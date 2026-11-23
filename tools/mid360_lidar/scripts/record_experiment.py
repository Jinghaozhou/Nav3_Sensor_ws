#!/usr/bin/env python3
"""Record raw sensors and exact localization inputs with a disk-space guard."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
TOPICS = ['/localization/selected_cloud', '/fast_lio/gravity', '/localization/keyframe_candidates', '/localization/keyframe_status', '/lidar_points', '/imu', '/fast_lio/odom', '/fast_lio/local_cloud',
          '/fast_lio/window_status', '/localization/input_snapshot',
          '/localization/status', '/localization/path', '/localization/match_markers',
          '/localization/map_path', '/localization/map_pose', '/tf', '/tf_static',
          '/fast_lio/local_map', '/fast_lio/local_map_global', '/Laser_map', '/path',
          '/localization_visualization/comparison/global_map',
          '/localization_visualization/comparison/local_map',
          '/localization_visualization/comparison/aligned_local_map',
          '/localization_visualization/comparison/labels',
          '/localization_visualization/comparison/map_path',
          '/localization_visualization/comparison/current_pose']

# Preserve one-shot reference maps and match the transient-local RViz subscriptions.
# Private evidence cannot replace the authoritative /localization/status.
TOPICS += ['/localization/race_control'] + [f'/localization_race/{source}/{topic}'
    for source in ('bbs','keyframes') for topic in ('status','input_snapshot','selected_cloud','keyframe_candidates','query_status')]
LATCHED_TOPICS = ['/tf_static', '/localization/status', '/localization/path',
                  '/localization/match_markers', '/localization/map_path',
                  '/localization/map_pose'] + [t for t in TOPICS if '/comparison/' in t]


LATCHED_TOPICS += ['/localization/race_control','/localization_race/bbs/status','/localization_race/keyframes/status']

def write_qos(path):
    path.write_text(''.join(
        f'{topic}:\n  reliability: reliable\n  durability: transient_local\n'
        f'  history: keep_last\n  depth: {100 if topic == "/tf_static" else 1}\n'
        for topic in LATCHED_TOPICS))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'bags', help='Recording parent directory')
    parser.add_argument('--min-free-gb', type=float, default=2., help='Stop recorder below this free space')
    parser.add_argument('--pointer-file', type=Path, default=ROOT / 'bags/latest_recording_path.txt',
                        help='File containing the new experiment directory for the localization terminal')
    parser.add_argument('--ready-file', type=Path,
                        help='Optional supervisor handshake, written after recorder storage opens')
    args = parser.parse_args()
    if args.min_free_gb < 1:
        parser.error('Keep at least 1GiB of free-space reserve')
    if not shutil.which('ros2'):
        parser.error('Source ROS Humble, Livox and the integrated KISS workspace first')
    # Confirm required message support without subscribing to large raw messages in Python.
    from livox_ros_driver2.msg import CustomMsg  # noqa: F401
    from kiss_matcher_localization.msg import LocalizationInput  # noqa: F401
    args.root = args.root.expanduser().resolve()
    args.root.mkdir(parents=True, exist_ok=True)
    available = shutil.disk_usage(args.root).free / 2**30
    if available < args.min_free_gb + .5:
        parser.error(f'Only {available:.2f}GiB free; choose another --root or free space first')
    session = args.root / datetime.now().strftime('experiment_%Y%m%d_%H%M%S_%f')
    session.mkdir()
    (session / 'source_configs').mkdir()
    for source in [ROOT / 'mid360_robot.yaml', ROOT / 'config/continuous_localization.yaml']:
        shutil.copy2(source, session / 'source_configs' / source.name)
    # Save the driver's loaded-package configuration as device provenance.
    # This is configured IP information, not a device serial or live parameter proof.
    from ament_index_python.packages import get_package_share_directory
    driver_config = Path(get_package_share_directory('livox_ros_driver2')) / 'config/MID360_config.json'
    shutil.copy2(driver_config, session / 'source_configs/MID360_config.json')
    # These are source configuration snapshots, not a claim about runtime overrides.
    qos = session / 'bag_qos.yaml'
    write_qos(qos)
    bag = session / 'bag'
    command = ['ros2', 'bag', 'record', '--storage', 'sqlite3', '--max-bag-size',
               '1073741824', '--max-cache-size', '134217728',
               '--qos-profile-overrides-path', str(qos), '-o', str(bag), *TOPICS]
    info = {'session': str(session), 'topics': TOPICS, 'command': command,
            'driver_config_snapshot_source': str(driver_config),
            'device_identity_note': 'Driver JSON IP is configuration, not a verified serial number',
            'created_local': datetime.now().astimezone().isoformat(),
            'source_config_note': 'Source files only; runtime launch overrides may differ',
            'free_gib_at_start': available, 'free_space_reserve_gib': args.min_free_gb}
    (session / 'recording.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
    args.pointer_file.parent.mkdir(parents=True, exist_ok=True)
    args.pointer_file.write_text(str(session) + '\n')
    print(f'实验目录：{session}', flush=True)
    print(f'可用空间 {available:.2f}GiB；低于 {args.min_free_gb:.1f}GiB时只停止录包，不停止扫描节点。', flush=True)
    print('先保持本终端运行，再启动建图；定位终端使用：', flush=True)
    print('ros2 launch launch/continuous_localization.launch.py rviz:=true record_inputs:=true '
          + shlex.quote('output:=' + str(session)), flush=True)
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    reason = 'recorder_exited'
    child = subprocess.Popen(command, start_new_session=True)
    ready_written = False
    try:
        while child.poll() is None:
            if args.ready_file and not ready_written and any(bag.glob('*.db3')):
                temporary = args.ready_file.with_name(args.ready_file.name + '.tmp')
                temporary.write_text(json.dumps({'session': str(session), 'recorder_pid': child.pid}))
                temporary.replace(args.ready_file)
                ready_written = True
            free = shutil.disk_usage(session).free / 2**30
            if stopping or free < args.min_free_gb:
                reason = 'user_stop' if stopping else 'low_disk_space'
                print(f'结束录包：{reason}，等待写盘完成……', flush=True)
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    print('录包仍在退出；请检查进程，不强杀以避免破坏数据。', flush=True)
                    child.wait()
                break
            time.sleep(1)
    finally:
        # If a Python error occurs, request a graceful shutdown of this recorder only.
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait()
        info.update(stop_reason=reason, recorder_exit_code=child.returncode,
                    stopped_local=datetime.now().astimezone().isoformat())
        (session / 'recording.json').write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
    result = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True)
    (session / 'bag_info.txt').write_text(result.stdout + result.stderr)
    print(result.stdout or result.stderr, flush=True)
    print('核对 /lidar_points、/imu、/localization/input_snapshot 和 /fast_lio/window_status 的消息数。', flush=True)
    return result.returncode or (child.returncode if child.returncode and child.returncode > 0 else 0)


if __name__ == '__main__':
    raise SystemExit(main())
