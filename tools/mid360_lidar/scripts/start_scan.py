#!/usr/bin/python3
"""Supervise recorder -> mapping -> localization; stop in reverse order."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


class ScanSession:
    def __init__(self, args):
        self.args = args
        self.children = {}
        self.logs = []
        self.stopping = False
        self.session = None
        self.events = []
        self.stopped = set()

    def note(self, event, **details):
        self.events.append(dict(time=datetime.now().astimezone().isoformat(), event=event, **details))
        if self.session:
            (self.session / 'one_click_session.json').write_text(json.dumps(
                dict(mode=self.args.mode, events=self.events), ensure_ascii=False, indent=2))

    def request_stop(self, *_):
        if not self.stopping:
            print('\n正在按顺序结束：定位 → 雷达 → 录包，请等待保存完成。', flush=True)
        self.stopping = True

    def start(self, name, command, logfile=None):
        output = None
        if logfile:
            output = logfile.open('w')
            self.logs.append(output)
        process = subprocess.Popen(command, cwd=ROOT, start_new_session=True,
                                   stdout=output, stderr=subprocess.STDOUT if output else None)
        self.children[name] = process
        self.note('started', component=name, pid=process.pid, command=command)
        return process

    def alive(self):
        if self.stopping:
            raise InterruptedError('user_stop')
        for name, process in self.children.items():
            if process.poll() is not None:
                raise RuntimeError(f'{name} 提前退出，返回值 {process.returncode}；结束整轮以避免继续扫描但未录制。')

    def wait_storage(self, ready, timeout=30):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.alive()
            if ready.is_file():
                return Path(json.loads(ready.read_text())['session'])
            time.sleep(.2)
        raise TimeoutError('录包存储未在30秒内准备好')

    def wait_window(self, timeout=90):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('one_click_wait_window_' + str(os.getpid()))
        received = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(PointCloud2, '/fast_lio/local_map_global',
                                 lambda msg: received.append(msg.header.stamp) if
                                 msg.header.frame_id == 'odom' and msg.width * msg.height > 0 else None, qos)
        end = time.monotonic() + timeout
        try:
            while not received and time.monotonic() < end:
                self.alive()
                rclpy.spin_once(node, timeout_sec=.2)
            if not received:
                raise TimeoutError('90秒未收到有效长窗口，请检查雷达连接；详见radar_console.log')
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def stop_all(self):
        # Children have separate process groups: the terminal SIGINT only reaches us.
        for name in ('localization', 'mapping', 'recording'):
            child = self.children.get(name)
            if child is None or name in self.stopped:
                continue
            if name == 'recording' and child.poll() is None:
                time.sleep(.5)  # Allow final queued status/TF messages to reach the recorder.
            if child.poll() is None:
                self.note('stopping', component=name, pid=child.pid)
                print(f'结束 {name}，等待退出……', flush=True)
                try:
                    # ROS launch / recorder supervisors forward SIGINT themselves.
                    # Signalling their whole group can deliver it twice to children.
                    child.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                while child.poll() is None:
                    try:
                        child.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        print(f'{name} 仍在保存/退出，继续等待，不强杀。', flush=True)
            self.note('stopped', component=name, returncode=child.returncode)
            self.stopped.add(name)
        for stream in self.logs:
            stream.close()

    def run(self):
        status = 0
        try:
            with tempfile.TemporaryDirectory(prefix='mid360_start_') as control:
                control = Path(control)
                ready, pointer = control / 'ready.json', control / 'session.txt'
                try:
                    self.start('recording', ['/usr/bin/python3', str(ROOT / 'scripts/record_experiment.py'),
                        '--root', str(self.args.bag_root), '--ready-file', str(ready),
                        '--pointer-file', str(pointer)], control / 'recording.log')
                    self.session = self.wait_storage(ready)
                    (ROOT / 'bags/latest_recording_path.txt').write_text(str(self.session) + '\n')
                    self.note('storage_ready', session=str(self.session))
                    shutil.copy2(ROOT/'config/acquisition_profile.json',self.session/'acquisition_profile.json')
                    print(f'本轮输出：{self.session}', flush=True)
                    self.start('mapping', ['ros2', 'launch', str(ROOT / 'launch/mid360_mapping.launch.py'),
                        'rviz:=' + str(not self.args.no_rviz).lower(), 'initial_window:=120',
                        'tracking_window:=30', 'window_frames:=1500'], self.session / 'radar_console.log')
                    print('录包已建立，等待雷达和Fast-LIO输出有效窗口……', flush=True)
                    self.wait_window()
                    self.note('window_ready')
                    command = ['ros2', 'launch', str(ROOT / ('launch/race_localization.launch.py' if self.args.mode=='race' else 'launch/continuous_localization.launch.py')),
                        'rviz:=' + str(not self.args.no_rviz).lower(), 'record_inputs:=true',
                        'output:=' + str(self.session), 'initial_region:=' + self.args.region,
                        'map_pcd:=' + str(map_file())]
                    if self.args.mode in ('building5','keyframes','hierarchical','bbs','virtual'):
                        command += ['region_multistart_enabled:=true',
                                    'region_seed_file:=' + str(ROOT / ('config/region_initialization_all_spaces.yaml' if self.args.mode in ('bbs','virtual','race') else 'config/region_initialization_space.yaml'))]
                    if self.args.mode in ('keyframes','hierarchical'):
                        command += ['keyframe_library:=' + str(self.args.keyframe_library)]
                        if self.args.mode!='race':command += ['keyframe_initialization_enabled:=true']
                        if self.args.mode == 'hierarchical': command += ['hierarchical_initialization_enabled:=true']
                    if self.args.mode == 'bbs':
                        from bbs_database import bbs_config_for_region
                        command += ['keyframe_initialization_enabled:=true','candidate_source:=bbs', 'bbs_config:='+str(bbs_config_for_region(self.args.region))]
                    if self.args.mode=='virtual':
                        from region_profiles import virtual_config_for_region
                        command += ['keyframe_initialization_enabled:=true','candidate_source:=virtual','hierarchical_initialization_enabled:=true','virtual_config:='+str(virtual_config_for_region(self.args.region))]
                    self.start('localization', command)
                    print('三部分均已启动；保留此终端，按一次Ctrl+C结束整轮。', flush=True)
                    while True:
                        self.alive()
                        time.sleep(.3)
                finally:
                    if self.session is None and pointer.is_file():
                        self.session = Path(pointer.read_text().strip())
                    self.stop_all()
                    if self.session and (control / 'recording.log').is_file():
                        shutil.copy2(control / 'recording.log', self.session / 'recording_console.log')
        except InterruptedError:
            self.note('requested_stop')
        except Exception as error:
            status = 1
            self.note('error', message=str(error))
            print(f'本轮启动/运行异常：{error}', flush=True)
        finally:
            self.stop_all()
        if self.session:
            print(f'保存位置：{self.session}', flush=True)
            if not self.args.no_render and (self.session / 'matching_metrics.csv').is_file():
                result = subprocess.run(['/usr/bin/python3', str(ROOT / 'scripts/summarize_localization.py'),
                                         str(self.session)], cwd=ROOT)
                self.note('postprocess_finished', returncode=result.returncode)
                if result.returncode:
                    status = 1
            if not getattr(self.args,'no_auto_library_update',False):
                result = subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/auto_keyframe_update.py'),str(self.session),'--map',str(map_file())],cwd=ROOT)
                self.note('auto_library_update_finished',returncode=result.returncode)
                if result.returncode:print('自动补库未通过，保留原库；请查看本轮auto_library_update.json。',flush=True)
            print('本轮已结束，原始bag及现有输出均保留。', flush=True)
        return status


def map_file():
    from workspace_paths import map_file as resolve_map
    return resolve_map()


def preflight(args):
    if args.mode=='race' and args.keyframe_library is not None:
        raise RuntimeError('race现使用BBS＋虚拟模板，不接受--keyframe-library；实采库对照请选择hierarchical或keyframes')
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    for package in ['livox_ros_driver2', 'fast_lio', 'kiss_matcher_localization',
                    'localization_visualizer', 'localization_path_recorder']:
        get_package_share_directory(package)
    args.bag_root = args.bag_root.expanduser().resolve()
    args.bag_root.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(args.bag_root).free / 2**30
    if free < 2.5:
        raise RuntimeError(f'剩余空间仅{free:.2f}GiB，不能开始录包')
    region_index=ROOT/'reference_maps/region_tracking/manifest.json'
    if not region_index.is_file():raise RuntimeError('区域维护索引未生成，请先运行prepare_region_tracking.py')
    if args.mode in ('building5','keyframes','hierarchical','bbs','race','virtual'):
        import yaml
        profile_path = ROOT / ('config/region_initialization_all_spaces.yaml' if args.mode in ('bbs','virtual','race') else 'config/region_initialization_space.yaml')
        profile = yaml.safe_load(profile_path.read_text())
        h = hashlib.sha256()
        with map_file().open('rb') as source:
            for chunk in iter(lambda: source.read(8*1024*1024), b''):
                h.update(chunk)
        if h.hexdigest() != profile['map_sha256']:
            raise RuntimeError('补全地图与候选库哈希不匹配')
        index_meta=json.loads(region_index.read_text())
        if index_meta['map_sha256']!=h.hexdigest():raise RuntimeError('区域索引属于其他地图，请重新生成')
        from build_keyframe_library import sha256
        if sha256(region_index.parent/'regions.npz')!=index_meta['index_sha256']:raise RuntimeError('区域索引文件校验失败')
    if args.mode in ('keyframes','hierarchical'):
        if args.region not in ('space_2','2'):raise RuntimeError('当前关键帧试验库仅支持space_2')
        from keyframe_database import KeyframeDatabase,validate_sensor_config,default_library
        args.keyframe_library = (args.keyframe_library or default_library()).expanduser().resolve()
        database=KeyframeDatabase(args.keyframe_library,map_file())
        args.keyframe_count=len(database.index)
        validate_sensor_config(ROOT/'mid360_robot.yaml',database)
        from ament_index_python.packages import get_package_prefix
        fastlio=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
        if b'/fast_lio/gravity' not in fastlio.read_bytes():raise RuntimeError('当前Fast-LIO尚无gravity话题，请重新编译/安装更新后的雷达工程')
    if args.mode in ('virtual','race'):
        from region_profiles import virtual_config_for_region
        from virtual_database import VirtualDatabase
        from keyframe_database import validate_sensor_config
        database=VirtualDatabase(map_file(),virtual_config_for_region(args.region));args.virtual_station_count=database.manifest['stations'];args.virtual_template_count=len(database.templates);validate_sensor_config(ROOT/'mid360_robot.yaml',database)
        from ament_index_python.packages import get_package_prefix
        binary=Path(get_package_prefix('kiss_matcher_localization'))/'lib/kiss_matcher_localization/navigation_localizer'
        if b'virtual_shortlist' not in binary.read_bytes():raise RuntimeError('请先编译安装虚拟候选消费支持')
        fastlio=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
        if b'/fast_lio/gravity' not in fastlio.read_bytes():raise RuntimeError('Fast-LIO缺少gravity输出')
    if args.mode in ('bbs','race'):

        from bbs_database import BbsDatabase,bbs_config_for_region
        from keyframe_database import validate_sensor_config
        database=BbsDatabase(map_file(),bbs_config_for_region(args.region))
        try: validate_sensor_config(ROOT/'mid360_robot.yaml',database)
        finally: database.close()
        from ament_index_python.packages import get_package_prefix
        installed=Path(get_package_prefix('kiss_matcher_localization'))/'lib/kiss_matcher_localization/navigation_localizer'
        if args.mode=='race' and b'race_virtual_sha256' not in installed.read_bytes():raise RuntimeError('请先编译安装BBS＋虚拟模板双路初始化定位器')
        if b'dynamic_candidates_only' not in installed.read_bytes(): raise RuntimeError('请先编译安装支持动态区域候选的定位器')
        if b'candidate_source' not in installed.read_bytes(): raise RuntimeError('定位器尚未安装BBS候选校验版本，请先编译')
        fastlio=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
        if b'/fast_lio/gravity' not in fastlio.read_bytes(): raise RuntimeError('当前Fast-LIO缺少gravity输出')
    rclpy.init(args=[])
    node = rclpy.create_node('one_click_preflight_' + str(os.getpid()))
    try:
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        critical = {'region_tracker','virtual_retriever','laser_mapping', 'livox_lidar_publisher', 'local_map_window', 'navigation_localizer',
                    'rosbag2_recorder', 'keyframe_retriever', 'bbs_retriever', 'initialization_race'}
        active = sorted({name for name, namespace in node.get_node_names_and_namespaces()} & critical)
        if active:
            raise RuntimeError('当前ROS域已存在扫描/录包节点，请先结束上一轮：' + ', '.join(active))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'预检通过：环境、地图、空间({free:.1f}GiB)、当前ROS域。模式={args.mode}', flush=True)
    if args.mode=='race':
        print(f'双路初始化：{args.region} BBS＋虚拟模板（{args.virtual_station_count}站位/{args.virtual_template_count}模板）；各自VGICP、歧义及两窗确认，主节点应用首个安全结果；不读取实采库。',flush=True)
        print('结束录包后'+('跳过自动补库（独立对照）。' if args.no_auto_library_update else '自动筛选space_2补库，下轮使用；原40帧冻结保留。'),flush=True)
    if args.mode=='virtual':
        print(f'虚拟模板：{args.region}，{args.virtual_station_count}个待核验站位/{args.virtual_template_count}模板→描述子Top24去重候选→分层VGICP→歧义/两窗；不是实采库，不自动补库。',flush=True)
    if args.mode == 'bbs':
        print(f'BBS地图搜索：{args.region}，全区域位置/朝向搜索→VGICP→歧义检查→两窗；需独立实扫核验。',flush=True)
    if args.mode in ('keyframes', 'hierarchical'):
        print(f'关键帧库：{args.keyframe_library}；候选数={len(database.index)}；实验模式，仍需现场核对位置和楼层。', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--keyframe-library', type=Path, help='Override the shared experimental reference library')
    parser.add_argument('--mode', choices=['building5', 'keyframes', 'hierarchical', 'bbs', 'race', 'virtual'], default='race',
                        help='race（默认）：BBS与虚拟模板竞争； building5: fixed VGICP seeds; keyframes: exhaustive database trial; hierarchical: grouped database search; bbs: map-only competing-peak BBS trial; virtual: selected region map-raycast template shortlist')
    parser.add_argument('--region', default='space_2', help='Actual current region, not navigation destination')
    parser.add_argument('--bag-root', type=Path, default=ROOT / 'bags')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--no-auto-library-update',action='store_true',help='Freeze reference selection for an independent experiment; no post-session admission')
    parser.add_argument('--no-render', action='store_true')
    parser.add_argument('--check', action='store_true', help='Read-only checks; do not start hardware or record')
    args = parser.parse_args()
    if args.mode in ('bbs','virtual','race') and args.region not in ('space_2','2'):
        args.no_auto_library_update=True
    try:
        preflight(args)
    except Exception as error:
        parser.exit(1, f'预检未通过：{error}\n')
    if args.check:
        return 0
    scan = ScanSession(args)
    signal.signal(signal.SIGINT, scan.request_stop)
    signal.signal(signal.SIGTERM, scan.request_stop)
    return scan.run()


if __name__ == '__main__':
    raise SystemExit(main())
