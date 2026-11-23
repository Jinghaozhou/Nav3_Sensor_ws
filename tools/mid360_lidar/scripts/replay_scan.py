#!/usr/bin/env python3
"""Replay recorded displays, without starting sensors or localization algorithms."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile

from record_experiment import ROOT, write_qos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='Experiment directory or its bag subdirectory')
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--domain', type=int, default=88, help='Dedicated replay ROS domain')
    parser.add_argument('--two-rviz', action='store_true', help='Also show recorded Fast-LIO displays')
    args = parser.parse_args()
    if args.rate <= 0 or not 0 <= args.domain <= 232:
        parser.error('rate must be positive; domain must be between 0 and 232')
    session = args.session.expanduser().resolve()
    bag = session if (session/'metadata.yaml').is_file() else session/'bag'
    if not (bag/'metadata.yaml').is_file():
        parser.error(f'Finalized bag metadata not found: {bag}')
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    children = []
    try:
        with tempfile.TemporaryDirectory(prefix='mid360_replay_') as temp:
            qos = Path(temp)/'playback_qos.yaml'
            write_qos(qos)
            configs = ['continuous_localization.rviz']
            if args.two_rviz:
                configs.append('fastlio_map.rviz')
            # Own RViz children only; do not touch any existing processes.
            logs = []
            for config in configs:
                log = open(Path(temp)/(config+'.log'), 'w')
                logs.append(log)
                children.append(subprocess.Popen(
                    ['rviz2', '-d', str(ROOT/'rviz'/config), '--ros-args',
                     '-p', 'use_sim_time:=true'], env=env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True))
            print(f'回放域 ROS_DOMAIN_ID={args.domain}，仅回放保存结果，不重新计算定位。', flush=True)
            print('等待RViz打开后，在本终端按回车开始；播放中空格暂停/继续，Ctrl-C退出。', flush=True)
            input()
            result = subprocess.run(
                ['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(args.rate),
                 '--qos-profile-overrides-path', str(qos)], env=env)
            print('播放结束。可在RViz中查看最后状态；按回车关闭本脚本打开的窗口。', flush=True)
            input()
            return result.returncode
    except (KeyboardInterrupt, EOFError):
        return 0
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.terminate()
        for log in locals().get('logs', []):
            log.close()


if __name__ == '__main__':
    raise SystemExit(main())
