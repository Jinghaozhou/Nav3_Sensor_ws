"""Exercise imported TF path recorder and visualization without a physical radar.

Run only in an isolated ROS domain with the integrated workspace sourced.
"""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise SystemExit('Set ROS_DOMAIN_ID=88 to isolate synthetic TF from live scans')
    rclpy.init()
    node = Node('test_imported_tf_path')
    broadcaster = TransformBroadcaster(node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received = {key: [] for key in ('path', 'pose', 'display_path', 'arrow')}
    for msg_type, topic, key in [
        (PathMsg, '/localization/map_path', 'path'),
        (PoseStamped, '/localization/map_pose', 'pose'),
        (PathMsg, '/localization_visualization/comparison/map_path', 'display_path'),
        (Marker, '/localization_visualization/comparison/current_pose', 'arrow'),
    ]:
        node.create_subscription(msg_type, topic,
                                 lambda msg, key=key: received[key].append(msg), qos)
    processes = []
    with tempfile.TemporaryDirectory(prefix='tf_path_test_') as folder:
        logs = []
        try:
            for package, executable, namespace in [
                ('localization_path_recorder', 'localization_path_recorder_node', '/localization'),
                ('localization_visualizer', 'localization_visualizer_node', '/localization_visualization'),
            ]:
                log = (Path(folder) / (package + '.log')).open('w')
                logs.append(log)
                binary = Path(get_package_prefix(package)) / 'lib' / package / executable
                processes.append(subprocess.Popen(
                    [str(binary), '--ros-args', '-r', '__ns:=' + namespace],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

            def advance(seconds, odom_x=None, correction=10.):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    assert all(p.poll() is None for p in processes), 'node exited'
                    if odom_x is not None:
                        stamp = node.get_clock().now().to_msg()
                        transforms = []
                        for parent, child, x in [('map', 'odom', correction),
                                                  ('odom', 'base_link', odom_x)]:
                            tf = TransformStamped()
                            tf.header.stamp = stamp
                            tf.header.frame_id = parent
                            tf.child_frame_id = child
                            tf.transform.translation.x = x
                            tf.transform.rotation.w = 1.
                            transforms.append(tf)
                        broadcaster.sendTransform(transforms)
                    rclpy.spin_once(node, timeout_sec=.03)

            advance(1.)
            assert not received['path'], 'must not publish a pose without TF'
            for x in (0., .2, .4):
                advance(.9, x)
            assert received['path'] and len(received['path'][-1].poses) >= 3
            path = received['path'][-1]
            assert path.header.frame_id == 'map'
            assert abs(path.poses[0].pose.position.x - 10.) < .01
            assert abs(path.poses[-1].pose.position.x - 10.4) < .01
            advance(.6, .4)
            assert received['display_path'] and received['arrow']
            display = received['display_path'][-1]
            assert display.header.frame_id == 'localization_comparison'
            assert abs(display.poses[-1].pose.position.x - 10.4) < .01
            assert abs(received['arrow'][-1].pose.position.z - .35) < .01

            # TF record preserves old accepted coordinates; it does not reproject history.
            advance(.9, .6, correction=11.)
            path = received['path'][-1]
            assert abs(path.poses[0].pose.position.x - 10.) < .01
            assert abs(path.poses[-1].pose.position.x - 11.6) < .01
            # New listeners receive the latched path even when TF is no longer fresh.
            advance(1.5)
            count = len(received['pose'])
            late = []
            node.create_subscription(PathMsg, '/localization/map_path', late.append, qos)
            advance(.7)
            assert len(received['pose']) == count, 'stale TF must not create new poses'
            assert late and len(late[-1].poses) >= 4, 'late subscriber must receive path'
            print('PASS: wait for TF, composed map pose, incremental history, display frame, '
                  'current arrow, stale TF rejection, transient-local late subscription')
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            for log in logs:
                log.close()
                print(Path(log.name).read_text())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
