"""Save the active Fast-LIO map; bounded service wait and truthful exit status."""
import argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60.0)
    args, ros_args = parser.parse_known_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    rclpy.init(args=ros_args)
    node = Node('mid360_save_map')
    try:
        client = node.create_client(Trigger, '/map_save')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/map_save unavailable; keep Fast-LIO running')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            raise RuntimeError('Service response timed out; server may still be writing')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        prefix = 'Map saved to '
        if not response.message.startswith(prefix):
            raise RuntimeError('Unexpected response; use the patched fast_lio package')
        path = Path(response.message[len(prefix):])
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError('Service reported success but map file is missing: ' + str(path))
        print(str(path))
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
