#!/usr/bin/env python3
"""Send an explicit current-location text/ID to the existing localization node."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('region', nargs='?', help='CSV region name, ID, current-location sentence, or 全图')
    parser.add_argument('--list', action='store_true', help='List known names/IDs without connecting to ROS')
    parser.add_argument('--labels', type=Path, default=Path(__file__).resolve().parents[2] / 'kiss_localization_ws1.1/src/localization/maps/region_labels.csv')
    parser.add_argument('--timeout', type=float, default=60.)
    args = parser.parse_args()
    if args.list:
        with args.labels.open(encoding='utf-8-sig', newline='') as source:
            for row in csv.DictReader(source):
                print(f"{row['region_id']}\t{row['region_name']}")
        return 0
    if args.region is None:
        parser.error('provide a current region or use --list')
    import rclpy
    from kiss_matcher_localization.srv import SetRegion
    rclpy.init()
    node = rclpy.create_node('set_localization_region')
    try:
        client = node.create_client(SetRegion, '/localization/set_region')
        if not client.wait_for_service(timeout_sec=5.):
            print('定位服务未启动：请先按手册启动终端2，并source当前定位工作空间。')
            return 1
        request = SetRegion.Request()
        request.region = args.region
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print('等待服务超时；请求可能仍排队，请查看定位终端REGION_HINT日志后再操作。')
            return 1
        result = future.result()
        print(json.dumps(dict(success=result.success, region_id=result.region_id,
                              region_name=result.region_name, target_points=result.target_points,
                              message=result.message), ensure_ascii=False, indent=2))
        return 0 if result.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
