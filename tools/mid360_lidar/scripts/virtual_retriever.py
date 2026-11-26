#!/usr/bin/python3
"""Publish virtual-template candidates only; never applies TF or updates a real bank."""
import argparse,json
from pathlib import Path
import rclpy
from keyframe_retriever import Retriever
from virtual_database import VirtualDatabase,DEFAULT_VIRTUAL_CONFIG
class VirtualRetriever(Retriever):
 def compute_query(self,q):
  result=super().compute_query(q)
  self.get_logger().info('VIRTUAL_QUERY '+json.dumps(self.db.last_metrics))
  return result
def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=DEFAULT_VIRTUAL_CONFIG);p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--race-controlled',action='store_true');a,ros=p.parse_known_args()
 a.node_name='virtual_retriever';a.query_label='VIRTUAL_QUERY';rclpy.init(args=ros);node=VirtualRetriever(a,VirtualDatabase(a.map,a.config))
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  if node.file:node.file.close()
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
