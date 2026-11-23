#!/usr/bin/python3
"""Receive on the ROS thread; search immutable snapshots on one worker thread."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from kiss_matcher_localization.msg import LocalizationResult
from keyframe_retriever import Retriever
from bbs_database import BbsDatabase, DEFAULT_BBS_CONFIG

TEMPORARY_QUERY_REASONS = frozenset({
    'no_short_cloud', 'short_cloud_warmup', 'query_odom_missing',
    'estimated_gravity_missing_at_query_stamp', 'bbs_search_pending'})

class BbsRetriever(Retriever):
    def __init__(self,a,database):
        a.node_name='bbs_retriever';a.query_label='BBS_QUERY'
        database.packet_reason='bbs_competitors_complete'
        super().__init__(a,database)
        self.tracking=False;self.last_status=0
        self.epoch=0;self.have_ready=False;self.pending=None;self.active=None
        self.future=None;self.closing=False;self.dispatched_ns=None
        self.search_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='bbs_search')
        self.result_timer=self.create_timer(.05,self.poll_search)
        self.create_subscription(LocalizationResult,'/localization/status',self.tracking_status,10)
        self.get_logger().warn('BBS_INIT async: continuous ROS reception, one search worker, latest pending query only; all VGICP/ambiguity/two-window guards retained')

    def invalidate_work(self):
        self.epoch+=1;self.pending=None;self.dispatched_ns=None;self.have_ready=False

    def reset(self):
        super().reset()
        self.invalidate_work()
        self.unavailable('query_epoch_reset')

    def tracking_status(self,msg):
        previous=self.tracking
        self.tracking=msg.applied;self.last_status=time.monotonic()
        if self.tracking!=previous:
            self.invalidate_work()
            if not self.tracking:self.unavailable('bbs_search_pending')

    def unavailable(self,why):
        if why not in TEMPORARY_QUERY_REASONS:self.invalidate_work()
        # Waiting for the next query must not overwrite a still usable packet.
        # The consumer rejects stale packets and never recounts a duplicate.
        if why in TEMPORARY_QUERY_REASONS and self.have_ready:
            if why!=self.last_reason:self.get_logger().info('BBS_QUERY waiting: '+why)
            self.last_reason=why
            self.status.publish(String(data=json.dumps(dict(ready=False,reason=why,waiting=True))))
        else:
            self.have_ready=False
            super().unavailable(why)

    def searching_enabled(self):
        return not self.closing and (not self.race_controlled or bool(self.race_generation)) and not (self.tracking and time.monotonic()-self.last_status<5)

    def encode_query(self):
        if not self.searching_enabled():return
        q=self.prepare_query()
        if q is None:return
        if q['n']==self.dispatched_ns or (self.pending and q['n']==self.pending[1]['n']):return
        self.pending=(self.epoch,q)  # Bounded slot: a newer query replaces it.
        self.poll_search()

    def run_search(self,q):
        result=self.compute_query(q)
        return result,dict(self.db.last_metrics)

    def poll_search(self):
        if self.closing:return
        if self.future is not None and self.future.done():
            future=self.future;epoch,q=self.active
            self.future=None;self.active=None
            try:
                result,metrics=future.result()
                if epoch!=self.epoch or not self.searching_enabled():
                    self.get_logger().info('BBS_QUERY discarded: epoch/tracking changed')
                else:
                    age=(self.get_clock().now().nanoseconds-q['n'])/1e9
                    if age>10 or age< -1:self.unavailable('bbs_result_stale_or_future')
                    else:
                        self.publish_query(q,result);self.have_ready=True
                        self.get_logger().info('BBS_QUERY async search=%.1fms total=%.1fms points=%d peaks=%d complete=%s age=%.3fs'%(
                            metrics['search_ms'],metrics['total_ms'],metrics['points'],metrics['peaks'],metrics['complete'],age))
            except Exception as error:
                if epoch==self.epoch and self.searching_enabled():
                    self.unavailable(type(error).__name__+': '+str(error))
                    self.get_logger().error('BBS_QUERY rejected: '+str(error))
        if self.future is None and self.pending is not None and self.searching_enabled():
            epoch,q=self.pending;self.pending=None
            if epoch!=self.epoch:return
            age=(self.get_clock().now().nanoseconds-q['n'])/1e9
            if age>2 or age< -1:
                self.unavailable('short_cloud_stale_or_future');return
            self.active=(epoch,q);self.dispatched_ns=q['n']
            self.future=self.search_executor.submit(self.run_search,q)
            if not self.have_ready:self.unavailable('bbs_search_pending')

    def close_search(self):
        # Join before closing worker pipes or its diagnostic CSV.
        self.closing=True;self.timer.cancel();self.result_timer.cancel();self.pending=None
        self.search_executor.shutdown(wait=True,cancel_futures=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--race-controlled',action='store_true')
    p.add_argument('--config',type=Path,default=DEFAULT_BBS_CONFIG)
    p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path)
    a,ros=p.parse_known_args();rclpy.init(args=ros)
    db=None;node=None
    try:
        db=BbsDatabase(a.map,a.config,a.output);node=BbsRetriever(a,db);rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        # Humble may report a failed take when SIGINT invalidates the context.
        # Real runtime failures while ROS is live must still propagate.
        if rclpy.ok():raise
    finally:
        if node:
            node.close_search()
            if node.file:node.file.close()
            node.destroy_node()
        if db:db.close()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
