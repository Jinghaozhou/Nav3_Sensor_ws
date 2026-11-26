#!/usr/bin/python3
"""Coordinate two private, independently confirmed validators and one TF owner."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import UInt64
from kiss_matcher_localization.msg import LocalizationResult
from kiss_matcher_localization.srv import RaceInitialization
from build_keyframe_library import ns


class RaceCoordinator(Node):
    def __init__(self, args):
        super().__init__('initialization_race')
        self.versions=json.loads(args.versions.read_text())
        if self.versions.get('sources', ['bbs','keyframes']) not in (['bbs','virtual'], ['bbs','keyframes']):
            raise ValueError('Unsupported race sources')
        self.generation=0;self.epoch=0;self.start_ns=0;self.waiting=False;self.pending=None
        self.queue=[];self.call=None;self.last_query=0
        self.control=self.create_publisher(UInt64,'/localization/race_control',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client=self.create_client(RaceInitialization,'/localization/race_initialization')
        for source in self.versions.get('sources', ['bbs','keyframes']):
            self.create_subscription(LocalizationResult,f'/localization_race/{source}/status',lambda m,s=source:self.proposal(s,m),10)
        args.output.mkdir(parents=True,exist_ok=True)
        self.log=(args.output/'race_events.jsonl').open('x')
        self.timer=self.create_timer(.05,self.tick)

    def note(self,**data):
        self.log.write(json.dumps(dict(time_ns=self.get_clock().now().nanoseconds,epoch=self.epoch,**data))+'\n');self.log.flush()

    def proposal(self,source,msg):
        # A first-window matched result never enters the race. Keep sources separate.
        if self.waiting and msg.matched and msg.applied and msg.reason=='applied' and ns(msg.local_map_stamp)>=self.start_ns:
            self.queue.append((source,msg));self.note(event='confirmed_proposal',source=source,cloud_ns=ns(msg.local_map_stamp),overlap=msg.overlap_percent,registration_ms=msg.registration_ms)

    def tick(self):
        if self.call is not None:
            if not self.call.done():return
            try:r=self.call.result()
            except Exception as e:
                self.note(event='service_error',reason=str(e));self.call=None;return
            submitted=self.pending;self.pending=None;self.call=None
            if submitted:
                self.note(event='decision',source=submitted[0],applied=r.result.applied,reason=r.result.reason)
                self.get_logger().info(f'RACE source={submitted[0]} epoch={self.epoch} applied={r.result.applied} reason={r.result.reason} overlap={r.result.overlap_percent:.2f}%')
            if r.waiting and r.epoch!=self.epoch:
                self.epoch=r.epoch;self.start_ns=self.get_clock().now().nanoseconds;self.queue=[]
                self.waiting=True;self.generation+=1;self.control.publish(UInt64(data=self.generation));self.note(event='race_open')
            elif not r.waiting and self.waiting:
                self.waiting=False;self.queue=[];self.control.publish(UInt64(data=0));self.note(event='race_closed',winner=submitted[0] if submitted else 'owner')
            elif submitted and not r.result.applied and self.waiting:
                # Private validator stops after confirmation. Re-arm both after a rejected/stale delivery.
                self.start_ns=self.get_clock().now().nanoseconds;self.queue=[]
                self.generation+=1;self.control.publish(UInt64(data=self.generation))
            return
        if not self.client.service_is_ready():return
        request=RaceInitialization.Request()
        if self.waiting and self.queue:
            source,msg=self.queue.pop(0);self.pending=(source,msg)
            request.submit=True;request.epoch=self.epoch;request.source=source;request.proposal=msg
            request.map_sha256=self.versions['map_sha256'];request.profile_sha256=self.versions[source+'_profile_sha256']
            request.library_sha256=self.versions[source+'_sha256']
        elif time.monotonic()-self.last_query<.5:return
        self.last_query=time.monotonic();self.call=self.client.call_async(request)


def main():
    p=argparse.ArgumentParser();p.add_argument('--versions',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a,ros=p.parse_known_args();rclpy.init(args=ros);node=RaceCoordinator(a)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
