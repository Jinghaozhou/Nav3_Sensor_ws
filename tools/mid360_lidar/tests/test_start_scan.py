"""Process-lifecycle tests with local dummy children; no ROS nodes or hardware."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

script = Path(__file__).resolve().parents[1] / 'scripts/start_scan.py'
spec = importlib.util.spec_from_file_location('start_scan', script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
REAL_POPEN = subprocess.Popen
WORKER = '''import json,os,signal,sys,time
from pathlib import Path
name,events,session,ready,pointer,mode=sys.argv[1:]
def note(text):
 with open(events,'a') as f:f.write(text+'\\n')
def stop(*args):
 note(name+':stop_requested')
 time.sleep(.05)
 note(name+':flushed')
 sys.exit(0)
signal.signal(signal.SIGINT,stop)
if name=='recording':
 Path(session).mkdir()
 Path(pointer).write_text(session)
 if mode!='storage_timeout':Path(ready).write_text(json.dumps({'session':session}))
print(name+' console',flush=True)
note(name+':ready')
if name=='mapping' and mode=='early_exit':sys.exit(3)
while True:time.sleep(.02)
'''


class Lifecycle(unittest.TestCase):
    def exercise(self, mode, scan_mode="building5",auto_update=False):
        with tempfile.TemporaryDirectory(prefix='scan_lifecycle_') as tmp:
            root = Path(tmp); project = root / 'project'; project.mkdir(); (project/'bags').mkdir()
            (project/'config').mkdir();(project/'config/acquisition_profile.json').write_text('{}')
            worker = root/'worker.py'; worker.write_text(WORKER)
            events = root/'events.txt'; session = project/'bags/experiment_test'
            args = argparse.Namespace(mode=scan_mode, region='entry', bag_root=project/'bags',
                                      no_rviz=True, no_render=True,no_auto_library_update=not auto_update,keyframe_library=project/'frozen_library')
            scan = module.ScanSession(args)
            timer = None
            def popen(command, **kwargs):
                nonlocal timer
                if 'record_experiment.py' in str(command):
                    role='recording'; ready=command[command.index('--ready-file')+1]; pointer=command[command.index('--pointer-file')+1]
                else:
                    role='mapping' if 'mid360_mapping.launch.py' in str(command) else 'localization'; ready=pointer='unused'
                if role=='localization' and scan_mode=='bbs':
                    self.assertIn('candidate_source:=bbs',command)
                    self.assertIn('keyframe_initialization_enabled:=true',command)
                    self.assertNotIn('hierarchical_initialization_enabled:=true',command)
                child = REAL_POPEN([sys.executable,str(worker),role,str(events),str(session),ready,pointer,mode],**kwargs)
                if role=='localization':
                    timer=threading.Timer(.3,scan.request_stop);timer.start()
                return child
            def window():
                time.sleep(.2)
                scan.alive()
            def postprocess(command,**kwargs):
                self.assertTrue(all(child.poll() is not None for child in scan.children.values()))
                self.assertIn('auto_keyframe_update.py',str(command));scan.note('test_auto_hook_after_all_stopped')
                return subprocess.CompletedProcess(command,0)
            original_wait = scan.wait_storage
            with patch.object(module,'ROOT',project), patch.object(module.subprocess,'Popen',side_effect=popen), \
                 patch.object(module.subprocess,'run',side_effect=postprocess), \
                 patch.object(scan,'wait_window',side_effect=window), \
                 patch.object(scan,'wait_storage',side_effect=lambda ready:original_wait(ready,timeout=2 if mode!='storage_timeout' else .3)):
                status=scan.run()
            if timer:timer.join()
            lines=events.read_text().splitlines()
            self.assertTrue(all(child.poll() is not None for child in scan.children.values()))
            self.assertTrue((session/'recording_console.log').is_file())
            self.assertIn('recording console',(session/'recording_console.log').read_text())
            if mode=='normal':
                self.assertEqual(status,0)
                for before,after in [('recording:ready','mapping:ready'),('mapping:ready','localization:ready'),
                    ('localization:flushed','mapping:stop_requested'),('mapping:flushed','recording:stop_requested')]:
                    self.assertLess(lines.index(before),lines.index(after),lines)
                self.assertEqual(lines.count('recording:stop_requested'),1)
                self.assertEqual((project/'bags/latest_recording_path.txt').read_text().strip(),str(session))
            else:
                self.assertEqual(status,1)
                self.assertNotIn('localization:ready',lines)
            return lines

    def test_ordered_start_stop_and_flushing(self):
        self.exercise('normal')

    def test_bbs_mode_lifecycle_and_candidate_source(self):
        self.exercise('normal',scan_mode='bbs')

    def test_race_mode_and_post_session_admission_order(self):
        self.exercise('normal',scan_mode='race',auto_update=True)

    def test_component_exit_stops_remaining_children(self):
        self.exercise('early_exit')

    def test_storage_timeout_preserves_partial_session_and_recorder_log(self):
        self.exercise('storage_timeout')


if __name__=='__main__':
    unittest.main()
