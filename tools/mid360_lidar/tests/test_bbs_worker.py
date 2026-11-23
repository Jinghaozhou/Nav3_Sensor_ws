"""Separated competing poses, incomplete search, and empty-source handling."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

class Worker(unittest.TestCase):
    def run_worker(self, maximum=8, timeout=3000, empty=False):
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp); rng=np.random.default_rng(5)
            source=rng.uniform([-2.,-1.5,-.8],[1.2,1.5,1.],(150,3)).astype('<f4')
            np.concatenate([source,source+[4.,0,0]]).astype('<f4').tofile(d/'target')
            (source[:0] if empty else source).tofile(d/'source')
            (d/'config').write_text(f'-.25 -.25 -.25 4.25 .25 .25 .25 3 2 {timeout} .5 .03 .6 .25 {maximum}\n')
            process=subprocess.run([str(ROOT/'build/bbs/bbs_worker'),str(d/'target'),str(d/'config')],
                input=str(d/'source')+'\n',capture_output=True,text=True,timeout=15)
            if empty:
                self.assertNotEqual(process.returncode,0); return
            self.assertEqual(process.returncode,0,process.stderr)
            return json.loads(process.stdout.splitlines()[-1])

    def test_two_equal_structures_are_both_retained(self):
        r=self.run_worker()
        self.assertTrue(r['complete']); self.assertFalse(r['timed_out'])
        x=[p['pose'][0][3] for p in r['peaks']]
        self.assertTrue(any(abs(v)<.3 for v in x),x)
        self.assertTrue(any(abs(v-4)<.3 for v in x),x)

    def test_candidate_limit_does_not_masquerade_as_complete(self):
        r=self.run_worker(maximum=1)
        self.assertTrue(r['peak_limit']); self.assertFalse(r['complete'])

    def test_timeout_does_not_masquerade_as_complete(self):
        r=self.run_worker(timeout=0)
        self.assertTrue(r['timed_out']); self.assertFalse(r['complete'])

    def test_empty_source_exits_without_crash(self): self.run_worker(empty=True)

if __name__=='__main__': unittest.main()
