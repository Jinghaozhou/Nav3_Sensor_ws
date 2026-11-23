import json,sys,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from virtual_database import VirtualDatabase,DEFAULT_VIRTUAL_CONFIG
from scan_context_descriptor import load_xyz
from workspace_paths import map_file
MAP=map_file()
class VirtualTests(unittest.TestCase):
 def test_wrong_map_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'wrong.pcd';p.write_text('wrong map')
   with self.assertRaisesRegex(ValueError,'mismatch'):VirtualDatabase(p)
 def test_changed_config_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   c=json.loads(DEFAULT_VIRTUAL_CONFIG.read_text());c['maximum_candidates']=23;p=Path(d)/'config.json';p.write_text(json.dumps(c))
   with self.assertRaisesRegex(ValueError,'mismatch'):VirtualDatabase(MAP,p)
 def test_templates_are_not_measured_references_and_self_query_is_bounded(self):
  db=VirtualDatabase(MAP);self.assertEqual(db.manifest['source'],'virtual_map_raycasts_not_measured_keyframes');self.assertFalse(db.manifest['field_validated']);self.assertEqual(len(db.index),24)
  row=db.templates[0];points=load_xyz(db.path/row['pcd']);h,_,_=db.retrieve(points,row['up_base'],np.eye(4))
  self.assertLessEqual(len(h),24);self.assertEqual(len({x['id'] for x in h}),len(h));self.assertTrue(any(np.linalg.norm(x['map_T_base']-np.array(row['map_T_base']))<1e-6 for x in h));self.assertTrue(db.last_metrics['shortlist_truncated'])
  self.assertEqual(db.packet_reason,'virtual_shortlist')
if __name__=='__main__':unittest.main()
