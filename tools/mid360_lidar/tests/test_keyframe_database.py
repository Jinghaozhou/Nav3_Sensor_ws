import unittest,sys,json
from pathlib import Path
import numpy as np
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from keyframe_database import KeyframeDatabase,rigid
from scan_context_descriptor import load_xyz
R=Path('/home/pc/人形');L=R/'导航工程/mid360_lidar/reference_maps/keyframe_libraries/space_2/experiment_20261109_170630_971075';M=R/'导航工程/kiss_localization_ws1.1/src/localization/maps/building_split_space.pcd'
class Tests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.db=KeyframeDatabase(L,M)
 def test_all_queries_exactly_preserve_offline_ranking_and_pose(self):
  p=R/'联调记录/20261109_201727_独立查询验证';qs=[json.loads(x) for x in (p/'query_index.jsonl').read_text().splitlines()]
  import csv
  with (p/'rankings.csv').open() as f:ranks=list(csv.DictReader(f))
  for q in qs:
   h,_,_=self.db.retrieve(load_xyz(p/q['pcd']),q['up_base'],q['odom_T_base']);expected=[x for x in ranks if x['query']==q['id']]
   self.assertEqual([x['id'] for x in h],[x['reference'] for x in expected]);self.assertEqual(len(h),28)
   for c in h:self.assertTrue(rigid(c['map_T_odom']))
 def test_map_mismatch_rejected(self):
  with patch('keyframe_database.sha256',return_value='0'*64):
   with self.assertRaisesRegex(ValueError,'SHA mismatch'):KeyframeDatabase(L,M)
 def test_pose_and_gravity_rejected(self):
  with self.assertRaises(ValueError):self.db.retrieve(np.ones((1000,3)),[0,0,1],np.zeros((4,4)))
  with self.assertRaises(ValueError):self.db.retrieve(np.ones((1000,3)),[0,0,0],np.eye(4))
 def test_nan_not_rigid(self):self.assertFalse(rigid(np.full((4,4),np.nan)))
if __name__=='__main__':unittest.main()
