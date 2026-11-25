import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from region_tracking import RegionClassifier,RegionState
from region_profiles import region_number,virtual_config_for_region
class RegionTests(unittest.TestCase):
 def test_indexed_support_matches_full_scan_with_multiple_heights(self):
  from scipy.spatial import cKDTree
  from build_virtual_map_library import support_neighborhood
  rng=np.random.default_rng(42);support=rng.uniform(-2,2,(3000,3));support=np.concatenate([support,[[.24,0,0],[0,0,.11],[0,0,0]]]);tree=cKDTree(support[:,:2])
  for foot in np.concatenate([rng.uniform(-1,1,(30,3)),[[0,0,0]]]):
   expected=support[(np.linalg.norm(support[:,:2]-foot[:2],axis=1)<.24)&(abs(support[:,2]-foot[2])<.11)]
   actual=support_neighborhood(tree,support,foot)
   self.assertEqual(sorted(map(tuple,expected)),sorted(map(tuple,actual)))
 def test_route_all_and_reject_typos(self):
  for i in range(1,14):self.assertEqual(region_number(f'space_{i}'),i);self.assertTrue(virtual_config_for_region(i).exists())
  for r in ('space_0','space_14','space_01','x2','2.0'):self.assertRaises(ValueError,region_number,r)
 def test_three_dimensions_distinguish_floors(self):
  rng=np.random.default_rng(8);a=rng.normal(0,.08,(100,3));xyz=np.concatenate([a,a+[0,0,3]])
  c=RegionClassifier(xyz,[2]*100+[3]*100)
  self.assertEqual(c.classify([0,0,.2])[0],2);self.assertEqual(c.classify([0,0,3.2])[0],3);self.assertEqual(c.classify([10,10,10])[0],0)
 def test_ambiguous_boundary_rejected(self):
  x=np.linspace(-.1,.1,48);a=np.column_stack([x,np.zeros(48),np.zeros(48)])
  c=RegionClassifier(np.concatenate([a,a]),[2]*48+[3]*48)
  self.assertEqual(c.classify([0,0,.5])[0],0)
 def test_confirm_switch_no_boundary_chatter(self):
  s=RegionState();self.assertIsNone(s.update(2,0));self.assertIsNone(s.update(2,1));self.assertEqual(s.update(2,2)['kind'],'initial')
  self.assertIsNone(s.update(3,3));self.assertIsNone(s.update(2,4));self.assertIsNone(s.update(3,5));self.assertIsNone(s.update(3,6));e=s.update(3,7);self.assertEqual((e['from_region'],e['to_region']),(2,3))
 def test_duplicate_stale_reset_and_recovery(self):
  s=RegionState();s.update(2,0)
  for _ in range(10):self.assertIsNone(s.update(2,0))
  s.update(2,1);s.update(2,2);s.update(0,3,False);self.assertEqual(s.current,0);self.assertEqual(s.last_known,2)
  s.update(2,4);s.update(2,5);self.assertEqual(s.update(2,6)['kind'],'recovered')
  s.update(3,.1);self.assertEqual(s.current,0);self.assertEqual(s.last_known,0)
if __name__=='__main__':unittest.main()
