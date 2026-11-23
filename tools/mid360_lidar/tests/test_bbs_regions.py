"""Region routing must keep independent BBS models and reject implicit fallbacks."""
import sys,json,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from bbs_database import bbs_config_for_region
class Regions(unittest.TestCase):
 def test_aliases(self):
  for n in (2,10):self.assertEqual(bbs_config_for_region(n),bbs_config_for_region(f'space_{n}'))
 def test_independent_models(self):
  a=json.loads(bbs_config_for_region(2).read_text());b=json.loads(bbs_config_for_region(10).read_text())
  self.assertEqual(a['region_id'],2);self.assertEqual(b['region_id'],10);self.assertNotEqual(a['model'],b['model'])
  for k in ('timeout_ms','maximum_candidates','minimum_initial_overlap_percent'):self.assertEqual(a[k],b[k])
 def test_unsupported_regions_rejected(self):
  for n in ('space_1','space_20','space_100','space_2 space_10'):
   with self.assertRaises(ValueError):bbs_config_for_region(n)
 def test_no_fake_entrance_seeds(self):
  import yaml
  profile=yaml.safe_load((ROOT/'config/region_initialization_bbs_space_10.yaml').read_text())
  self.assertEqual(profile['regions'][0]['region_id'],10);self.assertTrue(profile['regions'][0]['dynamic_candidates_only']);self.assertEqual(profile['regions'][0]['seeds'],[])
if __name__=='__main__':unittest.main()
