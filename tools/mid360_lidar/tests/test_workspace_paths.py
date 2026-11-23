import os,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import workspace_paths as paths
class PathsTest(unittest.TestCase):
 def test_sibling(self):
  with patch.object(paths,'ROOT',Path('/tmp/nav3/Nav3_Sensor_ws/tools/mid360_lidar')),patch.dict(os.environ,{},clear=True):
   self.assertEqual(paths.localization_workspace(),Path('/tmp/nav3/Nav3_Localization_ws'))
 def test_legacy(self):
  with patch.object(paths,'ROOT',Path('/tmp/legacy/mid360_lidar')),patch.dict(os.environ,{},clear=True):
   self.assertEqual(paths.localization_workspace(),Path('/tmp/legacy/kiss_localization_ws1.1'))
 def test_shared_map_and_override(self):
  with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{},clear=True):
   root=Path(d);p=root/'Nav3_ws/modules/Nav3_Sensor_ws/tools/mid360_lidar'
   shared=root/'Nav3_ws/data/maps/building_split_space.pcd';shared.parent.mkdir(parents=True);shared.touch()
   with patch.object(paths,'ROOT',p):
    self.assertEqual(paths.map_file(),shared)
    with patch.dict(os.environ,{'NAV3_MAP_PCD':str(root/'explicit.pcd')}):self.assertEqual(paths.map_file(),root/'explicit.pcd')
if __name__=='__main__':unittest.main()
