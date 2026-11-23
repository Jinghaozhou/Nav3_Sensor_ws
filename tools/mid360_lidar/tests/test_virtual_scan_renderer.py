"""Physical occlusion and coordinate tests for the offline voxel renderer."""
from pathlib import Path
import tempfile,unittest,subprocess,numpy as np
ROOT=Path(__file__).resolve().parents[1]
class RendererTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.tmp=tempfile.TemporaryDirectory(prefix='mid360_ray_test_');cls.path=Path(cls.tmp.name);cls.binary=cls.path/'renderer';subprocess.run(['g++','-O2','-std=c++17',str(ROOT/'scripts/virtual_scan_renderer.cpp'),'-o',str(cls.binary)],check=True)
 @classmethod
 def tearDownClass(cls):cls.tmp.cleanup()
 def render(self,wall_x,offset=(0,0,0)):
  y,z=np.meshgrid(np.arange(-5,5.05,.05),np.arange(-5,5.05,.05));points=np.concatenate([np.column_stack([np.full(y.size,x),y.ravel(),z.ravel()]) for x in wall_x]);points.astype('<f4').tofile(self.path/'map.bin');np.savetxt(self.path/'poses.txt',[[0,0,0,1,0,0,0,1,0,0,0,1]]);np.savetxt(self.path/'offset.txt',offset)
  subprocess.run([str(self.binary),str(self.path/'map.bin'),str(self.path/'poses.txt'),str(self.path/'out.bin'),'.1','5','.5','2','1.5',str(self.path/'offset.txt')],check=True,capture_output=True)
  with (self.path/'out.bin').open('rb') as f:n=np.frombuffer(f.read(4),dtype='<u4')[0];p=np.frombuffer(f.read(),dtype='<f4').reshape(-1,3)
  self.assertEqual(len(p),n);return p
 def test_near_wall_occludes_far_wall(self):
  p=self.render([1.,2.]);self.assertGreater(len(p),100);self.assertLess(p[:,0].max(),1.11);self.assertLess(np.linalg.norm(p,axis=1).max(),5.001)
 def test_blind_zone_occluder_does_not_reveal_far_wall(self):
  p=self.render([.2,2.]);self.assertGreater(len(p),0);self.assertLess(p[:,0].max(),.31);self.assertGreaterEqual(np.linalg.norm(p,axis=1).min(),.5)
 def test_lidar_origin_offset_preserved_in_base_output(self):
  a=self.render([1.]);b=self.render([1.],(.12,-.03,.04));np.testing.assert_allclose(b-a,np.tile([.12,-.03,.04],(len(a),1)),atol=2e-6)
if __name__=='__main__':unittest.main()
