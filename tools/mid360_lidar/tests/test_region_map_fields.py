"""Exercise categorical region IDs before voxelization, with two PCD datatypes."""
import struct,subprocess,tempfile
from pathlib import Path
binary='/home/pc/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/region_map_inspect'
with tempfile.TemporaryDirectory(prefix='region_fields_') as temp:
 root=Path(temp);labels=root/'labels.csv';labels.write_text('region_id,region_name\n0,其他区域\n1,5号楼入口楼梯\n')
 for pcd_type,fmt in [('F','f'),('U','I')]:
  cloud=root/f'{pcd_type}.pcd'
  head='VERSION 0.7\nFIELDS x y z region_id\nSIZE 4 4 4 4\nTYPE F F F '+pcd_type+'\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n'
  cloud.write_bytes(head.encode()+struct.pack('<fff'+fmt,.01,.01,.01,0)+struct.pack('<fff'+fmt,.02,.02,.02,1))
  r=subprocess.run([binary,str(cloud),str(labels),'1'],text=True,capture_output=True,check=True)
  assert 'MAP raw=2 voxel=1 labeled=1' in r.stdout,r.stdout
  assert 'REGION id=0 name=其他区域 raw=1 voxel=1' in r.stdout
  assert 'REGION id=1 name=5号楼入口楼梯 raw=1 voxel=1' in r.stdout
  assert subprocess.run([binary,str(cloud),str(labels),'999'],capture_output=True).returncode==1
 # Category 1.5 is invalid, even though coordinates are valid.
 cloud.write_bytes(head.replace('TYPE F F F U','TYPE F F F F').encode()+struct.pack('<ffff',.01,.01,.01,0)+struct.pack('<ffff',.02,.02,.02,1.5))
 r=subprocess.run([binary,str(cloud),str(labels)],text=True,capture_output=True)
 assert r.returncode==1 and 'non-integer' in r.stderr,r.stderr
print('PASS: FLOAT32/UINT32 labels, neighboring different labels in one voxel, unknown ID and fractional-label rejection')
