"""File-only test of space_1..space_13 name boundaries; no ROS traffic."""
import struct, subprocess, tempfile
from pathlib import Path
binary='/home/pc/kiss_localization_ws1.1/install/kiss_matcher_localization/lib/kiss_matcher_localization/region_map_inspect'
with tempfile.TemporaryDirectory(prefix='space_names_') as temp:
 root=Path(temp);labels=root/'labels.csv';labels.write_text('region_id,region_name\n'+''.join(f'{i},space_{i}\n' for i in range(1,14)))
 cloud=root/'map.pcd';head='VERSION 0.7\nFIELDS x y z region_id\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH 13\nHEIGHT 1\nPOINTS 13\nDATA binary\n'
 cloud.write_bytes(head.encode()+b''.join(struct.pack('<fffI',i,0,0,i) for i in range(1,14)))
 for i in range(1,14):
  for hint in [f'space_{i}',f'我现在在space_{i}',str(i)]:
   r=subprocess.run([binary,str(cloud),str(labels),hint],capture_output=True,text=True,check=True)
   assert f'RESOLVED id={i} target=1' in r.stdout,(hint,r.stdout)
 for hint in ['space_14','space_100','myspace_1','我要去space_10','我现在在space_1和space_10']:
  assert subprocess.run([binary,str(cloud),str(labels),hint],capture_output=True).returncode==1,hint
print('PASS: 13 exact/numeric/current-location names, prefix collisions, invalid/ambiguous/destination hints')
