"""Cache the exact production PCL voxel output once; no geometry approximation."""
from pathlib import Path
import fcntl
import subprocess
import tempfile
from ament_index_python.packages import get_package_prefix
from build_keyframe_library import sha256
ROOT=Path(__file__).resolve().parents[1]

def ensure_cache(map_path, labels_path):
    digest=sha256(map_path);parent=ROOT/'reference_maps/race_map_cache';parent.mkdir(exist_ok=True)
    cache=parent/(digest+'_voxel_0.1')
    with (parent/'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if not cache.exists():
            with tempfile.TemporaryDirectory(dir=parent,prefix='building_') as temp:
                temp=Path(temp);binary=Path(get_package_prefix('kiss_matcher_localization'))/'lib/kiss_matcher_localization/prepare_region_map_cache'
                subprocess.run([str(binary),str(map_path),str(labels_path),'.1',str(temp)],check=True)
                (temp/'source.txt').write_text(digest+' 0.1\n')
                (temp/'checksums.txt').write_text(''.join(p.name+' '+sha256(p)+'\n' for p in sorted(temp.iterdir()) if p.is_file()))
                temp.rename(cache)
    return cache
