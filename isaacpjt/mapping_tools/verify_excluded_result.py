"""Recheck native mapper output and source preservation without starting Kit."""
from pathlib import Path
import hashlib
import json
import numpy as np
from PIL import Image

p=Path(__file__).resolve().parent
r=json.loads((p/'exclusion_report.json').read_text())
v=json.loads((p/'map_validation.json').read_text())
a=np.load(p/'native_occupancy_buffer.npy')
w=np.fliplr(a)
lo=v['min_bound']
for region in r['regions']:
    x0,y0,x1,y1=region['bounds_xy']
    c0=int(np.ceil((x0+.20-lo[0])/.05));c1=int(np.floor((x1-.20-lo[0])/.05))
    r0=int(np.ceil((y0+.20-lo[1])/.05));r1=int(np.floor((y1-.20-lo[1])/.05))
    b=w[r0:r1,c0:c1]
    counts={'unknown':int(((b!=0)&(b!=1)).sum()),'free':int((b==0).sum()),'occupied':int((b==1).sum())}
    assert counts['free']==0 and counts['unknown']>0
    v['region_checks'][region['name']]={'interior_cells':int(b.size),'counts':counts}
    print(region['name'],counts)
v['counts']={'unknown':int(((a!=0)&(a!=1)).sum()),'free':int((a==0).sum()),'occupied':int((a==1).sum())}
assert hashlib.sha256(Path(r['source']).read_bytes()).hexdigest()==r['source_sha256']
expected=np.rot90(np.where(a==1,0,np.where(a==0,255,127)).astype(np.uint8),2)
png=p.parents[1]/'src/nova_carter/carter_navigation/maps/hospital_integration_human_map_excluded.png'
assert np.array_equal(np.asarray(Image.open(png)),expected)
v['checks_passed']=['source_sha256_unchanged','no_free_cells_in_exclusion_interiors','preview_equals_native_mapper_buffer_rotated_180']
(p/'map_validation.json').write_text(json.dumps(v,indent=2)+'\n')
print('ALL_CHECKS_PASSED')
