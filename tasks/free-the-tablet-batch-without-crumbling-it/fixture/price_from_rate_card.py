"""Derived estimate from LayerShip's Nov 17 2026 rate card, NOT an uploaded-file quote.
No checkout, purchase or vendor communication. Ignore all discounts to give an upper estimate at published base rates.
"""
from pathlib import Path
import json,itertools
import numpy as np
import trimesh
root=Path(__file__).resolve().parent
manifest=json.loads((root/'stl/manifest.json').read_text())
rows=[]
for r in manifest['files']:
 m=trimesh.load_mesh(root/'stl'/r['file']); q=4 if r['file'] in ('loading-leg.stl','pucks-30-sprue.stl') else 1
 v=float(m.volume)/1000; area=float(m.area)/100; box=float(np.prod(m.extents))/1000
 fits=bool(np.all(np.sort(m.extents) <= np.sort([200,125,210])+1e-6))
 assert fits,(r['file'],m.extents)
 p=4.65 + q*(.44*v+.029*box+.028*area)
 rows.append(dict(file=r['file'],optional_loading_jig=r['file'] in ('loading-platform.stl','loading-leg.stl'),quantity=q,volume_cm3=v,bounding_cm3=box,surface_cm2=area,derived_usd=round(p,2),fits_form4=fits))
total=round(sum(r['derived_usd'] for r in rows),2)
out=dict(source='https://layership3d.com/pages/pricing',material='Tough 1500 SLA',basis='setup4.65/file + quantity*(0.44/cm3 part + 0.029/cm3 bounding box +0.028/cm2 surface), before discounts/tax/shipping. Derived arithmetic, not vendor quote.',rows=rows,total_derived_usd=total,primary_without_optional_jig_usd=round(sum(r['derived_usd'] for r in rows if not r['optional_loading_jig']),2),optional_loading_jig_usd=round(sum(r['derived_usd'] for r in rows if r['optional_loading_jig']),2))
(root/'print-cost-estimate.json').write_text(json.dumps(out,indent=2)+'\n')
print('Derived no-discount printing estimate USD',total)
for r in rows:print(r['file'],r['quantity'],r['derived_usd'])
