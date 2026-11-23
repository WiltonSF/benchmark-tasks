"""Generate passive construction envelopes directly from evaluated task geometry. No simulation."""
from pathlib import Path
import sys, numpy as np
folder=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(folder))
import task
lines=['// Generated from task.py MACHINE/GRINDER; units mm, world x/y, counter z=0.', '// 3mm closed shells preserve every external dimension; passive, no functional internals.', 'module panelbox(s){difference(){cube(s,center=true);cube(s-[6,6,6],center=true);}}', 'module tubecyl(r,h){difference(){cylinder(r=r,h=h,center=true,$fn=64);if(r>6 && h>6)cylinder(r=r-3,h=h-6,center=true,$fn=64);}}']
for obj in (task.MACHINE,task.GRINDER):
 lines.append(f'// {obj.name}')
 for s in obj.shapes:
  assert s.orientation is None and s.axis=='z'
  at=(np.asarray(s.at)+np.asarray(obj.pos)-[0,0,task.TOP])*1000
  pre='translate(['+','.join(f'{v:.6g}' for v in at)+']) '
  if s.kind=='box': cmd='panelbox(['+','.join(f'{v*1000:.6g}' for v in s.size)+']);'
  else:cmd=f'tubecyl({s.size[0]*1000:.6g},{s.size[1]*1000:.6g});'
  lines.append(pre+cmd)
 low=np.min([np.asarray(s.at)-s.half for s in obj.shapes],axis=0)*1000
 high=np.max([np.asarray(s.at)+s.half for s in obj.shapes],axis=0)*1000
 print(obj.name,'depth,width,height mm',(high-low).round(6).tolist())
(folder/'fixture/enclosures.scad').write_text('\n'.join(lines)+'\n')
