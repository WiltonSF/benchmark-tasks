"""Knife exposure and guard clearance from current geometry, mm; no hardware claim."""
from pathlib import Path
import sys,json,itertools,hashlib,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import task as t
bar,blade=t.KNIFE.shapes[2:4]
def world(p,s):return (np.array(p.pos)+s.at-[0,0,t.TOP])*1000,s.half*1000
bc,bh=world(t.KNIFE,bar);kc,kh=world(t.KNIFE,blade)
projection=kc[2]+kh[2]-bc[2]-bh[2];assert abs(projection-2)<1e-7
assert min(blade.color[:3])>max(bar.color[:3])+.1
corners=kc+np.array(list(itertools.product((-1,1),repeat=3)))*kh
worst=(1e6,None,None);cover_margin=1e6
for deg in np.arange(175,190.001,.05):
 a=np.radians(deg);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
 pivot=(np.array(t.GUARD.pos)+t.guard.origin-[0,0,t.TOP])*1000
 for i,s in enumerate(t.guard.shapes):
  c=pivot+R@((np.array(s.at)-t.guard.origin)*1000);h=s.half*1000
  axes=np.concatenate([np.eye(3),R.T,*[np.cross(R.T,v) for v in np.eye(3)]])
  norms=np.linalg.norm(axes,axis=1);axes=axes[norms>1e-9]/norms[norms>1e-9,None]
  rad=np.abs(axes)@kh+np.abs(axes@R)@h;gap=float((np.abs((kc-c)@axes.T)-rad).max())
  assert gap>2.05,(deg,i,gap)
  if gap<worst[0]:worst=(gap,float(deg),i)
 # Main cover projects over every blade-edge corner; ray intersection must stay inside its X extent.
 s=t.guard.shapes[0];center=pivot+R@((np.array(s.at)-t.guard.origin)*1000)
 for x in (kc[0]-kh[0],kc[0]+kh[0]):
  # Intersect the vertical line with the centre plane of the cover (local z=0).
  z=pivot[2]-(R[0,2]*(x-pivot[0]))/R[2,2]
  q=R.T@(np.array([x,kc[1],z])-pivot)
  margin=min(q[0]-(s.at[0]-t.guard.origin[0]-s.half[0])*1000,(s.at[0]-t.guard.origin[0]+s.half[0])*1000-q[0])
  assert margin>2;cover_margin=min(cover_margin,float(margin))
out=dict(runtime_sha256=t.SOURCE_SHA256,exposed_edge_mm=float(projection),blade_span_mm=float(2*kh[1]),bar_top_mm=float(bc[2]+bh[2]),edge_top_mm=float(kc[2]+kh[2]),guard_angles_deg=[175,190],sample_step_deg=.05,minimum_guard_blade_separating_gap_mm=worst,conservative_between_sample_reserve_mm=.025,minimum_main_cover_in_plane_margin_mm=cover_margin,tab_to_blade_end_gap_mm=3,blade_color=list(blade.color),bar_color=list(bar.color),limitations='Analytical geometry and contrast only; visibility must also be inspected in current native media. Dull hardware stand-in unbuilt; no cutting simulation or hardware safety certification.')
print(json.dumps(out,indent=2))
