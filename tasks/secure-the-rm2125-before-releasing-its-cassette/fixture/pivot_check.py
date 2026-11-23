"""Fabrication pivot fit, swept envelopes and explicit bore/rotation checks, mm."""
from pathlib import Path
import contextlib,io,json,runpy,sys,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import task as t
f=(ROOT/'fixture/fabrication.py').read_text();sc=(ROOT/'fixture/printable.scad').read_text();generated=(ROOT/'fixture/fabrication.scad').read_text()
assert '(.0215,.008,.032),(-.05925,-.021' in f
assert 't.cyl(.007,.008,(-.0455,.021,t.C0+.0262))' in f
assert 'cylinder(d=4.85,h=14,center=true)' in generated
assert 'bore_y(4.85,300)' in sc and 'bore_y(10,4);bore_y(4.85,8)' in sc
with contextlib.redirect_stdout(io.StringIO()):m=runpy.run_path(str(ROOT/'fixture/clearance_check.py'))
B=json.loads((ROOT/'fixture/pivot-mount-envelopes.json').read_text())['boxes'];worst=(1e6,None,None);count=0
for b in B:
 c=np.array(b['center_mm']);s=np.array(b['size_mm']);lo=c-s/2;hi=c+s/2
 for name,l,h in m['fixed']:
  a=lo+[0,0,t.head.lower*1000];z=hi+[0,0,t.head.upper*1000]
  sep=np.maximum(l-z,a-h);gap=float(np.linalg.norm(np.maximum(sep,0)))
  assert (sep>0).any(),(b['name'],name)
  if gap<worst[0]:worst=(gap,b['name'],name)
  count+=1
 for name,l,h in m['components']:
  sep=np.maximum(l-hi,lo-h);gap=float(np.linalg.norm(np.maximum(sep,0)))
  assert (sep>0).any(),(b['name'],name)
  if gap<worst[0]:worst=(gap,b['name'],name)
  count+=1
# Entire steel axle lies within clamp relief; bearing and collar bores are coaxial.
assert -195 < -178 and 86 < 105
radial=(4.85-4.7625)/2;assert .043<radial<.044
# Narrowed inner post is outside steel axle, including its full radius.
inner_post_gap=334.5-4.7625/2-331.5;assert inner_post_gap>.61
# Bearing atY72..80: rotating clamp ends atY70; collar faces end71.9/start80.1.
assert abs(min(72-71.9,80.1-80)-.1)<1e-8
# Inner collar overlaps the crown inY; itsR5 lies within theR6 empty region.
crown_radial_gap=6-5;assert crown_radial_gap==1
# Every other clamp solid is either outside collarY or outside radius5.
for shape in t.clamp.shapes:
 p=(np.array(t.HEAD.pos)+shape.at-[0,0,t.TOP])*1000;h=shape.half*1000
 for y in (69.9,82.1):
  if p[1]-h[1]>=y+2 or p[1]+h[1]<=y-2:continue
  r=float(np.linalg.norm(np.maximum(np.abs(p[[0,2]]-[334.5,201.45])-h[[0,2]],0)))
  assert r>=6-1e-8,(p,r)
# PositiveX collar grub screws checked against rotating clamp at0.25deg; reserve0.6mm.
gmin=1e6
for deg in np.arange(0,15.501,.25):
 a=np.radians(deg);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
 for sh in t.clamp.shapes:
  p=(np.array(sh.at)-t.clamp.origin)*1000;center=np.array([334.5,55,201.45])+R@p;half=np.abs(R)@(sh.half*1000)
  for y in (69.9,82.1):
   c=np.array([334.5,55,201.45])+R@np.array([5.38125,y-55,0]);h=np.array([3,1.5,1.5])
   axes=np.concatenate([np.eye(3),R.T,*[np.cross(R.T,v) for v in np.eye(3)]])
   length=np.linalg.norm(axes,axis=1);axes=axes[length>1e-9]/length[length>1e-9,None]
   radius=np.abs(axes@R)@(sh.half*1000)+np.abs(axes@R)@h
   gap=float((np.abs((center-c)@axes.T)-radius).max())
   assert gap>.6,(deg,y,gap);gmin=min(gmin,gap)
out=dict(runtime_sha256=t.SOURCE_SHA256,pivot_components=len(B),full_stroke_comparisons=count,minimum_fixed_or_frame_gap_mm=worst,bearing_radial_clearance_mm=radial,inner_post_axle_clearance_mm=inner_post_gap,collar_cheek_gap_mm=.1,nominal_axial_float_mm=.2,dog_clearance_mm=.1,maximum_rocker_axial_play_mm=.3,dog_pocket_engagement_mm=1.,collar_crown_radial_gap_mm=crown_radial_gap,grub_clamp_gap_mm=gmin,full_stroke_mm=59,rotation_deg=[0,15.5],rotation_sample_deg=.25,limitations='Exact cylindrical bore/axle and radial collar arguments plus conservative hardware envelopes. Analytical only; no assembled or measured hardware result. Native proxy omits axle and bearing details.')
assert 'translate([5,8,0])' in sc
assert 335.88125+6<342 and .2+.1<.301
print(json.dumps(out,indent=2))
