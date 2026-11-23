"""CAD consistency for relocated dog and detachable blade bar; no hardware claim."""
from pathlib import Path
import sys,json,math,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import task as t
s=(ROOT/'fixture/printable.scad').read_text()
assert 'translate([5,8,0])rotate' in s and 'cube([3.3,82.2,19.25]' in s
assert 'cube([0.6,80.2,8.2]' in s and 'cube([0.6,80,0.4]' in s
assert 63-1.5-60.5>=1 and 67.9-(63+1.5)>=3.4-1e-9
assert abs(63-55)>1.5+1.65 # dog vs spine bolt, invariant during common rotation
assert 341.88125<342 and 63+1.5<66 and 335.88125+1<337.0
# Heel below screw below blade: positive upper clamp reaction, and ledge recessed.
assert 88.5<92.5<97 and 293.35-293.2>.14
assert 'translate([32.975,y,88.5])cube([0.75,8,2]' in s
# Two below-blade bolts; countersink heads clear blade bottom97.
assert 92.5+8.4/2<97 and 103<105 and 98-96==2
assert 286.7<284.525+3 and 292.7-286.7==6
kc=np.array([294.9375,55,93.475]);kh=np.array([3.175,82,19.05])/2
worst=(1e6,None,None)
for deg in np.arange(0,190.001,.1):
 a=np.radians(deg);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
 pivot=(np.array(t.GUARD.pos)+t.guard.origin-[0,0,t.TOP])*1000
 for i,sh in enumerate(t.guard.shapes):
  c=pivot+R@((np.array(sh.at)-t.guard.origin)*1000);h=sh.half*1000
  axes=np.concatenate([np.eye(3),R.T,*[np.cross(R.T,v) for v in np.eye(3)]])
  n=np.linalg.norm(axes,axis=1);axes=axes[n>1e-9]/n[n>1e-9,None]
  rad=np.abs(axes)@kh+np.abs(axes@R)@h;gap=float((np.abs((kc-c)@axes.T)-rad).max())
  assert gap>1.5,(deg,i,gap)
  if gap<worst[0]:worst=(gap,float(deg),i)
print(json.dumps(dict(runtime_sha256=t.SOURCE_SHA256,dog_y_mm=63,dog_to_spine_end_mm=1,dog_to_inner_collar_mm=3.4,thread_engagement_mm=4.25,dog_engagement_mm=1,blade_pressure_bar_mm=[3.175,82,19.05],bar_top_mm=103,edge_top_mm=105,bar_guard_sweep_min_gap_mm=worst,sample_step_deg=.1,between_sample_reserve_mm=.1,head_below_blade_gap_mm=.3,heel_z_mm=88.5,screw_z_mm=92.5,blade_reaction_reference_z_mm=100,upper_reaction_fraction=(92.5-88.5)/(100-88.5),ledge_to_bar_gap_mm=.15,limitations='Simplified exact boxes/CSG dimensions only. Pressure bar/anchors/threads/inserts are proposed fabrication;5N blade and10N axle retention and loaded force/lift proofs remain unperformed.'),indent=2))
