"""RDT60-R1 proposed physical kit, millimetres. Original CAD, CC BY 4.0.
Requires numpy, trimesh and manifold3d. Dimensions/forces require enrollment, not a validated build.
Run from this directory: python build_replica.py
"""
from pathlib import Path
import json, math
import numpy as np
import trimesh as tm
OUT=Path(__file__).resolve().parent/'stl'; OUT.mkdir(exist_ok=True)
N=32; H=3.7; SPLIT=2.05; GROOVE_OD=8.2; GROOVE_H=1.1
HOLES=[((c-2.5)*12.5,(r-4.5)*14) for r in range(10) for c in range(6)]
BOLTS=[(x,y) for x in (-44,44) for y in (-56,0,56)]
def box(s,p=(0,0,0)):
 m=tm.creation.box(s); m.apply_translation(p); return m
def frustum(d0,d1,h,p=(0,0,0)):
 a=np.arange(N)*2*np.pi/N
 v=np.r_[np.c_[d0/2*np.cos(a),d0/2*np.sin(a),np.zeros(N)],np.c_[d1/2*np.cos(a),d1/2*np.sin(a),np.full(N,h)],[[0,0,0],[0,0,h]]]
 f=[]
 for i in range(N):
  j=(i+1)%N; f.extend([(2*N,j,i),(2*N+1,N+i,N+j),(i,j,N+j),(i,N+j,N+i)])
 m=tm.Trimesh(vertices=v,faces=f,process=True)
 if m.volume<0:m.invert()
 m.apply_translation(p); return m
def cyl(d,h,p):return frustum(d,d,h,p)
def union(ms):return tm.boolean.union(ms,engine='manifold')
def diff(a,bs):return tm.boolean.difference([a,*bs],engine='manifold')
def save(n,m):
 assert m.is_watertight and m.volume>0,(n,m.is_watertight,m.volume)
 m.export(OUT/(n+'.stl')); print(n,round(m.volume,1),'mm^3',len(m.faces),'triangles')
 return dict(file=n+'.stl',volume_mm3=round(float(m.volume),2),watertight=bool(m.is_watertight),triangles=len(m.faces))
manifest=[]
tube=diff(box((30,30,55),(0,0,0)),[box((34,26,51),(0,0,0))])
tube=union([tube,box((2,26,51),(0,0,0))])
tube.apply_transform(tm.transformations.rotation_matrix(math.radians(2),(0,1,0)) @ tm.transformations.rotation_matrix(math.radians(2),(1,0,0)))
tube.apply_translation((0,250,H+27.5))
plate=union([box((120,152,H),(0,0,H/2)),box((30,177,10),(0,164.5,5)),tube,box((12,44,3),(2,250,H+58.8)),box((12,12,3.5),(0,250,H+55.75))])
cut=[]
for x,y in HOLES:
 cut.extend([frustum(6.5+2/3.7,5.5-2/3.7,H+4,(x,y,-2)),cyl(GROOVE_OD,GROOVE_H,(x,y,SPLIT-GROOVE_H/2))])
cut.extend(cyl(2.3,70,(x,y,-2)) for x,y in BOLTS)
plate=diff(plate,cut)
# 20 mm bolted half-lap lets each file fit a 200x125x210 mm build envelope after orientation.
for x in (-8,8):
 for y in (106,114):plate=diff(plate,[cyl(3.2,20,(x,y,-2))])
main=tm.boolean.intersection([plate,box((180,196,80),(0,22,40))],engine='manifold')
main=diff(main,[box((34,20,60),(0,110,35))])
ext=tm.boolean.intersection([plate,box((48,180,80),(0,190,40))],engine='manifold')
ext=diff(ext,[box((34,20,5),(0,110,2.5))])
assembled=[]
for n,z0,z1 in [('plate-bottom',0,SPLIT),('plate-top',SPLIT,11)]:
 m=tm.boolean.intersection([main,box((180,200,z1-z0),(0,22,(z0+z1)/2))],engine='manifold')
 manifest.append(save(n,m)); assembled.append(m)
manifest.append(save('handle-extension',ext)); assembled.append(ext)
reassembled=union(assembled)
assert abs(reassembled.volume-plate.volume)<.1, (reassembled.volume,plate.volume)
manifest.append(save('parking-block',box((30,25,12.9),(0,0,6.45))))
# Thirty pucks on removable 2 mm sprues. Cut/finish once; do not manufacture each trial.
puck_h=3.6; pieces=[box((78,48,2),(0,0,1))]
# Scaffold consists of thin lines, not a solid raft; allows inspection and removal with flush cutters.
pieces=[]
for y in (-20,-10,0,10,20): pieces.append(box((76,2,2),(0,y,1)))
for x in (-35,35):pieces.append(box((2,42,2),(x,0,1)))
for y in (-20,-10,0,10,20):
 for x in (-30,-18,-6,6,18,30):
  pieces.extend([cyl(2,4.2,(x,y,1)),frustum(6.42,5.447027027,puck_h,(x,y,5))])
manifest.append(save('pucks-30-sprue',union(pieces)))
# Proposed full lower-card load path through TAL220B-5kg, not a pin-only sensor.
# Coordinates relative to card centre; cell spans x[-47.5,7.5], z[-26.35,-13.65].
# Fixed top/frame carries the grasp; moving card is attached ONLY to the cell's x0 end.
body=union([box((90,3,25),(0,0,22.5)),box((20,12,25),(0,0,22.5)),
            box((10,14,22),(-40,0,-1)),box((12,14,3),(-40,0,11.5)),box((14,14,3),(-40,0,-12.15))])
body=diff(body,[cyl(5.4,10,(-40,0,-17)),cyl(9.4,50,(-40,0,-10.65))])
# Body/cover are unioned below; internal load cell purchased separately.
carrier=union([box((90,3,39),(0,0,-15.5)),cyl(1.8,15,(0,0,-47))])
carrier=diff(carrier,[box((15.6,18,45),(-40,0,-17)),box((56.6,14.3,14.3),(-20,0,-20))])
carrier=union([carrier,box((14,14,3),(0,0,-27.85))])
carrier=diff(carrier,[cyl(5.4,10,(0,0,-33)),cyl(9,10,(0,0,-14))])
manifest.append(save('pusher-carrier',carrier))
# Cover is fixed at left and clears moving end; never clamp it across both beam ends.
cover=diff(box((58,16,19),(-20,0,-19)),[box((56,14,18),(-20,0,-19.8)),
            box((45,4.6,5),(-10.5,0,-10)),box((6,4.6,22),(9,0,-19)),box((6,6,6),(-47,0,-21))])
body=union([body,cover]); manifest.append(save('pusher-body',body))
# Tool block: 92x4.5 slot, floor6, two shoulders12 high keep the loaded card upright above its central pin.
block=diff(box((100,40,20),(0,0,10)),[box((92,4.5,15),(0,0,13.5)),box((12,10.5,15),(0,0,13.5))])
block=union([block,box((38,4.5,12),(-25,0,12)),box((38,4.5,12),(25,0,12))])
block=union([block,box((30,10,19),(30,-7.5,28.5)),box((30,10,19),(30,7.5,28.5))])
manifest.append(save('spreader-block',block))
# Basic ring fit coupon: same groove/seat, no claim of calibrated forces from nominal dimensions.
coupon=box((36,12,3.7),(0,0,1.85))
cs=[]
for x in (-12,0,12): cs.extend([frustum(6.5+2/3.7,5.5-2/3.7,7.7,(x,0,-2)),cyl(8.2,1.1,(x,0,SPLIT-GROOVE_H/2))])
coupon=diff(coupon,cs)
for n,z0,z1 in [('coupon-bottom',0,SPLIT),('coupon-top',SPLIT,3.7)]:
 manifest.append(save(n,tm.boolean.intersection([coupon,box((40,16,z1-z0),(0,0,(z0+z1)/2))],engine='manifold')))
manifest.append(save('seating-gauge',union([cyl(20,3,(0,0,.1)),cyl(4,.1,(0,0,0))])))
(OUT/'manifest.json').write_text(json.dumps({'version':'RDT60-R1-proposed','units':'mm','ring':'5mm ID x1mm CS silicone; qualify actual force','pucks_sprue_quantity':4,'files':manifest},indent=2)+'\n')
