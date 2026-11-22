"""A complete unpowered CRICKIT controller rests underside-up in a service tray on the counter.
The copper repair route is visible beside the original gold contact pads. A plywood-handled horsehair
brush rests on a rack, with a small coating cup on the opposite side. The clear overcoat fluoresces
green under the inspection illumination; deposited material remains visible on the board.
"""
from __future__ import annotations
import json,math,os,pathlib,sys
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN,grip_rotation
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
# Local kit, small contact envelope: bristles themselves use the declared compliance rule.
parts.CONTACT_OFFSET=.0003;parts.REST_OFFSET=0.
DATA=json.loads((pathlib.Path(__file__).parent/'fixture/board.json').read_text())
def polygon_contains(points,vertices):
 poly=np.asarray(vertices,float);result=np.zeros(len(points),bool)
 for a,b in zip(poly,np.roll(poly,-1,axis=0)):
  if abs(b[1]-a[1])<1e-15:continue
  cross=((a[1]>points[:,1])!=(b[1]>points[:,1])) & (points[:,0]<(b[0]-a[0])*(points[:,1]-a[1])/(b[1]-a[1])+a[0])
  result ^= cross
 return result

GREEN=(.035,.22,.105);GOLD=(.82,.63,.18);BLACK=(.045,.045,.05);WOOD=(.57,.34,.13)
# The laminate rests level on four 16 mm M3 standoffs fixed in the service tray, under the board's own corner
# mounting holes U$43/U$44/U$49/U$50 (native Eagle positions, 2.7-3.8 mm from the nearest component body).
POSTS=((-.037719,.016129),(.037719,.016129),(.037719,-.016129),(-.037719,-.016129));POST_TOP=.022
BOARD_A=(.32,.03,TOP+POST_TOP+.0008);BOARD_B=(.32,-.03,TOP+POST_TOP+.0008)
# Complete octagonal 81.28 mm native laminate; published 85 mm overall includes component overhang.
board=[]
for y in np.arange(-.04014,.04015,.001):
 half=min(.04064,.05704-abs(y))
 board.append(parts.box((2*half,.001,.0016),(0,float(y),0),GREEN))
for c in DATA['components']:
 board.append(parts.box((c['sx']/1000,c['sy']/1000,c['h']/1000),(c['x']/1000,c['y']/1000,-.0008-c['h']/2000),BLACK))
# Bottom contact faces, native positions and radii. Through-hole pad annuli, not enlarged solid disks.
for p in DATA['pads']:
 r=p['r']/1000;d=p['drill']/2000;x=p['x']/1000;y=p['y']/1000
 if p.get('outline'):
  poly=np.asarray(p['outline'])/1000
  xx,yy=np.meshgrid(np.arange(poly[:,0].min(),poly[:,0].max()+.0002,.0004),np.arange(poly[:,1].min(),poly[:,1].max()+.0002,.0004));pp=np.c_[xx.ravel(),yy.ravel()]
  mask=polygon_contains(pp,poly)&(np.linalg.norm(pp-[x,y],axis=1)>=d)
  for q in pp[mask]:board.append(parts.box((.0004,.0004,.00008),(float(q[0]),float(q[1]),.00084),GOLD,collide=False))
 elif d>0:
  for a in np.arange(0,2*math.pi,math.pi/8):
   rr=(r+d)/2
   board.append(parts.box((max(.0002,r-d),max(.0002,2*rr*math.sin(math.pi/16)),.00008),(x+rr*math.cos(a),y+rr*math.sin(a),.00084),GOLD,collide=False))
 else:board.append(parts.cylinder(r,.00008,(x,y,.00084),GOLD,collide=False))
 if d>0:board.append(parts.cylinder(d,.00005,(x,y,.00089),(.015,.02,.015),collide=False))
# The mask opening is on the native ANALOG8 route. All surrounding laminate is still present.
TRACE_X=np.arange(-.02875,-.015,.0005);TRACE_Y=.010287;TRACE_W=.000254
board.append(parts.box((.014,TRACE_W,.00006),(-.022,TRACE_Y,.00085),(.88,.34,.09),collide=False))
BOARD=parts.Part('crickit_controller',pos=BOARD_A,mass=.0307,shapes=board,friction=(.6,.5))
# Native handle kept whole: dated maker chart, 196.85 x 11.1125 mm; 8 mm thickness is an estimate.
# Brush local x is its long axis, bristles at its far end, and local z points out of the wooden backing.
FACE_L=.034925;FACE_W=.00635;BUNDLE_L=.0045;BUNDLE_W=.0017
BRISTLE_X=np.linspace(.075-(FACE_L-BUNDLE_L)/2,.075+(FACE_L-BUNDLE_L)/2,7);BRISTLE_Y=np.linspace(-(FACE_W-BUNDLE_W)/2,(FACE_W-BUNDLE_W)/2,3);TRIM=.0111125
TIPS=np.array([(x,y,-.004-TRIM) for x in BRISTLE_X for y in BRISTLE_Y])
BRUSH_A=(.16,.14,TOP+.040);BRUSH_B=(.16,-.14,TOP+.040)
BRUSH=parts.Part('mg859_brush',pos=BRUSH_A,yaw=math.radians(30),mass=.020,shapes=[parts.box((.19685,.0111125,.008),(0,0,0),WOOD),
 *[parts.box((BUNDLE_L,BUNDLE_W,TRIM),(float(x),float(y),-.004-TRIM/2),(.65,.49,.25),collide=False) for x in BRISTLE_X for y in BRISTLE_Y]],friction=(.7,.6),torsional_patch_radius=.004,min_torsional_patch_radius=.002)
def trace_label():
 glyphs={'T':['11111','00100','00100','00100','00100','00100','00100'],'R':['11110','10001','10001','11110','10100','10010','10001'],'A':['01110','10001','10001','11111','10001','10001','10001'],'C':['01111','10000','10000','10000','10000','10000','01111'],'E':['11111','10000','10000','11110','10000','10000','11111']}
 return [parts.box((.0013,.0013,.00005),(-.055+row*.0015,-.0225+char*.009+col*.0015,.00603),(.93,.93,.86),collide=False) for char,letter in enumerate('TRACE') for row,line in enumerate(glyphs[letter]) for col,pixel in enumerate(line) if pixel=='1']

# Real service tray and rack supports are dimensioned in fixture/setup.svg, not a bare board coupon.
TRAY_A=(.32,.03,TOP);TRAY_B=(.32,-.03,TOP)
TRAY=parts.Part('service_tray',pos=TRAY_A,mass=.25,shapes=[parts.box((.12,.12,.006),(0,0,.003),(.15,.17,.18)),
 parts.box((.12,.005,.018),(0,.0575,.009),(.2,.22,.23)),parts.box((.12,.005,.018),(0,-.0575,.009),(.2,.22,.23)),
 # Standoffs hold the laminate level; the native component side hangs free above the tray floor.
 *[parts.cylinder(.0025,POST_TOP-.006,(x,y,.006+(POST_TOP-.006)/2),(.75,.72,.62)) for x,y in POSTS],*trace_label()],friction=(.8,.7))
RACKS=[]
for sy in (-.14,.14):
 RACKS.append(parts.Part('brush_rack_'+('left' if sy>0 else 'right'),pos=(.16,sy,TOP),yaw=math.radians(30 if sy>0 else -30),shapes=[parts.box((.13,.06,.006),(0,0,.003),(.16,.19,.22)),
  parts.box((.016,.04,.030),(-.06,0,.021),(.16,.19,.22)),parts.box((.016,.04,.030),(.025,0,.021),(.16,.19,.22))]))
CUP_A=(.22,.01,TOP+.010);CUP_B=(.22,-.01,TOP+.010)
CUP=parts.Part('coating_cup',pos=CUP_A,mass=.08,shapes=[parts.cylinder(.022,.004,(0,0,.002),(.52,.57,.6)),
 *[parts.box((.004,.035,.018),(s*.020,0,.013),(.52,.57,.6)) for s in (-1,1)],
 *[parts.box((.04,.004,.018),(0,s*.019,.013),(.52,.57,.6)) for s in (-1,1)],parts.box((.036,.034,.00004),(0,0,.004+5e-6/(.036*.034)),(.10,.8,.32),collide=False)],friction=(.8,.7))
BOTTLE=parts.Part('mg422b_bottle_in_spill_tray',pos=(.43,.12,TOP),shapes=[parts.box((.065,.065,.004),(0,0,.002),(.25,.25,.27)),parts.cylinder(.019,.068,(0,0,.038),(.85,.86,.8)),parts.cylinder(.012,.012,(0,0,.078),BLACK)])
CADDIES=[parts.Part('cup_caddy',pos=(.22,0.,TOP),shapes=[parts.box((.05,.07,.010),(0,0,.005),(.22,.25,.28))])]
OBJECTS=[TRAY,BOARD,BRUSH,CUP,*RACKS,*CADDIES,BOTTLE]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={BOARD.name:BOARD_A,BRUSH.name:BRUSH_A,TRAY.name:TRAY_A,CUP.name:CUP_A},yaw={BRUSH.name:math.radians(30)}),
 lots.Lot('B',place={BOARD.name:BOARD_B,BRUSH.name:BRUSH_B,TRAY.name:TRAY_B,CUP.name:CUP_B},yaw={BRUSH.name:math.radians(-30)}))
DEMO_TOUCHES=set()
# Film field covers every original board surface cell; it is not a success stamp at the requested centreline.
CELL=.0005
GX,GY=np.meshgrid(np.arange(-.04075,.0408,CELL),np.arange(-.040713,.0408,CELL));GRID=np.c_[GX.ravel(),GY.ravel()]
INSIDE=(np.abs(GRID).max(1)<=.04064)&(np.abs(GRID).sum(1)<=.05704)
TARGET=(GRID[:,0]>=-.029)&(GRID[:,0]<-.015)&(abs(GRID[:,1]-TRACE_Y)<.00013)
PAD_MASKS=[]
for p in DATA['pads']:
 d=np.linalg.norm(GRID-np.array([p['x'],-0+p['y']])/1000,axis=1)
 PAD_MASKS.append((polygon_contains(GRID,np.asarray(p['outline'])/1000) if p.get('outline') else d<=p['r']/1000)&(d>=p['drill']/2000))
 if p['drill']>0:INSIDE &= d>=p['drill']/2000
PADS=np.logical_or.reduce(PAD_MASKS)
AREA=CELL*CELL;MIN_FILM=.000025;MAX_POOL=.001
# A 0.5 mm grid yields 28 true-route longitudinal samples. Coverage is wet visible length, not cured film.

def new_material():
 return dict(volume=np.zeros(len(GRID)),wet=np.zeros(len(TIPS)),cup=5e-6,load_events=0,transferred=0.,lost=0.,bad=False,bridge=False,scratch=0.,scar_edges={},prev=None)
M=new_material();H={};_visuals={};_stage=None;_wet_caps={}

def copper_contains(points):
 points=np.atleast_2d(points);out=(points[:,0]>=-.029)&(points[:,0]<=-.015)&(abs(points[:,1]-TRACE_Y)<=TRACE_W/2)
 for p in DATA['pads']:
  dd=np.linalg.norm(points-np.array([p['x'],p['y']])/1000,axis=1)
  mask=polygon_contains(points,np.asarray(p['outline'])/1000) if p.get('outline') else dd<=p['r']/1000
  out |= mask&(dd>=p['drill']/2000)
 return out

def record_scar(a,b):
 # Unique 0.1 mm copper-path edges: reversing over the same short scar does not increase its length.
 n=max(1,int(math.ceil(np.linalg.norm(b[:2]-a[:2])/.00005)))
 points=np.linspace(a,b,n+1);valid=copper_contains(points[:,:2])&(.0008-points[:,2]>.003)
 previous=None
 for p,ok in zip(points,valid):
  q=tuple(np.round(p[:2]/.0001).astype(int))
  if ok and previous is not None and q!=previous:
   edge=tuple(sorted((previous,q)));M['scar_edges'][edge]=float(np.linalg.norm(np.array(q)-previous)*.0001)
  previous=q if ok else None
 # Sum unique edges in each connected physical scar; repeated motion cannot inflate a short scar.
 parent={}
 def find(a):
  parent.setdefault(a,a)
  while parent[a]!=a:parent[a]=parent[parent[a]];a=parent[a]
  return a
 for a,b in M['scar_edges']:
  x,y=find(a),find(b)
  if x!=y:parent[x]=y
 lengths={}
 for (a,b),length in M['scar_edges'].items():
  r=find(a);lengths[r]=lengths.get(r,0.)+length
 M['scratch']=max(lengths.values(),default=0.)
 if M['scratch']>=.001-1e-9:M['bad']=True

def load_brush(world_tips,cup,dt):
 rotation=parts._quat_to_matrix(cup.quat_xyzw)
 if rotation[2,2]<math.cos(math.radians(5)):
  M['lost']+=M['cup'];M['cup']=0.;M['bad']=True;return
 local=(world_tips-cup.pos)@rotation
 for j,tip in enumerate(local):
  level=.004+M['cup']/(.036*.034)
  if abs(tip[0])<.018 and abs(tip[1])<.017 and .004<=tip[2]<=level:
   amount=min(1e-9*dt,max(0.,3e-9-M['wet'][j]),M['cup'])
   M['wet'][j]+=amount;M['cup']-=amount
   if amount>0:M['load_events']+=1

def deposit(tip_positions,dt):
 """Integrate complete 3D tip trajectories; airborne parts of a stroke transfer no film."""
 if M['prev'] is None:M['prev']=tip_positions.copy();return dict(contacts=0,compression=0.,force=0.,splay=0.)
 previous=M['prev'];M['prev']=tip_positions.copy();contact_bundles=set();maxcomp=0.;force=0.;maxsplay=0.
 for j,(now,old) in enumerate(zip(tip_positions,previous)):
  distance=float(np.linalg.norm(now-old));n=max(1,int(math.ceil(distance/.00025)))
  # A straight step whose lower end is above the board surface cannot touch it, however fast it moves.
  if min(now[2],old[2])>.0008:continue
  # A step that could touch the board is integrated in 0.25 mm pieces; one over 100 mm in 0.05 s (2 m/s) cannot be
  # followed and fails closed.
  if distance>.1:M['bad']=True;continue
  for k in range(n):
   a=old+(now-old)*k/n;b=old+(now-old)*(k+1)/n;p=(a+b)/2
   if max(abs(p[0]),abs(p[1]))>.04064 or abs(p[0])+abs(p[1])>.05704:continue
   comp=.0008-p[2];maxcomp=max(maxcomp,comp)
   if comp<=0:continue
   contact_bundles.add(j);force+=100.*comp/n
   if comp>.003:record_scar(a,b)
   if M['wet'][j]<=0:continue
   splay=min(.004,max(0.,comp-.002)*2);maxsplay=max(maxsplay,splay)
   ids=np.flatnonzero((np.linalg.norm(GRID-p[:2],axis=1)<=.0011+splay)&INSIDE)
   if not len(ids):continue
   amount=min(M['wet'][j],len(ids)*AREA*.000075*(dt/n)*min(8.,max(.2,comp/.0005)))
   M['volume'][ids]+=amount/len(ids);M['wet'][j]-=amount;M['transferred']+=amount
 wet=M['volume']/AREA>=MIN_FILM
 if np.count_nonzero(wet&PADS)*AREA>=.5e-6-1e-12:M['bad']=True
 if np.any(wet&PADS):
  from scipy.ndimage import label
  regions,_=label(wet.reshape(GX.shape));seen={}
  for i,pad in enumerate(PAD_MASKS):
   for region in np.unique(regions.ravel()[pad&wet]):
    if region and region in seen and seen[region]!=i:M['bridge']=True;M['bad']=True
    if region:seen[region]=i
 return dict(contacts=len(contact_bundles),compression=maxcomp,force=force,splay=maxsplay)

def coverage():
 vals=M['volume'][TARGET]/AREA;wet=vals>=MIN_FILM
 fraction=float(wet.mean()) if len(wet) else 0.
 ids=np.flatnonzero(wet);gap=0.
 if len(ids)>1:
  gap=float(max(np.diff(ids)-1,default=0)*CELL)
 return fraction,gap,float(np.max(M['volume']/AREA,initial=0))

@lru_cache(maxsize=1)
def hand_mesh():
 from yam_arena import analysis
 result={}
 for name in ('link1','link2','link3','link4','link5','gripper','tip_left','tip_right'):
  cloud=[]
  for hull in analysis.link_hulls().get(name,[]):
   cloud.extend(hull.vertices)
   for plane in np.unique(hull.planes.round(7),axis=0):
    face=hull.vertices[np.abs(hull.vertices@plane[:3]-plane[3])<2e-6]
    if len(face)<3:continue
    c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(plane[:3],u)
    face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
    for a,b in zip(face,np.roll(face,-1,axis=0)):
     n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
     for i in range(n+1):
      for j in range(n-i+1):cloud.append(c+(a-c)*i/n+(b-c)*j/n)
  cloud=np.asarray(cloud)
  if not len(cloud):raise ValueError('unknown robot hull')
  _,idx=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True);result[name]=cloud[idx]
 return result

def hands(sim,need_clouds=True):
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[];peak=0.
 for arm in ('left_robot','right_robot'):
  robot=base.scene[arm];poses=parts._np(robot.data.body_link_pose_w)[0]
  effort=parts._np(robot.data.applied_torque)[0]
  from yam_arena.constants import TCP_OFFSET
  gp=poses[robot.body_names.index('gripper')];gr=parts._quat_to_matrix(gp[3:])
  H.setdefault('robots',{})[arm]=dict(tcp=(gp[:3]-origin+gr@np.asarray(TCP_OFFSET)).tolist(),quat=gp[3:].tolist(),finger=[float(effort[robot.joint_names.index(n)]) for n in ('joint7','joint8')])
  if not np.isfinite(poses).all() or not np.isfinite(effort).all():raise ValueError('unknown arm state')
  for n in ('joint7','joint8'):
   if n in robot.joint_names:peak=max(peak,abs(float(effort[robot.joint_names.index(n)])))
  for name,points in (hand_mesh().items() if need_clouds else []):
   p=poses[robot.body_names.index(name)];clouds.append(points@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
 return clouds,peak

def gap(clouds,spec,state):
 # Full oriented native bounds conservatively include every face, including concave voids.
 if not clouds:return -math.inf
 points=np.concatenate(clouds);r=parts._quat_to_matrix(state.quat_xyzw);lo,hi=spec._local_box()
 local=(points-state.pos)@r
 return float(np.linalg.norm(np.maximum(np.maximum(lo-local,local-hi),0.),axis=1).min())

def stamp(st):return tuple(float(v) for v in np.r_[st.pos,st.quat_xyzw,st.lin_vel,st.ang_vel,st.low,st.high])

def visual(sim,board):
 global _stage,_visuals,_wet_caps
 from pxr import Gf,UsdGeom
 from isaaclab.sim.utils import get_current_stage
 stage=get_current_stage()
 if stage is not _stage:_stage=stage;_visuals={};_wet_caps={}
 r=parts._quat_to_matrix(board.quat_xyzw)
 for i in np.flatnonzero(M['volume']/AREA>=MIN_FILM):
  if i not in _visuals:
   g=UsdGeom.Cube.Define(stage,f'/World/CoatingField/c{i}');g.CreateSizeAttr(1.);g.AddTranslateOp();g.AddScaleOp().Set(Gf.Vec3f(CELL*.94,CELL*.94,.00004));g.CreateDisplayColorAttr().Set([Gf.Vec3f(.10,.95,.38)]);_visuals[i]=g
  p=np.asarray(board.pos)+r@np.r_[GRID[i],.00092+min(.001,M['volume'][i]/AREA)]
  _visuals[i].GetOrderedXformOps()[0].Set(Gf.Vec3d(*p))
 # UV-A fluorescence shows actual liquid in the source cup and actual wet bristle ends.
 states=sim.states();tool=states[BRUSH.name];cup_path=parts.ROOT_PATHS.get(CUP.name,'')+'/s5';prim=stage.GetPrimAtPath(cup_path)
 if prim.IsValid():
  prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.,0.,.004+M['cup']/(.036*.034)))
  UsdGeom.Imageable(prim).CreateVisibilityAttr().Set('inherited' if M['cup']>0 else 'invisible')
 rt=parts._quat_to_matrix(tool.quat_xyzw);q=parts._matrix_to_quat(rt)
 for j in range(len(TIPS)):
  if j not in _wet_caps:
   g=UsdGeom.Cube.Define(stage,f'/World/BrushWetCaps/c{j}');g.CreateSizeAttr(1.);tr=g.AddTranslateOp();orientation=g.AddOrientOp();g.AddScaleOp().Set(Gf.Vec3f(BUNDLE_L*.95,BUNDLE_W*.95,.001));g.CreateDisplayColorAttr().Set([Gf.Vec3f(.10,.95,.38)]);_wet_caps[j]=(g,tr,orientation)
  g,tr,orientation=_wet_caps[j];p=np.asarray(tool.pos)+rt@(TIPS[j]+(0,0,.0005));tr.Set(Gf.Vec3d(*[float(v) for v in p]));orientation.Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*[float(v) for v in q[:3]])));g.CreateVisibilityAttr().Set('inherited' if M['wet'][j]>1e-12 else 'invisible')
 # Hide old episode cells, without any robot-triggered material reset.
 for i,g in _visuals.items():g.CreateVisibilityAttr().Set('inherited' if M['volume'][i]/AREA>=MIN_FILM else 'invisible')

SCORED=(BOARD,BRUSH,CUP,TRAY)
BACKING_POINTS=[]
for axis in range(3):
 other=[i for i in range(3) if i!=axis];half=np.array((.19685,.0111125,.008))/2
 for sign in (-1,1):
  for u in np.linspace(-half[other[0]],half[other[0]],int(math.ceil(2*half[other[0]]/.002))+1):
   for v in np.linspace(-half[other[1]],half[other[1]],int(math.ceil(2*half[other[1]]/.002))+1):
    p=np.zeros(3);p[axis]=sign*half[axis];p[other]=[u,v];BACKING_POINTS.append(p)
BACKING_POINTS=np.unique(np.asarray(BACKING_POINTS),axis=0)

def validate_state(state):
 for name,n in (('pos',3),('quat_xyzw',4),('lin_vel',3),('ang_vel',3),('low',3),('high',3)):
  a=np.asarray(getattr(state,name),float)
  if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('invalid '+name)
 if not .99<=np.linalg.norm(state.quat_xyzw)<=1.01:raise ValueError('invalid quaternion')
 if np.any(state.low>state.high):raise ValueError('invalid bounds')

def certificate(states):return tuple((spec.name,stamp(states[spec.name])) for spec in SCORED)

def supports(states):
 board,tool,cup,tray=(states[spec.name] for spec in SCORED)
 rt=parts._quat_to_matrix(tray.quat_xyzw);rb=parts._quat_to_matrix(board.quat_xyzw)
 contacts=[]
 # Laminate underside at each mounting hole must sit on its standoff top (1.5 mm) and over the standoff (4 mm).
 for x,y in POSTS:
  p=(np.asarray(board.pos)+rb@np.array((x,y,-.0008))-tray.pos)@rt
  contacts.append(bool(np.hypot(p[0]-x,p[1]-y)<=.004 and abs(p[2]-POST_TOP)<=.0015))
 board_ok=all(contacts) and np.linalg.norm((np.asarray(board.pos)-tray.pos)[:2])<=.005
 wood=BACKING_POINTS@parts._quat_to_matrix(tool.quat_xyzw).T+tool.pos
 rack_ok=False
 for rack in RACKS:
  local=(wood-np.asarray(rack.pos))@rack.rotation;hits=[]
  for x in (-.06,.025):
   hits.append(bool(np.any((abs(local[:,0]-x)<=.0085)&(abs(local[:,1])<=.0205)&(abs(local[:,2]-.036)<=.0015))))
  rack_ok |= all(hits)
 return bool(board_ok and rack_ok and abs(float(cup.low[2])-(TOP+.010))<=.0015 and abs(float(tray.low[2])-TOP)<=.0015)

def backing_depth(tool,board):
 points=BACKING_POINTS@parts._quat_to_matrix(tool.quat_xyzw).T+tool.pos
 old=H.get('backing');H['backing']=points.copy()
 rotation=parts._quat_to_matrix(board.quat_xyzw);depth=0.
 n=1 if old is None else max(1,int(math.ceil(np.linalg.norm(points-old,axis=1).max()/.001)))
 # Wood that stays above the board surface at both ends of the step cannot intrude (motion is linear in a step).
 lowest=min(float(np.min(((points if old is None else np.r_[points,old])-board.pos)@rotation[:,2])),1.)
 if lowest>.0008:return 0.
 if n>100:raise ValueError('unobserved backing sweep near the board')
 for a in np.linspace(0,1,n+1):
  q=(points if old is None else old+(points-old)*a)-board.pos;q=q@rotation
  inside=(np.abs(q[:,:2]).max(1)<=.04064)&(np.abs(q[:,:2]).sum(1)<=.05704)
  if np.any(inside):depth=max(depth,float(np.max(.0008-q[inside,2])))
 return depth

@parts.on_reset
def reset(sim):
 global M,H
 for g in _visuals.values():
  if g.GetPrim().IsValid():g.CreateVisibilityAttr().Set('invisible')
 for g,_,_ in _wet_caps.values():
  if g.GetPrim().IsValid():g.CreateVisibilityAttr().Set('invisible')
 if _stage is not None:
  prim=_stage.GetPrimAtPath(parts.ROOT_PATHS.get(CUP.name,'')+'/s5')
  if prim.IsValid():
   from pxr import Gf,UsdGeom
   prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.,0.,.004+5e-6/(.036*.034)));UsdGeom.Imageable(prim).CreateVisibilityAttr().Set('inherited')
 M=new_material();H=dict(known=False,bad=False,t=0.,held=0.,certificate=None,peak=0.,initial=None,metrics={},backing=None)

@parts.on_step
def step(sim):
 global H
 states={};b=tool=None
 dt=float(sim.t)-H.get('t',0.);H['t']=float(sim.t)
 try:
  states=sim.states()
  for spec in SCORED:validate_state(states[spec.name])
  b,tool,cup,tray=(states[spec.name] for spec in SCORED)
  if not 0<dt<=.100001:raise ValueError('unknown timestep')
  still=all(states[spec.name].resting(.003,.03) for spec in SCORED)
  supported=supports(states)
  need_clear=coverage()[0]>=.95 and still and supported
  clouds,force=hands(sim,need_clear);H['known']=True;H['peak']=max(H['peak'],force)
  if H['initial'] is None:H['initial']={spec.name:np.array(states[spec.name].pos) for spec in SCORED}
  if np.linalg.norm(b.pos-H['initial'][BOARD.name])>.005 or parts._quat_to_matrix(b.quat_xyzw)[2,2]<math.cos(math.radians(3)):H['bad']=True
  if np.linalg.norm(cup.pos-H['initial'][CUP.name])>.008 or np.linalg.norm(tray.pos-H['initial'][TRAY.name])>.008:H['bad']=True
  if parts._quat_to_matrix(tray.quat_xyzw)[2,2]<math.cos(math.radians(3)):H['bad']=True
  world=TIPS@parts._quat_to_matrix(tool.quat_xyzw).T+tool.pos
  load_brush(world,cup,dt)
  local=(world-b.pos)@parts._quat_to_matrix(b.quat_xyzw)
  metrics=deposit(local,dt);hard=backing_depth(tool,b)
  if metrics['compression']>.005 or hard>.005:H['bad']=True
  clear=need_clear and min(gap(clouds,spec,states[spec.name]) for spec in SCORED)>=.030
  released=bool(still and supported)
  H['held']=H['held']+dt if clear and released and not H['bad'] and not M['bad'] else 0.
  H['metrics']=dict(**metrics,backing_depth=hard,coverage=coverage()[0],gap=coverage()[1],pool=coverage()[2],clear=bool(clear),released=released,supported=supported,load=float(M['wet'].sum()),cup=M['cup'],lost=M['lost'],pad_area=float(np.count_nonzero((M['volume']/AREA>=MIN_FILM)&PADS))*AREA)
  H['certificate']=certificate(states);visual(sim,b)
 except (ValueError,KeyError,AttributeError,IndexError,TypeError) as error:
  H['known']=False;H['bad']=True;H['held']=0.;H['certificate']=None;H['error']=str(error)
 path=os.environ.get('COATING_TRACE')
 if os.environ.get('TRACE_OUT'):path=str(pathlib.Path(os.environ['TRACE_OUT']).with_name('coating.jsonl'))
 if path:
  row=dict(t=float(sim.t),lot=lots.current().name,known=H.get('known',False),bad=bool(H.get('bad',True) or M['bad']),bridge=bool(M['bridge']),held=H.get('held',0.),peak=H.get('peak',0.),metrics=H.get('metrics',{}),error=H.get('error'),robots=H.get('robots',{}),score=score(states))
  if b is not None:row['board']=b.pos.tolist()
  if tool is not None:row.update(tool=tool.pos.tolist(),quat=tool.quat_xyzw.tolist())
  row['objects']={spec.name:dict(pos=states[spec.name].pos.tolist(),quat=states[spec.name].quat_xyzw.tolist(),lin_vel=states[spec.name].lin_vel.tolist(),ang_vel=states[spec.name].ang_vel.tolist()) for spec in SCORED if spec.name in states}
  with pathlib.Path(path).open('a') as output:output.write(json.dumps(row)+'\n')

def score(objects):
 try:
  for spec in SCORED:validate_state(objects[spec.name])
  if not H.get('known') or H.get('bad') or M['bad'] or M['bridge']:return 0.
  if H.get('certificate')!=certificate(objects):return 0.
  frac,gap_,pool=coverage()
  if pool>=MAX_POOL or gap_>=.0005-1e-10:return 0.
  value=0.
  for threshold,tier in ((.25,.25),(.50,.5),(.80,.75),(.95,1.)):
   if frac+1e-9>=threshold:value=tier
  if value==1. and H['held']<2.-1e-9:value=.75
  return value
 except (KeyError,TypeError,ValueError,AttributeError):return 0.

def success(objects):return score(objects)==1.

# Demo constants. Calibration is per arm and fixed: it comes from earlier scripted traces of the visible tool (where
# the bristle tips landed relative to where they were commanded), never from live state, force, material or score.
RAKE=40.                     # degrees the bristle end is tipped down for dipping and coating
STROKE_HEADING={'left':math.radians(30.),'right':math.radians(-30.)}  # reachable wrist lean for each arm (offline IK)
# Leading-row middle bundle start, native x (mm). The three leading bundles lie across the handle, so with the right
# arm's -30 degree heading the bundle nearest the GPIO1:24 annulus trails the middle one by 1.16 mm; that arm starts
# 0.95 mm further along the trace and its first 0.5 mm sample is not counted on (27 of 28 still reach 95 percent).
START_X={'left':-.0285,'right':-.02755};END_X=-.0130
# along: the brush sits about 1 degree steeper in the grip than commanded, which pulls the tips 1.3 mm toward the hand.
# board/cup: remaining fixed landing offsets (x, y, z in m) measured in earlier scripted traces of each arm.
CAL={'left':dict(along=.0013,up=.0008,board=(0.,-.0012,0.),cup=(0.,0.,0.)),
     'right':dict(along=.0013,up=.0010,board=(-.00125,0.,0.),cup=(0.,0.,0.))}
STROKE_S=4.                  # seconds per pass; two passes, each from a fresh dip, in opposite directions. One 3 uL
                             # load lasts about 4.7 s at 1 mm compression, so a slow stroke runs dry before the far end

def demo():
 # Published visible starting positions only. No scorer, force, material or simulator pose feedback.
 lot=lots.current();p=np.asarray(lot.pos(BRUSH.name));b=np.asarray(lot.pos(BOARD.name));cup=np.asarray(lot.pos(CUP.name))
 arm='left' if p[1]>0 else 'right';yaw=lot.yaw.get(BRUSH.name,BRUSH.yaw);flat=parts._rot((0,0,1),yaw);down=flat@grip_rotation(0.)
 cal=CAL[arm];zrot=lambda h:parts._rot((0,0,1),h)
 def tool(h):return zrot(h)@parts._rot((0,1,0),math.radians(RAKE))
 def w(t,pos,f,label,rotation=down):return Waypoint(t,tuple(float(v) for v in pos),f,label,tilt._TiltedYaw(0.,rotation))
 grasp=p+flat@np.array((-.015,0.,0.));finger=-.004
 far=TIPS[-2]  # middle bundle of the leading row
 def tcp(tip,h):
  # TCP that puts the leading middle bundle at `tip`; the native grip point is 14.1 mm behind the brush centre.
  tip=np.asarray(tip,float)+zrot(h)@np.array((cal['along'],0.,0.))+(0,0,cal['up'])
  return tip-tool(h)@(far+np.array((.0141,0,0)))
 # Dip: the leading bundles 1.5 mm into the 4.1 mm-deep coating, 4 mm toward the handle side of the cup centre so
 # the wooden end clears the far wall; lift while backing off along the handle.
 hd=yaw;dip=cup+zrot(hd)@np.array((-.004,0,0))+(0,0,.0055)+cal['cup']
 up_cup=dip+zrot(hd)@np.array((-.002,0,0))+(0,0,.025)
 hs=STROKE_HEADING[arm];touch=.0008-.0010
 start=b+np.array((START_X[arm],TRACE_Y,touch))+cal['board'];end=b+np.array((END_X,TRACE_Y,touch))+cal['board']
 def load(tag,first):
  steps=[] if not first else [w(2.5,tcp(up_cup,hd),finger,'over service cup'),w(1.5,tcp(up_cup,hd),finger,'set rake',tool(hd)@grip_rotation(0.))]
  return steps+[w(2.,tcp(dip,hd),finger,tag+' dip leading bristles',tool(hd)@grip_rotation(0.)),w(3.5,tcp(dip,hd),finger,tag+' wet leading bristles',tool(hd)@grip_rotation(0.)),
   w(2.,tcp(up_cup,hd),finger,tag+' lift clear of cup',tool(hd)@grip_rotation(0.))]
 def coat(a,z,tag):
  rot=tool(hs)@grip_rotation(0.)
  return [w(3.,tcp(a+(0,0,.012),hs),finger,tag+' over repair',rot),w(1.2,tcp(a,hs),finger,tag+' touch repair',rot),
   w(STROKE_S,tcp(z,hs),finger,tag+' coat native trace',rot),w(1.5,tcp(z+(0,0,.006),hs),finger,tag+' lift off',rot)]
 plan=[w(2.,grasp+(0,0,.055),FINGER_OPEN,'approach native handle'),w(2.,grasp,FINGER_OPEN,'lower over handle'),w(1.5,grasp,finger,'grasp plywood'),w(2.,grasp+(0,0,.055),finger,'lift full brush'),
  *load('first',True),*coat(start,end,'first pass'),w(3.,tcp(up_cup,hd),finger,'back to cup',tool(hd)@grip_rotation(0.)),*load('reload',False),*coat(end,start,'second pass'),
  w(1.5,tcp(start+(0,0,.010),hs),finger,'unrake above board'),w(3.,grasp+(0,0,.055),finger,'return brush to rack'),w(2.,grasp+(0,0,.003),finger,'lower to rack'),
  w(1.5,grasp+(0,0,.003),FINGER_OPEN,'release full brush'),w(2.,grasp+(0,0,.070),FINGER_OPEN,'hands clear'),w(4.,grasp+(0,0,.070),FINGER_OPEN,'released still hold')]
 return {arm:plan}
