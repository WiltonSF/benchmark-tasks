"""A laboratory marking bench has a brass coupon supported in a bench vise, three numbered steel punches,
a ball-peen hammer on a cradle, a ruler and a work card showing 123. Each punch has a visible orientation stripe.
"""
from __future__ import annotations
import json,math,os,pathlib,sys
from functools import lru_cache
from dataclasses import replace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
GREY=(.5,.53,.56);STEEL=(.7,.72,.75);BRASS=(.75,.56,.23);BLACK=(.06,.06,.06);WHITE=(.9,.9,.85)
CX=.225;CY=0.;Z=TOP+.051;L=.073025;HALF=.00635
# Sharp-face line geometry: 9.525mm height; approximate Gothic centre-lines, not digit-segment boxes.
GLYPHS={1:[(-.002,.002),(0,.0047625),(0,-.0047625),(-.002,-.0047625),(.002,-.0047625)],
2:[(-.0028,.0035),(-.0015,.0047625),(.0015,.0047625),(.0028,.0035),(.0028,.002),(-.0028,-.0047625),(.0028,-.0047625)],
3:[(-.0028,.0047625),(.0028,.0047625),(0,0),(.0028,-.0015),(.0028,-.0035),(.0015,-.0047625),(-.0028,-.0047625)]}
def sample_glyph(d):
 pts=[];weights=[]
 for a,b in zip(GLYPHS[d],GLYPHS[d][1:]):
  a=np.array(a);b=np.array(b);length=np.linalg.norm(b-a);n=max(1,math.ceil(length/.00025))
  for u in (np.arange(n)+.5)/n:pts.append(a+(b-a)*u);weights.append(length/n)
 return np.array(pts),np.array(weights)
SAMPLES={d:sample_glyph(d) for d in (1,2,3)}
VISE=parts.Part('bench_vise',pos=(CX,0,TOP),contact_offset=.0001,shapes=[
 parts.box((.235,.146,.010),(0,0,.005),GREY),parts.box((.100,.050,.038),(0,0,.029),STEEL),
 parts.box((.0762,.012,.029),(0,.031,.0395),GREY),parts.box((.0762,.012,.029),(0,-.031,.0395),GREY),
 parts.cylinder(.006,.140,(0,-.090,.032),STEEL,'y'),parts.cylinder(.004,.075,(0,-.16,.032),STEEL,'x'),
 *[parts.cylinder(.004,.010,(x,y,.010),BLACK) for x in (-.100,.100) for y in (-.060,.060)]])
_coupon_shapes=[parts.box((.050,.050,.003),(0,0,0),BRASS)]
for x in (-.012,0.,.012):
 _coupon_shapes.extend([parts.box((.003,.0002,.00005),(x,0,.00155),(.10,.25,.8),collide=False),parts.box((.0002,.003,.00005),(x,0,.00155),(.10,.25,.8),collide=False)])
COUPON=parts.Part('brass_coupon',pos=(CX,0,Z-.0015),mass=.06375,contact_offset=.0001,report=('base',),shapes=_coupon_shapes)
START_YAWS=[math.radians(-20),math.radians(20),math.radians(-30)]
STARTS=[(.34,.09,TOP+.004+L/2),(.34,.16,TOP+.004+L/2),(.34,.23,TOP+.004+L/2)]
PUNCHES=[]
for d,p in zip((1,2,3),STARTS):
 # Full size steel shank, face protrusion included in body bottom; orange thumb orientation stripe.
 shapes=[parts.box((.0127,.0127,L-.001),(0,0,.0005),STEEL),parts.box((.003,.0002,.020),(0,.00645,.005),(.95,.25,.05),collide=False)]
 for a,b in zip(GLYPHS[d],GLYPHS[d][1:]):
  a=np.array(a);b=np.array(b);delta=b-a;angle=math.atan2(delta[1],delta[0]);q=(0,0,math.sin(angle/2),math.cos(angle/2))
  shapes.append(parts.box((float(np.linalg.norm(delta)),.0003,.001),(*((a+b)/2),-L/2+.0005),BLACK,quat=q))
  shapes.append(parts.box((float(np.linalg.norm(delta)),.0005,.0001),(*((a+b)/2),L/2+.0001),BLACK,quat=q,collide=False))
 PUNCHES.append(parts.Part('punch_'+str(d),pos=p,yaw=START_YAWS[d-1],mass=.090,shapes=shapes,contact_offset=.0001,report=('base',),friction=(.8,.65)))
# Hammer native handle (estimated detail), flat striking face down. Head at+120mm from the grip, handle extends-150mm.
HP=(.15,-.20,TOP+.11)
# Explicit material mass distribution: 8oz head, estimated18g steel collar and99.934g hickory handle.
_components=[(.226796,np.array((0,.120,-.005)),np.array((.0254,.0254,.08382))),(.018,np.array((0,.09714,0)),np.array((.0254,.02032,.0254))),(.099934,np.array((0,-.03112,0)),np.array((.025,.23624,.022)))]
_components=[(m,p+np.array((0.,-.070,0.)),size) for m,p,size in _components]
_HCOM=sum(m*p for m,p,size in _components)/.34473
_HTENSOR=np.zeros((3,3))
for m,p,size in _components:
 delta=p-_HCOM;_HTENSOR+=m*(np.diag((sum(size**2)-size**2)/12)+np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
_HDIAG,_HAXES=np.linalg.eigh(_HTENSOR)
if np.linalg.det(_HAXES)<0:_HAXES[:,0]*=-1
HAMMER=parts.Part('hammer',pos=HP,mass=.34473,torsional_patch_radius=.010,min_torsional_patch_radius=.008,mass_center=tuple(_HCOM),mass_inertia=tuple(_HDIAG),mass_axes=tuple(parts._matrix_to_quat(_HAXES)),contact_offset=.0001,report=('base',),friction=(.9,.75),shapes=[
 parts.box((.025,.23624,.022),(0,-.03112,0),(.55,.32,.15)),
 parts.cylinder(.01143,.06482,(0,.120,-.0095),STEEL),parts.cylinder(.01016,.019,(0,.120,.03241),STEEL),
 parts.box((.0254,.02032,.0254),(0,.09714,0),(.75,.12,.08)),
 # Flat-sided eye section of the head, maker width C25.4mm; the face and peen ends stay round.
 parts.box((.0254,.0254,.0254),(0,.120,0),STEEL)])
HAMMER=replace(HAMMER,shapes=tuple(replace(shape,at=(shape.at[0],shape.at[1]-.070,shape.at[2])) for shape in HAMMER.shapes))
RESTS=[]
for i,p in enumerate(STARTS):
 # Punch holder: 32x32x4mm base with a 16mm square socket 16mm deep, turned to the punch's start yaw.
 RESTS.append(parts.Part('punch_rest_'+str(i+1),pos=(p[0],p[1],TOP),yaw=START_YAWS[i],contact_offset=.0001,shapes=[parts.box((.032,.032,.004),(0,0,.002),GREY),
  parts.box((.024,.004,.016),(0,.010,.012),GREY),parts.box((.024,.004,.016),(0,-.010,.012),GREY),parts.box((.004,.016,.016),(.010,0,.012),GREY),parts.box((.004,.016,.016),(-.010,0,.012),GREY)]))
RESTS.append(parts.Part('hammer_cradle',pos=HP,shapes=[parts.box((.065,.245,.01),(0,-.090,-.033),GREY),parts.box((.04,.02,.012),(0,-.170,-.017),GREY),parts.box((.04,.012,.012),(0,.008,-.017),GREY)],contact_offset=.0001))
card=[parts.box((.065,.035,.001),(0,0,0),WHITE)]
for d in (1,2,3):
 for a,b in zip(GLYPHS[d],GLYPHS[d][1:]):
  a=np.array(a);b=np.array(b);v=b-a;q=(0,0,math.sin(math.atan2(v[1],v[0])/2),math.cos(math.atan2(v[1],v[0])/2))
  card.append(parts.box((float(np.linalg.norm(v)),.0005,.0001),(*((a+b)/2+((d-2)*.016,0)),.0006),BLACK,quat=q,collide=False))
CARD=parts.Part('work_card',pos=(.06,0,TOP+.001),shapes=card)
ruler=[parts.box((.120,.014,.001),(0,0,0),STEEL)]
for mm in range(0,121,1):ruler.append(parts.box((.00015,.006 if mm%10==0 else .003,.0001),(-.060+mm*.001,.003,.0006),BLACK,collide=False))
RULER=parts.Part('steel_rule',pos=(.12,-.084,TOP+.001),shapes=ruler)
OBJECTS=[VISE,COUPON,HAMMER,*PUNCHES,*RESTS,CARD,RULER]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'brass_coupon':COUPON.pos}),lots.Lot('B',place={'brass_coupon':(CX+.018,0,Z-.0015)}))
DEMO_TOUCHES=set()
# Estimated fully plastic pressure0.6GPa.45deg cutting edge: W=H*line_length*depth^2.
HARDNESS=.6e9;READ_DEPTH=.00005;MIN_SPEED=.35;EPS=1e-6

def fresh():return dict(known=False,fail='',marks=[],active=[None]*3,previous=None,hold=0.,time=0.,peak_force=0.,impulse=[0.]*3,energy=[0.]*3,backing=[0.]*3,hand=[0.]*3,clear=False,parked=False,certificate=None)
REC=fresh();VIEWS=None;BACK_VIEW=None;BARE_VIEW=None;HAND_VIEW=None;_origin=None

def imprint(d,energy,position,rotation,attitudes=None,substrate=None):
 pts,weights=SAMPLES[d];r=np.asarray(rotation);p=np.asarray(position)
 substrate=COUPON.state() if substrate is None else substrate
 sr=parts._quat_to_matrix(substrate.quat_xyzw);sp=np.asarray(substrate.pos)
 local=np.column_stack((pts,np.full(len(pts),-L/2)));world=local@r.T+p
 # Tilt first touches the low edge; solve local indentation volume using available actual-impact work.
 relative=local@(sr.T@r).T
 heights=relative[:,2]-relative[:,2].min()
 for attitude in attitudes or []:
  z=(local@np.asarray(attitude).T)[:,2];heights=np.maximum(heights,z-z.min())
 lo=0.;hi=.001
 for _ in range(45):
  mid=(lo+hi)/2;work=HARDNESS*float(np.sum(weights*np.maximum(0,mid-heights)**2))
  if work<energy:lo=mid
  else:hi=mid
 depth=np.maximum(0,(lo+hi)/2-heights)
 missing=0.;max_gap=0.
 for ok,w in zip(depth>=READ_DEPTH,weights):
  missing=0. if ok else missing+float(w);max_gap=max(max_gap,missing)
 material=(world-sp)@sr
 return dict(max_gap=max_gap,d=d,points=material[:,:2].tolist(),depth=depth.tolist(),coverage=float(weights[depth>=READ_DEPTH].sum()/weights.sum()),energy=float(energy),center=((p+r@np.array((0,0,-L/2))-sp)@sr)[:2].tolist(),yaw=math.atan2((sr.T@r)[1,0],(sr.T@r)[0,0]))

def valid_mark(mark,coupon):
 center=np.asarray(mark['center']);expected=np.array(((mark['d']-2)*.012,0.))
 return mark['coverage']>=.9-EPS and mark['max_gap']<=.0005+EPS and np.linalg.norm(center-expected)<=.004+EPS and abs(mark['yaw'])<=math.radians(20)+EPS

def tiers(marks,coupon,parked):
 if len(marks)>3:return 0.
 correct=[]
 if len({m['d'] for m in marks})!=len(marks):return 0.
 for m in marks:
  if m['d'] not in (1,2,3):return 0.
  error=np.linalg.norm(np.asarray(m['center'])-((m['d']-2)*.012,0.))
  if error>.004+EPS or abs(m['yaw'])>math.radians(20)+EPS:return 0.
  if valid_mark(m,coupon):correct.append(m)
 n=len(correct)
 if n<3:return n*.25
 marks=sorted(marks,key=lambda m:m['d']);centers=np.array([m['center'] for m in marks]);pitch=np.diff(centers[:,0]);baseline=np.ptp(centers[:,1])
 accurate=baseline<=.001+EPS and np.all(abs(pitch-.012)<=.001+EPS) and max(abs(m['yaw']) for m in marks)<=math.radians(10)+EPS
 return 1. if accurate and parked else .75

@lru_cache(maxsize=1)
def face_mesh():
 from yam_arena import analysis
 result={}
 for name in ('gripper','tip_left','tip_right'):
  cloud=[]
  for h in analysis.link_hulls()[name]:
   cloud.extend(h.vertices)
   for pl in np.unique(h.planes.round(7),axis=0):
    face=h.vertices[np.abs(h.vertices@pl[:3]-pl[3])<2e-6]
    if len(face)<3:continue
    c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(pl[:3],u);face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
    for a,b in zip(face,np.roll(face,-1,axis=0)):
     n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
     for i in range(n+1):
      for j in range(n-i+1):cloud.append(c+(a-c)*i/n+(b-c)*j/n)
  points=np.array(cloud);_,ix=np.unique(np.floor(points/.002).astype(int),axis=0,return_index=True);result[name]=points[ix]
 return result

def hands(sim):
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[]
 for arm in ('left_robot','right_robot'):
  asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
  for name,pts in face_mesh().items():
   p=poses[asset.body_names.index(name)];clouds.append(pts@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
 return clouds

def hand_gap(clouds,spec,s):
 return min(float(np.linalg.norm(np.maximum(np.abs((p-c)@r)-sz/2,0),axis=1).min()) for _,c,r,sz in spec.world_boxes({'base':(s.pos,parts._quat_to_matrix(s.quat_xyzw))}) for p in clouds)

def evidence_path():
 if os.environ.get('TRACE_OUT'):return pathlib.Path(os.environ['TRACE_OUT']).with_name('stamp.jsonl')
 if '--log-dir' in sys.argv:return pathlib.Path(sys.argv[sys.argv.index('--log-dir')+1]).parent/'stamp.jsonl'
 return None

def render_marks():
 from isaaclab.sim.utils import get_current_stage
 from pxr import UsdGeom,Gf
 stage=get_current_stage()
 root='/World/PermanentMarks'
 if stage.GetPrimAtPath(root):stage.RemovePrim(root)
 anchor=UsdGeom.Xform.Define(stage,root)
 anchor.AddTranslateOp().Set(Gf.Vec3d(*map(float,np.asarray(REC.get('coupon_pos',COUPON.pos))+_origin)))
 q=REC.get('coupon_quat',(0,0,0,1));anchor.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
 for i,m in enumerate(REC['marks']):
  for j,(xy,dep) in enumerate(zip(m['points'],m['depth'])):
   if dep<.00001:continue
   g=UsdGeom.Cylinder.Define(stage,f'{root}/m{i}p{j}');g.CreateRadiusAttr(max(.00010,dep));g.CreateHeightAttr(.000025)
   g.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]),float(xy[1]),.00152));g.CreateDisplayColorAttr().Set([Gf.Vec3f(.11,.08,.03)])

def move_marks(coupon):
 from isaaclab.sim.utils import get_current_stage
 from pxr import Gf
 prim=get_current_stage().GetPrimAtPath('/World/PermanentMarks')
 if prim:
  prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*map(float,np.asarray(coupon.pos)+_origin)))
  q=coupon.quat_xyzw;prim.GetAttribute('xformOp:orient').Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))

@parts.on_reset
def reset(sim):
 global REC,VIEWS,BACK_VIEW,BARE_VIEW,HAND_VIEW,_origin
 REC=fresh();VIEWS=None;_origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
 from isaaclab_physx.physics import PhysxManager
 from isaaclab.sim.utils import get_current_stage
 from pxr import UsdPhysics
 stage=get_current_stage();paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName() in ('gripper','tip_left','tip_right') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
 if len(paths)!=6:raise ValueError('missing hand contact paths')
 sv=PhysxManager.get_physics_sim_view();VIEWS=[]
 BARE_VIEW=sv.create_rigid_contact_view([parts._PATHS['hammer']],filter_patterns=[[parts._PATHS['brass_coupon']]],max_contact_data_count=256)
 BACK_VIEW=sv.create_rigid_contact_view([parts._PATHS['brass_coupon']],filter_patterns=[[parts._PATHS['bench_vise']+'/s1']],max_contact_data_count=256)
 leftpaths=[p for p in paths if '/LeftArm/' in p]
 if len(leftpaths)!=3:raise ValueError('missing left hand paths '+str(paths))
 HAND_VIEW=sv.create_rigid_contact_view([parts._PATHS['hammer']],filter_patterns=[leftpaths],max_contact_data_count=256)
 for spec in PUNCHES:
  p=parts._PATHS[spec.name]
  VIEWS.append([sv.create_rigid_contact_view([p],filter_patterns=[[parts._PATHS['hammer']]],max_contact_data_count=256),sv.create_rigid_contact_view([p],filter_patterns=[[parts._PATHS['brass_coupon']]],max_contact_data_count=256),sv.create_rigid_contact_view([p],filter_patterns=[paths],max_contact_data_count=256)])
 render_marks()

@parts.on_physics_step
def physical(sim):
 r=REC;dt=float(sim.env.unwrapped.sim.get_physics_dt());states=sim.states();hammer=states['hammer'];coupon=states['brass_coupon']
 r['known']=True;r['coupon_pos']=coupon.pos.tolist();r['coupon_quat']=coupon.quat_xyzw.tolist();sr=parts._quat_to_matrix(coupon.quat_xyzw)
 r['backing_force']=float(np.linalg.norm(parts._np(BACK_VIEW.get_contact_force_matrix(dt=dt)).reshape(-1,3),axis=1).sum())
 if np.linalg.norm(parts._np(BARE_VIEW.get_contact_force_matrix(dt=dt)).reshape(-1,3),axis=1).sum()>1.:r['fail']='bare hammer hit specimen'
 if np.linalg.norm(parts._np(HAND_VIEW.get_contact_force_matrix(dt=dt)).reshape(-1,3),axis=1).sum()>1.:r['fail']='hammer struck punch hand'
 for st in [hammer,coupon,*[states[p.name] for p in PUNCHES]]:
  if not np.isfinite(np.r_[st.pos,st.quat_xyzw,st.lin_vel,st.ang_vel]).all() or not .9<np.linalg.norm(st.quat_xyzw)<1.1:r['known']=False;r['fail']='unavailable rigid state';return
 if not np.isfinite(r['backing_force']):r['known']=False;r['fail']='unavailable backing observation';return
 start=np.asarray(lots.current().pos('brass_coupon'))
 if np.linalg.norm(coupon.pos-start)>.003 or abs(parts._quat_to_matrix(coupon.quat_xyzw)[2,2])<math.cos(math.radians(5)):r['fail']='stock escaped vise'
 current={}
 for i,spec in enumerate(PUNCHES):
  s=states[spec.name];rot=parts._quat_to_matrix(s.quat_xyzw);tip=s.pos+rot@np.array((0,0,-L/2));head=s.pos+rot@np.array((0,0,L/2))
  hrot=parts._quat_to_matrix(hammer.quat_xyzw);face=hammer.pos+hrot@np.array((0,.050,-.04191));hv=hammer.lin_vel+np.cross(hammer.ang_vel,face-hammer.pos);pv=s.lin_vel+np.cross(s.ang_vel,head-s.pos)
  closing=max(0.,float(-(hv-pv)[2]));com_velocity=hammer.lin_vel+np.cross(hammer.ang_vel,hrot@_HCOM)
  tensor=hrot@_HTENSOR@hrot.T;kinetic=.5*HAMMER.mass*float(com_velocity@com_velocity)+.5*float(hammer.ang_vel@tensor@hammer.ang_vel)
  current[i]=(closing,kinetic,float(-hv[2]))
  vectors=[parts._np(v.get_contact_force_matrix(dt=dt)).reshape(-1,3) for v in VIEWS[i]]
  force=abs(float(vectors[0][:,2].sum()));back,hand=[float(np.linalg.norm(v,axis=1).sum()) for v in vectors[1:]]
  top_strike=abs(face[2]-head[2])<=.003 and np.linalg.norm(face[:2]-head[:2])<=.019 and hrot[2,2]>=math.cos(math.radians(20))
  if not np.isfinite([force,back,hand,closing,kinetic]).all():r['known']=False;r['fail']='unavailable contact or velocity';return
  r['peak_force']=max(r['peak_force'],force);r['backing'][i]=max(r['backing'][i],back);r['hand'][i]=max(r['hand'][i],hand)
  active=r['active'][i]
  # Only a native, specific hammer/punch contact can arm the irreversible event.
  if force>1. and top_strike and active is None:
   previous=(r['previous'] or {}).get(i,(0.,0.,0.));incoming=previous[0];incident=previous[1];supported=back>1. and r['backing_force']>1. and abs(tip[2]-Z)<=.0015 and np.max(abs(tip[:2]-coupon.pos[:2]))<=.025
   footprint=np.column_stack((SAMPLES[i+1][0],np.full(len(SAMPLES[i+1][0]),-L/2)))@rot.T+s.pos
   active=dict(substrate=coupon,down_speed=previous[2],footprint=((footprint-coupon.pos)@sr)[:,:2],slip=0.,attitudes=[],incident=incident,speed=incoming,impulse=0.,peak=0.,supported=supported,hand=hand>1.,pose=np.asarray(s.pos).copy(),rotation=rot.copy(),since=0.,quiet=0.)
   r['active'][i]=active
  if active is not None:
   active['impulse']+=force*dt
   if force>1. and back>1.:
    footprint=np.column_stack((SAMPLES[i+1][0],np.full(len(SAMPLES[i+1][0]),-L/2)))@rot.T+s.pos
    active['slip']=max(active['slip'],float(np.linalg.norm(((footprint-coupon.pos)@sr)[:,:2]-active['footprint'],axis=1).max()));active['attitudes'].append(sr.T@rot)
   if force>active['peak']:active['pose']=np.asarray(s.pos).copy();active['rotation']=rot.copy();active['substrate']=coupon
   active['peak']=max(active['peak'],force);active['supported'] |= back>1. and force>1. and r['backing_force']>1. and abs(tip[2]-Z)<=.0015;active['hand'] |= hand>1. and force>1.;active['since']+=dt;active['quiet']=active['quiet']+dt if force<=1. else 0.
   if active['quiet']>=.025 or active['since']>=.15:
    # Half impulse * incident speed, capped by measured incoming tool kinetic energy.
    energy=min(.5*active['impulse']*active['speed'],active['incident'])
    r['impulse'][i]+=active['impulse'];r['energy'][i]+=energy
    if active['speed']>=MIN_SPEED and active['down_speed']>=.1 and active['supported'] and energy>.001:
     mark=imprint(i+1,energy,active['pose'],active['rotation'],active['attitudes'],active['substrate']);mark['slip']=active['slip'];r['marks'].append(mark)
     if active['slip']>.0005+EPS:r['fail']='die footprint slipped more than0.5mm during impact'
     if not active['hand']:r['fail']='strike without counterholding punch'
     if len(r['marks'])>3 or sum(m['d']==i+1 for m in r['marks'])>1:r['fail']='second impression'
     if energy>.40:r['fail']='overstrike damage proxy'
     render_marks()
    r['active'][i]=None
 r['previous']=current

@parts.on_step
def step(sim):
 r=REC;states=sim.states();t=float(sim.t);dt=t-r['time'];r['time']=t;move_marks(states['brass_coupon'])
 clouds=hands(sim);r['clear']=all(hand_gap(clouds,spec,states[spec.name])>=.05-EPS for spec in [COUPON,HAMMER,*PUNCHES])
 r['parked']=all(np.linalg.norm(states[p.name].pos-np.asarray(p.pos))<=.015 and states[p.name].resting(.005,.1) for p in [HAMMER,*PUNCHES])
 still=all(states[p.name].resting(.005,.1) for p in [COUPON,HAMMER,*PUNCHES])
 positions=np.array([states[p.name].pos for p in [COUPON,HAMMER,*PUNCHES]]);quats=np.array([states[p.name].quat_xyzw for p in [COUPON,HAMMER,*PUNCHES]])
 anchor=r.get('anchor');anchor_q=r.get('anchor_q')
 moved=anchor is None or np.linalg.norm(positions-anchor,axis=1).max()>.0005 or np.max(2*np.arccos(np.clip(abs(np.sum(quats*anchor_q,axis=1)),0,1)))>math.radians(1)
 if not r['clear'] or not still:r['hold']=0.;r['anchor']=None;r['anchor_q']=None
 elif moved:r['hold']=0.;r['anchor']=positions.copy();r['anchor_q']=quats.copy()
 else:r['hold']+=max(0.,dt)
 r['certificate']=certificate(states)
 if evidence_path() is not None:
  row={k:v for k,v in r.items() if k not in ('active','previous','certificate','anchor','anchor_q')};row.update(t=t,score=score(states),objects={n:dict(p=s.pos.tolist(),q=s.quat_xyzw.tolist(),v=s.lin_vel.tolist()) for n,s in states.items() if n in ['hammer','brass_coupon','punch_1','punch_2','punch_3']})
  row['grippers']={}
  from yam_arena.constants import TCP_OFFSET
  for arm in ('left_robot','right_robot'):
   robot=sim.env.unwrapped.scene[arm];poses=parts._np(robot.data.body_link_pose_w)[0];pose=poses[robot.body_names.index('gripper')];joint=parts._np(robot.data.joint_pos)[0]
   row['grippers'][arm]=dict(tcp=(pose[:3]-_origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist(),quat=pose[3:].tolist(),finger=float(joint[robot.joint_names.index('joint7')]))
  with evidence_path().open('a') as f:f.write(json.dumps(row)+'\n')

def certificate(objects):
 return tuple(np.concatenate([np.r_[objects[p.name].pos,objects[p.name].quat_xyzw,objects[p.name].lin_vel,objects[p.name].ang_vel] for p in [COUPON,HAMMER,*PUNCHES]]))

def score(objects):
 try:
  r=REC
  if not r['known'] or r['fail'] or not r['clear'] or r['hold']<1.-EPS or r['certificate']!=certificate(objects):return 0.
  return tiers(r['marks'],objects['brass_coupon'],r['parked'])
 except (KeyError,ValueError,TypeError):return 0.
def success(objects):return score(objects)==1.

LEAN=75.;GRASP_DZ=.002;PUNCH_XY=(0.,0.)
# Measured open-loop corrections from the lot-A trace: the held punch leaned1.0degree toward+x, so the punch hand's
# commanded rotation is turned back about world y. The hammer is pinched across its head at the eye, directly above
# the striking face, so the blow's reaction passes through the pinch instead of rolling the tool in the fingers (a
# handle grip 50mm behind the head rolled 27degrees at every impact). The aim offset corrects the measured drift of
# the face toward-x/-y during the stroke.
LEFT_TRIM_DEG=-1.3
HAMMER_LEAN=30.
STAGE=np.array((0.,-.040,.080))
# Per character: face height above the punch head when aligned, commanded depth below the head, stroke time,
# finger setting while the punch is seated and struck, hammer x/y aim offset.
STRIKES=[(.080,.085,.15,-.0060,(.0035,.0085))]*3
def demo():
 # Published starts and visible coupon layout only; fixed timing, no object-state feedback.
 # The punch hand keeps one constant75degree lean (fingertips toward +x, pads on the punch's y faces), so the grip never
 # rotates the punch: only its yaw changes between the start rest and the coupon.
 left=[];right=[];OPEN=-.035;GRIP=-.0030;HGRIP=-.0040;elapsed=[0.,0.];trim=parts._rot((0,1,0),math.radians(LEFT_TRIM_DEG));work=[False]
 def W(arm,dt,p,f,label,angle=None,lean=None):
  if angle is None:angle=0. if arm==0 else math.pi/2
  seq=left if arm==0 else right
  rotation=tilt._TiltedYaw(angle,(trim if work[0] else np.eye(3))@tilt.rotation(angle,LEAN,angle)) if arm==0 else tilt._TiltedYaw(angle,tilt.rotation(angle,HAMMER_LEAN,math.pi/2))
  seq.append(Waypoint(dt,tuple(p),f,label,rotation));elapsed[arm]+=dt
 def sync():
  end=max(elapsed)
  for a,seq in [(0,left),(1,right)]:
   if elapsed[a]<end:W(a,end-elapsed[a],seq[-1].pos,seq[-1].finger,'wait',float(seq[-1].yaw))
 hp=np.asarray(HP)
 hg=hp+(0,.053,0)
 W(1,2,hg+(0,0,.07),OPEN,'approach hammer');W(1,2,hg,OPEN,'lower');W(1,1,hg,HGRIP,'grasp head at eye');W(1,2,hg+(0,0,.06),HGRIP,'lift hammer')
 W(0,7,np.asarray(STARTS[0])+(0,.04,.08),OPEN,'wait clear');sync()
 cp=lots.current().pos('brass_coupon')
 for i,start in enumerate(STARTS):
  yaw=START_YAWS[i]
  up,down,stroke,seat,aim=STRIKES[i]
  target=np.array((cp[0]+(i-1)*.012,cp[1],Z+L/2));grasp=np.asarray(start)+np.array((0,0,GRASP_DZ))
  goal=target+np.array((PUNCH_XY[0],PUNCH_XY[1],GRASP_DZ));strike=np.array((target[0]+aim[0],target[1]+aim[1],Z+L+.04191))
  W(0,2,grasp+(0,0,.075),OPEN,'approach punch',yaw);W(0,2,grasp,OPEN,'lower to shank',yaw);W(0,1,grasp,GRIP,'grasp shank',yaw);W(0,2,grasp+(0,0,.075),GRIP,'lift punch',yaw);work[0]=True;W(0,3,goal+(0,0,.035),GRIP,'register character');W(0,2,goal,GRIP,'rest face on brass')
  W(1,10 if i==0 else 4,strike+STAGE,HGRIP,'stage hammer clear of punch hand');W(0,.6,goal,seat,'seat punch');sync()
  W(1,2,strike+(0,0,up),HGRIP,'align flat face');sync()
  W(1,stroke,strike+(0,0,-down),HGRIP,'single strike');W(1,.35,strike+(0,0,up),HGRIP,'retract after strike');W(0,stroke+.35,goal,seat,'counterhold strike');sync()
  W(1,1,strike+STAGE,HGRIP,'lift hammer off punch');W(1,3,hg+(0,0,.06),HGRIP,'return hammer')
  W(0,1,goal,seat,'hold until hammer clear');W(0,2,goal,GRIP,'regrip punch')
  W(0,2,goal+(0,0,.04),GRIP,'lift punch clear');work[0]=False;W(0,3,grasp+(0,0,.075),GRIP,'return punch',yaw);W(0,2,grasp,GRIP,'set punch down',yaw);W(0,1,grasp,OPEN,'release punch',yaw);W(0,2,grasp+(0,0,.08),OPEN,'withdraw',yaw);sync()
 W(1,2,hg-(0,0,.002),HGRIP,'lower cradle');W(1,1,hg-(0,0,.002),OPEN,'release hammer');W(1,2,hg+(0,0,.09),OPEN,'withdraw hammer');sync()
 W(0,2,(.20,.25,TOP+.12),OPEN,'park left');W(1,2,(.08,-.02,TOP+.14),OPEN,'park right');W(0,2,(.20,.25,TOP+.12),OPEN,'still');W(1,2,(.08,-.02,TOP+.14),OPEN,'still')
 return {'left':left,'right':right}
