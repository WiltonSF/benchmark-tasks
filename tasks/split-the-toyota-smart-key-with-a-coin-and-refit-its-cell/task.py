"""A Toyota HYQ14FBA smart-key shell, a quarter in a shallow slotted rest, and two CR2032 cells on a desk mat.
The shell has a seam notch and a circular cell seat. One cell is enclosed; the replacement rests on a narrow tray pedestal.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from mechanism import hands
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, grip_rotation
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.05
BLACK=(.12,.13,.15); TRIM=(.62,.65,.68); STEEL=(.76,.78,.8); GREEN=(.18,.65,.24); RED=(.8,.22,.15)
# True-size envelope estimate from the selected shell photograph, and clearance around a standard 20 mm cell; not an internal photo scale.
L,W,T=.070,.040,.015; X,Y=.320,.025; START_Z=TOP+.0325
PIVOT=np.array((-.033,0.,0.)); NOTCH=np.array((.023,-.0185,0.)); GRIP=np.array((0.,0.,-.004))
OPEN_TORQUE=.12 # N m estimate: 10 N thumb force at a quarter's 12.13 mm radius, rounded.
OPEN_EFFORT=.48 # 7.5 N at 64 mm: 0.12 N m at a 16 mm coin jaw contact span.
LATCH_EFFORT=.72 # Closing snap ramp: 12 N at a 60 mm cover lever arm.
CLOSE_FORCE=12.; CLOSE_TRAVEL=.020; CLOSE_K=LATCH_EFFORT/CLOSE_TRAVEL; CLOSE_DAMP=.05; MIN_LOADED_TURN=math.radians(2.); CLOSED_Q=.0045; LOCK_K=8.; LOCK_DAMP=.008; FREE_DAMP=.00015
COIN_R=.01213; COIN_T=.00175; CELL_R=.010; CELL_T=.0032
COIN_POS=(.235,-.165,TOP+.03813); CELL_POS=(.350,-.180,TOP+.0356)
SEAT=np.array((.012,0.,.0016)); CELL_BOTTOM=0.
# Base: perimeter, recessed circular seat, and a real 2.8 mm seam notch at its front-right margin.
def b(size,at=(0,0,0),c=BLACK):return parts.box(size,at,c)
base_shapes=[b((.070,.040,.002),(0,0,-.0065)),b((.023,.040,.0058),(-.0235,0,-.003)),
 b((.046,.006,.0058),(.012,.017,-.003)),b((.021,.006,.0058),(-.0005,-.017,-.003)),
 b((.006,.028,.0058),(.032,0,-.003)),b((.009,.028,.0058),(-.0065,0,-.003)),
 # Rigid board and four standoffs place the cell above the base perimeter for honest removal.
 b((.046,.026,.0016),(.012,0,-.0008),(.14,.40,.24))]
# Recess the notch in both jaws. No collider represents empty space.
# Lower notch jaw sits at -1.4mm, upper jaw at +1.4mm (2.8mm opening).
for xx in (-.006,.030):
 for yy in (-.008,.008):base_shapes.append(b((.003,.003,.0039),(xx,yy,-.00355),BLACK))
for angle in np.arange(16)*2*math.pi/16:
 if math.sin(angle)>-.3:continue # open downhill side of the molded seat
 centre=(SEAT[0]+.0113*math.cos(angle),.0113*math.sin(angle),.001)
 base_shapes.append(parts.box((.0014,.0044,.002),centre,TRIM,quat=tuple(parts._matrix_to_quat(parts._rot((0,0,1),angle)))))
base_shapes += [b((.020,.005,.002),(.023,-.0175,-.0024),TRIM)]
lid_shapes=[b((.066,.034,.002),(0,0,.0065)),b((.066,.005,.0058),(0,.0145,.003)),
 b((.0448,.005,.0058),(-.0106,-.0145,.003)),b((.005,.024,.0058),(-.0305,0,.003)),
 b((.005,.024,.0058),(.0305,0,.003)),b((.020,.005,.002),(.023,-.0175,.0024),TRIM)]
# Four visible button islands; not actuated or claimed as an RF/electrical test.
lid_shapes += [b((.010,.016,.0006),(xx,0,.0078),(.32,.33,.35)) for xx in (-.022,-.007,.008,.023)]
def extruded(poly,z0,z1,color=BLACK):
 n=len(poly);points=[(float(x),float(y),float(z)) for z in (z0,z1) for x,y in poly]
 faces=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(j,(j+1)%n,(j+1)%n+n,j+n) for j in range(n)]
 return parts.hull(points,faces,color)
# Mouth is 26 mm wide, tapering to the 20 mm load-bearing inner jaws over 5 mm.
# The outer right relief wraps the 70 mm shell's corner. These dimensions are a declared geometry estimate.
base_shapes += [extruded(((.010,-.020),(.013,-.015),(.013,-.014),(.010,-.014)),-.0059,-.0001),
 extruded(((.035,-.018333333),(.035,-.014),(.033,-.014),(.033,-.015)),-.0059,-.0001)]
lid_shapes += [extruded(((.0118,-.017),(.013,-.015),(.013,-.012),(.0118,-.012)),.0001,.0059)]
LID=parts.Hinge('cover',tuple(lid_shapes),tuple(PIVOT),(0,-1,0),0.,1.65,mass=.012,
 spring=LOCK_K,damping=LOCK_DAMP,max_effort=2.,target=-OPEN_EFFORT/LOCK_K)
FOB=parts.Part('toyota_smart_key',pos=(X,Y,START_Z),mass=.018,shapes=tuple(base_shapes),joints=(LID,),
 friction=(.7,.55),contact_offset=.00012,report=('base','cover'))
# Quarter stands vertically, circular faces along y, lower edge extends below the fingers.
COIN=parts.Part('quarter',pos=COIN_POS,mass=.00567,shapes=(parts.prism(COIN_R,COIN_T,32,color=STEEL,quat=tuple(parts._matrix_to_quat(parts._rot((0,0,1),math.pi/2)))),),
 friction=(.6,.45),contact_offset=.00010,report=('base',))
def cell(name,pos,color):
 return parts.Part(name,pos=tuple(map(float,pos)),mass=.003,shapes=(parts.cylinder(CELL_R,CELL_T,color=STEEL),),
  visuals=(parts.Visual('plus',((-.006,-.001,.00163),(.006,-.001,.00163),(.006,.001,.00163),(-.006,.001,.00163),
   (-.001,-.006,.00164),(.001,-.006,.00164),(.001,.006,.00164),(-.001,.006,.00164)),((0,1,2),(0,2,3),(4,5,6),(4,6,7)),color),),
  friction=(.3,.25),contact_offset=.00008,report=('base',))
OLD=cell('old_cell',(X+SEAT[0],Y,START_Z+SEAT[2]),RED)
NEW=cell('new_cell',CELL_POS,GREEN)
MAT=parts.Part('desk_mat',pos=(.32,0,TOP+.0005),shapes=(b((.44,.50,.001),c=(.31,.38,.40)),),contact_offset=.0001)
REST=parts.Part('fob_pickup_rest',pos=(X,Y,TOP+.003),shapes=(b((.010,.030,.0215),(-.020,0,.01075),(.7,.67,.56)),
 b((.010,.030,.0215),(.020,0,.01075),(.7,.67,.56))),contact_offset=.0001)
def tray(name,pos,color,coin=False):
 # Shallow real tray, with narrow central pedestal to present a flat cell above the fingers' bottom reach.
 sh=[b((.064,.064,.002),(0,0,.001),color)]
 sh += [b((.064,.002,.006),(0,s*.031,.005),color) for s in (-1,1)]
 sh += [b((.002,.060,.006),(s*.031,0,.005),color) for s in (-1,1)]
 if coin:
  sh += [b((.024,.003,.004),(0,s*.003,.027),color) for s in (-1,1)]
  sh += [b((.024,.012,.023),(0,0,.0135),color)]
 else:sh += [b((.008,.008,.032),(0,0,.018),color)]
 return parts.Part(name,pos=pos,mass=.15,shapes=tuple(sh),contact_offset=.0001)
COIN_TRAY=tray('coin_rest',(COIN_POS[0],COIN_POS[1],TOP+.001),(.7,.62,.41),True)
NEW_TRAY=tray('new_cell_tray',(CELL_POS[0],CELL_POS[1],TOP),(.25,.5,.28))
OLD_TRAY=parts.Part('old_cell_tray',pos=(.320,.045,TOP+.001),shapes=(b((.110,.100,.002),(0,0,.001),(.55,.25,.20)),
 *[b((.110,.003,.006),(0,s*.0485,.005),(.55,.25,.20)) for s in (-1,1)],
 *[b((.003,.094,.006),(s*.0535,0,.005),(.55,.25,.20)) for s in (-1,1)]),contact_offset=.0001)
OBJECTS=[MAT,REST,COIN_TRAY,NEW_TRAY,OLD_TRAY,FOB,COIN,OLD,NEW]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'quarter':COIN_POS}),
 lots.Lot('B',place={'quarter':(.205,-.180,COIN_POS[2]),'coin_rest':(.205,-.180,TOP+.001),
 'new_cell':(.385,-.150,CELL_POS[2]),'new_cell_tray':(.385,-.150,TOP)}))
# lots.py moves only rigid objects; fixed tray coordinates are handled in its lot declaration below.
DEMO_TOUCHES={'coin_rest','new_cell_tray','fob_pickup_rest','desk_mat'}

# History and contact latch fail closed. Only reset callbacks clear irreversible records.
def fresh():return dict(known=False,zero=False,why='',unlatched=False,split=False,latched=True,reclosed=False,
 old_out=False,ever_open=False,hold=0.,t=0.,last=None,views=None,handview=None,peak_torque=0.,peak_close=0.,
 current_torque=0.,current_close=0.,right_coin_grip=0.,loaded_turn=0.,loaded_work=0.,coin_angle=None,history=[],events=[],closing=False,prevq=0.,cert=None,clear=False,base_hold=0.)
S=fresh()
def event(text):
 S['events'].append(text);print('[fob]',text,flush=True)
def fail(text):
 if not S['zero']:event('ZERO '+str(text));S.update(zero=True,why=str(text))
def rot(s):return parts._quat_to_matrix(s.quat_xyzw)
def signature(o):
 return tuple(np.concatenate([np.r_[o[n].pos,o[n].quat_xyzw,o[n].lin_vel,o[n].ang_vel,
  list(getattr(o[n],'joints',{}).values())] for n in (FOB.name,COIN.name,OLD.name,NEW.name)]))
def local(o,name):return (np.asarray(o[name].pos)-o[FOB.name].pos)@rot(o[FOB.name])
def seated(o):
 p=local(o,NEW.name);normal=rot(o[FOB.name])[:,2];up=rot(o[NEW.name])[:,2]
 return bool(np.linalg.norm(p[:2]-SEAT[:2])<=.002+1e-9 and abs(p[2]-SEAT[2])<=.001+1e-9 and normal@up>=math.cos(math.radians(12)))
def old_in_tray(o):
 p=np.asarray(o[OLD.name].pos);c=np.asarray(OLD_TRAY.pos)
 return bool(abs(p[0]-c[0])<=.041 and abs(p[1]-c[1])<=.036 and TOP<=p[2]<=TOP+.009)
def clear_hands(clouds,o):
 if not clouds or any(not len(p) or not np.isfinite(p).all() for p in clouds):return False
 for spec in (FOB,COIN,OLD,NEW):
  st=o[spec.name]; poses={'base':(st.pos,rot(st))}
  for n,(p,q) in getattr(st,'links',{}).items():poses[n]=(p,parts._quat_to_matrix(q))
  for _,c,r,sz in spec.world_boxes(poses):
   if any(np.linalg.norm(np.maximum(abs((p-c)@r)-sz/2,0),axis=1).min()<=.005 for p in clouds):return False
 return True

def cover_gap(q):
 q=max(0.,float(q));return .0002+.066*math.sin(q)+.0001*(math.cos(q)-1.)

def score(o):
 try:
  if not S['known'] or S['zero'] or S['cert']!=signature(o) or not S['split'] or S['hold']<1.-1e-8:return 0.
  q=o[FOB.name].joints['cover']
  if not S['clear']:return 0.
  if not seated(o):return .25
  oldp=local(o,OLD.name)
  removed=np.linalg.norm(np.maximum(abs(oldp)-np.array((.035,.020,.0075)),0.))>=.011
  if not S['old_out'] or not removed:return .25
  if cover_gap(q)>.0005+1e-9 or not S['reclosed']:return .5
  return 1. if old_in_tray(o) else .75
 except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(o):return score(o)==1.

def write_latch(sim,latched):
 import torch
 asset=sim.env.unwrapped.scene[FOB.name];ids=[asset.joint_names.index('cover')];dev=asset.device
 k=LOCK_K if latched else 0.
 asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]],device=dev),joint_ids=ids)
 asset.write_joint_damping_to_sim_index(damping=torch.tensor([[LOCK_DAMP if latched else FREE_DAMP]],device=dev),joint_ids=ids)
 asset.write_joint_position_limit_to_sim_index(limits=torch.tensor([[[0.,.040 if latched else 1.65]]],device=dev),joint_ids=ids)
 sim.set_target(FOB.name,'cover',-OPEN_EFFORT/LOCK_K if latched else 0.)
 asset.set_joint_effort_target_index(target=torch.tensor([[0.]],device=dev),joint_ids=ids)
 asset.write_data_to_sim()

@parts.before_reset
def before_reset(sim):write_latch(sim,True)
@parts.on_reset
def reset(sim):
 global S
 S=fresh();hands.reset();write_latch(sim,True)
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)

def arr(x):
 x=x.torch if hasattr(x,'torch') else x
 return np.asarray(x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy() if hasattr(x,'numpy') else x)

def read_contacts(sim,o):
 from isaaclab_physx.physics import PhysxManager
 from yam_arena.robot import asset_link_paths
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);root=parts._PATHS[FOB.name]
 if S['views'] is None:
  physics=PhysxManager.get_physics_sim_view();links=asset_link_paths();env=base.scene.env_prim_paths[0]
  arm_paths=[[env+'/'+arm+links[n] for n in ('tip_left','tip_right','gripper')] for arm in ('LeftArm','RightArm')]
  S['views']=physics.create_rigid_contact_view([parts._PATHS[COIN.name]],filter_patterns=[[root+'/base',root+'/cover']],max_contact_data_count=256)
  S['handview']=physics.create_rigid_contact_view([root+'/base',root+'/cover',parts._PATHS[COIN.name]],filter_patterns=[sum(arm_paths,[])]*3,max_contact_data_count=256)
 dt=float(base.sim.get_physics_dt());rf=rot(o[FOB.name]);hinge=rf@np.array((0,-1.,0));axis=hinge
 pivot=np.asarray(o[FOB.name].pos)+rf@PIVOT;q=float(o[FOB.name].joints['cover']);lr=parts._rot((0,-1,0),q)
 def packet(view):
  force,point,normal,distance,count,start=(arr(x) for x in view.get_contact_data(dt=dt))
  return force.reshape(-1),point.reshape(-1,3)-origin,normal.reshape(-1,3),distance.reshape(-1),count.reshape(-1),start.reshape(-1)
 fm=arr(S['views'].get_contact_force_matrix(dt=dt)).reshape(2,3)
 force,point,normal,distance,count,start=packet(S['views']);tau=0.;useful=0.;jaw_load=[];details=[]
 for channel,(number,index) in enumerate(zip(count,start)):
  number,index=int(number),int(index)
  if not number:jaw_load.append(0.);continue
  if index<0 or index+number>len(force):raise ValueError('contact buffer')
  sl=slice(index,index+number);cp=(point[sl]-o[FOB.name].pos)@rf
  if channel==1:cp=(cp-PIVOT)@lr+PIVOT
  centre=np.array((.023,-.0175,-.0024 if channel==0 else .0024))
  keep=np.max(abs(cp-centre)-np.array((.010,.0025,.001)),axis=1)<=.0002
  vv=normal[sl]*force[sl,None]
  # Calibrate normal direction against the native force-on-sensor matrix, not an assumed pair convention.
  if np.dot(vv.sum(0),fm[channel])<0:vv=-vv
  active=abs(force[sl])>.01
  if active.any():details.append(dict(jaw=int(channel),points=cp[active].tolist(),forces=force[sl][active].tolist(),distance=distance[sl][active].tolist(),vectors=(vv[active]@rf).tolist()))
  tau+=float((np.cross(point[sl]-o[COIN.name].pos,vv)@axis)[keep].sum())
  if channel==1:useful+=float((np.cross(point[sl]-pivot,-vv)@hinge)[keep].sum())
  jaw_load.append(float(abs(force[sl][keep]).sum()))
 opposing=abs(tau) if len(jaw_load)==2 and min(jaw_load)>.05 else 0.
 matrix=arr(S['handview'].get_contact_force_matrix(dt=dt)).reshape(3,6,3)
 hold=float(np.linalg.norm(matrix[0,:3],axis=1).sum());coin_grip=float(np.linalg.norm(matrix[2,3:],axis=1).sum())
 close=max(0.,-float(matrix[1,3:].sum(0)@rf[:,2]));close_moment=0.
 if S['unlatched'] and q<.2:
  hf,hp,hn,hdistance,hcount,hstart=packet(S['handview'])
  for channel in (9,10,11):
   number,index=int(hcount[channel]),int(hstart[channel])
   if not number:continue
   sl=slice(index,index+number);fw=hn[sl]*hf[sl,None]
   if np.dot(fw.sum(0),matrix[1,channel-6])<0:fw=-fw
   close_moment-=float((np.cross(hp[sl]-pivot,fw)@hinge).sum())
 S.update(contact_details=details,jaw_load=jaw_load,raw_coin_torque=tau,useful_lid_moment=useful,closing_moment=close_moment)
 return opposing,close,hold,coin_grip

@parts.on_physics_step
def latch_physics(sim):
 if S['zero'] or S['reclosed']:return
 try:
  from types import SimpleNamespace
  base=sim.env.unwrapped;asset=base.scene[FOB.name];origin=parts._np(base.scene.env_origins[0])
  pp=parts._np(asset.data.body_link_pose_w)[0,asset.body_names.index('base')]
  cc=parts._np(base.scene[COIN.name].data.root_link_pose_w)[0]
  q=float(parts._np(asset.data.joint_pos)[0,asset.joint_names.index('cover')]);rf=parts._quat_to_matrix(pp[3:])
  low=pp[:3]-origin-abs(rf)@np.array((.035,.020,.0081))
  o={FOB.name:SimpleNamespace(pos=pp[:3]-origin,quat_xyzw=pp[3:],low=low,joints={'cover':q}),
     COIN.name:SimpleNamespace(pos=cc[:3]-origin,quat_xyzw=cc[3:])}
  torque,close,hold,coin_grip=read_contacts(sim,o)
  S.update(current_torque=torque,current_close=close,base_hold=hold,right_coin_grip=coin_grip)
  S['peak_torque']=max(S['peak_torque'],torque);S['peak_close']=max(S['peak_close'],close)
  angle=math.atan2(-(rf.T@rot(o[COIN.name])[:,1])[0],(rf.T@rot(o[COIN.name])[:,1])[2])
  paired=bool(min(S['jaw_load'])>.05 and hold>=1. and coin_grip>=1. and low[2]>TOP+.050)
  oldangle=S.get('phys_angle');previous=S.get('paired_before',False)
  resisting_now=False
  if paired and previous and oldangle is not None:
   da=(angle-oldangle+math.pi)%(2*math.pi)-math.pi
   if abs(da)<.10:
    # Net angular progress and signed contact work: tiny reversals return work instead of erasing an entire turn.
    oldtau=S.get('previous_torque_signed',S['raw_coin_torque']);tau=S['raw_coin_torque']
    S['loaded_turn']+=da
    S['loaded_work']=max(0.,S['loaded_work']-.5*(oldtau+tau)*da)
    resisting_now=bool(-tau*da>=-1e-8)
   else:S['loaded_turn']=0.;S['loaded_work']=0.
  elif not paired:S['loaded_turn']=0.;S['loaded_work']=0.
  S.update(phys_angle=angle,paired_before=paired,previous_resistance=torque,previous_torque_signed=S['raw_coin_torque'],coin_angle=angle)
  energy=OPEN_EFFORT*.004+.5*LOCK_K*.004**2
  if not S['unlatched'] and paired and resisting_now and torque>=OPEN_TORQUE and q>=.004 and S['useful_lid_moment']>=OPEN_EFFORT+LOCK_K*q-.002 and abs(S['loaded_turn'])>=MIN_LOADED_TURN and S['loaded_work']>=energy:
   S.update(latched=False,unlatched=True,effort_mode=0);write_latch(sim,False)
   event(f'contact split torque={torque:.4f}Nm q={q:.4f} work={S["loaded_work"]:.5f}J')
  if S['unlatched'] and q>=.25:S.update(split=True,ever_open=True)
  if S['ever_open'] and not S['reclosed']:
   # A 1.2mm elastic snap lip: 12N at full compression over a60mm arm.
   # Implicit native drive damping prevents an explicit torque step from kicking the free cover.
   in_clip=q<CLOSE_TRAVEL
   if in_clip!=S.get('clip_drive'):
    import torch
    ids=[asset.joint_names.index('cover')];dev=asset.device
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[CLOSE_K if in_clip else 0.]],device=dev),joint_ids=ids)
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([[CLOSE_DAMP if in_clip else FREE_DAMP]],device=dev),joint_ids=ids)
    sim.set_target(FOB.name,'cover',CLOSE_TRAVEL if in_clip else 0.)
    asset.set_joint_effort_target_index(target=torch.tensor([[0.]],device=dev),joint_ids=ids);asset.write_data_to_sim()
    S['clip_drive']=in_clip
   S['effort_mode']=max(0.,CLOSE_K*(CLOSE_TRAVEL-q)) if in_clip else 0.
   if q<=CLOSED_Q and close>=CLOSE_FORCE and S['closing_moment']>=LATCH_EFFORT-.002:
    S.update(latched=True,reclosed=True);write_latch(sim,True);event(f'contact snap close={close:.3f}N moment={S["closing_moment"]:.4f}Nm')
  S['physics_samples']=S.get('physics_samples',0)+1
 except Exception as e:fail('native latch '+type(e).__name__+': '+str(e))

@parts.on_step
def step(sim):
 try:
  o=sim.states();dt=float(sim.t)-S['t'];S['t']=float(sim.t)
  torque,close,hold,coin_grip=read_contacts(sim,o);S.update(current_torque=torque,current_close=close,base_hold=hold,right_coin_grip=coin_grip)
  S['peak_torque']=max(S['peak_torque'],torque);S['peak_close']=max(S['peak_close'],close)
  q=float(o[FOB.name].joints['cover']);old_local=local(o,OLD.name)
  if S['split'] and np.linalg.norm(old_local-SEAT)>.030:S['old_out']=True
  S['prevq']=q
  for n in (FOB.name,COIN.name,OLD.name,NEW.name):
   st=o[n]
   if not np.isfinite(np.r_[st.pos,st.quat_xyzw,st.lin_vel,st.ang_vel]).all():fail('unknown pose')
   if not (.10<=st.low[0] and st.high[0]<=.54 and st.low[1]>=-.25 and st.high[1]<=.25 and st.low[2]>=TOP-.003):fail('part left mat')
  clouds,peak=hands.hand_state(sim);S['peak_finger']=max(S.get('peak_finger',0.),peak);S['clear']=clear_hands(clouds,o)
  still=all(o[n].resting(.003,.05) for n in (FOB.name,COIN.name,OLD.name,NEW.name))
  hist={n:np.asarray(o[n].pos).copy() for n in (FOB.name,COIN.name,OLD.name,NEW.name)}
  for n in (FOB.name,COIN.name,OLD.name,NEW.name):hist[n+'_rot']=rot(o[n]).copy()
  hist['cover']=np.asarray(o[FOB.name].links['cover'][0]).copy();hist['cover_q']=q
  S['history'].append((float(sim.t),hist))
  S['history']=[v for v in S['history'] if float(sim.t)-v[0]<=1.10]
  for _,old in S['history']:
   if any(np.trace(hist[n+'_rot'].T@old[n+'_rot'])<1+2*math.cos(math.radians(1.)) for n in (FOB.name,COIN.name,OLD.name,NEW.name)):still=False
   if any(np.linalg.norm(hist[n]-old[n])>.001 for n in (FOB.name,COIN.name,OLD.name,NEW.name,'cover')) or abs(q-old['cover_q'])>math.radians(1.):still=False
  base=sim.env.unwrapped;asset=base.scene[FOB.name]
  cover_speed=float(parts._np(asset.data.joint_vel)[0,asset.joint_names.index('cover')])
  still=still and abs(cover_speed)<=.02
  eligible=S['clear'] and still
  S['hold']=S['hold']+dt if eligible and S.get('eligible',False) and 0<dt<=.10001 else 0.
  S['eligible']=eligible;S['known']=True;S['cert']=signature(o)
  if not S.get('receipt',False) and score(o)==1.:
   S['receipt']=True;event(f'FULL lot={lots.current().name} t={sim.t:.2f}s peak_coin={S["peak_torque"]:.5f}Nm peak_close={S["peak_close"]:.2f}N sampled_peak_finger={S.get("peak_finger",0.):.2f}N')
  if os.environ.get('TRACE_OUT'):
   row={k:v for k,v in S.items() if k not in ('last','cert','views','handview','history')};row.update(new_seated=seated(o),old_in_tray=old_in_tray(o),new_local=local(o,NEW.name).tolist(),q=q,score=score(o),lot=lots.current().name,parts={n:dict(pos=o[n].pos.tolist(),quat=o[n].quat_xyzw.tolist(),low=o[n].low.tolist(),high=o[n].high.tolist()) for n in (FOB.name,COIN.name,OLD.name,NEW.name)})
   from yam_arena.constants import TCP_OFFSET
   row['hands']={};row['hand_quaternions']={}
   for arm in ('left_robot','right_robot'):
    rob=sim.env.unwrapped.scene[arm];p=parts._np(rob.data.body_link_pose_w)[0,rob.body_names.index('gripper')]
    row['hand_quaternions'][arm]=p[3:].tolist()
    row['hands'][arm]=(p[:3]-parts._np(sim.env.unwrapped.scene.env_origins[0])+parts._quat_to_matrix(p[3:])@np.array(TCP_OFFSET)).tolist()
   with pathlib.Path(os.environ['TRACE_OUT']).with_name('fob.jsonl').open('a') as f:f.write(json.dumps(row,default=lambda v:v.item() if isinstance(v,np.generic) else v.tolist())+'\n')
 except Exception as e:
  S['known']=False;fail(type(e).__name__+': '+str(e))

# Synchronized open-loop waypoints use the published visible layout only.
def demo():
 plan={'left':[],'right':[]};clock={'left':0.,'right':0.}
 def add(arm,t,p,f,label,yaw=0.,lean=0.,heading=0.,matrix=None):
  w=tilt.Waypoint(t,tuple(map(float,p)),f,label,yaw,lean,heading)
  if matrix is not None:w.yaw=tilt._TiltedYaw(float(yaw),matrix)
  plan[arm].append(w);clock[arm]+=t
 def wait(arm,target):
  if clock[arm]<target:
   w=plan[arm][-1];plan[arm].append(Waypoint(target-clock[arm],w.pos,w.finger,'wait',w.yaw));clock[arm]=target
 def both():
  t=max(clock.values());wait('left',t);wait('right',t)
 start=np.array(FOB.pos);pick=start+GRIP
 cp=np.array(lots.current().pos(COIN.name));npick=np.array(lots.current().pos(NEW.name))
 held=np.array((X,Y,TOP+.085));r=parts._rot((1,0,0),-math.pi/3)
 # Fixed calibration for the two published, camera-visible pickup positions.
 normal_bias=-.0005 if cp[0]>.220 else .0001
 lp=held+r@GRIP;co0=np.array((NOTCH[0]+.003,-.027,0.))
 tcp=held+r@(co0+np.array((0,-.007,0)))
 fg=-.030;coinfg=-.008;cyaw=math.pi/2;lyaw=-math.pi/2
 add('left',1.5,pick+(0,0,.11),FINGER_OPEN,'approach shell',yaw=lyaw)
 add('left',1.8,pick,FINGER_OPEN,'base end grip',yaw=lyaw)
 add('left',1.2,pick,fg,'pinch base ends',yaw=lyaw)
 add('left',2.,pick+(0,0,.05),fg,'lift off pickup rest',yaw=lyaw)
 add('left',2.5,lp,fg,'hold shell at sixty degrees',yaw=lyaw,lean=60.,heading=-math.pi/2)
 add('right',1.5,cp+(0,0,.095),FINGER_OPEN,'approach quarter rim',yaw=cyaw)
 add('right',1.8,cp+(0,0,.007),FINGER_OPEN,'diameter rim grip',yaw=cyaw)
 add('right',1.2,cp+(0,0,.007),coinfg,'pinch quarter rim',yaw=cyaw)
 add('right',2.,cp+(0,0,.09),coinfg,'lift quarter',yaw=cyaw)
 both()
 add('right',2.5,tcp+(0,-.025,.044),coinfg,'line coin edge at notch',yaw=cyaw,lean=33,heading=math.pi/2)
 add('right',2.,tcp+r@np.array((0,.0025,normal_bias)),coinfg,'insert edge into actual seam',yaw=cyaw,lean=33,heading=math.pi/2)
 # Roll about the lower notch-jaw contact, not the coin centre: the working hand rises as the coin turns.
 for turn in (10.,20.,30.,40.):
  aa=math.radians(turn);pivot_shift=(parts._rot((0,-1,0),aa)-np.eye(3))@np.array((.008,0,COIN_T/2))
  pt=tcp+r@(pivot_shift+np.array((0,.0025+.006*min(turn/10.,1.),normal_bias)))
  add('right',.8,pt,coinfg,'roll coin on lower notch jaw',yaw=cyaw+aa,lean=33,heading=math.pi/2)
 both()
 # Continue levering with the same coin; the hand holds the case and turns it slightly to give the working wrist room.
 for q,turn in ((.35,30),(.80,55)):
  alpha=-math.radians(20)*q/1.65;rr=parts._rot((0,0,1),alpha)@r;lidr=parts._rot((0,-1,0),q)
  co=PIVOT+lidr@(co0-PIVOT)+lidr@np.array((0,0,.0007))
  tcp1=held+rr@(co+np.array((0,-.007,0)))
  add('left',1.2,held+rr@GRIP,fg,'hold reaction during cover lift',yaw=lyaw+alpha,lean=60,heading=alpha-math.pi/2)
  add('right',1.2,tcp1,coinfg,'lever cover past upright',yaw=cyaw+math.radians(turn)+alpha,lean=33,heading=math.pi/2+alpha)
 add('right',1.,tcp1+np.array((.020,-.015,.015)),coinfg,'clear coin from open cover',yaw=cyaw+math.radians(55)-math.radians(20)*.8/1.65,lean=33,heading=math.pi/2-math.radians(20))
 add('right',2.5,cp+(0,0,.09),coinfg,'return quarter upright',yaw=cyaw)
 add('right',1.8,cp+(0,0,.007),coinfg,'coin over rest',yaw=cyaw)
 add('right',1.,cp+(0,0,.007),FINGER_OPEN,'release coin',yaw=cyaw)
 add('right',1.5,cp+(0,0,.09),FINGER_OPEN,'clear coin',yaw=cyaw)
 both()
 add('left',2.,lp,fg,'let spent cell slide into tray',yaw=lyaw,lean=60,heading=-math.pi/2)
 add('left',3.,start+GRIP+(0,0,.045),fg,'level shell above table supports',yaw=lyaw)
 add('left',4.,start+GRIP+(0,0,-.0005),fg,'set base on table pickup rests',yaw=lyaw)
 add('left',2.,start+GRIP+(0,0,-.0005),fg,'settle supported case before release',yaw=lyaw)
 add('left',1.5,start+GRIP+(0,0,-.0005),FINGER_OPEN,'release supported base',yaw=lyaw)
 add('left',2.,start+GRIP+(0,0,.10),FINGER_OPEN,'withdraw straight above shell',yaw=lyaw)
 add('left',1.5,(.20,.22,TOP+.14),FINGER_OPEN,'withdraw holding hand',yaw=lyaw)
 both()
 # Regrip the supported base by its sides before opening the light, released cover.
 support=start+np.array((0.,.024,-.004));sidefg=-.030
 def steady():
  add('left',1.5,support+(0,0,.065),FINGER_OPEN,'above supported base sides',yaw=lyaw,lean=60.,heading=-math.pi/2)
  add('left',1.5,support,FINGER_OPEN,'base sides below seam',yaw=lyaw,lean=60.,heading=-math.pi/2)
  add('left',1.,support,sidefg,'steady base on table',yaw=lyaw,lean=60.,heading=-math.pi/2)
 def unsteady(level=False):
  release=support.copy();rr=tilt.rotation(lyaw,60.,-math.pi/2)
  if level:
   # Fixed correction from visible, repeated case settling in the published layout; no live state.
   qc=np.array((.0192,.020,.0098,.9996));qc/=np.linalg.norm(qc)
   rr=parts._quat_to_matrix(qc).T@rr;release+=np.array((0,0,-.003))
   add('left',1.5,release,sidefg,'level held base onto broad rests',yaw=lyaw,matrix=rr)
   add('left',1.,release,sidefg,'settle case before jaw release',yaw=lyaw,matrix=rr)
  add('left',1.,release,FINGER_OPEN,'fully release supported base',yaw=lyaw,matrix=rr)
  add('left',1.5,release+(0,0,.09),FINGER_OPEN,'withdraw above base',yaw=lyaw,matrix=rr)
  add('left',1.,(.20,.22,TOP+.14),FINGER_OPEN,'clear working space',yaw=lyaw)
 steady();both()
 # Finish lifting the released cover only after the end grip is clear of its hinge sweep.
 # Gravity brings the free cover to the physical closing ramp near0.02rad during setdown.
 cover_grip=np.array((.020,0.,.016))
 lidp=start+PIVOT+parts._rot((0,-1,0),CLOSE_TRAVEL)@(cover_grip-PIVOT)
 add('right',2.,lidp+(0,0,.040),FINGER_OPEN,'approach released cover rim',lean=math.degrees(CLOSE_TRAVEL),heading=0.)
 add('right',1.5,lidp,FINGER_OPEN,'rim above physical closing ramp',lean=math.degrees(CLOSE_TRAVEL),heading=0.)
 add('right',1.2,lidp,-.0155,'grip cover sides',lean=math.degrees(CLOSE_TRAVEL),heading=0.)
 for q in (.35,.80,1.25,1.73):
  lidp=start+PIVOT+parts._rot((0,-1,0),q)@(cover_grip-PIVOT)
  add('right',1.2,lidp,-.0155,'lift free cover past upright',lean=min(85.,math.degrees(q)),heading=0.)
 add('right',1.,lidp,-.0155,'settle against actual open stop',lean=85.,heading=0.)
 add('right',1.,lidp,FINGER_OPEN,'release cover at open stop',lean=85.,heading=0.)
 add('right',1.5,lidp+(0,0,.040),FINGER_OPEN,'clear vertical cover',lean=85.,heading=0.)
 both();unsteady(level=True);both()
 # Cell placement and case closure follow while the supporting hand is clear.
 add('right',2.,npick+(0,0,.08),FINGER_OPEN,'approach presented cell')
 add('right',1.5,npick+(0,0,.010),FINGER_OPEN,'rim pickup over pedestal')
 add('right',1.,npick+(0,0,.010),-.009,'pinch twenty millimetre rim')
 add('right',1.5,npick+(0,0,.07),-.009,'lift new cell')
 seat=start+SEAT
 add('right',2.,seat+(0,0,.06),-.009,'plus face remains up')
 add('right',2.,seat+(0,0,.012),-.009,'lower cell to seat')
 add('right',1.,seat+(0,0,.012),FINGER_OPEN,'release in seat')
 add('right',1.5,seat+(0,0,.06),FINGER_OPEN,'clear seat')
 add('right',1.5,(.40,-.12,TOP+.13),FINGER_OPEN,'clear base regrip')
 both();steady();both()
 lid_open=start+PIVOT+parts._rot((0,-1,0),1.65)@(cover_grip-PIVOT)
 add('right',2.,lid_open+(0,0,.035),-.025,'approach open cover',lean=85.,heading=0.)
 add('right',1.5,lid_open,-.025,'cover rim',lean=85.,heading=0.)
 add('right',1.,lid_open,-.0155,'hold cover rim',lean=85.,heading=0.)
 for q in (1.25,.80,.35,.15):
  lidp=start+PIVOT+parts._rot((0,-1,0),q)@(cover_grip-PIVOT)
  add('right',1.2,lidp,-.0155,'fold cover down',lean=min(85.,math.degrees(q)),heading=0.)
 add('right',1.,lidp,-.025,'release cover above snap ramp',lean=math.degrees(.15),heading=0.)
 add('right',1.5,start+np.array((.010,0,.060)),-.025,'above cover for palm press')
 add('right',.6,start+np.array((.010,0,.060)),0.,'close fingers for press')
 add('right',2.,start+np.array((.010,0,.010)),0.,'press contact latch')
 add('right',1.,start+np.array((.010,0,.010)),0.,'hold snap pressure')
 add('right',2.,start+np.array((.010,0,-.015)),0.,'increase real compression through clip')
 add('right',1.5,start+np.array((.010,0,-.015)),0.,'hold final compression')
 add('right',1.5,start+np.array((.010,0,.060)),0.,'release pressure')
 add('right',1.5,(.22,-.22,TOP+.15),FINGER_OPEN,'hands clear')
 both();unsteady();both();add('left',3.,(.20,.22,TOP+.14),FINGER_OPEN,'released final hold');add('right',3.,(.22,-.22,TOP+.15),FINGER_OPEN,'one second still')
 return plan
