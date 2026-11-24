"""A Toyota HYQ14FBA smart-key shell in two detachable snap-fit halves on a narrow stand, a Wera 2035 0.6 x 3.5 mm flat
screwdriver on a cradle, two CR2032 cells (one in the shell's pocket, one on a pedestal), a cover rest and a spent-cell tray.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from mechanism import hands
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.05

# ---- Shell and tool figures. Each is one named value so a measured reading can replace it without a redesign. ----
KEY_MODEL='HYQ14FBA'      # FCC label read on one physical key, Nov 23 2026
CASE_L,CASE_W,CASE_T=.071,.040,.015   # length measured with calipers on one key; width/thickness estimates
KEY_MASS=.0304            # kg, reported key reading; interpret as assembled with cell for mass accounting
CELL_MASS=.003            # kg, Energizer CR2032 datasheet
CASE_MASS=KEY_MASS-CELL_MASS  # shell + inert board; cell inclusion and base/cover allocation require enrollment
COVER_DETACHES=True       # measured specimen separates fully, no hinge
SNAP_COUNT=2              # counted on the measured specimen
SNAP_END_OFFSET=.0052     # m, clip centres measured from each end
SNAP_X=(-(CASE_L/2-SNAP_END_OFFSET),CASE_L/2-SNAP_END_OFFSET)
SNAP_FORCE=10.            # N each snap holds before it slips past its lip (estimate, unmeasured)
SNAP_TRAVEL=.0012         # m lip engagement before a snap lets go (estimate, unmeasured)
CLOSE_FORCE=12.           # N retained conservative gate, above measured centre-press range 7.0-9.1 N; not a reading
CLOSE_READINGS_G=(711,864,923,843,799)  # kitchen scale, 5 centre thumb presses on one key; mean 828 g / 8.1 N
OPEN_CLEAR_ABOVE_SUPPORT=.005 # m, first split must be clear of the tallest passive support
CLOSED_GAP=.0005          # m largest allowed seam gap at any corner for a closed case
NOTCH_W=.007              # m, measured along seam with calipers
NOTCH_GAP=.0012           # m gap between halves inside the slot (estimate)
NOTCH_MOUTH=.0024         # m gap at the slot mouth (lead-in chamfer, estimate)
NOTCH_DEPTH=.0026         # m, toothpick probe and ruler reading
TOOL='Wera 2035 Kraftform Micro 05118012001'
TOOL_BLADE_T,TOOL_BLADE_W=.0006,.0035  # m, maker figures (tip thickness x width)
TOOL_LEN,TOOL_HANDLE_LEN,TOOL_SHAFT_D=.177,.097,.004  # m, overall (distributor page), handle (maker), blade shank diameter (maker)
TOOL_FLAT_LEN=.010       # m, estimated flattened tip length; verify transition at enrollment
TOOL_HANDLE_FLATS=.013   # m maximum handle width: Wera datasheet 177x13x13 mm; hex profile estimated
TOOL_MASS=.020            # kg: Wera datasheet
CELL_RETAINED='pocket'    # cell loose in a 1.6 mm deep ring on the board: stays in when tilted, falls out upside down (estimate)
CELL_RING_ID,CELL_RING_H=.021,.0016   # m (estimate)
GRIP_LIMIT=50.            # N: i2rt YAM gripper driver cap (yam_arena constants, linear_4310.yml)
OPEN_LEVER=(SNAP_X[1]-SNAP_X[0])/(SNAP_X[1]+CASE_L/2)  # blade force at the notch that loads the notch snap to SNAP_FORCE

BLACK=(.12,.13,.15); TRIM=(.62,.65,.68); STEEL=(.76,.78,.8); GREEN=(.18,.65,.24); RED=(.8,.22,.15); BOARD=(.14,.40,.24)
CX,CY=.340,.000; SEAM_Z=TOP+.073          # case centre at its seam, resting on the stand
QCASE=(0.,0.,math.sin(math.pi/4),math.cos(math.pi/4))   # case long axis along world +y; notch end faces the right arm
RCASE=parts._quat_to_matrix(QCASE)
SEAT=np.array((.012,0.,-.0023))           # seated cell centre, case frame
CELL_R,CELL_T=.010,.0032
TRAY_C=(.400,.100); TRAY_IN=(.094,.074); TRAY_WALL=.015
NEW_POS=(.440,-.130,TOP+.0356); DRIVER_POS=(.290,-.125,TOP+.050); COVER_REST=(.400,-.060)

def b(size,at=(0,0,0),c=BLACK,collide=True):return parts.box(size,at,c,collide=collide)
def hull_box(xs,ys,z_in,z_out,c=BLACK):
 """Wedge: a box in x,y whose top (or bottom) height changes from z_in at x=xs[1] to z_out at x=xs[0]."""
 (x0,x1),(y0,y1)=xs,ys;zb=min(z_in,z_out)-.0049 if z_in<0 else max(z_in,z_out)+.0049
 pts=[(x0,y0,z_out),(x1,y0,z_in),(x1,y1,z_in),(x0,y1,z_out),(x0,y0,zb),(x1,y0,zb),(x1,y1,zb),(x0,y1,zb)]
 return parts.hull(pts,((0,1,2,3),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)),c)
L2,W2=CASE_L/2,CASE_W/2;H=CASE_T/2;G=NOTCH_W/2
OPEN_LOW_MIN=max(TOP+.050,SEAM_Z-H+OPEN_CLEAR_ABOVE_SUPPORT)
# Base half (cell side), z from -7.5 mm to the seam. Walls stop 0.1 mm short of the seam.
base_shapes=[b((CASE_L,CASE_W,.002),(0,0,-H+.001)),
 *[b((CASE_L,.002,.0054),(0,s*(W2-.001),-.0028)) for s in (-1,1)],
 b((.002,CASE_W-.004,.0054),(L2-.001,0,-.0028)),
 *[b((.002,W2-.002-G,.0054),(-L2+.001,s*(G+(W2-.002-G)/2),-.0028)) for s in (-1,1)],
 hull_box((-L2,-L2+NOTCH_DEPTH),(-G,G),-NOTCH_GAP/2,-NOTCH_MOUTH/2),   # lower jaw with lead-in
 b((.046,.026,.0016),(SEAT[0],0,-H+.002+.0008),BOARD)]
# Locating lip inside the cover walls (0.1 mm clearance, 0.8 mm chamfer), kept clear of the notch slot and of the
# finger path used to place the cell.
def lip(x0,x1,y0,y1,side):
 prof=((.0021,-.0001),(.0035,-.0001),(.0035,.0015),(.0029,.0015))
 if side in ('+y','-y'):
  s=1 if side=='+y' else -1;pts=[(x,s*(W2-d),z) for x in (x0,x1) for d,z in prof]
 else:
  s=1 if side=='+x' else -1;pts=[(s*(L2-d),y,z) for y in (y0,y1) for d,z in prof]
 return parts.hull(pts,((0,1,2,3),(4,5,6,7),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)),TRIM)
base_shapes += [lip(-.030,-.010,0,0,'+y'),lip(-.030,-.010,0,0,'-y'),lip(0,0,-.005,.005,'+x'),
 lip(0,0,-.012,-.004,'-x'),lip(0,0,.004,.012,'-x')]
# Cell pocket: a ring of 16 segments on the board, inner diameter CELL_RING_ID, height CELL_RING_H.
BOARD_TOP=-H+.002+.0016
for a in np.arange(16)*2*math.pi/16:
 rr=CELL_RING_ID/2+.0007
 base_shapes.append(parts.box((.0014,.0044,CELL_RING_H),(SEAT[0]+rr*math.cos(a),rr*math.sin(a),BOARD_TOP+CELL_RING_H/2),TRIM,quat=tuple(parts._matrix_to_quat(parts._rot((0,0,1),a)))))
assert abs(SEAT[2]-(BOARD_TOP+CELL_T/2))<1e-9
# Cover half (button side), seam to +7.5 mm, with the upper jaw of the notch.
cover_shapes=[b((CASE_L,CASE_W,.002),(0,0,H-.001)),
 *[b((CASE_L,.002,.0054),(0,s*(W2-.001),.0028)) for s in (-1,1)],
 b((.002,CASE_W-.004,.0054),(L2-.001,0,.0028)),
 *[b((.002,W2-.002-G,.0054),(-L2+.001,s*(G+(W2-.002-G)/2),.0028)) for s in (-1,1)],
 hull_box((-L2,-L2+NOTCH_DEPTH),(-G,G),NOTCH_GAP/2,NOTCH_MOUTH/2)]
cover_shapes += [b((.010,.016,.0006),(xx,0,H+.0003),(.32,.33,.35),collide=False) for xx in (-.022,-.007,.008,.023)]
BASE=parts.Part('key_base',pos=(CX,CY,SEAM_Z),quat=QCASE,mass=CASE_MASS-.012,shapes=tuple(base_shapes),friction=(.7,.55),contact_offset=.00012,report=('base',))
COVER=parts.Part('key_cover',pos=(CX,CY,SEAM_Z),quat=QCASE,mass=.012,shapes=tuple(cover_shapes),friction=(.7,.55),contact_offset=.00012,report=('base',))
# Screwdriver, frame at the grasp point on the handle axis, +x toward the tip; blade wide face horizontal.
HEX=(0.,0.,0.,1.)
DR=TOOL_HANDLE_FLATS/2
driver_shapes=(parts.prism(DR,TOOL_HANDLE_LEN,6,(-.045+TOOL_HANDLE_LEN/2,0,0),(.85,.75,.15),quat=HEX),
 parts.prism(TOOL_SHAFT_D/2,TOOL_LEN-TOOL_HANDLE_LEN-TOOL_FLAT_LEN,8,(-.045+TOOL_HANDLE_LEN+(TOOL_LEN-TOOL_HANDLE_LEN-TOOL_FLAT_LEN)/2,0,0),STEEL),
 b((TOOL_FLAT_LEN,TOOL_BLADE_W,TOOL_BLADE_T),(-.045+TOOL_LEN-TOOL_FLAT_LEN/2,0,0),STEEL))
TIP=-.045+TOOL_LEN
DRIVER=parts.Part('screwdriver',pos=DRIVER_POS,quat=QCASE,mass=TOOL_MASS,shapes=driver_shapes,friction=(.8,.6),contact_offset=.0001,report=('base',))
def cell(name,pos,color):
 return parts.Part(name,pos=tuple(map(float,pos)),mass=CELL_MASS,shapes=(parts.cylinder(CELL_R,CELL_T,color=STEEL),),
  visuals=(parts.Visual('plus',((-.006,-.001,.00163),(.006,-.001,.00163),(.006,.001,.00163),(-.006,.001,.00163),
   (-.001,-.006,.00164),(.001,-.006,.00164),(.001,.006,.00164),(-.001,.006,.00164)),((0,1,2),(0,2,3),(4,5,6),(4,6,7)),color),),
  friction=(.3,.25),contact_offset=.00008,report=('base',))
OLD=cell('old_cell',np.array((CX,CY,SEAM_Z))+RCASE@SEAT,RED)
NEW=cell('new_cell',NEW_POS,GREEN)
MAT=parts.Part('desk_mat',pos=(.32,0,TOP+.0005),shapes=(b((.44,.50,.001),c=(.31,.38,.40)),),contact_offset=.0001)
# Fixed stand under the case's notch half; the held end overhangs it so the fingers reach the base sides.
STAND_Y0,STAND_Y1=CY-.033,CY+.006
STAND=parts.Part('fob_stand',pos=(CX,(STAND_Y0+STAND_Y1)/2,TOP+.001),
 shapes=(b((.030,STAND_Y1-STAND_Y0,SEAM_Z-H-TOP-.001),(0,0,(SEAM_Z-H-TOP-.001)/2),(.7,.67,.56)),),contact_offset=.0001)
PED_TOP=TOP+.030
COVER_STAND=parts.Part('cover_rest',pos=(COVER_REST[0],COVER_REST[1],TOP+.001),
 shapes=(b((.030,.064,.002),(0,0,.001),(.6,.6,.62)),b((.024,.056,PED_TOP-TOP-.001),(0,0,(PED_TOP-TOP-.001)/2),(.6,.6,.62))),contact_offset=.0001)
def tray_new(pos):
 c=(.25,.5,.28);sh=[b((.064,.064,.002),(0,0,.001),c)]
 sh += [b((.064,.002,.006),(0,s*.031,.005),c) for s in (-1,1)]+[b((.002,.060,.006),(s*.031,0,.005),c) for s in (-1,1)]
 sh += [b((.008,.008,.032),(0,0,.018),c)]
 return parts.Part('new_cell_tray',pos=pos,mass=.15,shapes=tuple(sh),contact_offset=.0001)
NEW_TRAY=tray_new((NEW_POS[0],NEW_POS[1],TOP))
def driver_rest(pos):
 # Three passive blocks: two under the 97 mm handle away from the pickup fingers, one under the shank.
 c=(.7,.62,.41);ax=DRIVER_POS[2]-pos[2];sh=[b((.030,.160,.002),(0,.005,.001),c)]
 for y,floor in ((DRIVER_POS[1]-.032,DR*math.sqrt(3)/2),(DRIVER_POS[1]+.032,DR*math.sqrt(3)/2),(DRIVER_POS[1]+.095,TOOL_SHAFT_D/2)):
  h=ax-floor-.002
  sh.append(b((.020,.008,h),(0,y-pos[1],.002+h/2),c))
 return parts.Part('driver_rest',pos=pos,mass=.15,shapes=tuple(sh),contact_offset=.0001)
REST_A=(DRIVER_POS[0],-.060,TOP+.001)
DRIVER_REST=driver_rest(REST_A)
tw=TRAY_IN[0]/2+.0015;th=TRAY_IN[1]/2+.0015
OLD_TRAY=parts.Part('old_cell_tray',pos=(TRAY_C[0],TRAY_C[1],TOP+.001),shapes=(b((TRAY_IN[0]+.006,TRAY_IN[1]+.006,.002),(0,0,.001),(.55,.25,.20)),
 *[b((TRAY_IN[0]+.006,.003,TRAY_WALL),(0,s*th,TRAY_WALL/2+.002),(.55,.25,.20)) for s in (-1,1)],
 *[b((.003,TRAY_IN[1],TRAY_WALL),(s*tw,0,TRAY_WALL/2+.002),(.55,.25,.20)) for s in (-1,1)]),contact_offset=.0001)
OBJECTS=[MAT,STAND,COVER_STAND,OLD_TRAY,NEW_TRAY,DRIVER_REST,BASE,COVER,DRIVER,OLD,NEW]
LOTS=lots.declare(OBJECTS,lots.Lot('A'),
 lots.Lot('B',place={'screwdriver':(.310,DRIVER_POS[1],DRIVER_POS[2]),'driver_rest':(.310,REST_A[1],REST_A[2]),
  'new_cell':(.460,-.130,NEW_POS[2]),'new_cell_tray':(.460,-.130,TOP)}))
DEMO_TOUCHES={'fob_stand','cover_rest','desk_mat'}
PARTS_SCORED=(BASE.name,COVER.name,DRIVER.name,OLD.name,NEW.name)

# ---- Records. History fails closed; only reset clears it. ----
def fresh():return dict(known=False,zero=False,why='',latched=[True,True],split=False,cover_off=False,old_out=False,
 hold=0.,t=0.,cert=None,clear=False,views=None,history=[],events=[],peak_blade=0.,peak_press=0.,peak_finger=0.,
 blade_cover=0.,blade_base=0.,left_base=0.,right_tool=0.,press=0.,press_x=0.,snap_d=[0.,0.],relatched=[False,False],split_info=None,relatched_info=[None,None],loaded_turn=0.,loaded_work=0.,raw_tool_torque=0.,current_torque=0.,peak_torque=0.,press_moment=0.)
S=fresh()
def event(text):S['events'].append(text);print('[fob]',text,flush=True)
def fail(text):
 if not S['zero']:event('ZERO '+str(text));S.update(zero=True,why=str(text))
def rot(s):return parts._quat_to_matrix(s.quat_xyzw)
def rel(o):
 """Cover pose in the base frame: (rotation, translation)."""
 rb,rc=rot(o[BASE.name]),rot(o[COVER.name])
 return rb.T@rc,rb.T@(np.asarray(o[COVER.name].pos)-o[BASE.name].pos)
def snap_offsets(o):
 r,t=rel(o);return [float((r@np.array((x,0,0))+t-np.array((x,0,0)))[2]) for x in SNAP_X]
def cover_gap(o):
 r,t=rel(o)
 return max(float(np.linalg.norm(r@np.array(c)+t-np.array(c))) for c in [(sx*L2,sy*W2,0.) for sx in (-1,1) for sy in (-1,1)])
def local(o,name):return (np.asarray(o[name].pos)-o[BASE.name].pos)@rot(o[BASE.name])
def seated(o):
 p=local(o,NEW.name);normal=rot(o[BASE.name])[:,2];up=rot(o[NEW.name])[:,2]
 return bool(np.linalg.norm(p[:2]-SEAT[:2])<=.002+1e-9 and abs(p[2]-SEAT[2])<=.001+1e-9 and normal@up>=math.cos(math.radians(12)))
def old_in_tray(o):
 p=np.asarray(o[OLD.name].pos)
 return bool(abs(p[0]-TRAY_C[0])<=TRAY_IN[0]/2 and abs(p[1]-TRAY_C[1])<=TRAY_IN[1]/2 and TOP<=p[2]<=TOP+.003+CELL_R+.001)
def signature(o):
 return tuple(np.concatenate([np.r_[o[n].pos,o[n].quat_xyzw,o[n].lin_vel,o[n].ang_vel] for n in PARTS_SCORED]))
def clear_hands(clouds,o):
 if not clouds or any(not len(p) or not np.isfinite(p).all() for p in clouds):return False
 for spec in (BASE,COVER,DRIVER,OLD,NEW):
  st=o[spec.name]
  for _,c,r,sz in spec.world_boxes({'base':(st.pos,rot(st))}):
   if any(np.linalg.norm(np.maximum(abs((p-c)@r)-sz/2,0),axis=1).min()<=.005 for p in clouds):return False
 return True

def score(o):
 try:
  if not S['known'] or S['zero'] or S['cert']!=signature(o) or S['hold']<1.-1e-8 or not S['clear']:return 0.
  if not (S['split'] and S['cover_off']):return 0.
  if not seated(o):return .25
  oldp=local(o,OLD.name)
  removed=np.linalg.norm(np.maximum(abs(oldp)-np.array((L2,W2,H)),0.))>=.011
  if not S['old_out'] or not removed:return .25
  if not all(S['latched']) or not all(S['relatched']) or cover_gap(o)>CLOSED_GAP+1e-9:return .5
  return 1. if old_in_tray(o) else .75
 except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(o):return score(o)==1.

# ---- Snap joints: two six-axis joints between the halves, written into the running stage. ----
# Each holds the cover against its lip with a force-capped drive (SNAP_FORCE, reached at SNAP_TRAVEL) and locates it
# sideways; the same joint force acts on both halves. A snap is let go only past its travel under robot contact, and
# re-engaged only by a press that gives that snap at least half of CLOSE_FORCE with the seam closed.
_SNAP_PATHS=[]
def _stage():
 from isaaclab.sim.utils import get_current_stage
 return get_current_stage()
def author_snaps(sim):
 from pxr import Gf,Sdf,UsdGeom,UsdPhysics
 if _SNAP_PATHS:return
 env=sim.env.unwrapped.scene.env_prim_paths[0];UsdGeom.Scope.Define(_stage(),f'{env}/key_snaps')
 for k,x in enumerate(SNAP_X):
  j=UsdPhysics.Joint.Define(_stage(),f'{env}/key_snaps/snap{k}')
  j.CreateJointEnabledAttr().Set(False)
  j.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS[BASE.name])]);j.CreateBody1Rel().SetTargets([Sdf.Path(parts._PATHS[COVER.name])])
  j.CreateLocalPos0Attr().Set(Gf.Vec3f(float(x),0.,0.));j.CreateLocalPos1Attr().Set(Gf.Vec3f(float(x),0.,0.))
  j.CreateLocalRot0Attr().Set(Gf.Quatf(1.));j.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
  j.CreateExcludeFromArticulationAttr().Set(True);j.CreateCollisionEnabledAttr().Set(True)
  for axis,k_,d_,cap in (('transX',20000.,5.,40.),('transY',20000.,5.,40.),('transZ',SNAP_FORCE/SNAP_TRAVEL,2.,SNAP_FORCE)):
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force');d.CreateStiffnessAttr(k_);d.CreateDampingAttr(d_)
   d.CreateMaxForceAttr(cap);d.CreateTargetPositionAttr(0.)
  _SNAP_PATHS.append(str(j.GetPath()))
def set_snap(k,on):
 from pxr import UsdPhysics
 UsdPhysics.Joint(_stage().GetPrimAtPath(_SNAP_PATHS[k])).GetJointEnabledAttr().Set(bool(on))

@parts.before_reset
def before_reset(sim):
 # Both snaps are engaged in the declared closed start, including the stock reset settle.
 # No physics step occurs between this reset hook and the environment restoring initial body poses.
 for k in range(len(_SNAP_PATHS)):set_snap(k,True)
@parts.on_reset
def reset(sim):
 global S
 S=fresh();hands.reset();author_snaps(sim)
 for k in range(SNAP_COUNT):set_snap(k,True)
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)

def arr(x):
 x=x.torch if hasattr(x,'torch') else x
 return np.asarray(x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy() if hasattr(x,'numpy') else x)

def read_contacts(sim,o):
 from isaaclab_physx.physics import PhysxManager
 from yam_arena.robot import asset_link_paths
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);dt=float(base.sim.get_physics_dt())
 P=parts._PATHS
 if S['views'] is None:
  physics=PhysxManager.get_physics_sim_view();links=asset_link_paths();env=base.scene.env_prim_paths[0]
  arm=[env+'/'+a+links[n] for a in ('LeftArm','RightArm') for n in ('tip_left','tip_right','gripper')]
  S['views']=(physics.create_rigid_contact_view([P[DRIVER.name]],filter_patterns=[[P[BASE.name],P[COVER.name]]],max_contact_data_count=256),
   physics.create_rigid_contact_view([P[BASE.name],P[COVER.name],P[DRIVER.name]],filter_patterns=[arm]*3,max_contact_data_count=512))
 tv,hv=S['views'];rb=rot(o[BASE.name]);n=rb[:,2]
 tm=arr(tv.get_contact_force_matrix(dt=dt)).reshape(2,3)
 hm=arr(hv.get_contact_force_matrix(dt=dt)).reshape(3,6,3)
 blade_base,blade_cover=float(np.linalg.norm(tm[0])),float(np.linalg.norm(tm[1]))
 tf,tp,tn,_,tc,ts=(arr(x) for x in tv.get_contact_data(dt=dt))
 tf=tf.reshape(-1);tp=tp.reshape(-1,3)-origin;tn=tn.reshape(-1,3);tc=tc.reshape(-1);ts=ts.reshape(-1)
 axis=rot(o[DRIVER.name])[:,0];torque=0.;jaw_load=[]
 for ch in (0,1):
  count,start=int(tc[ch]),int(ts[ch]);sl=slice(start,start+count)
  cp=(tp[sl]-o[(BASE if ch==0 else COVER).name].pos)@rot(o[(BASE if ch==0 else COVER).name])
  keep=(abs(cp[:,0]+L2-NOTCH_DEPTH/2)<=NOTCH_DEPTH/2+.0003)&(abs(cp[:,1])<=G+.0003)&(abs(cp[:,2])<=.004)
  blade_cp=(tp[sl]-o[DRIVER.name].pos)@rot(o[DRIVER.name])
  keep &= (blade_cp[:,0]>=TIP-TOOL_FLAT_LEN-.0003)&(blade_cp[:,0]<=TIP+.0003)&(abs(blade_cp[:,1])<=TOOL_BLADE_W/2+.0003)&(abs(blade_cp[:,2])<=TOOL_BLADE_T/2+.0003)
  vv=tn[sl]*tf[sl,None]
  if np.dot(vv.sum(0),tm[ch])<0:vv=-vv
  torque+=float((np.cross(tp[sl]-o[DRIVER.name].pos,vv)@axis)[keep].sum())
  jaw_load.append(float(abs(tf[sl][keep]).sum()))
 S.update(raw_tool_torque=torque,current_torque=abs(torque) if min(jaw_load)>.05 else 0.,jaw_load=jaw_load)
 S['peak_torque']=max(S['peak_torque'],S['current_torque'])
 left_base=float(np.linalg.norm(hm[0,:3],axis=1).sum());left_cover=float(np.linalg.norm(hm[1,:3],axis=1).sum())
 right_cover=float(np.linalg.norm(hm[1,3:],axis=1).sum());right_tool=float(np.linalg.norm(hm[2,3:],axis=1).sum())
 # Press: right-hand force on the cover along the base's inward normal, and where it acts along the case.
 press=max(0.,-float(hm[1,3:].sum(0)@n));press_x=0.
 if press>.5:
  f,p,nn,_,cnt,st=(arr(x) for x in hv.get_contact_data(dt=dt));f=f.reshape(-1);p=p.reshape(-1,3)-origin;cnt=cnt.reshape(-1);st=st.reshape(-1)
  w=0.;acc=0.
  for ch in (9,10,11):
   c,s=int(cnt[ch]),int(st[ch])
   if c:x=((p[s:s+c]-o[BASE.name].pos)@rb)[:,0];acc+=float((abs(f[s:s+c])*x).sum());w+=float(abs(f[s:s+c]).sum())
  press_x=acc/w if w>0 else 0.
 moment=press*min(abs(press_x-SNAP_X[0]),abs(press_x-SNAP_X[1]))
 S.update(press_moment=moment,blade_base=blade_base,blade_cover=blade_cover,left_base=left_base,left_cover=left_cover,right_cover=right_cover,right_tool=right_tool,press=press,press_x=press_x)
 S['peak_blade']=max(S['peak_blade'],blade_cover);S['peak_press']=max(S['peak_press'],press)

@parts.on_physics_step
def snap_physics(sim):
 if S['zero']:return
 try:
  from types import SimpleNamespace
  base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);o={}
  for n in (BASE.name,COVER.name,DRIVER.name):
   pp=parts._np(base.scene[n].data.root_link_pose_w)[0];o[n]=SimpleNamespace(pos=pp[:3]-origin,quat_xyzw=pp[3:])
  read_contacts(sim,o);d=snap_offsets(o);S['snap_d']=d
  rb=rot(o[BASE.name]);low=float(o[BASE.name].pos[2]-abs(rb[2])@np.array((L2,W2,H)))
  held=S['left_base']>=1.
  angle=math.atan2((rot(o[BASE.name]).T@rot(o[DRIVER.name])[:,1])[2],(rot(o[BASE.name]).T@rot(o[DRIVER.name])[:,1])[1])
  paired=min(S['jaw_load'])>.05 and S['right_tool']>=1. and held and low>OPEN_LOW_MIN
  da=(angle-S.get('last_angle',angle)+math.pi)%(2*math.pi)-math.pi
  if paired and S.get('paired_before',False) and abs(da)<.1:
   S['loaded_turn']+=da
   S['loaded_work']=max(0.,S['loaded_work']-.5*(S['raw_tool_torque']+S.get('last_torque',S['raw_tool_torque']))*da)
  elif not paired:S['loaded_turn']=0.;S['loaded_work']=0.
  S.update(last_angle=angle,last_torque=S['raw_tool_torque'],paired_before=paired)

  # Notch snap: let go only when the blade, held in the right hand, loads both halves and the snap is past its travel.
  if S['latched'][0] and d[0]>=SNAP_TRAVEL and S['right_tool']>=1. and held and low>OPEN_LOW_MIN \
     and min(S['jaw_load'])>=SNAP_FORCE*OPEN_LEVER:
   S['latched'][0]=False;set_snap(0,False);S['split']=True
   S['split_info']=dict(t=float(sim.t),torque=S['current_torque'],turn=S['loaded_turn'],work=S['loaded_work'],jaw_load=list(S['jaw_load']),blade_cover=S['blade_cover'],blade_base=S['blade_base'],d=d[0],left=S['left_base'])
   event(f'notch snap released by blade: cover {S["blade_cover"]:.2f}N base {S["blade_base"]:.2f}N travel {d[0]*1000:.2f}mm left {S["left_base"]:.1f}N')
  # Far snap: let go past its travel while a hand pulls the cover and the other holds the base.
  if S['latched'][1] and not S['latched'][0] and d[1]>=SNAP_TRAVEL and S['right_cover']>=1. and held:
   S['latched'][1]=False;set_snap(1,False);event(f'far snap released by hand pull: travel {d[1]*1000:.2f}mm right {S["right_cover"]:.1f}N')
  # Re-engage: each snap needs its share of the press (lever rule about the other snap) with the seam closed there.
  if S['cover_off']:
   r,t=rel(o)
   for k in range(SNAP_COUNT):
    if S['latched'][k]:continue
    other=SNAP_X[1-k];share=S['press']*abs(S['press_x']-other)/abs(SNAP_X[1]-SNAP_X[0]) if min(SNAP_X)<=S['press_x']<=max(SNAP_X) else 0.
    off=r@np.array((SNAP_X[k],0,0))+t-np.array((SNAP_X[k],0,0))
    moment=S['press']*abs(S['press_x']-other)
    if S['press']>=CLOSE_FORCE and share>=CLOSE_FORCE/2 and abs(off[2])<=CLOSED_GAP and np.hypot(off[0],off[1])<=CLOSED_GAP:
     S['relatched_info'][k]=dict(t=float(sim.t),press=S['press'],press_x=S['press_x'],share=share,offset_mm=(off*1000).tolist())
     S['latched'][k]=True;S['relatched'][k]=True;set_snap(k,True);event(f'snap {k} re-engaged: press {S["press"]:.2f}N at x={S["press_x"]*1000:.1f}mm share {share:.2f}N')
  S['physics_samples']=S.get('physics_samples',0)+1
 except Exception as e:fail('native snap '+type(e).__name__+': '+str(e))

@parts.on_step
def step(sim):
 try:
  o=sim.states();dt=float(sim.t)-S['t'];S['t']=float(sim.t)
  r,t=rel(o)
  if S['split'] and not any(S['latched']) and np.linalg.norm(t)>.020:S['cover_off']=True
  if S['split'] and np.linalg.norm(local(o,OLD.name)-SEAT)>.030:S['old_out']=True
  for n in PARTS_SCORED:
   st=o[n]
   if not np.isfinite(np.r_[st.pos,st.quat_xyzw,st.lin_vel,st.ang_vel]).all():fail('unknown pose')
   if not (.10<=st.low[0] and st.high[0]<=.54 and st.low[1]>=-.25 and st.high[1]<=.25 and st.low[2]>=TOP-.003):fail('part left mat: '+n)
  clouds,peak=hands.hand_state(sim);S['finger_load']=peak;S['peak_finger']=max(S['peak_finger'],peak);S['clear']=clear_hands(clouds,o)
  if peak>S.get('peak_finger_logged',0.)+5.:S['peak_finger_logged']=peak;event(f'finger load {peak:.1f}N at t={sim.t:.2f}s')
  still=all(o[n].resting(.003,.05) for n in PARTS_SCORED)
  hist={n:(np.asarray(o[n].pos).copy(),rot(o[n]).copy()) for n in PARTS_SCORED}
  S['history'].append((float(sim.t),hist));S['history']=[v for v in S['history'] if float(sim.t)-v[0]<=1.10]
  for _,old in S['history']:
   for n in PARTS_SCORED:
    if np.linalg.norm(hist[n][0]-old[n][0])>.001 or np.trace(hist[n][1].T@old[n][1])<1+2*math.cos(math.radians(1.)):still=False
  eligible=S['clear'] and still
  S['hold']=S['hold']+dt if eligible and S.get('eligible',False) and 0<dt<=.10001 else 0.
  S['eligible']=eligible;S['known']=True;S['cert']=signature(o)
  if not S.get('receipt',False) and score(o)==1.:
   S['receipt']=True;event(f'FULL lot={lots.current().name} t={sim.t:.2f}s peak_blade={S["peak_blade"]:.2f}N peak_press={S["peak_press"]:.2f}N peak_finger={S["peak_finger"]:.2f}N')
  if os.environ.get('TRACE_OUT'):
   row={k:v for k,v in S.items() if k not in ('cert','views','history','events')}
   row.update(new_seated=seated(o),old_in_tray=old_in_tray(o),new_local=local(o,NEW.name).tolist(),old_local=local(o,OLD.name).tolist(),
    cover_rel=t.tolist(),gap=cover_gap(o),score=score(o),lot=lots.current().name,
    parts={n:dict(pos=o[n].pos.tolist(),quat=o[n].quat_xyzw.tolist()) for n in PARTS_SCORED})
   from yam_arena.constants import TCP_OFFSET
   row['finger_contacts']=hands.fingertip_contact_forces(sim).tolist()
   row['hands']={};row['hand_poses']={}
   for arm in ('left_robot','right_robot'):
    rob=sim.env.unwrapped.scene[arm];p=parts._np(rob.data.body_link_pose_w)[0,rob.body_names.index('gripper')]
    row['hand_poses'][arm]=dict(gripper_pos=p[:3].tolist(),quat=p[3:].tolist(),finger_joints=parts._np(rob.data.joint_pos)[0,-2:].tolist())
    row['hands'][arm]=(p[:3]-parts._np(sim.env.unwrapped.scene.env_origins[0])+parts._quat_to_matrix(p[3:])@np.array(TCP_OFFSET)).tolist()
   with pathlib.Path(os.environ['TRACE_OUT']).with_name('fob.jsonl').open('a') as f:f.write(json.dumps(row,default=lambda v:v.item() if isinstance(v,np.generic) else v.tolist())+'\n')
 except Exception as e:
  S['known']=False;fail(type(e).__name__+': '+str(e))

# ---- Demo: synchronized open-loop waypoints from the published layout only (no live state). ----
CAL=dict(left_z=0.,right_z=0.,driver_grasp_roll=30.,tool_z=0.,cover_z=0.,drop_dx=.0015,drop_dy=0.,cover_dx=.0004,cover_dy=.00055,cover_yaw=.1)   # fixed corrections, set from traces of the published layout
PRY_DY,PRY_Z=.050,TOP+.140   # case lifted clear of the table and toward the right hand for the split
WORK_Z=SEAM_Z+.0005              # case returned onto the stand while left grip remains for cover/cell work
def W(p,dy=0.):return np.array((CX,CY+dy,0.))+RCASE@np.asarray(p,float)
def demo():
 plan={'left':[],'right':[]};clock={'left':0.,'right':0.}
 def add(arm,t,p,f,label,yaw=0.,lean=0.,heading=0.):
  plan[arm].append(tilt.Waypoint(t,tuple(map(float,p)),f,label,yaw,lean,heading));clock[arm]+=t
 def wait(arm,target):
  if clock[arm]<target:
   w=plan[arm][-1];plan[arm].append(tilt.Waypoint(target-clock[arm],w.pos,w.finger,'wait',float(w.yaw),w.tilt,w.heading));clock[arm]=target
 def both():
  t=max(clock.values());wait('left',t);wait('right',t)
 lot=lots.current();dp=np.array(lot.pos(DRIVER.name));npick=np.array(lot.pos(NEW.name))
 Y=math.pi/2;HL=-math.pi/2;HR=math.pi/2
 # Left: horizontal grasp of the base half only from the far end, pads below the seam.
 lg=np.array((.036,0,-.0115))
 def L(z_seam,dy=0.):return W(lg,dy)+np.array((0,0,z_seam+CAL['left_z']))
 def l(t,p,f,label,roll=0.):add('left',t,p,f,label,yaw=-Y+math.radians(roll),lean=90,heading=HL)
 fL=-(W2-.003)
 l(2.,L(SEAM_Z)+(0,0,.050),FINGER_OPEN,'above far end, pads level')
 l(2.,L(SEAM_Z),FINGER_OPEN,'pads beside base sides')
 l(1.2,L(SEAM_Z),fL,'pinch base half below seam')
 l(2.,L(SEAM_Z+.030),fL,'lift key off stand')
 l(2.5,L(PRY_Z,PRY_DY),fL,'hold key clear of table')
 # Right: screwdriver from its cradle, horizontal, blade flat.
 def r(t,p,f,label,twist=0.):
  roll=twist+CAL['driver_grasp_roll']  # pinch opposite hex flats; tool starts blade-flat, wrist is rolled30deg
  add('right',t,p+parts._rot((0,1,0),-math.radians(roll))@np.array((0,0,.003)),f,label,yaw=Y,lean=roll,heading=0.)
 def rd(t,p,f,label,yaw_bias=0.):add('right',t,p,f,label,yaw=Y+math.radians(yaw_bias))
 fT=-(TOOL_HANDLE_FLATS/2-.0055)
 r(2.,dp+(0,0,.040),FINGER_OPEN,'above handle, fingers level')
 r(2.,dp+(0,0,CAL['tool_z']),FINGER_OPEN,'pads beside handle flats')
 r(1.2,dp+(0,0,CAL['tool_z']),fT,'grip handle')
 r(1.5,dp+(0,0,.030),fT,'lift screwdriver')
 both()  # hold the driver outside the key footprint until the left hand has reached the free-air pry pose
 tip_in=W((-L2+.004,0,0),PRY_DY)+np.array((.0029,.0041,.0015)) + np.array((0,0,PRY_Z+CAL['right_z']))
 tcp_in=tip_in-np.array((0,TIP,0))
 r(1.5,np.array((dp[0],dp[1],PRY_Z+.035)),fT,'raise driver clear of held key')
 r(2.,tcp_in-(0,.030,0)+(0,0,.035),fT,'align above and behind notch')
 r(1.5,tcp_in-(0,.020,0),fT,'lower blade behind notch')
 r(.8,tcp_in-(0,.020,0),fT,'settle level blade behind notch')
 r(2.,tcp_in,fT,'blade into notch')
 for a in (10,20,30,40,50):
  correction=np.array((np.interp(a,[0,40,70,80],[0,0,-.009,-.009]),0,np.interp(a,[0,20,40,70,80],[0,.003,.006,.017,.019])))
  r(.8,tcp_in+correction,fT,'twist blade in notch',a)
 r(1.,tcp_in+(-.003,0,.0096666667),fT,'hold twist',50)
 r(1.5,tcp_in,fT,'untwist')
 r(1.5,tcp_in-(0,.025,0),fT,'withdraw blade')
 r(3.,dp+(0,0,.030),fT,'carry screwdriver back')
 r(1.5,dp+(0,0,CAL['tool_z']+.002),fT,'set into cradle')
 r(1.,dp+(0,0,CAL['tool_z']+.002),FINGER_OPEN,'release screwdriver')
 r(1.5,dp+(0,0,.050),FINGER_OPEN,'lift clear of handle')
 add('right',2.,(.40,-.12,TOP+.15),FINGER_OPEN,'right hand aside',yaw=Y)
 both()
 # Left brings the opened key over the stand; right lifts the cover off (the far snap lets go) and parks it.
 l(2.5,L(WORK_Z),fL,'hold opened key at work height')
 both()
 fC=-(W2-.004);tz=.0128+.001
 cov=lambda z:np.array((CX,CY,z+tz))
 rd(2.,cov(WORK_Z+CAL['cover_z'])+(0,0,.025),FINGER_OPEN,'above cover')
 rd(1.5,cov(WORK_Z+CAL['cover_z']),FINGER_OPEN,'pads on cover sides above seam')
 rd(1.2,cov(WORK_Z+CAL['cover_z']),fC,'grip cover')
 rd(2.,cov(WORK_Z+CAL['cover_z'])+(0,0,.030),fC,'pull cover off base, compensate arm compliance')
 park=lambda z:np.array((COVER_REST[0],COVER_REST[1],z+tz))
 cz=PED_TOP-H+.002
 cover_travel=WORK_Z+tz+.026
 rd(1.2,(CX,CY,cover_travel),fC,'bring free cover to level carry height')
 rd(3.,(COVER_REST[0],COVER_REST[1],cover_travel),fC,'carry cover level over rest')
 rd(1.5,park(cz+.001),fC,'lower cover over rest')
 rd(1.,park(cz+.001),FINGER_OPEN,'release cover')
 rd(1.5,park(cz)+(0,0,.050),FINGER_OPEN,'clear cover')
 # Left tips the spent cell out over the tray while the right parks the cover, then returns.
 wait('left',clock['right'])
 inv=np.array((TRAY_C[0]-.020,TRAY_C[1]+.011,TOP+.110))
 l(3.,inv,fL,'carry base over tray')
 l(2.5,inv,fL,'roll base upside down',120.)
 l(1.5,inv,fL,'spent cell falls into tray',120.)
 l(2.5,inv,fL,'roll base upright')
 l(3.,L(WORK_Z),fL,'base back at work height')
 # Right places the new cell into the pocket of the held base.
 cz2=.012;fN=-(CELL_R-.003)
 rd(2.,npick+(0,0,.035),FINGER_OPEN,'above new cell')
 rd(1.5,npick+(0,0,cz2),FINGER_OPEN,'pads at cell rim')
 rd(1.,npick+(0,0,cz2),fN,'pinch cell rim')
 rd(1.5,npick+(0,0,.035),fN,'lift new cell plus up')
 both()
 drop=W(SEAT)+np.array((CAL['drop_dx'],CAL['drop_dy'],WORK_Z+CAL['left_z']+CELL_RING_H+.0003))
 rd(1.5,(.38,-.06,drop[2]+.015+cz2),fN,'carry cell toward key')
 rd(1.5,drop+(0,0,.015+cz2),fN,'over pocket')
 rd(1.5,drop+(0,0,cz2),fN,'lower cell to ring')
 rd(1.,drop+(0,0,cz2),FINGER_OPEN,'release cell into pocket')
 rd(1.5,drop+(0,0,cz2+.020),FINGER_OPEN,'clear pocket')
 both()
 # Left sets the base on the stand and keeps holding it; right fetches the cover, seats it and presses it home.
 l(2.,L(SEAM_Z+.0005),fL,'set base on stand, still held')
 rd(2.,park(cz)+(0,0,.040),FINGER_OPEN,'above parked cover')
 rd(1.5,park(cz),FINGER_OPEN,'pads on parked cover')
 rd(1.2,park(cz),fC,'grip cover')
 rd(1.5,park(cz)+(0,0,.040),fC,'lift cover off rest')
 both()
 cover_bias=np.array((CAL['cover_dx'],CAL['cover_dy'],0))
 rd(3.,cov(SEAM_Z+.003)+(0,0,.030)+cover_bias,fC,'carry cover over base',CAL['cover_yaw'])
 rd(2.5,cov(SEAM_Z+.006)+cover_bias,fC,'cover above locating lip',CAL['cover_yaw'])
 rd(2.,cov(SEAM_Z+.006)+cover_bias,fC,'settle cover clear of locating lip',CAL['cover_yaw'])
 rd(3.,cov(SEAM_Z+.0002)+cover_bias,fC,'lower cover slowly onto lip',CAL['cover_yaw'])
 rd(.8,cov(SEAM_Z+.0002)+cover_bias,fC,'settle cover on lip',CAL['cover_yaw'])
 rd(1.,cov(SEAM_Z+.0002)+cover_bias,FINGER_OPEN,'release cover',CAL['cover_yaw'])
 # Measured real trial closes by a centre thumb press. Keep the original12 N/6 N-per-end gates.
 # The fixed downward targets compensate measured wrist compliance; actual contact forces/poses are audited.
 top=np.array((CX-.0006,CY+.0065,SEAM_Z+H+.0128))
 rd(1.2,top+(0,0,.010),FINGER_OPEN,'above centre of cover')
 rd(.6,top+(0,0,.010),0.,'close fingers for centre press')
 rd(1.,top+(0,0,.002),0.,'fingertips above cover centre')
 for depth in (.003,.008,.013,.018,.023,.028,.033,.038):
  rd(.7,top-(0,0,depth),0.,'centre press, compensate wrist compliance')
 rd(1.,top-(0,0,.038),0.,'hold centre press')
 rd(1.5,top+(0,0,.015),0.,'lift off cover')
 rd(1.5,(.40,-.15,TOP+.15),FINGER_OPEN,'right hand clear')
 both()
 l(1.,L(SEAM_Z+.0005),FINGER_OPEN,'open left pads')
 l(1.5,L(SEAM_Z+.0005)+(0,0,.060),FINGER_OPEN,'lift pads clear of key')
 l(2.,(.33,.12,TOP+.11),FINGER_OPEN,'left hand clear')
 both();l(3.,(.33,.12,TOP+.11),FINGER_OPEN,'released final hold');rd(3.,(.40,-.15,TOP+.15),FINGER_OPEN,'still')
 return plan
