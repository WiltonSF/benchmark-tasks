"""A black custom bezel stands on its lower edge in front of a partly populated
bench rack. Two tabs are at its right end. The opposite end has a sprung latch
and a yellow position flag. The target is the second unit; the ten units above
are empty. A complete lower server, target chassis, rails and top beam surround it."""
import pathlib,sys,math,os,json
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts
from mechanics import geometry as g,native_loads,release
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=.1
parts.CONTACT_OFFSET=.00001;parts.REST_OFFSET=0.
BLACK=(.12,.14,.17);STEEL=(.55,.58,.62);BLUE=(.12,.35,.55);YELLOW=(.95,.76,.08)
TARGET=np.array((.370,-.002,TOP+.066675));START=(.220,0,TOP+.022225)
# Every fixed envelope used by the native preflight is retained here.
RACK_BOXES=[]
def rb(name,c,size,color=STEEL):RACK_BOXES.append((name,np.array(c,float),np.array(size,float),color))
# Folded2mm channels, inside the conservatively checked36 x16mm post envelopes.
for sy in (-1,1):
    for x in (.370,.610):
        for dx in (-.017,.017):rb('upright flange',(x+dx,sy*.23255,TOP+.280),(.002,.016,.560),BLACK)
        
        if x==.370:
            for za,zb in ((0,.04435),(.089,.560)):rb('upright web',(x,sy*.23955,TOP+(za+zb)/2),(.032,.002,zb-za),BLACK)
        else:rb('upright web',(x,sy*.23955,TOP+.280),(.032,.002,.560),BLACK)
rb('top crossbar',(.37,0,TOP+.560),(.036,.4986,.020),BLACK)
# A real chassis-footprint opening keeps the lower floor at the reviewed table datum.
rb('base front',(.375,0,TOP+.005),(.070,.520,.010),BLACK)
rb('base rear',(.625,0,TOP+.005),(.030,.520,.010),BLACK)
for sy in (-1,1):rb('base side',(.510,sy*.2375,TOP+.005),(.200,.045,.010),BLACK)
for i in range(2):
    rb('server front',(.412,0,TOP+(i+.5)*.04445),(.004,.430,.04445),BLUE)
    rb('roof',(.51,0,TOP+(i+1)*.04445-.001),(.200,.430,.002))
    rb('floor',(.51,0,TOP+i*.04445+.001),(.200,.430,.002))
    rb('rear',(.609,0,TOP+(i+.5)*.04445),(.002,.430,.04445))
    for sy in (-1,1):rb('side',(.51,sy*.214,TOP+(i+.5)*.04445),(.200,.002,.04445))
# Four real stand-offs define the final plane; all are far from the hand grips.
for y in (-.100,.100):
    for z in (-.018225,.018225):rb('bezel stop',(.39645,y+TARGET[1],TARGET[2]+z),(.0271,.010,.006))
fixed=[parts.box(tuple(s),tuple(c),co) for _,c,s,co in RACK_BOXES]
BOLT_SHAPES=[parts.cylinder(.005,.006,(.347,sy*.23255,TARGET[2]+z),STEEL,axis='x') for sy in (-1,1) for z in (-.015875,.015875)]
fixed+=BOLT_SHAPES
RACK_BASE_COUNT=len(fixed)
# Right capture sockets: front wall, back wall, outer wall and three vertical bands.
zc=TARGET[2]
def fixed_box(lo,hi,color=STEEL):
    lo=np.array(lo);hi=np.array(hi);fixed.append(parts.box(tuple(hi-lo),tuple((hi+lo)/2),color))
fixed_box((.355,-.2413,zc-.022225),(.37125,-.2185,zc+.022225))
fixed_box((.380,-.2413,zc-.022225),(.385,-.2185,zc+.022225))
fixed_box((.37125,-.2413,zc-.022225),(.380,-.2335,zc+.022225))
for a,b in ((-.022225,-.016),(-.008,.008),(.016,.022225)):
    fixed_box((.37125,-.2335,zc+a),(.380,-.2185,zc+b))
RIGHT_RECEIVER_SHAPES=fixed[RACK_BASE_COUNT:]
# Left receiver has an actual behind-catch window and load-bearing top/bottom walls.
for a,b in ((-.022225,-.0055),(.0055,.022225)):
    fixed_box((.355,.2185,zc+a),(.385,.2413,zc+b))
fixed_box((.382,.2185,zc-.0055),(.385,.2413,zc+.0055))
fixed_box((.355,.2383,zc-.0055),(.382,.2413,zc+.0055))
fixed_box((.356,.2185,zc-.0055),(.365,.2383,zc+.0055))
LEFT_RECEIVER_SHAPES=fixed[RACK_BASE_COUNT+len(RIGHT_RECEIVER_SHAPES):]
RACK=parts.Part('rack_and_chassis',pos=(0,0,0),shapes=fixed,friction=(.35,.25))
# Free perforated frame. Its25mm depth is the actual grip width, not a new handle.
bs=[parts.box((.025,.430,.008),(0,0,z),BLACK) for z in (-.018225,.018225)]
for y in (-.210,):bs.append(parts.box((.025,.010,.04445),(0,y,0),BLACK))
for z in (-.0146125,.0146125):bs.append(parts.box((.025,.010,.015225),(0,.210,z),BLACK))
# Vent grid across the front; a real window exposes the indicator's motion.
for z in (-.010,0,.010):bs.append(parts.box((.004,.370,.004),(-.0105,-.015,z),BLACK))
for y in np.arange(-.190,.171,.040):bs.append(parts.box((.004,.004,.02845),(-.0105,float(y),0),BLACK))
for z in (-.0086125,.0086125):bs.append(parts.box((.004,.040,.011225),(-.0105,.195,z),BLACK))
bs.append(parts.box((.004,.013,.02845),(-.0105,.2085,0),BLACK))
FRAME_SHAPES=list(bs)
# Fixed tabs, captured at two separated heights in the right receiver.
for z in (-.012,.012):bs.append(parts.box((.005,.010,.005),(.005,-.220,z),STEEL))
TAB_SHAPES=bs[len(FRAME_SHAPES):]
# Buildable slide guides:14.3 x8.3 bore, with0.15mm nominal bearing clearance per side.
for z in (-.005825,.005825):bs.append(parts.box((.017,.024,.00335),(.003,.202,z),STEEL))
for x in (-.004825,.010825):bs.append(parts.box((.00135,.010,.0083),(x,.209,0),STEEL))
bs+=g.ring_y(.004,.0018,.156,.159,STEEL)
bs.append(parts.box((.0145,.003,.011225),(-.00425,.159,.0091125),STEEL))
# Stem cross-pin reaches a side lug at each travel end; the nose passes beside the lugs.
for y in (.17084,.183):
    bs.append(parts.box((.00485,.002,.004),(-.006575,y,0),STEEL))
    bs.append(parts.box((.00485,.002,.012725),(-.006575,y,.0083625),STEEL))
GUIDE_SHAPES=bs[len(FRAME_SHAPES)+len(TAB_SHAPES):]
profile=[(-.004,.184),(.010,.184),(.010,.214),(0,.224),(-.004,.224)]
ls=[g.extrusion(profile,-.004,.004,BLACK),
    parts.cylinder(.0015,.036,(.003,.172,0),STEEL,axis='y'),
    parts.cylinder(.003,.002,(.003,.18092,0),STEEL,axis='y'),
    parts.cylinder(.001,.014,(-.001,.18092,0),STEEL,axis='x'),
    parts.cylinder(.0015,.019,(-.0055,.194,0),STEEL,axis='x'),
    parts.box((.002,.006,.005),(-.015,.194,0),YELLOW)]
BEZEL=parts.Part('bezel',pos=START,mass=.342,shapes=bs,friction=(.35,.25),joints=[
    parts.Slider('latch',tuple(ls),axis=(0,-1,0),travel=.008,mass=.008,spring=464.,damping=.5,max_effort=10.)])
OBJECTS=[RACK,BEZEL]
REC={};FACES={};HANDS=('gripper','tip_left','tip_right')
def rot(q):return parts._quat_to_matrix(q)
def frame_points(s):return np.array([[-.0125,y,z] for y in (-.200,0,.200) for z in (-.018,.018)])@rot(s.quat_xyzw).T+s.pos
def tabs_seated(s):
    R=rot(s.quat_xyzw)
    for z0 in (-.012,.012):
        p=np.array([[x,y,z] for x in (.0025,.0075) for y in (-.225,-.215) for z in (z0-.0025,z0+.0025)])@R.T+s.pos
        if not(p[:,0].min()>=.37125-.0001 and p[:,0].max()<=.380+.0001 and p[:,1].min()>=-.2335-.0001 and p[:,1].min()<=-.2235 and p[:,2].min()>=zc+z0-.004-.0001 and p[:,2].max()<=zc+z0+.004+.0001):return False
    return True
def frame_aligned(s):
    R=rot(s.quat_xyzw)
    return bool((np.abs(frame_points(s)[:,0]-.3575)<=.002+1e-6).all() and abs(s.pos[1]+.002)<=.003+1e-6 and abs(s.pos[2]-TARGET[2])<=.002+1e-6 and math.acos(float(np.clip(R[2,2],-1.,1.)))<=math.radians(3)+1e-6)
def fitted(s):return bool(tabs_seated(s) and frame_aligned(s))
def clear_hands(sim,o):
    from yam_arena import analysis
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);hulls=analysis.link_hulls();expected=set(hulls)
    if not expected or not set(HANDS)<=expected:return None
    for side in ('left_robot','right_robot'):
        a=base.scene[side];found=set()
        for name,p in zip(a.body_names,parts._np(a.data.body_link_pose_w)[0]):
            if name not in expected:continue
            found.add(name)
            if not np.isfinite(p).all() or np.linalg.norm(p[3:])<1e-8:return None
            if name not in FACES:
                hh=hulls.get(name,[])
                if not hh:return None
                FACES[name]=dict(vertices=np.concatenate([h.vertices for h in hh]),cloud=None)
            cache=FACES[name];R=rot(p[3:]);w=cache['vertices']@R.T+p[:3]-origin;pts=None
            targets=[o['bezel'],o['rack_and_chassis']] if name in HANDS else [o['bezel']]
            for s in targets:
                if not np.isfinite(np.r_[s.low,s.high]).all():return None
                if np.any(w.max(0)<s.low-.05) or np.any(w.min(0)>s.high+.05):continue
                if cache['cloud'] is None:
                    q=np.concatenate([release.faces(h) for h in hulls[name]]);_,ix=np.unique(np.round(q/.0015).astype(int),axis=0,return_index=True);cache['cloud']=q[ix]
                if pts is None:pts=cache['cloud']@R.T+p[:3]-origin
                if analysis.box_signed_distance(pts,(s.low+s.high)/2,np.eye(3),s.high-s.low).min()<=.005:return False
        if found!=expected:return None
    return True
def apply_pull(sim,N):
    import torch
    a=sim.env.unwrapped.scene['bezel'];p=sim.states()['bezel'].pos+parts._np(sim.env.unwrapped.scene.env_origins[0]);dev=sim.env.unwrapped.device
    a.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[[-float(N),0.,0.]]],device=dev),positions=torch.tensor([[[float(x) for x in p]]],device=dev),body_ids=[0],env_ids=torch.tensor([0],dtype=torch.int32,device=dev),is_global=True)
    REC['app']=float(N);native_loads.pull_active=bool(N>0);native_loads.pull_N=float(N)
@parts.on_reset
def reset(sim):
    REC.update(known=False,fresh=False,last_native=0,clear=False,bad=False,hold=0.,t=None,still=False,proof=0.,proof_ticks=0,proof_force=None,proof_ok=False,app=0.,latch_prev=None,warned=False)
    native_loads.reset(sim);apply_pull(sim,0.);sim.set_target('bezel','latch',-.5/464.)
@parts.on_step
def observe(sim):
    try:
        o=sim.states();s=o['bezel'];now=float(sim.t);dt=.05 if REC['t'] is None else max(0,min(.1,now-REC['t']));REC['t']=now
        if not np.isfinite(np.r_[s.pos,s.quat_xyzw,list(s.joints.values())]).all() or np.linalg.norm(s.quat_xyzw)<1e-8:raise ValueError('unknown bezel state')
        clear=clear_hands(sim,o)
        fresh=native_loads.ticks>REC['last_native'];REC['last_native']=native_loads.ticks
        if clear is None or native_loads.invalid or not fresh:raise ValueError('unknown or stale native hand/load state '+str(native_loads.error))
        if native_loads.peak>40.01:REC['bad']=True
        q=s.joints['latch'];fit=fitted(s);flag=q<=.001
        still=s.resting(.008,.15) and (REC['latch_prev'] is None or abs(q-REC['latch_prev'])<=.008*max(dt,1e-6));REC['latch_prev']=q
        if REC['app'] and (native_loads.max_out>.002+1e-6 or native_loads.max_move>.002+1e-6 or not fit):REC['bad']=True
        ready=fit and flag and clear and still
        if ready and not REC['proof_ok']:
            old=REC['app']
            if old==0.:native_loads.max_out=0.;native_loads.max_move=0.;native_loads.baseline=None;native_loads.proof_hold=0.;native_loads.proof_ticks=0;REC['proof']=0.
            apply_pull(sim,min(10.,old+1.))
            REC['proof']=native_loads.proof_hold
            REC['proof_ticks']=native_loads.proof_ticks
            REC['proof_force']=None if native_loads.force_world is None else native_loads.force_world.tolist()
            if REC['proof']>=1.-1e-6 and native_loads.proof_ticks>0 and not REC['bad']:
                REC['proof_ok']=True
                print('[bezel pull certificate] '+json.dumps(dict(t=now,proof_s=REC['proof'],native_samples=REC['proof_ticks'],physics_dt=native_loads.physics_dt,force_world=REC['proof_force'],max_3d_motion_m=native_loads.max_move,max_front_error_m=native_loads.max_out,finger_peak_N=native_loads.peak)),flush=True)
                apply_pull(sim,0.)
        elif not ready and REC['app']:apply_pull(sim,0.);REC['proof']=0.
        if REC['proof_ok'] and not(fit and flag):REC['proof_ok']=False
        REC.update(known=True,fresh=True,clear=clear,still=still,hold=REC['hold']+dt if clear and still and not REC['bad'] else 0.)
        if os.environ.get('TRACE_OUT') and round(now*20)%20==0:
            print('[bezel diagnostics] '+json.dumps(dict(t=now,pos=s.pos.tolist(),quat=s.quat_xyzw.tolist(),tabs=tabs_seated(s),fit=fit,latch=q,clear=clear,still=still,load=native_loads.peak,bad=REC['bad'],app=REC['app'],proof=REC['proof'],proof_ok=REC['proof_ok'],out=native_loads.max_out,move=native_loads.max_move,reaction=native_loads.reaction,native_ticks=native_loads.ticks,proof_ticks=native_loads.proof_ticks,physics_dt=native_loads.physics_dt,force_world=None if native_loads.force_world is None else native_loads.force_world.tolist())),flush=True)
    except Exception as e:
        if not REC.get('warned'):print('[bezel measurement] '+repr(e),flush=True);REC['warned']=True
        if REC.get('app'):apply_pull(sim,0.)
        REC.update(known=False,fresh=False,clear=False,hold=0.)
def score(o):
    r=getattr(o.get('measurement'),'bezel_probe',REC)
    if not(r.get('known') and r.get('fresh') and r.get('clear') and r.get('still') and not r.get('bad') and r.get('hold',0)>=1.-1e-6):return 0.
    s=o['bezel']
    if not fitted(s):return .25 if tabs_seated(s) else 0.
    f=np.asarray(r.get('proof_force') if r.get('proof_force') is not None else [],float)
    proved=r.get('proof_ok') and r.get('proof',0)>=1.-1e-6 and r.get('proof_ticks',0)>0 and f.shape==(3,) and np.isfinite(f).all() and np.allclose(f,(-10.,0.,0.),rtol=0,atol=.001)
    return 1. if proved and s.joints.get('latch',1)<=.001 else .5
def success(o):return score(o)==1.
def demo():
    out={};carry_z=TARGET[2]+.0045 # Fixed scripted waypoint correction from development trace; no pose feedback.
    stages=[(2.,(.220,0,TOP+.124225),0,FINGER_OPEN,'over'),(2.,START,0,FINGER_OPEN,'lower'),(1.,START,0,-.0115,'grasp'),(2.,(.220,0,carry_z),0,-.0115,'lift'),(2.,(.250,.010,carry_z),8,-.0115,'open'),]
    a=math.radians(8);c=(.370-.215*math.sin(a),-.217+.215*math.cos(a),carry_z)
    stages.extend([(2.5,(c[0],c[1]+.010,c[2]),8,-.0115,'approach sockets'),(2.,c,8,-.0115,'insert tabs')])
    for deg in (6,4,2,0):
        a=math.radians(deg);stages.append((1.5,(.370-.215*math.sin(a),-.217+.215*math.cos(a),carry_z),deg,-.0115,'pivot and latch'))
    stages.extend([(3.,(TARGET[0]+.015,TARGET[1],carry_z),0,-.0115,'press against physical stops'),(1.,(TARGET[0]+.015,TARGET[1],carry_z),0,-.0115,'hold latch push'),(1.5,(TARGET[0],TARGET[1],carry_z),0,-.0115,'relieve push'),(1.5,(TARGET[0],TARGET[1],carry_z),0,-.016,'loosen'),(2.,(TARGET[0],TARGET[1],carry_z+.050),0,-.016,'lift clear'),(2.,(.240,-.002,carry_z+.050),0,FINGER_OPEN,'withdraw'),(6.,(.240,-.002,carry_z+.050),0,FINGER_OPEN,'pull check and settle')])
    for side,sgn in (('left',1),('right',-1)):
        way=[]
        for dur,c,deg,f,label in stages:
            a=math.radians(deg);p=np.array(c)+np.array((-math.sin(a)*sgn*.1625,math.cos(a)*sgn*.1625,.015775))
            way.append(Waypoint(dur,tuple(p),f,label,(-sgn*math.pi/2)+a))
        out[side]=way
    return out
