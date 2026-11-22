"""A blue C-A-T Gen 7 training tourniquet surrounds a chocked and strapped training
thigh. Its rod and attached two-sided clip sit above the band. A sealed bladder
feeds an aneroid gauge with a green sector. Both grippers start open.
"""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts
import lots
import tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import FINGER_OPEN, Waypoint, grip_rotation

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = .05  # the independent scorer already enforces the full second
INSTRUCTION = 'Tighten the C-A-T trainer until its gauge is green, lock its rod in its attached clip, then move both hands clear.'
HERE = Path(__file__).resolve().parent
BLUE, DARK, FOAM = (.08,.30,.76), (.12,.13,.15), (.68,.53,.37)
PIVOT = np.array((.230, 0., TOP+.1505))
ROD_LENGTH, ROD_RADIUS = .150, .006
CLIP_MOUTH = .0107
FIXTURE_YAW = 0.
START_YAW = math.radians(126.)+FIXTURE_YAW
SEAT_TILT = math.radians(21.)
HOOK_RADIUS = .050
CLEARANCE = .010
SAMPLE_MARGIN = .006  # >3mm face raster +1.5mm voxel coverage and0.8mm rod ribs

# Development priors only. The SOF Gen 5 study and previous submission do not
# measure the specified C-A-T trainer. Physical calibration remains open.
DEVELOPMENT = {'torque_nm_at_250':1.8, 'snap_in_n':10., 'snap_out_n':10.,
               'pressure_points':[[0.,60.],[30.,90.],[78.,138.],[150.,210.],[210.,270.]]}

def read_calibration():
    path = HERE/'evidence'/'trainer-measurements.json'
    if not path.exists():
        return DEVELOPMENT.copy(), False, 'specified-trainer measurements absent'
    try:
        c = json.loads(path.read_text())
        if c['product'] != 'NAR C-A-T Gen 7 blue trainer 30-0033':
            raise ValueError('wrong product')
        if c.get('measured') is not True or not c['date_utc'] or not c['specimen_id']:
            raise ValueError('missing measurement provenance')
        for key in ('instrument','method','raw_file','raw_sha256'):
            if not c.get(key): raise ValueError('missing '+key)
        raw = (HERE/'evidence'/c['raw_file']).resolve()
        if not raw.is_relative_to((HERE/'evidence').resolve()) or not raw.is_file():
            raise ValueError('missing raw readings')
        if hashlib.sha256(raw.read_bytes()).hexdigest()!=c['raw_sha256']:
            raise ValueError('raw readings hash mismatch')
        readings = c['winding_readings']
        for angle in (78.,150.,210.):
            rows = [r for r in readings if r['angle_deg']==angle]
            if len(rows)<3: raise ValueError('three winding repeats required per angle')
            if any(not all(math.isfinite(float(r[k])) and float(r[k])>0 for k in
                           ('lever_m','tightening_force_n','holding_force_n','pressure_mmhg')) for r in rows):
                raise ValueError('invalid winding reading')
        snaps=c['snap_readings']
        if len(snaps)<3 or any(not all(math.isfinite(float(r[k])) and float(r[k])>0
                                      for k in ('insert_n','extract_n')) for r in snaps):
            raise ValueError('three positive snap repeats required')
        # Preserve the higher tightening resistance, never average it downward
        # with the lower holding force. Interpolate measured torque by angle.
        c['torque_points'] = [[a,max(r['lever_m']*r['tightening_force_n'] for r in readings if r['angle_deg']==a)]
                              for a in (78.,150.,210.)]
        c['snap_in_n'] = max(r['insert_n'] for r in snaps)
        c['snap_out_n'] = max(r['extract_n'] for r in snaps)
        pp = np.array(c['pressure_points'],float)
        if pp.shape[1]!=2 or len(pp)<5 or not np.isfinite(pp).all() or not np.all(np.diff(pp[:,0])>0):
            raise ValueError('invalid measured pressure curve')
        if abs(pp[0,0])>1e-9 or abs(pp[0,1]-60)>2 or np.any(np.diff(pp[:,1])<0):
            raise ValueError('invalid pressure enrollment')
        return c, True, 'hardware evidence loaded; independent verification still required'
    except (ValueError,KeyError,TypeError,IndexError) as e:
        return DEVELOPMENT.copy(),False,'invalid calibration: '+str(e)

CALIBRATION, CALIBRATED, CALIBRATION_NOTE = read_calibration()

def pressure(angle):
    if not CALIBRATED: return min(300.,60.+abs(math.degrees(angle)))
    points=np.array(CALIBRATION['pressure_points'])
    return float(np.interp(abs(math.degrees(angle)),points[:,0],points[:,1]))

def winding_load(angle):
    return (1.94/math.radians(210))*abs(angle)

def fixture_point(x,y,z):
    c,si=math.cos(FIXTURE_YAW),math.sin(FIXTURE_YAW)
    return (float(.23+c*(x-.23)-si*y),float(si*(x-.23)+c*y),float(z))

# The joint base is fixed because the full-size training cylinder is strapped
# into real chocks. The elastic winding torque reacts through that base.
THIGH=parts.Part('strapped_training_thigh',pos=(.23,0.,TOP+.0655),yaw=FIXTURE_YAW,shapes=[
    parts.cylinder(.0635,.400,color=FOAM,axis='y'),
    parts.cylinder(.0695,.038,color=BLUE,axis='y'),
    parts.cylinder(.066,.050,color=DARK,axis='y')],friction=(.6,.5))
CHOCKS=[]
for y in (-.15,.15):
    for x in (-.0707,.0707):
        CHOCKS.append(parts.Part('chock_'+str(len(CHOCKS)),pos=fixture_point(.23+x,y,TOP+.0175),yaw=FIXTURE_YAW,
                       shapes=[parts.box((.030,.040,.035),color=(.55,.36,.20))],friction=(.6,.5)))
# Securing straps at either end are separate from the scored central band.
STRAPS=[parts.Part('end_strap_'+str(i),pos=fixture_point(.23,y,TOP+.0655),yaw=FIXTURE_YAW,shapes=[
    parts.cylinder(.0645,.020,color=(.26,.27,.29),axis='y')],friction=(.6,.5))
    for i,y in enumerate((-.15,.15))]
WINDLASS=parts.Part('cat_windlass',pos=tuple(PIVOT),yaw=START_YAW,shapes=[
    parts.box((.030,.022,.005),(0,0,-.0175),BLUE)],joints=[
    parts.Hinge('winding',shapes=(parts.box((.020,.015,.005),(0,0,-.007),BLUE),),
                pivot=(0,0,0),axis=(0,0,1),lower=math.radians(-240),upper=math.radians(240),
                mass=.010,spring=1.94/math.radians(210),damping=.20,max_effort=5.),
    parts.Hinge('rod_tilt',shapes=(parts.cylinder(ROD_RADIUS,ROD_LENGTH,color=DARK,axis='x'),parts.cylinder(.0068,.004,(-.065,0,0),DARK,axis='x'),parts.cylinder(.0068,.004,(.065,0,0),DARK,axis='x')),
                pivot=(0,0,0),axis=(0,1,0),lower=-math.radians(30),upper=math.radians(30),
                mass=.030,spring=.10,damping=.025,max_effort=2.,parent='winding')],friction=(.8,.6))

# Attached bilateral clip: two physical channels. Each has two spring lips;
# compression, retention and release all come from contact geometry. There is
# no state-driven latch, joint target change, weld, or robot/object pose write.
CLIPS=[]
for side,psi in [('hip',math.radians(156)+FIXTURE_YAW),('knee',math.radians(204)+FIXTURE_YAW)]:
    floorz=-ROD_RADIUS-.0015
    r=parts._rot((0,0,1),psi)
    center=PIVOT+r@np.array((HOOK_RADIUS*math.cos(SEAT_TILT),0,-HOOK_RADIUS*math.sin(SEAT_TILT)))
    lips=[]
    # At the 11 mm mouth a 12 mm rod spreads the lips 0.5 mm each.
    # k is a development estimate; match the whole vertical push/pull curve
    # to measured insert/extract forces in the offline contact probe.
    lip_k=50000.  # N/m; independent CPU force/displacement probe, estimated clip
    for sign,label in [(-1,'a'),(1,'b')]:
        at=(0,sign*.00610,.003)
        lips.append(parts.Slider('lip_'+label,shapes=(parts.cylinder(.00075,.010,at,DARK,axis='x'),),
                    axis=(0,sign,0),travel=.002,mass=.008,spring=lip_k,damping=40.,max_effort=40.))
    CLIPS.append(parts.Part('attached_clip_'+side,pos=tuple(center),yaw=psi,pitch=SEAT_TILT,
        shapes=[parts.box((.012,.016,.003),(0,0,floorz),DARK),
                parts.box((.012,.002,.012),(0,-.0075,floorz+.006),DARK),
                parts.box((.012,.002,.012),(0,.0075,floorz+.006),DARK)],joints=lips,friction=(.35,.25)))
GAUGE_POS=np.array((.360,-.180,TOP+.105))
GAUGE=parts.Part('aneroid_gauge',pos=tuple(GAUGE_POS),mass=.20,shapes=[
    parts.cylinder(.026,.010,color=(.92,.92,.87)),
    parts.box((.012,.012,.100),(0,0,-.055),DARK),
    parts.box((.060,.060,.008),(0,0,-.101),DARK)],friction=(.6,.5))
_DIAL_OP=None

def draw_gauge(stage,root):
    from pxr import Gf,Sdf,UsdGeom,UsdShade
    texture=str(HERE/'assets'/'dial.png');material=UsdShade.Material.Define(stage,root+'/dial_material')
    shader=UsdShade.Shader.Define(stage,root+'/dial_material/surface');shader.CreateIdAttr('UsdPreviewSurface')
    tex=UsdShade.Shader.Define(stage,root+'/dial_material/texture');tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(texture)
    uv=UsdShade.Shader.Define(stage,root+'/dial_material/uv');uv.CreateIdAttr('UsdPrimvarReader_float2')
    uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    face=UsdGeom.Mesh.Define(stage,root+'/dial_face');n=64
    pts=[(0.,0.,.0056)]+[(.0255*math.cos(i*2*math.pi/n),.0255*math.sin(i*2*math.pi/n),.0056) for i in range(n)]
    face.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);face.CreateFaceVertexCountsAttr([3]*n)
    face.CreateFaceVertexIndicesAttr([k for i in range(n) for k in (0,1+i,1+(i+1)%n)])
    face.CreateSubdivisionSchemeAttr('none');face.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(face).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(
        [Gf.Vec2f(.5+p[0]/.051,.5+p[1]/.051) for p in pts])
    UsdShade.MaterialBindingAPI.Apply(face.GetPrim()).Bind(material)
    indicator=UsdGeom.Xform.Define(stage,root+'/indicator')
    pointer=UsdGeom.Mesh.Define(stage,root+'/indicator/pointer')
    pointer.CreatePointsAttr([Gf.Vec3f(-.0014,-.003,.007),Gf.Vec3f(.0014,-.003,.007),Gf.Vec3f(0,.022,.007)])
    pointer.CreateFaceVertexCountsAttr([3]);pointer.CreateFaceVertexIndicesAttr([0,1,2]);pointer.CreateSubdivisionSchemeAttr('none')
    pointer.CreateDisplayColorAttr([Gf.Vec3f(.7,.02,.02)])
    indicator.AddRotateZOp().Set(81.)  # 60 mmHg at reset
parts.VISUAL_HOOKS[GAUGE.name]=draw_gauge
PLATE=[]
for i,x in enumerate((-.005,-.015,-.025,-.035,-.045)):
    h=math.sqrt(.0695**2-x*x)
    PLATE.append(parts.Part('cat_stabilization_plate_'+str(i),pos=fixture_point(float(PIVOT[0]+x),0.,TOP+.0655+h+.0013),yaw=FIXTURE_YAW,
                 pitch=math.atan2(x,h),shapes=[parts.box((.010,.038,.0025),color=DARK)],friction=(.4,.3)))
OBJECTS=[THIGH,*CHOCKS,*STRAPS,*PLATE,WINDLASS,*CLIPS,GAUGE]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={GAUGE.name:GAUGE.pos}),
                  lots.Lot('B',place={GAUGE.name:(.400,.220,TOP+.105)}))
DEMO_TOUCHES={THIGH.name,*(c.name for c in CLIPS)}  # fingertip contact while grasping/seating; arm links still checked

def rod_pose(s):
    p,q=s.links['rod_tilt']
    return np.asarray(p),parts._quat_to_matrix(q)

def geometry(s):
    p,r=rod_pose(s)
    if not np.isfinite(np.r_[p,r.ravel(),list(s.joints.values())]).all():
        raise ValueError('nonfinite rod state')
    angle=float(s.joints['winding']); tipped=math.asin(float(np.clip(-r[2,0],-1,1)))
    ends=[p+sign*HOOK_RADIUS*r[:,0] for sign in (-1,1)]
    seated=False
    for clip in CLIPS:
        c=np.array(clip.pos); cr=clip.rotation
        for end in ends:
            d=(end-c)@cr
            if abs(d[0])<=.005 and abs(d[1])<=.0009 and -.0015<=d[2]<=.0025:
                seated=True
    return dict(angle=angle,pressure=pressure(angle),seated=bool(seated),tilt=tipped)

def value(snapshot,certified=True):
    if not certified or not snapshot.get('known') or snapshot.get('invalid'):
        return 0.
    p=snapshot['pressure']; locked=snapshot['seated'] and snapshot.get('snap_verified',False)
    if locked and p<250.-1e-6: return .25
    if 250.-1e-6<=p<=300.+1e-6 and not snapshot.get('body_touch'):
        if locked and snapshot.get('clear') and snapshot.get('still') and snapshot.get('hold_s',0)>=1.-1e-9:
            return 1.
        if locked or (snapshot.get('held') and snapshot.get('hand_contact_n',0)>=1. and abs(snapshot['tilt'])<=math.radians(5)+1e-9):
            return .5
    return 0.

_STATE={'known':False,'invalid':False}
_CERTIFICATE=None
_LAST=None
_CLOCK=0.
_FACE_CACHE={}

def stamp(s):
    return tuple(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel,list(s.joints.values()),
                       *[np.r_[p,q] for p,q in s.links.values()]])

def hand_points(sim):
    from yam_arena import analysis
    global _FACE_CACHE
    if not _FACE_CACHE:
        from scipy.spatial import ConvexHull
        for name,hulls in analysis.link_hulls().items():
            samples=[]
            for h in hulls:
                v=np.asarray(h.vertices);samples.append(v)
                for face in ConvexHull(v).simplices:
                    a,b,c=v[face];n=max(1,int(math.ceil(max(np.linalg.norm(b-a),np.linalg.norm(c-a),np.linalg.norm(c-b))/.003)))
                    samples.append(np.array([a+(b-a)*i/n+(c-a)*j/n for i in range(n+1) for j in range(n+1-i)]))
            cloud=np.concatenate(samples);_,ix=np.unique(np.floor(cloud/.0015).astype(int),axis=0,return_index=True)
            _FACE_CACHE[name]=cloud[ix]
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[]
    for side in ('left','right'):
        robot=base.scene[side+'_robot'];poses=parts._np(robot.data.body_link_pose_w)[0]
        for name,cloud in _FACE_CACHE.items():
            if name not in robot.body_names:continue
            p=poses[robot.body_names.index(name)]
            clouds.append(cloud@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
    if len(clouds)<12 or any(not np.isfinite(c).all() for c in clouds):raise ValueError('missing robot surfaces')
    return clouds

_VIEWS=None
_SNAP={}

def contact_forces(sim):
    global _VIEWS
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.robot import asset_link_paths
    if _VIEWS is None:
        paths=parts.ROOT_PATHS; links=asset_link_paths();envpath=sim.env.unwrapped.scene.env_prim_paths[0]
        handpaths=[envpath+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in links]
        lips=[paths[c.name]+'/'+j.name for c in CLIPS for j in c.joints]
        floors=[paths[c.name]+'/base' for c in CLIPS]
        sv=PhysxManager.get_physics_sim_view()
        _VIEWS=(sv.create_rigid_contact_view([paths[WINDLASS.name]+'/rod_tilt'],filter_patterns=[handpaths+lips+floors],max_contact_data_count=512),len(handpaths))
    view,nh=_VIEWS
    f=parts._np(view.get_contact_force_matrix(dt=sim.env.unwrapped.sim.get_physics_dt())).reshape(-1,3)
    if len(f)!=nh+6 or not np.isfinite(f).all():raise ValueError('unknown native contacts')
    return f[:nh],f[nh:nh+4].reshape(2,2,3),f[nh+4:]

def body_gap(clouds,states):
    v=np.concatenate(clouds);best=math.inf
    for radius,length in ((.0635,.400),(.0695,.038),(.066,.050)):
        d=(v-np.array(THIGH.pos))@THIGH.rotation;q=np.column_stack((np.hypot(d[:,0],d[:,2])-radius,np.abs(d[:,1])-length/2))
        signed=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.maximum(q[:,0],q[:,1]),0)
        best=min(best,float(signed.min()))
    gs=states[GAUGE.name]
    for _,c,r,size in GAUGE.world_boxes({'base':(np.asarray(gs.pos),parts._quat_to_matrix(gs.quat_xyzw))}):
        best=min(best,float(np.linalg.norm(np.maximum(np.abs((v-c)@r)-size/2,0),axis=1).min()))
    return best

def clearance(clouds,s):
    p,r=rod_pose(s);v=np.concatenate(clouds);d=(v-p)@r
    gap=float(np.linalg.norm(np.maximum(np.abs(d)-(.075,.006,.006),0),axis=1).min())
    for clip in CLIPS:
        for _,c,r,size in clip.world_boxes({'base':(np.array(clip.pos),clip.rotation)}):
            gap=min(gap,float(np.linalg.norm(np.maximum(np.abs((v-c)@r)-size/2,0),axis=1).min()))
    return gap

def clip_retained(entered,joints):
    """Current lips must overlap the rod after a witnessed physical entry."""
    displacements=[float(joints[name]) for name in ('lip_a','lip_b')]
    if not all(math.isfinite(v) for v in displacements):raise ValueError('unknown clip lip pose')
    mouth=CLIP_MOUTH+sum(max(0.,v) for v in displacements)
    return bool(entered and mouth<=2*ROD_RADIUS-.0002),float(mouth)

def new_snap():
    return dict(armed=False,passed=False,verified=False,peak=0.,work=0.,down=0.,
                last_z=None,last_line=False,last_end=None,last_fz=0.,last_contact=False,
                seen_lips=0,low_z=None)

def snap_update(q,end_index,d,forces):
    """Measure one insertion, including sequential contact with beveled lips.

    The rod axis first approaches above the mouth. New depth contributes work
    only with consecutive real rod-lip contact; force-free gaps add nothing.
    Each lip must contact during that same entry. An upward reversal or lost
    alignment discards the record. Verification creates no constraint.
    """
    d=np.asarray(d,float);forces=np.asarray(forces,float)
    if d.shape!=(3,) or forces.shape!=(2,3) or not np.isfinite(np.r_[d,forces.ravel()]).all():
        raise ValueError('invalid snap observation')
    z=float(d[2]);line=bool(abs(d[0])<.006 and abs(d[1])<.0015)
    norms=np.linalg.norm(forces,axis=1);bits=sum(1<<i for i,n in enumerate(norms) if n>.1)
    fz=float(forces[:,2].sum());contact=bool(bits and fz>0)
    same=q['last_end']==int(end_index) and q['last_line']
    previous_z=q['last_z'];previous_fz=q['last_fz'];previous_contact=q['last_contact']
    if not line or not same:
        q.clear();q.update(new_snap())
    elif q['verified']:
        if z<-.002 or z>.003:q.clear();q.update(new_snap())
    elif q['armed']:
        if q['low_z'] is not None and z>float(q['low_z'])+.0002:
            q.clear();q.update(new_snap())
        else:
            # Record actual new contact distance, never free descent or repeated
            # motion over an already traversed depth. Asymmetric loading is real
            # clip entry; each bead still needs its own rod-contact witness.
            if q['low_z'] is None:q['low_z']=z
            advance=max(0.,float(q['low_z'])-z)
            if contact:
                q['peak']=max(q['peak'],fz)
                if previous_contact and previous_fz>0:
                    q['work']+=min(previous_fz,fz)*advance;q['down']+=advance
            q['low_z']=min(float(q['low_z']),z);q['seen_lips'] |= int(bits)
            if q['seen_lips']==3 and q['peak']>=10. and q['work']>=.001 and q['down']>=.001:
                q['passed']=True
            # The lip force normally reverses after passing the bead. Completed
            # physical work remains evidence while the same entry reaches seat.
            if -.0015<=z<.0015 and q['passed']:q['verified']=True
    if line and z>.008:q['armed']=True
    q['last_z']=z;q['last_line']=line;q['last_end']=int(end_index)
    q['last_fz']=fz;q['last_contact']=contact
    return bool(q['verified'] and line and abs(d[0])<=.005 and abs(d[1])<=.0009 and -.0015<=z<=.0025)

@parts.on_reset
def reset(sim=None):
    global _CERTIFICATE,_LAST,_CLOCK,_VIEWS,_SNAP
    _STATE.clear();_STATE.update(known=False,invalid=False,snap_verified=False,hold_s=0.)
    _CERTIFICATE=None;_LAST=None;_CLOCK=0.;_VIEWS=None
    _SNAP={c.name:new_snap() for c in CLIPS}
    if sim is not None:
        from isaaclab.app.settings_manager import get_settings_manager
        get_settings_manager().set_bool('/physics/disableContactProcessing',False)

@parts.on_step
def observe(sim):
    global _CERTIFICATE,_LAST,_CLOCK
    try:
        states=sim.states();s=states[WINDLASS.name];g=geometry(s);now=float(sim.t);dt=now-_CLOCK
        if not 0<dt<=.100001: raise ValueError('invalid observation interval')
        if g['seated'] or g['pressure']>=250.:
            clouds=hand_points(sim);gap=clearance(clouds,s);body_distance=body_gap(clouds,states);body_touch=body_distance<SAMPLE_MARGIN
        else:
            gap=0.;body_touch=False;body_distance=None
        hands,lip_forces,floors=contact_forces(sim)
        p,r=rod_pose(s);ends=[p+sign*HOOK_RADIUS*r[:,0] for sign in (-1,1)]
        verified=False;peaks=[];mouths=[]
        for i,clip in enumerate(CLIPS):
            q=_SNAP[clip.name];cr=clip.rotation;c=np.array(clip.pos)
            ends_local=[(e-c)@cr for e in ends];end_index=min(range(2),key=lambda i:np.linalg.norm(ends_local[i][:2]));d=ends_local[end_index]
            entered=snap_update(q,end_index,d,lip_forces[i]@cr)
            retained,mouth=clip_retained(entered,states[clip.name].joints);verified |= retained;mouths.append(mouth)
            peaks.append(q['peak'])
        speed=math.inf if _LAST is None else abs(g['angle']-_LAST['angle'])/dt
        ts=math.inf if _LAST is None else abs(g['tilt']-_LAST['tilt'])/dt
        still=speed<=math.radians(1) and ts<=math.radians(1)
        hand_n=float(np.linalg.norm(hands,axis=1).sum())
        eligible=g['seated'] and verified and 250<=g['pressure']<=300 and gap>=CLEARANCE+SAMPLE_MARGIN and still and not body_touch
        hold=_STATE.get('hold_s',0)+dt if eligible else 0.
        _STATE.update(g,known=True,clear=gap>=CLEARANCE+SAMPLE_MARGIN,held=hand_n>=1.,hand_contact_n=hand_n,
                      body_touch=body_touch,body_gap_sample_m=body_distance,sampled_clearance_m=gap,still=still,snap_verified=verified,hold_s=hold,clip_force_peaks=peaks,
                      lip_forces=lip_forces.tolist(),clip_base_forces=floors.tolist(),clip_mouths_mm=[float(v*1000) for v in mouths],hand_force_peak=float(np.linalg.norm(hands,axis=1).max()))
        _CLOCK=now;_LAST=g;_CERTIFICATE=stamp(s)
        _STATE['hand_left_n']=float(np.linalg.norm(hands[:len(hands)//2],axis=1).sum())
        _STATE['hand_right_n']=float(np.linalg.norm(hands[len(hands)//2:],axis=1).sum())
        _STATE['grippers']={}
        from yam_arena.constants import TCP_OFFSET
        base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0])
        for side in ('left','right'):
            robot=base.scene[side+'_robot'];bp=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
            jq=parts._np(robot.data.joint_pos)[0]
            _STATE['grippers'][side]=dict(quat=bp[3:].tolist(),tcp=(bp[:3]-origin+parts._quat_to_matrix(bp[3:])@np.array(TCP_OFFSET)).tolist(),finger_q=[float(jq[robot.joint_names.index(n)]) for n in ('joint7','joint8')])
        from pxr import UsdGeom
        from isaaclab.sim.utils import get_current_stage
        gauge_root=parts.ROOT_PATHS[GAUGE.name]
        prim=get_current_stage().GetPrimAtPath(gauge_root+'/indicator')
        ops=UsdGeom.Xformable(prim).GetOrderedXformOps()
        ops[0].Set(135-270*min(g['pressure'],300)/300)
    except (KeyError,ValueError,AttributeError,IndexError,TypeError) as e:
        _CLOCK=float(sim.t)
        _STATE.update(known=False,invalid=True,hold_s=0.,error=type(e).__name__+': '+str(e))
    if os.environ.get('TRACE_OUT'):
        row=dict(t=float(sim.t),calibrated=CALIBRATED,state=_STATE,contacts=_SNAP)
        with Path(os.environ['TRACE_OUT']).with_name('windlass.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
    try:
        s=objects[WINDLASS.name]
        if stamp(s)!=_CERTIFICATE:return 0.
        # Native contact observations and physical retention gate every credited state.
        return value(_STATE,True)
    except (KeyError,TypeError,ValueError,AttributeError):return 0.

def success(objects):return score(objects)==1.


def demo():
    """Open-loop two-handed regrip and seating from the visible fixed layout."""
    from yam_arena import ik,station
    from yam_arena.constants import READY
    plans={'left':[],'right':[]};theta0=START_YAW
    gain=1.20;handoff=130.;retake=165.
    def point(theta,sign,dip=0.,radius=.065,above=0.,pad_height=0.):
        rot=parts._rot((0,0,1),theta)@parts._rot((0,1,0),dip)
        return PIVOT+rot@np.array((sign*radius,0.,pad_height))+np.array((0.,0.,above))
    def add(arm,t,p,f,label,yaw,actual,lean=None,dip=0.):
        if lean is None:lean=-45. if arm=='left' else 0.
        axis=(-math.sin(actual),math.cos(actual),0.)
        rot=parts._rot(axis,dip)@parts._rot((-math.sin(yaw),math.cos(yaw),0),math.radians(lean))@grip_rotation(yaw)
        plans[arm].append(Waypoint(t,tuple(float(v) for v in p),f,label,tilt._TiltedYaw(0,rot)))
    def right(t,a,f,label,above=0.,dip=0.,branch=1,loaded=True,angle_lead=0.,z_lead=0.):
        cmd=theta0+math.radians(a*(gain if loaded else 1.)+angle_lead)
        add('right',t,point(cmd,1,dip,above=above+z_lead),f,label,theta0+math.radians(a)-branch*math.pi,theta0+math.radians(a),dip=dip)
    def left(t,a,f,label,above=0.,dip=0.,radius=.065,loaded=False,angle_lead=0.,z_lead=0.):
        cmd=theta0+math.radians(a*(gain if loaded else 1.)+angle_lead)
        add('left',t,point(cmd,-1,dip,radius,above+z_lead),f,label,theta0+math.radians(a),theta0+math.radians(a),dip=dip)
    right(2.,0,FINGER_OPEN,'approach rod',above=.015,loaded=False)
    right(1.,0,FINGER_OPEN,'lower to rod',loaded=False)
    right(2.,0,-.0045,'grasp rod',loaded=False)
    for a in np.linspace(0,handoff,27)[1:]:right(.25,float(a),-.0045,'wind first stroke')
    first_end=5.+26*.25
    # Explicit stationary READY hold: the other arm does not approach through
    # the turning forearm while the first loaded stroke is in progress.
    ready_p,ready_r=ik.tcp_pose(np.asarray(READY),station.BASES['left'])
    plans['left'].append(Waypoint(first_end,tuple(ready_p),FINGER_OPEN,'wait at ready',tilt._TiltedYaw(0,ready_r)))
    right(6.,handoff,-.0045,'wait counterhold')
    left(2.,handoff,FINGER_OPEN,'approach counterhold',above=.025)
    left(1.,handoff,FINGER_OPEN,'lower counterhold')
    left(2.,handoff,-.0045,'counterhold')
    left(1.,handoff,-.0045,'take winding load',loaded=True)
    right(1.,handoff,FINGER_OPEN,'release first grip')
    right(1.,handoff,FINGER_OPEN,'clear before wrist reset',above=.015)
    add('right',1.,(.13,-.22,TOP+.18),FINGER_OPEN,'reset wrist away',0.,0.,lean=0.)
    add('right',4.5,(.13,-.22,TOP+.18),FINGER_OPEN,'wait for counterhold turn',0.,0.,lean=0.)
    right(1.5,retake,FINGER_OPEN,'return after wrist reset',above=.025,branch=2,loaded=False)
    right(1.,retake,FINGER_OPEN,'retake rod',branch=2,loaded=False)
    right(2.,retake,-.0045,'close second grip',branch=2,loaded=False)
    right(1.,retake,-.0045,'take load',branch=2)
    right(3.,retake,-.0045,'wait counterhold release',branch=2)
    left(3.,handoff,-.0045,'maintain winding load',loaded=True)
    for a in np.linspace(handoff,retake,15)[1:]:left(.25,float(a),-.0045,'counterhold turns beyond wrist limit',loaded=True)
    left(6.5,retake,-.0045,'maintain winding load',loaded=True)
    left(1.,retake,FINGER_OPEN,'release counterhold',loaded=True)
    left(2.,retake,FINGER_OPEN,'withdraw counterhold',above=.06,loaded=True)
    for a in np.linspace(retake,210,16)[1:]:right(.25,float(a),-.0045,'wind to green',branch=2)
    right(5.,210,-.0045,'hold for seating grasp',branch=2)
    left(3.75,210,FINGER_OPEN,'approach attached clip',above=.025,radius=.073)
    left(2.,210,FINGER_OPEN,'lower seating grasp',radius=.073)
    left(3.,210,-.0045,'grasp seating end',radius=.073)
    # Measured clip-phase compliance compensation; jaw angles retain intended
    # rod alignment. The level pause presents the rod above the real lip mouth.
    right(2.,210,-.0045,'align above clip',branch=2,angle_lead=-10.5)
    left(2.,210,-.0045,'align above clip',radius=.073)
    right(1.,210,-.0045,'pause above mouth',branch=2,angle_lead=-10.5)
    left(1.,210,-.0045,'pause above mouth',radius=.073)
    for u in np.linspace(0,1,8)[1:]:
        dip=-SEAT_TILT*float(u)
        right(5./7,210,-.0045,'allow rod tilt',dip=dip,branch=2,angle_lead=-10.5,z_lead=.015*u)
        left(5./7,210,-.0045,'press rod through lips',dip=dip,radius=.073,angle_lead=0.,z_lead=-.015*u)
    right(6.,210,-.0045,'hold seated rod',dip=-SEAT_TILT,branch=2,angle_lead=-10.5,z_lead=.015)
    left(2.5,210,-.0045,'seat under load',dip=-SEAT_TILT,radius=.073,angle_lead=0.,z_lead=-.015)
    left(1.,210,FINGER_OPEN,'release seating hand',dip=-SEAT_TILT,radius=.073,angle_lead=0.,z_lead=-.015)
    left(1.5,210,FINGER_OPEN,'lift from attached clip',above=.10,dip=-SEAT_TILT,radius=.073,angle_lead=0.,z_lead=-.015)
    add('left',2.,(.13,.24,TOP+.14),FINGER_OPEN,'withdraw seating hand',0.,0.,lean=0.)
    add('left',4.,(.13,.24,TOP+.14),FINGER_OPEN,'hands clear',0.,0.,lean=0.)
    right(1.,210,FINGER_OPEN,'release retained rod',dip=-SEAT_TILT,branch=2,angle_lead=-10.5,z_lead=.015)
    add('right',2.,(.13,-.22,TOP+.14),FINGER_OPEN,'withdraw turning hand',0.,0.,lean=0.)
    add('right',2.,(.13,-.22,TOP+.14),FINGER_OPEN,'released stillness',0.,0.,lean=0.)
    for side,waypoints in plans.items():
        q=None
        for w in waypoints:
            q,pe,re=ik.solve(np.array(w.pos),ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)),q_init=q,base_pos=station.BASES[side],iters=160)
            if pe>.001 or re>math.radians(.5):raise ValueError('unreachable fixed demo endpoint '+side+' '+w.label+' '+str((pe*1000,math.degrees(re),w.pos)))
            w.joint_target=tuple(float(v) for v in q);w.endpoint_ik_mm=float(pe*1000);w.endpoint_ik_deg=float(math.degrees(re))
    return plans
