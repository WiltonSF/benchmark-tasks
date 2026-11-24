"""A soft-robot fabrication bench. A grey bench-mounted precision vise, head turned so its jaws close vertically, clamps the exposed 50 mm end of a horizontal
13 mm stainless-steel casting core. A pink 170 mm silicone actuator wall (12.7 mm bore; stock 15.9 mm outside, simulated 16.1 mm across flats) sits on
the rest of the core, its far end flush with the core tip. A blue parts tray lies beyond the core tip, a
millimetre ruler beside it, and an alcohol wash bottle stands at the back of the bench.
"""
from __future__ import annotations
import hashlib, json, math, os, pathlib, sys
import numpy as np
from dataclasses import replace
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.station import TABLE_CENTER, TABLE_SIZE
from yam_arena.demo import Waypoint
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.05
SOURCE_SHA256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
GEOM_EPS=1e-6

# ---- Supplied phone readings and explicitly separate numerical estimates ----
# Obtained by phone from a technician who works with this product (Nov 2026):
# matching specimen tube/rod pulls 11, 11, 13 N; rounded mean 11.7 N.
# These readings do not supply a force/displacement curve or a damage threshold.
WITHDRAW_READINGS_N=(11.,11.,13.)
WITHDRAW_N=11.7
LOAD_SCALE=float(os.environ.get('SOFTCORE_LOAD_SCALE','1'))  # sensitivity control only; final runs use1
CORE_MU=.15*LOAD_SCALE                   # numerical contact calibration, not measured tube/steel friction
BORE_CONTACT_K=125000.                  # native implicit contact penalty N/m/contact, numerically calibrated
BORE_CONTACT_D=.5                       # native contact damping N s/m/contact (estimate)
CONTACT_SEP=.00010; CONTACT_MIN_N=.001  # contact-data qualification, not an artificial force trigger
CLAMP_N=500.*float(os.environ.get('SOFTCORE_CLAMP_SCALE','1'))  # jaw drive estimate (hand-tightened bench vise); zero-preload control
JAW_MU=.90   # serrated steel vise jaws on a steel rod (estimate)
WALL_FRICTION=(1.2,1.0)  # dry silicone exterior estimate; see README study analogue, not a stock/YAM measurement
# ---- Wall damage, derived openly from the stock datasheet and tube dimensions (estimates, not measured onsets) ----
# NewAge Silcon Die B tear strength 120 ppi = 21.0 N/mm; nominal wall 1/16 in = 1.5875 mm.
TEAR_STRENGTH_N_PER_MM=21.0; WALL_T_MM=1.5875
CUT_N=TEAR_STRENGTH_N_PER_MM*WALL_T_MM   # 33.3 N: one finger pad pressed through the full wall section
RIM_N=CUT_N/2                            # 16.7 N: a cut end is supported on one side only (assumption)
OFF_AXIS=.004; ANGLE=15.                 # lever geometry; tear also needs >= CUT_N current native bore load
HAND_N=2*CUT_N                           # 66.7 N: all links of one hand together (both pads at CUT_N)
# ---- First-release bond of the cast wall to its core (Step 3b: a quarter turn frees the cast, then withdraw) ----
# Estimate, not measured: a silicone/steel bond of ~43.0 kPa shear over the 12.7 x 170 mm bore (6,783 mm2) gives
# BOND_TWIST = 43.0 kPa x 6.783e-3 m2 x 6.35 mm = 1.85 N m and BOND_AXIAL = 1.85/0.00635 = 292 N.
# The bond shears progressively as the wall turns: its torque rises linearly with twist and it lets go only at
# BOND_TURN degrees. Simulated one-hand controls: one hand turned a 1.40 N m bond with 58 N total grip (pad friction
# plus pressure on the 12 wall flats) but slipped on the wall at about 1.66 N m with 60 N grip; two hands stalled at
# about 2.06 N m. 1.85 N m lies between them.
BOND_SHEAR_PA=43.0e3
BOND_TWIST=BOND_SHEAR_PA*math.pi*.0127*.170*.00635      # 1.85 N m about the core axis at release
BOND_AXIAL=BOND_TWIST/.00635                             # 292 N native PhysX break force (pull/shear) -> tear
BOND_BEND=6.0                                            # N m native PhysX break torque (lever) -> tear
BOND_TURN=60.                                            # deg of twist at which the sheared bond lets go
BOND_K=BOND_TWIST/math.radians(BOND_TURN)                # 1.77 N m/rad progressive bond torsional stiffness
CLAMP_SLIP=.001; CLAMP_TURN=2.
JAW_TARGET=.001   # vise screw: jaw drive target 1 mm past first contact (2 mm sank the core 1.6 mm into the jaws)

# ---- Geometry (m) ----
X=.26; Z=TOP+.080
L=.170; CORE_L=.220; R=.0065             # wall length; core length and true radius (13.0 mm)
N_WALL=12; N_CORE=24                      # wall panels; core sides (faces at 0, 90, 180, 270 deg in both)
RI=.00635                               # unloaded stock bore12.7mm; interference is native compliant contact
CLEAR=RI-R                              # negative150um: nominal interference, no free hover clearance
RO=.00806                                 # loaded outside radius (wall volume kept)
CORE_Y=-.025; JAW_Y=-.115; JAW_W=.0254; JAW_H=.0508; JAW_T=.012
PINK=(.88,.56,.60); STEEL=(.62,.64,.67); GREY=(.30,.32,.35); DARK=(.18,.19,.21)

def ring(r_face,k,n):
    """Corner points (x,z) of side k of an n-gon whose face centre is at angle 2*pi*k/n, face distance r_face."""
    da=2*math.pi/n; rc=r_face/math.cos(da/2); a,b=(k-.5)*da,(k+.5)*da
    return (rc*math.cos(a),rc*math.sin(a)),(rc*math.cos(b),rc*math.sin(b))
def prism(poly,y0,y1,color,friction=None):
    p=[(x,y,z) for y in (y0,y1) for x,z in poly]; n=len(poly)
    faces=[tuple(range(n)),tuple(range(2*n-1,n-1,-1))]+[(i,i+n,(i+1)%n+n,(i+1)%n) for i in range(n)]
    return parts.hull(p,faces,color,friction=friction)
def wall():
    """12 box panels (native PhysX boxes collide robustly; thin convex hulls tunnelled through the core)."""
    out=[]; t=RO-RI; w=2*RO*math.tan(math.pi/N_WALL); rm=(RI+RO)/2
    for k in range(N_WALL):
        th=2*math.pi*k/N_WALL; ph=math.pi/2-th
        out.append(parts.box((w,L,t),at=(rm*math.cos(th),0.,rm*math.sin(th)),quat=(0.,math.sin(ph/2),0.,math.cos(ph/2)),color=PINK))
    return tuple(out)
CORE_POLY=[ring(R,k,N_CORE)[0] for k in range(N_CORE)]
# Wall: 12.1 cm3 of Shore A50 silicone at 1.15 g/cm3 = 14 g (estimate). Core: 13 mm x 220 mm 304 steel = 0.231 kg.
BODY=parts.Part('actuator',pos=(X,0.,Z-CLEAR),mass=.014,shapes=wall(),friction=WALL_FRICTION,contact_offset=.0002,report=('base',))
WALL_CENTRES=np.array([s.at for s in BODY.shapes]);WALL_ROT=np.array([s.R for s in BODY.shapes]);WALL_HALF=np.array([s.half for s in BODY.shapes])
CORE=parts.Part('core',pos=(X,CORE_Y,Z),mass=.231,shapes=(prism(CORE_POLY,-CORE_L/2,-.060,STEEL,friction=(JAW_MU,JAW_MU)),prism(CORE_POLY,-.060,CORE_L/2,STEEL,friction=(CORE_MU,CORE_MU))),
    friction=(CORE_MU,CORE_MU),contact_offset=.0002,report=('base',))
BODY=replace(BODY,max_depenetration_velocity=2.)
CORE=replace(CORE,max_depenetration_velocity=2.,shapes=(CORE.shapes[0],replace(CORE.shapes[1],compliance=(BORE_CONTACT_K,BORE_CONTACT_D))))
# Bench vise (PanaVise 201 class): head tilted so the jaws close vertically on the exposed core end. Bolted base,
# fixed lower jaw, screw-driven upper jaw (position drive 1 mm past contact = CLAMP_N). The core is a free body
# held only by jaw contact: gravity and the robot's loads reach the bench through the jaws.
_jb=Z-R-JAW_T-TOP                         # lower jaw bottom above the bench
VISE=parts.Part('bench_vise',pos=(X,JAW_Y,TOP),mass=None,shapes=(
    parts.box((.125,.125,.012),at=(0,0,.006),color=DARK),
    parts.box((.040,.040,_jb-.012),at=(0,0,.012+(_jb-.012)/2),color=GREY),
    parts.box((JAW_H,JAW_W,JAW_T),at=(0,0,_jb+JAW_T/2),color=DARK,friction=(JAW_MU,JAW_MU)),
    parts.box((.012,JAW_W,2*JAW_T+2*R+.016),at=(-JAW_H/2-.006,0,_jb+(2*JAW_T+2*R+.016)/2),color=GREY)),
    joints=(parts.Slider('jaw',shapes=(parts.box((JAW_H,JAW_W,JAW_T),at=(0,0,Z+R+JAW_T/2-TOP),color=DARK,friction=(JAW_MU,JAW_MU)),),
                         axis=(0.,0.,-1.),travel=.003,extend=.020,mass=.10,spring=CLAMP_N/.001,damping=200.,
                         max_effort=2*CLAMP_N,target=JAW_TARGET),),
    contact_offset=.002,report=('base','jaw'))
TRAY_XY={'A':(.32,.205),'B':(.345,.195)}
TRAY=parts.Part('parts_tray',pos=(*TRAY_XY['A'],TOP+.003),mass=.16,shapes=(parts.box((.100,.200,.006),color=(.45,.62,.73)),)+tuple(parts.box(s,at=p,color=(.45,.62,.73)) for s,p in [((.006,.200,.012),(-.053,0,.003)),((.006,.200,.012),(.053,0,.003)),((.112,.006,.012),(0,-.103,.003)),((.112,.006,.012),(0,.103,.003))]),friction=(.7,.5),contact_offset=.0002)
RULER=parts.Part('millimetre_rule',pos=(.44,.24,TOP+.001),shapes=(parts.box((.020,.220,.002),color=(.88,.87,.79)),)+tuple(parts.box((.012,.0005,.0003),at=(0,-.100+.010*k,.0012),color=(.12,.12,.12)) for k in range(21)))
BOTTLE=parts.Part('ipa_wash_bottle',pos=(.62,-.22,TOP+.050),shapes=(parts.cylinder(.030,.100,color=(.85,.88,.92)),parts.cylinder(.008,.030,at=(0,0,.060),color=(.95,.55,.20))))
OBJECTS=[BODY,CORE,VISE,TRAY,RULER,BOTTLE]
# Re-enrol the two interface bodies at the declared pose BEFORE each trial's final settle.
# This discards previous-trial roll/contact history at reset; it never places a body during a trial.
LOTS=lots.declare(OBJECTS,*[lots.Lot(n,place={'actuator':BODY.pos,'core':CORE.pos,'parts_tray':(*TRAY_XY[n],TOP+.003)},
    nominal_reset=('actuator','core')) for n in ('A','B')])
DEMO_TOUCHES=set()

def fresh(): return dict(known=False,zero=False,why='',physics=0,observed_physics=None,runtime=None,steps=0,pair=0.,peak_pair=0.,
                         clear=None,still=None,prev=None,t=0.,view=None,iview=None,jaw_valid=False,edge=0.,maxoff=0.,maxangle=0.,start=None,
                         engaged=False,iface={},jaw={},squeeze=0.,crush={},pad_peak=0.,
                         bond='pending',bond_path=None,bond_rel=None,twist=0.,max_twist=0.,peak_bond_torque=0.,
                         axial_slip=0.,turned=False,release_due=False,hand_crush={},core0=None)
S=fresh()
def fail(msg):
    if not S['zero']: S.update(zero=True,why=str(msg))
def validate_states(o,names=('actuator','core','parts_tray')):
    for name in names:
        st=o[name]
        for value,size in ((st.pos,3),(st.quat_xyzw,4),(st.lin_vel,3),(st.ang_vel,3),(st.low,3),(st.high,3)):
            if np.shape(value)!=(size,) or not np.isfinite(value).all():raise ValueError('invalid '+name+' state reading')
        if np.any(st.low>st.high) or abs(np.linalg.norm(st.quat_xyzw)-1.)>1e-3:raise ValueError('invalid '+name+' state geometry')
def axis_distance(b,c):
    """Exact minimum distance between the two finite centreline segments."""
    ba=parts._quat_to_matrix(b.quat_xyzw)[:,1];ca=parts._quat_to_matrix(c.quat_xyzw)[:,1]
    p0=b.pos-ba*L/2;p1=b.pos+ba*L/2;q0=c.pos-ca*CORE_L/2;q1=c.pos+ca*CORE_L/2
    def point_segment(p,a,z):
        u=z-a;t=float(np.clip((p-a)@u/(u@u),0.,1.))
        return float(np.linalg.norm(p-a-t*u))
    distances=[point_segment(p0,q0,q1),point_segment(p1,q0,q1),point_segment(q0,p0,p1),point_segment(q1,p0,p1)]
    u=p1-p0;v=q1-q0;w=p0-q0;a=float(u@u);bv=float(u@v);cc=float(v@v);d=float(u@w);e=float(v@w)
    den=a*cc-bv*bv
    if den>1e-14:
        ss=(bv*e-cc*d)/den;tt=(a*e-bv*d)/den
        if 0.<=ss<=1. and 0.<=tt<=1.:distances.append(float(np.linalg.norm(w+ss*u-tt*v)))
    return min(distances)
def relative(o):
    """Axial overlap of the wall on the core (m), with the radial offset and axis angle between them."""
    b,c=o['actuator'],o['core']; rb=parts._quat_to_matrix(b.quat_xyzw); rc=parts._quat_to_matrix(c.quat_xyzw)
    axis=rc[:,1]; d=b.pos-c.pos; axial=float(d@axis); radial=d-axial*axis
    dot=float(np.clip(rb[:,1]@axis,-1,1)); angle=math.degrees(math.acos(abs(dot)))
    lo=max(-CORE_L/2,axial-L/2); hi=min(CORE_L/2,axial+L/2); overlap=max(0.,hi-lo)
    if axis_distance(b,c)>RO+R+.001:overlap=0.   # side by side after release, not threaded on
    return axis,overlap,(lo+hi)/2,float(np.linalg.norm(radial)),angle

def on_wall(p,b):
    """Current contact lies on a wall panel, with a 1 mm allowance for compliant contact error."""
    local=(np.asarray(p)-b.pos)@parts._quat_to_matrix(b.quat_xyzw)
    panel=np.einsum('ni,nij->nj',local-WALL_CENTRES,WALL_ROT)
    return bool(np.any(np.all(np.abs(panel)<=WALL_HALF+.001,axis=1)))

def interface(o,contacts=(),dt=1/240):
    """Observe loaded native bore contact and latch damage; never apply a task-generated wrench.
    PhysX compliant collision and Coulomb friction supply the full interface load and equal reaction.
    """
    b,c=o['actuator'],o['core'];axis,overlap,mid,off,angle=relative(o)
    if overlap>.010:
        S['maxoff']=max(S['maxoff'],off);S['maxangle']=max(S['maxangle'],angle)
    axial=float((b.pos-c.pos)@axis)
    raw_overlap=max(0.,min(CORE_L/2,axial+L/2)-max(-CORE_L/2,axial-L/2))
    active=[]
    for p,n,sep in contacts:
        p=np.asarray(p,float)
        if p.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite([n,sep]).all() or n<0:
            raise ValueError('invalid bore contact')
        if n<CONTACT_MIN_N or sep>CONTACT_SEP:continue
        # Current native contact must be on BOTH reported body envelopes, not merely near the core.
        if any(np.any(p<st.low-.001) or np.any(p>st.high+.001) for st in (b,c)):continue
        if not on_wall(p,b):continue
        active.append((p,float(n)))
    load=float(sum(n for _,n in active))
    if load>=CUT_N and raw_overlap>.010 and (off>OFF_AXIS or angle>ANGLE):
        fail('tear: wall levered off axis against >=33.3 N native core contact')
    S['engaged']=bool(active and overlap>0.)
    S['contact_n']=float(sum(n for _,n in active))
    return np.zeros(3),np.zeros(3),c.pos+axis*mid,overlap

def rim_damage(rim):
    """Any one hand link pressing within 5 mm of a cut end above RIM_N (16.7 N, half the derived 33.3 N)."""
    for force in rim.values():
        if not np.isfinite(force) or force<0:raise ValueError('invalid rim contact')
        if force>RIM_N:fail('tear: cut-end contact above 16.7 N');return True
    return False

def contact_view():
    from isaaclab_physx.physics import PhysxManager
    from isaaclab.sim.utils import get_current_stage
    from pxr import Usd,UsdPhysics
    stage=get_current_stage()
    arm_paths=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()) if any(v in str(p.GetPath()) for v in ('/LeftArm/','/RightArm/')) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not arm_paths:raise ValueError('missing arm contact paths')
    S['contact_paths']=arm_paths
    return PhysxManager.get_physics_sim_view().create_rigid_contact_view([parts._PATHS['actuator'],parts._PATHS['core']],filter_patterns=[arm_paths,arm_paths],max_contact_data_count=512)
def arr(x):return np.asarray(x.numpy() if hasattr(x,'numpy') else x)
def damage_contacts(o,dt):
    """Any invalid contact reading permanently zeros this trial, even after recovery."""
    try:
        _damage_contacts(o,dt)
    except Exception as e:
        S['known']=False
        fail('invalid contact data: '+str(e))

def _damage_contacts(o,dt):
    if not np.isfinite(dt) or dt<=0:raise ValueError('invalid contact timestep')
    if np.shape(S.get('org'))!=(3,) or not np.isfinite(S['org']).all():raise ValueError('invalid contact origin')
    if S['view'] is None:S['view']=contact_view()
    forces,points,normals,seps,counts,starts=[arr(a) for a in S['view'].get_contact_data(dt=dt)]
    paths=S.get('contact_paths')
    if not paths or counts.shape!=(2,len(paths)) or starts.shape!=counts.shape:raise ValueError('incomplete contact index table')
    if not np.isfinite(counts).all() or not np.isfinite(starts).all():raise ValueError('nonfinite contact indices')
    if np.any(counts<0) or np.any(counts!=np.floor(counts)) or np.any(starts!=np.floor(starts)):raise ValueError('invalid contact indices')
    forces=forces.reshape(-1);points=points.reshape(-1,3);normals=normals.reshape(-1,3);seps=seps.reshape(-1)
    if len({len(forces),len(points),len(normals),len(seps)})!=1:raise ValueError('contact buffers disagree')
    squeeze={};pad={};edge=0.;edge_points=[];rim={}; b=o['actuator'];r=parts._quat_to_matrix(b.quat_xyzw);org=S['org'];used=np.zeros(len(forces),dtype=bool)
    for k in range(2):
        for j,(n,s) in enumerate(zip(counts[k],starts[k])):
            n,s=int(n),int(s)
            if n==0:continue
            if n<0 or s<0 or s+n>len(forces):raise ValueError('contact buffer incomplete')
            if used[s:s+n].any():raise ValueError('overlapping active contact ranges')
            used[s:s+n]=True
            if not all(np.isfinite(a[s:s+n]).all() for a in (forces,points,normals,seps)):raise ValueError('nonfinite contact reading')
            if np.any(np.abs(np.linalg.norm(normals[s:s+n],axis=1)-1.)>1e-3):raise ValueError('contact normals are not unit vectors')
            st=o['actuator' if k==0 else 'core'];world=points[s:s+n]-org
            if np.any(world<st.low-.005) or np.any(world>st.high+.005):raise ValueError('contact point outside reported part bounds')
            if k==1:continue
            loaded=(np.abs(forces[s:s+n])>=CONTACT_MIN_N)&(seps[s:s+n]<=CONTACT_SEP)
            loaded &= np.array([on_wall(p,b) for p in world])
            if not loaded.any():continue
            fs=np.abs(forces[s:s+n])[loaded]; path=paths[j]
            arm='LeftArm' if '/LeftArm/' in path else 'RightArm'
            squeeze[arm]=squeeze.get(arm,0.)+float(fs.sum()); pad[path]=pad.get(path,0.)+float(fs.sum())
            loc=(points[s:s+n][loaded]-org-b.pos)@r
            if not np.isfinite(loc).all():raise ValueError('nonfinite local contact position')
            hit=np.abs(loc[:,1])>L/2-.005   # within 5 mm of a cut end
            edge+=float(fs[hit].sum())
            if hit.any():
                rim[path]=rim.get(path,0.)+float(fs[hit].sum())
                edge_points.append(dict(part=path,local=loc[hit].tolist(),force=fs[hit].tolist()))
    S['edge']=max(S['edge'],edge); S['rim_forces']=rim; S['hand_squeeze']=squeeze
    if rim_damage(rim) and 'edge_event' not in S:S['edge_event']=edge_points
    total=sum(squeeze.values()); S['squeeze']=max(S['squeeze'],total); S['pad_forces']=pad
    S['pad_peak']=max([S['pad_peak']]+list(pad.values()))
    # One hand link pressing the wall above CUT_N; time-qualified so one solver impulse is not a tear.
    S['crush']={p:(S['crush'].get(p,0.)+dt if f>CUT_N else 0.) for p,f in pad.items()}
    if any(v>.10 for v in S['crush'].values()):fail('tear: one hand link pressed the wall above 33.3 N for 0.1 s')
    S['hand_crush']={a:(S['hand_crush'].get(a,0.)+dt if f>HAND_N else 0.) for a,f in squeeze.items()}
    if any(v>.10 for v in S['hand_crush'].values()):fail('tear: one hand pressed the wall above 66.7 N in total for 0.1 s')

def interface_views():
    """Native bore, vise and reverse bore contacts; jaw readings also qualify the clamp."""
    from isaaclab_physx.physics import PhysxManager
    sv=PhysxManager.get_physics_sim_view(); v=parts._PATHS['bench_vise']
    return (sv.create_rigid_contact_view([parts._PATHS['actuator']],filter_patterns=[[parts._PATHS['core']]],max_contact_data_count=256),
            sv.create_rigid_contact_view([parts._PATHS['core']],filter_patterns=[[v+'/base',v+'/jaw']],max_contact_data_count=256),
            sv.create_rigid_contact_view([parts._PATHS['core']],filter_patterns=[[parts._PATHS['actuator']]],max_contact_data_count=256))
def native_contacts(dt):
    """Only active ranges are read: unused GPU buffer slots are undefined."""
    if S['iview'] is None:S['iview']=interface_views()
    out={};bore=[]
    for key,view in zip(('wall_on_core','jaws_on_core','core_on_wall'),S['iview']):
        fs,pts,ns,seps,counts,starts=[arr(a) for a in view.get_contact_data(dt=dt)]
        fs=fs.reshape(-1);pts=pts.reshape(-1,3);ns=ns.reshape(-1,3);seps=seps.reshape(-1)
        expected=(1,2 if key=='jaws_on_core' else 1)
        if counts.shape!=expected or starts.shape!=expected:raise ValueError('incomplete interface index table')
        if len({len(fs),len(pts),len(ns),len(seps)})!=1:raise ValueError('interface buffer lengths')
        if not np.isfinite(counts).all() or not np.isfinite(starts).all():raise ValueError('interface index finite')
        rows=[];seen=set();by_filter=[];loaded_by_filter=[];normal_net=np.zeros(3);normal_moment=np.zeros(3)
        for countrow,startrow in zip(counts,starts):
            for count,first in zip(countrow,startrow):
                if count!=int(count) or first!=int(first) or count<0:raise ValueError('interface indices')
                count,first=int(count),int(first);total=0.;loaded_total=0.
                if count and (first<0 or first+count>len(fs)):raise ValueError('interface buffer bounds')
                for j in range(first,first+count):
                    if j in seen:raise ValueError('overlapping interface ranges')
                    seen.add(j)
                    if not all(np.isfinite(x).all() for x in (fs[j],pts[j],ns[j],seps[j])):raise ValueError('nonfinite interface')
                    if abs(np.linalg.norm(ns[j])-1.)>1e-3:raise ValueError('interface normal')
                    force=float(abs(fs[j]));p=pts[j]-S['org'];sep=float(seps[j])
                    normal_net+=float(fs[j])*ns[j];normal_moment+=np.cross(p,float(fs[j])*ns[j])
                    total+=force;rows.append((p,force,sep))
                    if force>=CONTACT_MIN_N and sep<=CONTACT_SEP:loaded_total+=force
                by_filter.append(total);loaded_by_filter.append(loaded_total)
        out[key]=dict(normal_sum=sum(r[1] for r in rows),points=len(rows),by_filter=by_filter,loaded_by_filter=loaded_by_filter,min_sep=min((r[2] for r in rows),default=None),loaded=sum(1 for _,n,sep in rows if n>=CONTACT_MIN_N and sep<=CONTACT_SEP))
        out[key]['normal_net']=normal_net.tolist()
        out[key]['normal_moment_world']=normal_moment.tolist()
        if key in ('wall_on_core','core_on_wall'):
            ff,fp,fc,fi=[arr(a) for a in view.get_friction_data(dt=dt)]
            ff=ff.reshape(-1,3);fp=fp.reshape(-1,3)
            if fc.shape!=(1,1) or fi.shape!=(1,1) or len(ff)!=len(fp):raise ValueError('friction buffer shape')
            count,first=float(fc[0,0]),float(fi[0,0])
            if not np.isfinite([count,first]).all() or count!=int(count) or first!=int(first) or count<0:raise ValueError('friction indices')
            count,first=int(count),int(first)
            if count and (first<0 or first+count>len(ff)):raise ValueError('friction buffer bounds')
            forces=ff[first:first+count];points=fp[first:first+count]-S['org']
            if not np.isfinite(forces).all() or not np.isfinite(points).all():raise ValueError('invalid native friction')
            out[key]['friction_net']=forces.sum(axis=0).tolist()
            out[key]['friction_moment_world']=np.cross(points,forces).sum(axis=0).tolist()
            out[key]['friction_points']=count
        if key=='wall_on_core':bore=rows
    S['iface']=out
    b,c=(out[k] for k in ('wall_on_core','core_on_wall'))
    bf=np.array(b['normal_net'])+np.array(b['friction_net']);cf=np.array(c['normal_net'])+np.array(c['friction_net'])
    bm=np.array(b['normal_moment_world'])+np.array(b['friction_moment_world']);cm=np.array(c['normal_moment_world'])+np.array(c['friction_moment_world'])
    S['native_wrench']=dict(body_force=bf.tolist(),core_force=cf.tolist(),body_moment_world=bm.tolist(),core_moment_world=cm.tolist(),net_force=(bf+cf).tolist(),net_moment_world=(bm+cm).tolist())
    S['peak_native_axial']=max(S.get('peak_native_axial',0.),abs(float(bf[1])))
    S['peak_reaction_error']=max(S.get('peak_reaction_error',0.),float(np.linalg.norm(bf+cf)))
    S['jaw_valid']=len(out['jaws_on_core']['loaded_by_filter'])==2 and min(out['jaws_on_core']['loaded_by_filter'])>.1
    return bore

# ---- Bond: a native PhysX revolute constraint between core and wall, authored fresh for every trial ----
# Its axis is the core axis. Axial pull, radial shear and lever loads are carried rigidly up to the native break
# force/torque (BOND_AXIAL/BOND_BEND); PhysX breaking it that way is a tear. Twist about the axis acts on a
# torsional drive of stiffness BOND_K; when the twist torque the hands apply reaches BOND_TWIST the bond lets go
# for good (the quarter turn breaking the cast free). The bond only resists the hands; it never moves the wall.
_GEN=[0]
def _stage():
    from isaaclab.sim.utils import get_current_stage
    return get_current_stage()
def bond_release():
    path=S.get('bond_path')
    if path:
        from pxr import UsdPhysics
        j=UsdPhysics.RevoluteJoint.Get(_stage(),path)
        if j:j.GetJointEnabledAttr().Set(False)
    S['bond_path']=None
def bond_author(sim,o):
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics
    stage=_stage();env=sim.env.unwrapped.scene.env_prim_paths[0]
    UsdGeom.Scope.Define(stage,env+'/softcore_bond');_GEN[0]+=1
    b,c=o['actuator'],o['core'];rb=parts._quat_to_matrix(b.quat_xyzw);rc=parts._quat_to_matrix(c.quat_xyzw)
    path=f"{env}/softcore_bond/g{_GEN[0]}"
    j=UsdPhysics.RevoluteJoint.Define(stage,path)
    j.CreateJointEnabledAttr().Set(False)
    j.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS['core'])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(parts._PATHS['actuator'])])
    j.CreateAxisAttr('Y')
    anchor=np.asarray(b.pos,float)
    for pos_attr,rot_attr,r,pp in ((j.CreateLocalPos0Attr,j.CreateLocalRot0Attr,rc,c.pos),(j.CreateLocalPos1Attr,j.CreateLocalRot1Attr,rb,b.pos)):
        pos_attr().Set(Gf.Vec3f(*(float(v) for v in r.T@(anchor-np.asarray(pp,float)))))
        x,y,z,w=parts._matrix_to_quat(r.T@rc)
        rot_attr().Set(Gf.Quatf(float(w),float(x),float(y),float(z)))
    j.CreateCollisionEnabledAttr().Set(True)
    j.CreateExcludeFromArticulationAttr().Set(True)
    j.CreateBreakForceAttr().Set(float(BOND_AXIAL))
    j.CreateBreakTorqueAttr().Set(float(BOND_BEND))
    drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');per_deg=math.pi/180
    drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(BOND_K*per_deg);drive.CreateDampingAttr(.02*per_deg)
    drive.CreateTargetPositionAttr(0.);drive.CreateMaxForceAttr(10.)
    j.CreateJointEnabledAttr().Set(True)
    S.update(bond='bonded',bond_path=path,bond_rel=dict(rel=rc.T@rb,off=rc.T@(np.asarray(b.pos,float)-np.asarray(c.pos,float))))
def rotvec(m):
    cth=float(np.clip((np.trace(m)-1)/2,-1,1));a=math.acos(cth)
    if a<1e-9:return np.zeros(3)
    return np.array([m[2,1]-m[1,2],m[0,2]-m[2,0],m[1,0]-m[0,1]])/(2*math.sin(a))*a
def bond_motion(o):
    """(twist about the core axis deg, off-axis bend deg, axial and radial offset m) relative to the bond pose."""
    b,c=o['actuator'],o['core'];rb=parts._quat_to_matrix(b.quat_xyzw);rc=parts._quat_to_matrix(c.quat_xyzw);r0=S['bond_rel']
    v=rotvec((rc.T@rb)@r0['rel'].T);twist=math.degrees(float(v[1]));bend=math.degrees(float(np.hypot(v[0],v[2])))
    d=rc.T@(np.asarray(b.pos,float)-np.asarray(c.pos,float))-r0['off']
    return twist,bend,float(d[1]),float(np.hypot(d[0],d[2]))
def bond_physics(o):
    if S['bond_rel'] is None:return
    twist,bend,axial,radial=bond_motion(o)
    S['twist']=twist;S['max_twist']=max(S['max_twist'],abs(twist));S['axial_slip']=max(S['axial_slip'],abs(axial))
    if S['bond']=='bonded':
        S['peak_bond_torque']=max(S['peak_bond_torque'],BOND_K*math.radians(abs(twist)))
        if abs(axial)>.0005 or radial>.0005 or bend>1.5:
            fail('tear: bonded wall pulled or levered off before the quarter turn freed it')
        elif abs(twist)>=BOND_TURN:S['release_due']=True;S['turned']=True

@parts.before_reset
def unbond(sim):
    bond_release()   # the previous trial's bond never holds the wall through the next reset
@parts.on_reset
def reset(sim):
    global S
    bond_release()
    S=fresh()
    S['runtime']=sim.embodiment
    sim.set_target('bench_vise','jaw',JAW_TARGET)   # tighten the vise screw (the stock reset may zero drive targets)
@parts.on_physics_step
def physics(sim):
    try:
        base=sim.env.unwrapped; S['org']=parts._np(base.scene.env_origins[0]);dt=float(base.physics_dt)
        if not np.isfinite(dt) or dt<=0:raise ValueError('invalid physics timestep')
        o={p.name:parts._read(base,p) for p in (BODY,CORE)}
        validate_states(o,('actuator','core'))
        contacts=native_contacts(dt)
        if S['steps']>0 and not core_clamped(o['core']):fail('core moved outside clamped pose or jaw contact missing')
        interface(o,contacts,dt); bond_physics(o); b,c=o['actuator'],o['core']
        S['physics']+=1
        if S['steps']>0:
            S['max_core_drift']=max(S.get('max_core_drift',0.),float(np.linalg.norm(c.pos-np.array(CORE.pos))))
            S['max_core_turn']=max(S.get('max_core_turn',0.),2*math.degrees(math.acos(float(np.clip(abs(c.quat_xyzw[3]),0,1)))))
            wrench=S['native_wrench'];force=np.array(wrench['body_force']);moment=np.array(wrench['body_moment_world'])-np.cross(c.pos,force)
            S['peak_native_torque']=max(S.get('peak_native_torque',0.),abs(float(moment@parts._quat_to_matrix(c.quat_xyzw)[:,1])))
            S['peak_moment_error']=max(S.get('peak_moment_error',0.),float(np.linalg.norm(wrench['net_moment_world'])))
            S['min_jaw_load']=min(S.get('min_jaw_load',float('inf')),min(S['iface']['jaws_on_core']['loaded_by_filter']))
        damage_contacts(o,dt)
        if os.environ.get('DEBUG_SUBSTEP') and S['physics']<400:
            with open(os.environ['DEBUG_SUBSTEP'],'a') as fh:fh.write(json.dumps(dict(n=S['physics'],b=b.pos.tolist(),bv=b.lin_vel.tolist(),c=c.pos.tolist(),eng=S['engaged'],why=S['why'],iface=S.get('iface')))+'\n')
    except Exception as e:S['known']=False;fail('physics unavailable: '+str(e))

# Face samples, including interiors. Conservative 3 mm guard on 30 mm clearance.
_HAND={}
def local_hand(n):
    if n not in _HAND:
        from yam_arena import analysis
        chunks = []
        for h in analysis.link_hulls()[n]:
            chunks.append(h.vertices)
            _, plane_ids = np.unique(np.round(h.planes, 6), axis=0, return_index=True)
            for normal, dd in zip(h.planes[plane_ids, :3], h.planes[plane_ids, 3]):
                face = h.vertices[np.abs(h.vertices @ normal - dd) < 1e-6]
                if len(face) < 3: continue
                c = face.mean(0); u = face[np.argmax(np.linalg.norm(face - c, axis=1))] - c
                if np.linalg.norm(u) < 1e-12: continue
                u /= np.linalg.norm(u); v = np.cross(normal, u)
                uv = np.column_stack(((face - c) @ u, (face - c) @ v)); order = np.lexsort((uv[:, 1], uv[:, 0]))
                def turn(i, j, k):
                    aa = uv[j] - uv[i]; bb = uv[k] - uv[j]; return aa[0] * bb[1] - aa[1] * bb[0]
                lo = []; hi = []
                for ii_ in order:
                    while len(lo) > 1 and turn(lo[-2], lo[-1], ii_) <= 1e-12: lo.pop()
                    lo.append(int(ii_))
                for ii_ in order[::-1]:
                    while len(hi) > 1 and turn(hi[-2], hi[-1], ii_) <= 1e-12: hi.pop()
                    hi.append(int(ii_))
                face = face[lo[:-1] + hi[:-1]]
                for aa, bb in zip(face, np.roll(face, -1, axis=0)):
                    nn = max(1, math.ceil(max(np.linalg.norm(aa - c), np.linalg.norm(bb - c), np.linalg.norm(bb - aa)) / .004))
                    ii, jj = np.meshgrid(np.arange(nn + 1), np.arange(nn + 1), indexing='ij')
                    valid = ii + jj <= nn
                    chunks.append(c + (aa - c) * ii[valid, None] / nn + (bb - c) * jj[valid, None] / nn)
        all_pts = np.concatenate(chunks)
        _, idx = np.unique(np.round(all_pts / .002).astype(np.int64), axis=0, return_index=True)
        _HAND[n] = all_pts[np.sort(idx)]
    return _HAND[n]

def hand_points(sim):
    base=sim.env.unwrapped; org=parts._np(base.scene.env_origins[0]);out=[]
    for arm in ('left_robot','right_robot'):
        a=base.scene[arm];p=parts._np(a.data.body_link_pose_w)[0]
        for name in ('link5','gripper','tip_left','tip_right'):
            local_hand(name)
            s=p[list(a.body_names).index(name)];out.append(_HAND[name]@parts._quat_to_matrix(s[3:]).T+s[:3]-org)
    h=np.concatenate(out)
    if not np.isfinite(h).all():raise ValueError('invalid hand state')
    return h

def hand_clear(h,o,margin=.033):
    """Every hand point at least 30 mm (plus 3 mm guard) from the wall's and the core's outer envelopes."""
    for name,half in (('actuator',np.array([RO,L/2,RO])),('core',np.array([R,CORE_L/2,R]))):
        s=o[name];r=parts._quat_to_matrix(s.quat_xyzw);loc=(h-s.pos)@r
        if np.linalg.norm(np.maximum(np.abs(loc)-half,0),axis=1).min()<margin:return False
    return True

def observe(o,h,t):
    if not np.isfinite(t) or t<0:S['known']=False;fail('invalid observation time');return
    if h is None or len(h)==0 or not np.isfinite(h).all():S['known']=False;fail('hand state unavailable');return
    try:validate_states(o)
    except (KeyError,ValueError,TypeError,AttributeError) as e:S['known']=False;fail('object state unavailable: '+str(e));return
    previous=S['observed_physics']
    if S['physics']<=0 or (previous is not None and S['physics']<=previous):
        S['known']=False;fail('physics/contact updates stopped');return
    S['observed_physics']=S['physics']
    for name in ('actuator','core','parts_tray'):
        if o[name].low[2]<TOP-.01:fail('part dropped off bench')
    if S['core0'] is None:S['core0']=(np.asarray(o['core'].pos,float).copy(),np.asarray(o['core'].quat_xyzw,float).copy())
    if not core_clamped(o['core']):fail('core moved outside clamped pose or jaw contact missing')
    chunks=[]
    for name in ('actuator','core','parts_tray'):
        st=o[name]; spec={'actuator':BODY,'core':CORE,'parts_tray':TRAY}[name]
        for _,c,r,sz in spec.world_boxes({'base':(st.pos,parts._quat_to_matrix(st.quat_xyzw))}):
            chunks.append(c+np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*(sz/2)@r.T)
    snap=np.concatenate(chunks)
    if S['prev'] is None or np.max(np.linalg.norm(snap-S['prev'],axis=1))>.001:S['still']=t;S['prev']=snap.copy()
    clear=hand_clear(h,o);S['clear']=(S['clear'] if S['clear'] is not None else t) if clear else None
    S.update(known=True,t=t,steps=S['steps']+1)

def bench_supported(st):
    centre=np.asarray(TABLE_CENTER[:2]);half=np.asarray(TABLE_SIZE[:2])/2
    return bool(abs(float(st.low[2])-TOP)<=.002+GEOM_EPS and np.all(st.low[:2]>=centre-half-GEOM_EPS) and np.all(st.high[:2]<=centre+half+GEOM_EPS))

def core_clamped(c):
    """The core is still where the vise holds it: within 1 mm and 2 degrees of its clamped pose at the trial's first
    observation (after the reset settle), and both jaws loaded."""
    ref=S.get('core0')
    if ref is None:return False
    p0,q0=ref
    if np.linalg.norm(np.asarray(c.pos)-p0)>CLAMP_SLIP+GEOM_EPS:return False
    q=np.asarray(c.quat_xyzw,float);turn=2*math.degrees(math.acos(float(np.clip(abs(q@q0),0,1))))
    return bool(S.get('jaw_valid',False) and turn<=CLAMP_TURN+1e-9)

def evaluate(o):
    if not S['known'] or S['zero'] or S['physics']==0 or S['steps']==0:return 0.
    b,c,tr=o['actuator'],o['core'],o['parts_tray']; axis,overlap,_,_,_=relative(o)
    if not core_clamped(c):return 0.
    if S.get('bond')!='turn' or not S.get('turned'):return 0.   # the quarter turn must have freed the wall
    # Complete separation, including tilted ends: bounding envelopes must be 5 mm apart.
    gap=np.linalg.norm(np.maximum(np.maximum(b.low-c.high,c.low-b.high),0))
    if overlap>GEOM_EPS or gap<.005-GEOM_EPS:return 0.
    # Every tier requires release, support and stillness.
    if S['clear'] is None or S['still'] is None or S['t']-max(S['clear'],S['still'])<1.-1e-6:return 0.
    r=parts._quat_to_matrix(tr.quat_xyzw)
    upright=bool(r[2,2]>=math.cos(math.radians(5.)))
    world_points=body_vertices(b);pp=(world_points-tr.pos)@r
    on_floor=abs(float(pp[:,2].min())-.003)<=.002+GEOM_EPS
    contained=bool(np.max(np.abs(pp[:,0]))<=.050+GEOM_EPS and np.max(np.abs(pp[:,1]))<=.100+GEOM_EPS)
    in_cavity=bool(upright and bench_supported(tr) and tr.resting(.008,.15) and on_floor and contained)
    if not (bench_supported(b) or in_cavity) or not b.resting(.008,.15):return 0.
    if not in_cavity:return .25
    if S['t']-max(S['clear'],S['still'])<2.-1e-6:return .5
    centre=(b.pos-tr.pos)@r
    if abs(centre[0])>.010+GEOM_EPS or abs(centre[1])>.010+GEOM_EPS:return .75
    return 1.
def score(o):
    try:
        validate_states(o)
        if S['physics']>0:
            if S['observed_physics']!=S['physics']:raise ValueError('physics not covered by a current observation')
            runtime=S.get('runtime')
            if runtime is not None:
                now=float(runtime._t)
                if not np.isfinite(now) or abs(now-S['t'])>1e-8:raise ValueError('stale observation clock')
        return float(evaluate(o))
    except (KeyError,ValueError,TypeError,AttributeError) as e:
        S['known']=False;fail('scoring state unavailable: '+str(e));return 0.
def success(o):return score(o)==1.

def body_vertices(st):
    """All actual wall-panel vertices, transformed into world coordinates."""
    r=parts._quat_to_matrix(st.quat_xyzw); corners=np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)],float)
    return np.concatenate([st.pos+(np.asarray(s.at)+(corners*s.half)@s.R.T)@r.T for s in BODY.shapes])

@parts.on_step
def step(sim):
    if S.get('last_t')==sim.t:return
    S['last_t']=sim.t
    try:
        o=sim.states()
        if S['bond']=='pending':bond_author(sim,o)
        if S['bond']=='bonded' and S['release_due']:
            bond_release();S['bond']='turn'
            print(f"[softcore] t={sim.t:.2f}s bond released by twist {S['peak_bond_torque']:.3f} N m",file=sys.stderr,flush=True)
        h=hand_points(sim);observe(o,h,float(sim.t))
        if S['start'] is None:S['start']={n:o[n].pos.tolist() for n in ('actuator','core','parts_tray')}
        log=str(pathlib.Path(os.environ['TRACE_OUT']).with_name('release.jsonl')) if os.environ.get('TRACE_OUT') else os.environ.get('RELEASE_LOG')
        if log and S['steps']%5==0:
            row={k:S.get(k) for k in ('known','zero','why','physics','steps','pair','peak_pair','edge','squeeze','maxoff','maxangle','clear','still','start','engaged','iface','jaw_valid','torque','contact_n','slip','twist','hand_squeeze','rim_forces','edge_event','max_core_drift','max_core_turn','native_wrench','peak_native_axial','peak_reaction_error','peak_native_torque','peak_moment_error','min_jaw_load','bond','twist','max_twist','peak_bond_torque','axial_slip','turned','pad_peak','pad_forces')}
            row.update(task_sha256=SOURCE_SHA256,t=float(sim.t),lot=lots.current().name,score=score(o),clamped=core_clamped(o['core']),overlap=relative(o)[1],
                       objects={n:dict(pos=o[n].pos.tolist(),quat=o[n].quat_xyzw.tolist(),lin_vel=o[n].lin_vel.tolist(),ang_vel=o[n].ang_vel.tolist()) for n in ('actuator','core','parts_tray')})
            if 'bench_vise' in o:row['jaw']=getattr(o['bench_vise'],'joints',{})
            base=sim.env.unwrapped;row['grippers']={}
            for arm in ('left_robot','right_robot'):
                ar=base.scene[arm];p=parts._np(ar.data.body_link_pose_w)[0]
                row['grippers'][arm]={n:p[list(ar.body_names).index(n),:3].tolist() for n in ('gripper','tip_left','tip_right')}
                row['grippers'][arm]['joints']=parts._np(ar.data.joint_pos)[0].tolist()
            with open(log,'a') as f:f.write(json.dumps(row)+'\n')
    except Exception as e:S['known']=False;fail('observer unavailable: '+str(e))

GRIP_DZ=.008      # TCP above the wall axis: pads straddle the wall's equator
TWIST_HOLD=-.0023 # grip for freeing the bond: 5.8 mm x 5000 N/m = about 29 N per pad (below the 33.3 N cut limit)
HOLD=-.0050       # grip after the bond is free (regrasp, withdrawal, carry): about 15 N per pad
OPEN=-.020
ONE_HAND_HOLD=-.0019  # single-hand control: about 31 N per pad, just under the 33.3 N cut limit
def demo():
    """Open-loop supported quarter turn and withdrawal, with the core held only by vise-jaw contacts."""
    tx,ty=lots.current().pos('parts_tray')[:2]
    zg=Z+GRIP_DZ; up=Z+.040; ly,ry=.040,-.040; stroke=L+.040
    zc=TOP+.006+RO+.001
    def w(d,x,y,z,f,label,arm,roll=15.):
        yaw=-math.pi/2 if arm=='left' else math.pi/2
        rot=parts._rot((0,1,0),math.radians(roll))
        offset=rot@np.array([0.,0.,GRIP_DZ])-np.array([0.,0.,GRIP_DZ])
        pose=tuple(np.array([x,y,z])+offset)
        orientation=rot@tilt.rotation(yaw,25.,yaw)
        return Waypoint(d,pose,f,label,tilt._TiltedYaw(yaw,orientation))
    plans={}
    for arm,y in (('left',ly),('right',ry)):
        plans[arm]=[
            w(2,X,y,up,OPEN,'above wall',arm),
            w(2,X,y,zg,OPEN,'straddle wall',arm),
            w(1,X,y,zg,TWIST_HOLD,'grip wall away from cut ends',arm),
            w(1,X,y,zg,TWIST_HOLD,'support wall',arm),
            w(3,X,y,zg,TWIST_HOLD,'turn wall first30degrees',arm,-15.),
            w(3,X,y,zg,TWIST_HOLD,'turn wall next30degrees',arm,-45.),
            w(3,X,y,zg,TWIST_HOLD,'complete90degree turn',arm,-75.),
            w(2,X,y,zg,TWIST_HOLD,'support after turn',arm,-75.),
        ]
        # Reorient each OPEN hand while the other holds the turned wall. Do not roll the wall back.
        if arm=='left':
            plans[arm] += [w(1,X,y,zg,OPEN,'left releases for regrasp',arm,-75.),
                w(3,X,y,zg,OPEN,'left reorients open fingers',arm),
                w(1,X,y,zg,HOLD,'left regrips turned wall',arm),
                w(1,X,y,zg,HOLD,'left takes support',arm),
                w(6,X,y,zg,HOLD,'left supports while right reorients',arm)]
        else:
            plans[arm] += [w(6,X,y,zg,HOLD,'right supports while left reorients',arm,-75.),
                w(1,X,y,zg,OPEN,'right releases for regrasp',arm,-75.),
                w(3,X,y,zg,OPEN,'right reorients open fingers',arm),
                w(1,X,y,zg,HOLD,'right regrips turned wall',arm),
                w(1,X,y,zg,HOLD,'both hands support before withdrawal',arm)]
        plans[arm] += [w(10,X,y+stroke,zg,HOLD,'withdraw with both hands along core axis',arm),
            w(2,X,y+stroke,zg,HOLD,'support after complete separation',arm)]
    plans['left'] += [
        w(3,X,ly+stroke,zg,HOLD,'wait for right hand to release and clear', 'left'),
        w(1,X,ly+stroke,zg+.010,HOLD,'lift clear', 'left'),
        w(3,tx,ty+ly,zc+GRIP_DZ+.040,HOLD,'carry over tray','left'),
        w(2,tx,ty+ly,zc+GRIP_DZ+.002,HOLD,'lower onto tray floor','left'),
        w(1,tx,ty+ly,zc+GRIP_DZ+.002,OPEN,'release in tray','left'),
        w(2,tx,ty+ly,TOP+.15,OPEN,'left hand clear','left',0.),
        w(4,tx,ty+ly,TOP+.15,OPEN,'settle','left',0.),
    ]
    plans['right'] += [
        w(1,X,ry+stroke,zg,OPEN,'let go after separation','right',0.),
        w(2,X,ry+stroke-.020,TOP+.17,OPEN,'right hand up before returning','right',0.),
        w(2,.24,-.12,TOP+.17,OPEN,'right hand above vise','right',0.),
        w(20,.20,-.20,TOP+.17,OPEN,'park','right',0.),
    ]
    if os.environ.get('SOFTCORE_ONE_HAND')=='1':
        plans['left']=[replace(p,finger=ONE_HAND_HOLD) if p.finger in (HOLD,TWIST_HOLD) else p for p in plans['left']]
        plans['right']=[w(55.,.20,-.20,TOP+.15,OPEN,'one-hand control: right hand stays parked','right',0.)]
    return plans

parts._softcore_namespace=globals()
