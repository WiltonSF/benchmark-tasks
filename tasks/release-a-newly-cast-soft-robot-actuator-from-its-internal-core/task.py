"""An actuator fabrication bench. A pale cured semilunar silicone actuator surrounds a steel half-round casting
core supported horizontally on two small saddles. The core protrudes toward the right arm. Open vise jaws, printed
mold halves, a casting mat and a scrap cup sit behind the assembly. A parts tray and millimetre ruler sit to the left.
"""
from __future__ import annotations
import hashlib, json, math, os, pathlib, sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.station import TABLE_CENTER, TABLE_SIZE
from yam_arena.demo import Waypoint, FINGER_OPEN, FINGER_CLOSED
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.05
SOURCE_SHA256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
GEOM_EPS=1e-6
X=.32; Z=TOP+.080; L=.170; CORE_L=.220; R=.00635; RI=.00655; RO=.00935
PINK=(.85,.58,.59); STEEL=(.56,.58,.61); GREY=(.32,.34,.37)
# Declared estimates: 400 N/m bristle adhesion plus 0.2 N s/m damping,
# capped at 6 N times overlap fraction; radial foundation 1500 N/m capped 12 N.
DRAG=6.; RADIAL_K=1500.; FORCE_CAP=12.; TEAR_FORCE=18.; OFF_AXIS=.004; ANGLE=15.

def prism(poly,y0,y1,color,friction=None):
    p=[(x,y,z) for y in (y0,y1) for x,z in poly]; n=len(poly)
    faces=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    return parts.hull(p,faces,color,friction=friction)
def shell():
    out=[parts.box((2*RO,L,.003),at=(0,0,-.0015),color=PINK)]
    for k in range(12):
        a,b=k*math.pi/12,(k+1)*math.pi/12
        out.append(prism([(RI*math.cos(a),RI*math.sin(a)),(RO*math.cos(a),RO*math.sin(a)),
                          (RO*math.cos(b),RO*math.sin(b)),(RI*math.cos(b),RI*math.sin(b))],-L/2,L/2,PINK))
    return tuple(out)
BODY=parts.Part('actuator',pos=(X,0.,Z),mass=.024,shapes=shell(),friction=(.7,.55),contact_offset=.0001,report=('base',))
CORE_POLY=[(R*math.cos(k*math.pi/16),R*math.sin(k*math.pi/16)) for k in range(17)]
CORE=parts.Part('core',pos=(X,-.025,Z),mass=.109,shapes=(prism(CORE_POLY,-CORE_L/2,-.060,STEEL,friction=(.8,.6)),prism(CORE_POLY,-.060,CORE_L/2,STEEL,friction=(.12,.10))),friction=(.12,.10),contact_offset=.0001,report=('base',))

# Saddles carry the native flat underside, not a welded/grasp-assisted assembly.
SADDLES=parts.Part('casting_saddles',pos=(X,0.,TOP),shapes=tuple(parts.box((.010,.012,.077),at=(0,y,.0385),color=GREY) for y in (-.065,.065)),friction=(.5,.4),contact_offset=.0001)
TRAY_XY={'A':(.32,.205),'B':(.345,.195)}
TRAY=parts.Part('parts_tray',pos=(*TRAY_XY['A'],TOP+.003),mass=.16,shapes=(parts.box((.100,.200,.006),color=(.45,.62,.73)),)+tuple(parts.box(s,at=p,color=(.45,.62,.73)) for s,p in [((.006,.200,.012),(-.053,0,.003)),((.006,.200,.012),(.053,0,.003)),((.112,.006,.012),(0,-.103,.003)),((.112,.006,.012),(0,.103,.003))]),friction=(.7,.5),contact_offset=.0002)
MAT=parts.Part('casting_mat',pos=(.56,0,TOP+.001),shapes=(parts.box((.15,.35,.002),color=(.63,.67,.65)),))
VISE=parts.Part('open_bench_vise',pos=(.56,-.18,TOP),shapes=(parts.box((.10,.10,.018),at=(0,0,.009),color=GREY),parts.box((.10,.025,.075),at=(0,-.034,.04),color=GREY),parts.box((.10,.025,.075),at=(0,.034,.04),color=GREY)))
MOLD=parts.Part('printed_mold_halves',pos=(.56,.10,TOP+.016),shapes=(parts.box((.045,.190,.032),at=(-.03,0,0),color=(.72,.71,.49)),parts.box((.045,.190,.032),at=(.03,0,0),color=(.72,.71,.49))))
RULER=parts.Part('millimetre_rule',pos=(.44,.29,TOP+.001),shapes=(parts.box((.020,.220,.002),color=(.88,.87,.79)),)+tuple(parts.box((.012,.0005,.0003),at=(0,-.100+.010*k,.0012),color=(.12,.12,.12)) for k in range(21)))
CUP=parts.Part('scrap_cup',pos=(.65,-.27,TOP+.020),shapes=(parts.cylinder(.025,.04,color=(.62,.62,.64)),))
OBJECTS=[BODY,CORE,SADDLES,TRAY,MAT,VISE,MOLD,RULER,CUP]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'parts_tray':TRAY.pos}),lots.Lot('B',place={'parts_tray':(*TRAY_XY['B'],TOP+.003)}))
DEMO_TOUCHES=set()

def fresh(): return dict(known=False,zero=False,why='',physics=0,observed_physics=None,runtime=None,steps=0,force=0.,peak=0.,work=0.,last_t=None,clear=None,still=None,prev=None,t=0.,view=None,edge=0.,load=0.,maxoff=0.,maxangle=0.,start=None)
S=fresh()
def fail(msg):
    if not S['zero']: S.update(zero=True,why=str(msg))
def validate_states(o,names=('actuator','core','parts_tray')):
    for name in names:
        st=o[name]
        for value,size in ((st.pos,3),(st.quat_xyzw,4),(st.lin_vel,3),(st.ang_vel,3),(st.low,3),(st.high,3)):
            if np.shape(value)!=(size,) or not np.isfinite(value).all():raise ValueError('invalid '+name+' state reading')
        if np.any(st.low>st.high) or np.linalg.norm(st.quat_xyzw)<1e-12:raise ValueError('invalid '+name+' state geometry')
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
    b,c=o['actuator'],o['core']; rb=parts._quat_to_matrix(b.quat_xyzw); rc=parts._quat_to_matrix(c.quat_xyzw)
    axis=rb[:,1]; d=c.pos-b.pos; axial=float(d@axis); radial=d-axial*axis
    # The uncapped half-round profile mates in either end-for-end direction.
    dot=float(np.clip(rc[:,1]@axis,-1,1)); angle=math.degrees(math.acos(abs(dot)))
    # Same-size 170 mm cavity and 220 mm core, whose initial protrusion is 50 mm at -y.
    overlap=max(0.,min(L/2,axial+CORE_L/2)-max(-L/2,axial-CORE_L/2))
    # Axial projections alone can overlap across a large transverse gap after release.
    if axis_distance(b,c)>math.hypot(RO,.003)+R+.0002:overlap=0.
    return axis,axial,radial,angle,overlap

def resistance(o,dt=1/240):
    b,c=o['actuator'],o['core']; axis,axial,radial,angle,overlap=relative(o)
    raw_overlap=max(0.,min(L/2,axial+CORE_L/2)-max(-L/2,axial-CORE_L/2))
    previous=S.get('interface_engaged',False)
    # A lateral escape from an existing interface must still tear; spatial gating
    # must not turn that bypass into a force-free successful extraction.
    if previous and raw_overlap>.01 and (np.linalg.norm(radial)>OFF_AXIS or angle>ANGLE):fail('tear: off-axis extraction')
    active=overlap>0. and (previous or (angle<=ANGLE and axis_distance(b,c)<=RI+.0005))
    if not active:overlap=0.
    S['interface_engaged']=bool(active)
    v=np.asarray(c.lin_vel)-np.asarray(b.lin_vel); va=float(v@axis)
    # Elasto-plastic adhesion bristles integrate actual relative velocity. 400 N/m
    # corresponds to15 mm elastic axial stretch at6 N (8.8% of this body).
    limit=DRAG*min(1.,overlap/L)
    slip=S.get('slip',0.)+va*dt if overlap>0 else 0.
    slip=float(np.clip(slip,-limit/400.,limit/400.))
    S['slip']=slip
    axial_force=float(np.clip(-400.*slip-.20*va,-limit,limit))
    f=axial_force*axis
    rad=-RADIAL_K*radial if overlap>0 else np.zeros(3)
    if np.linalg.norm(rad)>FORCE_CAP:rad*=FORCE_CAP/np.linalg.norm(rad)
    f+=rad
    S['force']=float(np.linalg.norm(f));S['peak']=max(S['peak'],S['force'])
    S['maxoff']=max(S['maxoff'],float(np.linalg.norm(radial))) if overlap>.01 else S['maxoff']
    S['maxangle']=max(S['maxangle'],angle) if overlap>.01 else S['maxangle']
    if overlap>.01 and (np.linalg.norm(radial)>OFF_AXIS or angle>ANGLE):fail('tear: off-axis extraction')
    if S['force']>TEAR_FORCE:fail('tear: extraction load above 18 N')
    return f,overlap,va

def apply(asset,f,t):
    import torch
    if not np.isfinite(f).all() or not np.isfinite(t).all():raise ValueError('nonfinite applied wrench')
    ff=torch.tensor(np.asarray(f).reshape(1,1,3),device=asset.device,dtype=torch.float32)
    tt=torch.tensor(np.asarray(t).reshape(1,1,3),device=asset.device,dtype=torch.float32)
    asset.set_external_force_and_torque(ff,tt,is_global=True)
    asset.write_data_to_sim()

def rim_damage(rim):
    """2 N on EACH opposing fingertip means an unsupported rim pinch; 8 N single contact tears.
    The material datasheet >30 N/mm is not an assembly calibration. These conservative
    estimates distinguish a brief one-sided brush from actually gripping the thin cut wall.
    """
    for force in rim.values():
        if not np.isfinite(force) or force<0:raise ValueError('invalid rim contact')
        if force>8.:fail('tear: single rim contact above8 N');return True
    for arm in ('LeftArm','RightArm'):
        values=[sum(v for p,v in rim.items() if '/'+arm+'/' in p and p.endswith('/'+tip)) for tip in ('tip_left','tip_right')]
        if min(values)>2.:fail('tear: opposing fingers grip thin rim above2 N each');return True
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
    total=np.zeros(2);pull=np.zeros(3);edge=0.;edge_points=[];rim={}; b=o['actuator'];r=parts._quat_to_matrix(b.quat_xyzw);org=S['org'];used=np.zeros(len(forces),dtype=bool)
    for k in range(2):
        for j,(n,s) in enumerate(zip(counts[k],starts[k])):
            n,s=int(n),int(s)
            if n==0:continue
            if n<0 or s<0 or s+n>len(forces):raise ValueError('contact buffer incomplete')
            if used[s:s+n].any():raise ValueError('overlapping active contact ranges')
            used[s:s+n]=True
            # Capacity beyond these indexed ranges is unused storage, not a reading.
            if not all(np.isfinite(a[s:s+n]).all() for a in (forces,points,normals,seps)):raise ValueError('nonfinite contact reading')
            if np.any(np.abs(np.linalg.norm(normals[s:s+n],axis=1)-1.)>1e-3):raise ValueError('contact normals are not unit vectors')
            st=o['actuator' if k==0 else 'core'];world=points[s:s+n]-org
            if np.any(world<st.low-.005) or np.any(world>st.high+.005):raise ValueError('contact point outside reported part bounds')
            fs=np.abs(forces[s:s+n]);total[k]+=float(fs.sum())
            if k==1:pull+=np.sum(fs[:,None]*normals[s:s+n],axis=0)
            else:
                loc=(points[s:s+n]-org-b.pos)@r
                if not np.isfinite(loc).all():raise ValueError('nonfinite local contact position')
                # Gripping an unsupported cut rim (<5 mm from either end) can nick a 2 mm wall.
                hit=np.abs(loc[:,1])>L/2-.005
                edge+=float(fs[hit].sum())
                if hit.any():
                    path=S.get('contact_paths',['unknown']*len(counts[k]))[j]
                    rim[path]=rim.get(path,0.)+float(fs[hit].sum())
                    edge_points.append(dict(part=path,local=loc[hit].tolist(),force=fs[hit].tolist()))
    if not np.isfinite(total).all() or not np.isfinite(pull).all() or not np.isfinite(edge):raise ValueError('nonfinite contact aggregate')
    S['compression']=max(S.get('compression',0.),float(total[0])); S['edge']=max(S['edge'],edge); S['load']=max(S['load'],float(np.linalg.norm(pull)))
    if rim_damage(rim):
        if 'edge_event' not in S:S['edge_event']=edge_points
    S['rim_forces']=rim
    # Time-qualified to avoid treating one contact-solver impulse as a sustained material tear.
    S['overload']=S.get('overload',0.)+dt if np.linalg.norm(pull)>TEAR_FORCE else 0.
    S['crush']=S.get('crush',0.)+dt if total[0]>45. else 0.
    if S['overload']>.10:fail('tear: sustained core pull above 18 N')
    if S['crush']>.10:fail('tear: sustained body compression above 45 N')

@parts.on_reset
def reset(sim):
    global S
    S=fresh()
    S['runtime']=sim.embodiment
@parts.on_physics_step
def physics(sim):
    try:
        base=sim.env.unwrapped; S['org']=parts._np(base.scene.env_origins[0]);dt=float(base.physics_dt)
        if not np.isfinite(dt) or dt<=0:raise ValueError('invalid physics timestep')
        o={p.name:parts._read(base,p) for p in (BODY,CORE)}
        validate_states(o,('actuator','core'))
        f,overlap,va=resistance(o,dt); b,c=o['actuator'],o['core']; point=(b.pos+c.pos)/2
        apply(base.scene['core'],f,np.cross(point-c.pos,f));apply(base.scene['actuator'],-f,np.cross(point-b.pos,-f))
        axis=relative(o)[0]; vel=np.asarray(c.lin_vel)
        prev=S.get('core_velocity')
        demand=abs(float((CORE.mass*(vel-prev)/dt-f)@axis)) if prev is not None else 0.
        S['core_velocity']=vel.copy();S['pull_est']=demand;S['pull_peak']=max(S.get('pull_peak',0.),demand)
        S['pull_over']=S.get('pull_over',0.)+dt if demand>TEAR_FORCE else 0.
        if S['pull_over']>.10:fail('tear: inferred axial pull above 18 N for0.1s')
        S['applied_core']=f.tolist();S['applied_body']=(-f).tolist()
        S['physics']+=1;S['work']+=max(0.,-float(f@(c.lin_vel-b.lin_vel)))*dt
        damage_contacts(o,dt)
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
    from yam_arena import analysis
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
    # Body's filled oriented outer envelope is conservative (includes cavity).
    for name,half,offset in [('actuator',np.array([RO,L/2,(RO+.003)/2]),np.array([0,0,(RO-.003)/2])),('core',np.array([R,CORE_L/2,R/2]),np.array([0,0,R/2]))]:
        s=o[name];r=parts._quat_to_matrix(s.quat_xyzw);loc=(h-s.pos)@r-offset
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
        st=o[name]
        if st.low[2]<TOP-.01:fail('part dropped off bench')
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

def evaluate(o):
    if not S['known'] or S['zero'] or S['physics']==0 or S['steps']==0:return 0.
    b,c,tr=o['actuator'],o['core'],o['parts_tray'];axis,axial,radial,angle,overlap=relative(o)
    # Complete separation, including tilted ends: bounding envelopes must be 5 mm apart.
    gap=np.linalg.norm(np.maximum(np.maximum(b.low-c.high,c.low-b.high),0))
    if overlap>GEOM_EPS or gap<.005-GEOM_EPS:return 0.
    # Every tier requires release, support and stillness, superseding research held-body tiers.
    if S['clear'] is None or S['still'] is None or S['t']-max(S['clear'],S['still'])<1.-1e-6:return 0.
    r=parts._quat_to_matrix(tr.quat_xyzw)
    upright=bool(r[2,2]>=math.cos(math.radians(5.)))
    tray_on_bench=bench_supported(tr)
    world_points=body_vertices(b);pp=(world_points-tr.pos)@r
    on_floor=abs(float(pp[:,2].min())-.003)<=.002+GEOM_EPS
    contained=bool(np.max(np.abs(pp[:,0]))<=.050+GEOM_EPS and np.max(np.abs(pp[:,1]))<=.100+GEOM_EPS)
    in_cavity=bool(upright and tray_on_bench and tr.resting(.008,.15) and on_floor and contained)
    on_bench=bench_supported(b)
    if not (on_bench or in_cavity) or not bench_supported(c) or not b.resting(.008,.15) or not c.resting(.008,.15):return 0.
    if not in_cavity:return .25
    if S['t']-max(S['clear'],S['still'])<2.-1e-6:return .5
    # Centred within 10 mm is a ruler-visible quality tier; no additional extraction work by scorer.
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
    """All actual convex-panel vertices, transformed into world coordinates."""
    r=parts._quat_to_matrix(st.quat_xyzw);chunks=[]
    for shape in BODY.shapes:
        if shape.kind=='hull':v=np.asarray(shape.points,float)
        else:v=np.asarray(shape.at)+np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*shape.half
        chunks.append(st.pos+v@r.T)
    return np.concatenate(chunks)

@parts.on_step
def step(sim):
    if S['last_t']==sim.t:return
    S['last_t']=sim.t
    try:
        o=sim.states();h=hand_points(sim);observe(o,h,float(sim.t))
        if S['start'] is None:S['start']={n:o[n].pos.tolist() for n in ('actuator','core','parts_tray')}
        log=str(pathlib.Path(os.environ['TRACE_OUT']).with_name('release.jsonl')) if os.environ.get('TRACE_OUT') else os.environ.get('RELEASE_LOG')
        if log and S['steps']%5==0:
            row={k:S[k] for k in ('known','zero','why','physics','steps','force','peak','work','edge','load','maxoff','maxangle','clear','still','start')}
            row.update(rim_forces=S.get('rim_forces'),edge_event=S.get('edge_event'),pull_est=S.get('pull_est'),pull_peak=S.get('pull_peak'),compression=S.get('compression'))
            row['applied_core']=S.get('applied_core');row['applied_body']=S.get('applied_body')
            row.update(task_sha256=SOURCE_SHA256,t=float(sim.t),lot=lots.current().name,score=score(o),objects={n:dict(pos=o[n].pos.tolist(),quat=o[n].quat_xyzw.tolist(),low=o[n].low.tolist(),high=o[n].high.tolist(),lin_vel=o[n].lin_vel.tolist(),ang_vel=o[n].ang_vel.tolist()) for n in ('actuator','core','parts_tray')})
            row['grippers']={}
            base=sim.env.unwrapped;org=parts._np(base.scene.env_origins[0])
            for arm in ('left_robot','right_robot'):
                ar=base.scene[arm];p=parts._np(ar.data.body_link_pose_w)[0]
                row['grippers'][arm]={n:p[list(ar.body_names).index(n),:3].tolist() for n in ('gripper','tip_left','tip_right')}
                row['grippers'][arm]['joints']=parts._np(ar.data.joint_pos)[0].tolist()
            with open(log,'a') as f:f.write(json.dumps(row)+'\n')
    except Exception as e:S['known']=False;fail('observer unavailable: '+str(e))

def demo():
    # Open loop, only published visible coordinates. Tray position is visible layout data.
    tx,ty=lots.current().pos('parts_tray')[:2]; z=Z+.008
    def w(d,x,y,z,f,label):return Waypoint(d,(float(x),float(y),float(z)),f,label,math.pi/2)
    hold=-.0060;rod=-.0020;op=-.020
    left=[w(2,X,.040,Z+.060,op,'above actuator'),w(2,X,.040,z,op,'straddle reinforced body'),w(1,X,.040,z,hold,'hold body'),
          w(1,X,.052,z,hold,'take up measured arm deflection'),w(8,X,.052,z,hold,'counterhold while core withdraws'),w(1,X,.052,Z+.060,hold,'lift released actuator'),
          w(3,tx,ty+.040,Z+.060,hold,'carry to tray'),w(2,tx,ty+.040,TOP+.006+.016+.003,hold,'lower into tray'),
          w(1,tx,ty+.040,TOP+.025,op,'release in tray'),w(2,tx,ty+.040,TOP+.13,op,'hands clear'),w(14,tx,ty+.040,TOP+.13,op,'settle')]
    zr=Z+.010
    right=[w(2,X,-.125,Z+.060,op,'above core handle'),w(2,X,-.125,zr,op,'straddle core'),w(1,X,-.125,zr,rod,'grip core'),
           w(7,X,-.315,zr,rod,'draw core axially with resistance'),w(2,X,-.315,Z+.060,rod,'core clear of body'),
           w(3,.22,-.27,Z+.060,rod,'park core on bench'),w(2,.22,-.27,TOP+.019,rod,'lower core'),w(1,.22,-.27,TOP+.019,op,'release core'),
           w(2,.22,-.27,TOP+.13,op,'right hand clear'),w(15,.22,-.27,TOP+.13,op,'settle')]
    if os.environ.get('SOFTCORE_ONE_HAND')=='1':
        left=[w(30.,.17,.32,TOP+.13,op,'one-hand countercheck: left stays clear')]
    for point in left:point.yaw=-math.pi/2
    right=[tilt.Waypoint(p.duration,p.pos,p.finger,p.label,yaw=p.yaw,tilt=25.,heading=math.pi/2) for p in right]
    return {'left':left,'right':right}

parts._softcore_namespace=globals()
