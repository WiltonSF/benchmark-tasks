"""A laboratory preparation counter holds an original 100 mm precast protein-gel cassette on an inverted
100 mm crystallizing dish. Two transparent styrene plates enclose an 80 mm hydrated gel. A flat gel knife lies
beside it. A complete, disconnected electrophoresis housing stands behind the opening area. A millimetre rule lies nearby. The cassette has an open well side and three bonded margins.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
import numpy as np
from dataclasses import replace
from types import SimpleNamespace
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, FINGER_CLOSED
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.05; EPS=1e-7
CX,CY=.325,0.; P=.100; PT=.002; G=.001; RISE=.050
BOTTOM_Z=TOP+RISE+PT/2; SEAM=TOP+RISE+PT+G/2; UPPER_Z=TOP+RISE+PT+G+PT/2
# Manufacturer sizes: gel 80x80x1; cassette 100x100; EI9010 handle130, blade90x45 mm.
# Image-based estimates: plates2 mm, rim10 mm, handle22x14 mm, blade1.2 mm.
# Bond and adhesion parameters are estimates, not material measurements.
BOND_K=700.; BOND_CAP=1.5; BREAK=1.2; SHEAR_K=250.; GEL_ADH=.12; GEL_K=120.
KNIFE_M=.050; BLADE_T=.0012; TIP=.155; BLADE_START=.065; BW=.045
CLEAR=(.64,.83,.88); GELCOL=(.72,.84,.91); STEEL=(.76,.78,.79); BLACK=(.16,.17,.18)
def prism(poly,z0,z1,color):
    n=len(poly); pts=[(float(x),float(y),float(z)) for z in (z0,z1) for x,y in poly]
    fs=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(j,(j+1)%n,(j+1)%n+n,j+n) for j in range(n)]
    return parts.hull(pts,fs,color)
def face_box(sz,at=(0,0,0),color=CLEAR): return parts.box(sz,at,color)
# Notched upper well edge and slotted lower foot are geometric, not labels.
# Lower plate retains its native full well lip. Upper notch exposes the 10 mm margin.
# Notch depth is estimated from the maker's 100 mm cassette / 80 mm gel envelope.
LOWER=parts.Part('slotted_plate',pos=(CX,CY,BOTTOM_Z),mass=.024,shapes=(
    face_box((.100,.100,PT)),),friction=(.4,.35),contact_offset=.00015,report=('base',))
UPPER=parts.Part('notched_plate',pos=(CX,CY,UPPER_Z),mass=.022,shapes=(
    face_box((.090,.100,PT),(-.005,0,0)),face_box((.010,.010,PT),(.045,-.045,0)),
    face_box((.010,.010,PT),(.045,.045,0))),friction=(.4,.35),contact_offset=.00015,report=('base',))
GEL=parts.Part('hydrated_gel',pos=(CX,CY,SEAM),mass=.0064,
    shapes=(face_box((.080,.080,G),color=GELCOL),),friction=(.35,.3),contact_offset=.00008,report=('base',))
# Bevel on the blade's nose and sides: convex wedge, continuous collisions.
bladepoly=[(.065,-.018),(.078,-BW/2),(TIP,-BW/2),(TIP,BW/2),(.078,BW/2),(.065,.018)]
# blade underside aligned with the handle's bottom for a reproducible flat tool start.
BLADE_Z=-.0064
KNIFE=parts.Part('gel_knife',pos=(.185,-.220,TOP+.007),mass=KNIFE_M,shapes=(
    face_box((.130,.022,.014),color=BLACK),parts.hull([[0.065, -0.018, -0.0065], [0.078, -0.0225, -0.0065], [0.155, -0.0225, -0.0065], [0.155, 0.0225, -0.0065], [0.078, 0.0225, -0.0065], [0.065, 0.018, -0.0065], [0.065, -0.018, -0.0063], [0.078, -0.0225, -0.0063], [0.155, -0.0225, -0.0063], [0.155, 0.0225, -0.0063], [0.078, 0.0225, -0.0063], [0.065, 0.018, -0.0063], [0.07, -0.0175, -0.007], [0.15, -0.0175, -0.007], [0.15, 0.0175, -0.007], [0.07, 0.0175, -0.007], [0.07, -0.0175, -0.0058], [0.15, -0.0175, -0.0058], [0.15, 0.0175, -0.0058], [0.07, 0.0175, -0.0058]],[[6, 7, 16], [1, 12, 0], [5, 4, 15], [10, 19, 11], [1, 13, 12], [1, 13, 2], [1, 6, 7], [1, 6, 0], [8, 1, 2], [8, 1, 7], [8, 7, 16], [8, 17, 16], [14, 13, 2], [14, 3, 2], [14, 4, 3], [14, 4, 15], [14, 13, 12], [14, 15, 12], [19, 6, 11], [19, 6, 16], [19, 17, 16], [19, 18, 17], [5, 6, 0], [5, 6, 11], [5, 15, 12], [5, 12, 0], [9, 8, 2], [9, 3, 2], [9, 8, 17], [9, 18, 17], [10, 4, 3], [10, 9, 3], [10, 19, 18], [10, 9, 18], [10, 5, 11], [10, 5, 4]],STEEL)),
    friction=(.65,.5),contact_offset=.00015,report=('base',))
DISH=parts.Part('inverted_crystallizing_dish',pos=(CX,CY,TOP),mass=.22,shapes=(
    parts.cylinder(.050,.002,(0,0,RISE-.001),(.8,.87,.88)),
    *[parts.box((.004,.024,RISE-.002),(.047*math.cos(k*math.pi/6),.047*math.sin(k*math.pi/6),(RISE-.002)/2),(.8,.87,.88),quat=tuple(parts._matrix_to_quat(parts._rot((0,0,1),k*math.pi/6)))) for k in range(12)]),
    friction=(.6,.5),contact_offset=.0002)
# Full-size inert cell housing (current guide110x120x160); complete envelope, not the smaller blot module.
HOUSING=parts.Part('disconnected_cell_housing',pos=(.590,.225,TOP),shapes=(
    face_box((.110,.120,.006),(0,0,.003),(.50,.62,.65)),
    *[face_box((.006,.120,.148),(x,0,.080),(.5,.62,.65)) for x in (-.052,.052)],
    *[face_box((.098,.006,.148),(0,y,.080),(.5,.62,.65)) for y in (-.057,.057)],
    face_box((.058,.035,.148),(0,0,.080),(.25,.28,.3)),face_box((.110,.120,.006),(0,0,.157),(.4,.5,.52))),contact_offset=.001)
RULE=parts.Part('steel_rule',pos=(.51,-.25,TOP+.0005),shapes=(face_box((.150,.018,.001),color=(.9,.9,.85)),))
# Ink labels and millimetre rule are visual only; their geometry never alters contacts.
def flat_marks(name,rects,z,color=(.1,.1,.1)):
    pts=[];tris=[]
    for x,y,w,h in rects:
        k=len(pts);pts.extend([(x-w/2,y-h/2,z),(x+w/2,y-h/2,z),(x+w/2,y+h/2,z),(x-w/2,y+h/2,z)]);tris.extend([(k,k+1,k+2),(k,k+2,k+3)])
    return parts.Visual(name,tuple(pts),tuple(tris),color)
marks=[(-.045,0.,.001,.008)]  # 1
for cx,cy,digit in [(0.,.045,2),(0.,-.045,3)]:
    marks.extend([(cx,cy+v,.005,.0008) for v in (-.0035,0,.0035)])
    marks.extend([(cx+.0021,cy+.0017,.0008,.0035),(cx+(-.0021 if digit==2 else .0021),cy-.0017,.0008,.0035)])
UPPER=replace(UPPER,visuals=(flat_marks('edge_numbers',marks,PT/2+.00003),))
RULE=replace(RULE,visuals=(flat_marks('millimetre_ticks',[(i*.001-.075,0.,.00015,.010 if i%10==0 else .006 if i%5==0 else .003) for i in range(151)],.00051),))
OBJECTS=[LOWER,UPPER,GEL,KNIFE,DISH,HOUSING,RULE]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'gel_knife':KNIFE.pos}),lots.Lot('B',place={'gel_knife':(.215,-.255,TOP+.007)}))
DEMO_TOUCHES=set(); SCORED=(LOWER,UPPER,GEL,KNIFE,DISH)
# 3x5 seam anchors. Open well edge +x; only near x and the two y sides are bonded.
ANCHORS=np.array([(x,y,0.) for x,y in [*((-.047,y) for y in np.linspace(-.035,.035,5)),*((x,.047) for x in np.linspace(-.035,.035,5)),*((x,-.047) for x in np.linspace(-.035,.035,5))]])
N=len(ANCHORS)
def fresh(): return dict(bond=np.ones(N,bool),witness=np.zeros(3,bool),zero=False,why='',known=False,phys=0,steps=0,t=0.,clear=None,still=None,prev=None,views=None,pressview=None,hold_N=0.,peak=0.,knife_load=0.,events=[],last_tip=None,cut_length=0.,cutting=False,cut_v=np.zeros(3),delta=None,adh=np.ones(2,bool),gdelta=None)
S=fresh()
def fail(why):
    if not S['zero']: S.update(zero=True,why=str(why));event('ZERO '+str(why))
def event(s):
    if len(S['events'])<100:S['events'].append(s)
def rot(st):return parts._quat_to_matrix(st.quat_xyzw)
def world(st,points):return np.asarray(st.pos)+np.asarray(points)@rot(st).T
def edge_points(st):
    # Blade perimeter; dense sampling also catches side-edge entry.
    out=[]
    for a,b in zip(bladepoly,bladepoly[1:]+bladepoly[:1]):
        for u in np.linspace(0,1,25):out.append((*((1-u)*np.array(a)+u*np.array(b)),BLADE_Z))
    return world(st,out)
def anchors(o):
    a=world(o['slotted_plate'],ANCHORS+np.array([0,0,PT/2+G/2]));b=world(o['notched_plate'],ANCHORS-np.array([0,0,PT/2+G/2]))
    return a,b

def material_rule(o,knife_load,dt,contact_points=None,contact_forces=None):
    a,b=anchors(o);d=b-a; S['delta']=d
    ep=edge_points(o['gel_knife']); dist=np.linalg.norm(ep[:,None,:]-a[None,:,:],axis=2).min(0)
    normal=rot(o['slotted_plate'])[:,2]; gap=d@normal
    tension=np.maximum(gap,0)*BOND_K
    # Work is measured load+native blade proximity+actual plate opening, never a pose-only sweep.
    local=np.zeros(N,bool)
    # A loaded rigid blade transmits a pressure patch along one native seam. Native contact
    # manifolds expose only a few endpoints; same-edge load is distributed over the blade's
    # actual footprint, not an arbitrary 25 mm ball around each manifold endpoint.
    edge_load=np.zeros(3)
    if contact_points is not None and len(contact_points):
        cp=(np.asarray(contact_points)-o['slotted_plate'].pos)@rot(o['slotted_plate'])
        cf=np.asarray(contact_forces)
        cp_world=np.asarray(contact_points)
        low=(cp_world-o['slotted_plate'].pos)@rot(o['slotted_plate'])[:,2]-PT/2
        high=(cp_world-o['notched_plate'].pos)@rot(o['notched_plate'])[:,2]+PT/2
        in_gap=(low>=-.0003-EPS)&(high<=.0003+EPS)
        for side,(axis,edge) in enumerate(((0,-.047),(1,.047),(1,-.047))):
            mask=in_gap&(np.abs(cp[:,axis]-edge)<=.014+EPS)
            edge_load[side]=float(cf[mask].sum())
            local[side*5:(side+1)*5]=edge_load[side]>=.20-EPS
    S['edge_load']=edge_load
    freeing=S['bond']&(dist<=.014+EPS)&local&(gap>=.0006-EPS)
    for k in np.flatnonzero(freeing):S['bond'][k]=False;event(f'bond{k} released at {S["t"]:.2f}')
    for side in range(3):
        k=side*5;opened=gap[k:k+5]>=.002-EPS
        if not S['bond'][k:k+5].any() and np.any(opened[:-1]&opened[1:]):S['witness'][side]=True
    # Excessive unworked tension tears the gel/plate assembly; history never clears inside a trial.
    if np.any(S['bond']&(gap>.004+EPS)):fail('plate pulled >4 mm at an unworked bonded margin')
    if np.any(S['bond']&(np.linalg.norm(d,axis=1)>.008+EPS)):fail('bonded cassette sheared >8 mm')
    # Conserved gel slab, with explicit blade-path damage and adhesive strain states.
    g=o['hydrated_gel']; q=(ep-g.pos)@rot(g);inside=(np.abs(q[:,0])<.040)&(np.abs(q[:,1])<.040)&(np.abs(q[:,2])<=G/2+.0001)
    S['cutting']=bool(inside.any());S['cut_v']=np.zeros(3)
    if inside.any():
        if S['last_tip'] is not None:
            travel=q-S['last_tip'];S['cut_v']=rot(g)@travel[inside].mean(0)/max(dt,1e-9)
            S['cut_length']+=float(np.linalg.norm(travel[inside],axis=1).max(initial=0.))
        if S['cut_length']>=.001-EPS:fail('blade swept >=1 mm through native gel')
    S['last_tip']=q.copy()
    # Both faces have equal estimated wet adhesion. Gravity/contact decide which retains the gel.
    gd=[]
    for k,name in enumerate(('slotted_plate','notched_plate')):
        plate=o[name];sign=1 if k==0 else -1
        target=plate.pos+rot(plate)@np.array([0,0,sign*(PT+G)/2]);delta=np.asarray(g.pos)-target
        gd.append(delta)
        if S['adh'][k] and np.linalg.norm(delta)>.0015:S['adh'][k]=False;event(f'gel face{k} adhesion released')
    S['gdelta']=gd
    if np.linalg.norm(gd[0])>.003 and np.linalg.norm(gd[1])>.003:fail('gel lost support from both original plates')

def wrench(asset,f,t):
    import torch
    dev=asset.device
    ff=torch.tensor([[list(map(float,f))]],dtype=torch.float32,device=dev);tt=torch.tensor([[list(map(float,t))]],dtype=torch.float32,device=dev)
    ee=torch.tensor([0],device=dev)
    comp=getattr(asset,'permanent_wrench_composer',None)
    if comp is not None:comp.set_forces_and_torques_index(forces=ff,torques=tt,env_ids=ee,is_global=True)
    else:asset.set_external_force_and_torque(ff,tt,env_ids=ee,is_global=True)

def forces(sim,o):
    fs={n:np.zeros(3) for n in ('slotted_plate','notched_plate','hydrated_gel','gel_knife')};ts={n:np.zeros(3) for n in fs}
    # Native implicit D6 seam restraint supplies opposing plate forces and torques.
    # Its rest geometry is the intact cassette; stiffness decreases only as loaded native seams fracture.
    update_joint()
    if S['gdelta'] is not None:
        for k,n in enumerate(('slotted_plate','notched_plate')):
            if S['adh'][k]:
                f=-GEL_K*np.asarray(S['gdelta'][k]);mag=np.linalg.norm(f)
                if mag>GEL_ADH:f*=GEL_ADH/mag
                fs['hydrated_gel']+=f;fs[n]-=f
    # Accidental blade entry also does physical work: 0.6 N smooth cutting drag, equal reaction on gel.
    v=S['cut_v'];speed=float(np.linalg.norm(v))
    if S['cutting'] and speed>1e-8:
        drag=-.6*min(speed/.010,1.)*v/speed
        fs['gel_knife']+=drag;fs['hydrated_gel']-=drag
    for n in fs:wrench(sim.env.unwrapped.scene[n],fs[n],ts[n])

def arr(x):return np.asarray(x.numpy() if hasattr(x,'numpy') else x)
def contacts():
    from isaaclab_physx.physics import PhysxManager
    return PhysxManager.get_physics_sim_view().create_rigid_contact_view([parts._PATHS['gel_knife']],filter_patterns=[[parts._PATHS['slotted_plate'],parts._PATHS['notched_plate'],parts._PATHS['hydrated_gel']]],max_contact_data_count=256)
def build_joint():
    from pxr import UsdPhysics, Gf, Sdf
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage();path=parts._PATHS['slotted_plate']+'/native_bonds'
    joint=UsdPhysics.Joint.Define(stage,path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS['slotted_plate'])])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(parts._PATHS['notched_plate'])])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0,0,(PT+G)/2))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,-(PT+G)/2))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
    drives=[]
    for name in ('transX','transY','transZ','rotX','rotY','rotZ'):
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),name);drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
        lin=name.startswith('trans');base_k=(1500. if name!='transZ' else 700.) if lin else .2/(180/math.pi)
        drive.CreateStiffnessAttr(base_k);drive.CreateDampingAttr(2. if lin else .01/(180/math.pi));drive.CreateMaxForceAttr(12. if lin else .15)
        drives.append((drive,base_k))
    joint.CreateJointEnabledAttr(True);S['joint']=joint;S['drives']=drives;S['joint_n']=15

def update_joint():
    n=int(S['bond'].sum())
    if n==S.get('joint_n'):return
    joint=S.get('joint')
    if joint is None:raise ValueError('native bond joint missing')
    if n==0:joint.GetJointEnabledAttr().Set(False)
    else:
        for axis,(d,k) in enumerate(S['drives']):
            # Long remaining styrene bridge still bends under load: normal floor140N/m.
            # Order estimate: 10x2mm GPPS strip,75mm span,E~2.96GPa; weld not calibrated.
            scale=max(n/15.,.20) if axis==2 else n/15.
            d.GetStiffnessAttr().Set(k*scale)
    S['joint_n']=n

@parts.on_reset
def reset(sim):
    global S
    S=fresh();build_joint()
@parts.on_physics_step
def physics(sim):
    try:
        S['t']=float(sim.t);dt=float(sim.env.unwrapped.physics_dt)
        base=sim.env.unwrapped
        org=S.get('origin')
        if org is None:org=parts._np(base.scene.env_origins[0]);S['origin']=org
        import torch
        names=('slotted_plate','notched_plate','hydrated_gel','gel_knife')
        batch=parts._np(torch.stack([base.scene[name].data.root_link_pose_w[0] for name in names]));o={}
        for name,p in zip(names,batch):o[name]=SimpleNamespace(pos=p[:3]-org,quat_xyzw=p[3:])
        if S['views'] is None:
            S['views']=contacts()
            from isaaclab_physx.physics import PhysxManager
            from isaaclab.sim.utils import get_current_stage
            from pxr import Usd,UsdPhysics
            paths=[str(p.GetPath()) for p in Usd.PrimRange(get_current_stage().GetPseudoRoot(),Usd.TraverseInstanceProxies()) if '/LeftArm/' in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            S['pressview']=PhysxManager.get_physics_sim_view().create_rigid_contact_view([parts._PATHS['slotted_plate']],filter_patterns=[paths],max_contact_data_count=256)
        press=arr(S['pressview'].get_contact_force_matrix(dt=dt)).reshape(-1,3);S['hold_N']=float(np.linalg.norm(press,axis=1).sum())
        ff,points,_,_,counts,starts=(arr(v) for v in S['views'].get_contact_data(dt=dt))
        ff=ff.reshape(-1);points=points.reshape(-1,3);cp=[];cf=[]
        for channel,(num,start) in enumerate(zip(counts.reshape(-1),starts.reshape(-1))):
            num,start=int(num),int(start)
            if num<0 or (num and (start<0 or start+num>len(ff))):raise ValueError('contact buffer incomplete')
            if num and channel<2:cp.extend(points[start:start+num]-org);cf.extend(np.abs(ff[start:start+num]))
        load=float(sum(cf));S['knife_load']=load;S['peak']=max(S['peak'],load)
        material_rule(o,load,dt,cp,cf);forces(sim,o);S['phys']+=1
    except Exception as e: S.update(known=False);fail('unknown material/contact: '+str(e))

# Sparse face-sampled arm geometry. Any missing state fails closed.
_HAND = {}
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
def hands(sim):
    base=sim.env.unwrapped;org=parts._np(base.scene.env_origins[0]);out=[]
    for name in ('left_robot','right_robot'):
        asset=base.scene[name];p=parts._np(asset.data.body_link_pose_w)[0]
        for n in ('link5','gripper','tip_left','tip_right'):
            v=p[list(asset.body_names).index(n)];out.append(local_hand(n)@parts._quat_to_matrix(v[3:]).T+v[:3]-org)
    return np.concatenate(out)
def box_gap(h,spec,st):
    pp={'base':(st.pos,rot(st))};return min(float(np.linalg.norm(np.maximum(np.abs((h-c)@r)-size/2,0),axis=1).min()) for _,c,r,size in spec.world_boxes(pp))
def observe(o,h,t):
    if h is None or not len(h) or not np.isfinite(h).all():S['known']=False;fail('arm state unknown');return
    clear=True;snap=[]
    for spec in SCORED:
        st=o[spec.name]
        if not np.isfinite(st.pos).all() or not np.isfinite(st.quat_xyzw).all():S['known']=False;fail('object state unknown');return
        if st.low[2]<TOP-.01:fail(spec.name+' fell off counter')
        if abs(st.pos[1])>.63 or not -.17<st.pos[0]<.88:fail(spec.name+' left counter')
        near=h[np.all((h>=st.low-.04)&(h<=st.high+.04),axis=1)]
        if len(near) and box_gap(near,spec,st)<.036:clear=False
        r=rot(st);corners=np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        snap.append(st.pos+(corners*np.asarray(spec.size)/2)@r.T)
    snap=np.concatenate(snap)
    if S['prev'] is None or np.max(np.linalg.norm(snap-S['prev'],axis=1))>.001:S['still']=t;S['prev']=snap.copy()
    S['clear']=(t if S['clear'] is None else S['clear']) if clear else None
    S.update(known=True,steps=S['steps']+1,t=float(t))
def retained(o):
    g=o['hydrated_gel'];out=[]
    for name in ('slotted_plate','notched_plate'):
        p=o[name];q=(g.pos-p.pos)@rot(p)
        out.append(abs(abs(q[2])-(PT+G)/2)<=.0015+EPS and np.linalg.norm(q[:2])<=.003+EPS and abs(float(rot(p)[:,2]@rot(g)[:,2]))>math.cos(math.radians(5)))
    return any(out)
def supported(st,dish=None):
    on_counter=TOP-.001-EPS<=st.low[2]<=TOP+.0015+EPS
    on_dish=False
    if dish is not None:
        on_dish=(np.linalg.norm(st.pos[:2]-dish.pos[:2])<=.008+EPS and
                 dish.high[2]-.001-EPS<=st.low[2]<=dish.high[2]+.0015+EPS and
                 abs(rot(dish)[2,2])>=math.cos(math.radians(5))-EPS)
    return bool((on_counter or on_dish) and st.resting(.008,.15) and abs(rot(st)[2,2])>=math.cos(math.radians(5))-EPS)

def plates_clear(o):
    a=o['slotted_plate'];b=o['notched_plate'];ra,rb=rot(a),rot(b)
    # A separating-axis gap >=10 mm certifies every point of the two rigid plates clear.
    axes=[ra[:,j] for j in range(3)]+[rb[:,j] for j in range(3)]
    axes += [np.cross(ra[:,j],rb[:,k]) for j in range(3) for k in range(3)]
    half=np.array([.050,.050,PT/2]);delta=b.pos-a.pos
    for axis in axes:
        n=np.linalg.norm(axis)
        if n<1e-9:continue
        axis=axis/n;gap=abs(float(delta@axis))-float(np.abs(axis@ra)@half)-float(np.abs(axis@rb)@half)
        if gap>=.010-EPS:return True
    return False

def score(o):
    if not S['known'] or S['zero'] or S['phys']==0 or S['steps']==0:return 0.
    freed=[not S['bond'][i*5:(i+1)*5].any() and S['witness'][i] for i in range(3)];n=sum(freed)
    if not n or not retained(o):return 0.
    if S['clear'] is None or S['still'] is None or S['t']-max(S['clear'],S['still'])<1.-EPS:return 0.
    sep=plates_clear(o)
    if n==3 and sep and retained(o):
        held=S['clear'] is not None and S['still'] is not None and S['t']-max(S['clear'],S['still'])>=1.-EPS
        if held and all(supported(o[x.name],o[DISH.name] if x.name in (LOWER.name,UPPER.name) else None) for x in (LOWER,UPPER,KNIFE,DISH)):return 1.
        return .75
    return .5 if n>=2 else .25

def success(o):return bool(score(o)>=1.-EPS)
@parts.on_step
def step(sim):
    try:
        o=sim.states();observe(o,hands(sim),float(sim.t))
        log=os.environ.get('TRACE_OUT');log=str(pathlib.Path(log).with_name('cassette.jsonl')) if log else os.environ.get('CASSETTE_LOG')
        if log and S['steps']%5==0:
            row={k:S[k] for k in ('t','phys','steps','zero','why','peak','knife_load','hold_N','events','cut_length','clear','still')};row.update(bonds=S['bond'].astype(int).tolist(),adh=S['adh'].astype(int).tolist(),score=score(o),lot=lots.current().name)
            for n in ('slotted_plate','notched_plate','hydrated_gel','gel_knife'):row[n]=dict(pos=np.round(o[n].pos,5).tolist(),quat=np.round(o[n].quat_xyzw,4).tolist())
            row['gaps_mm']=np.round(S['delta']*1000,3).tolist() if S['delta'] is not None else None;row['witness']=S['witness'].astype(int).tolist();row['edge_load']=np.round(S.get('edge_load',np.zeros(3)),3).tolist()
            base=sim.env.unwrapped;org=parts._np(base.scene.env_origins[0]);row['hands']={}
            for n in ('left_robot','right_robot'):
                asset=base.scene[n];pp=parts._np(asset.data.body_link_pose_w)[0];v=pp[list(asset.body_names).index('gripper')];row['hands'][n]={'pos':(v[:3]-org).round(5).tolist(),'quat':v[3:].round(5).tolist()}
            with open(log,'a') as ff:ff.write(json.dumps(row)+'\n')
    except Exception as e:S['known']=False;fail('observer: '+str(e))

# Entire plan is open-loop from published camera-visible tool/cassette starts.
def demo():
    from yam_arena import ik,station
    from yam_arena.demo import READY
    def park(d,arm):
        p,r=ik.tcp_pose(np.array(READY),station.BASES[arm]);return Waypoint(float(d),tuple(map(float,p)),FINGER_CLOSED,'hands clear',tilt._TiltedYaw(0.,r))
    L=[];R=[]
    def w(d,p,f,label,a=0.):return tilt.Waypoint(float(d),tuple(map(float,p)),float(f),label,yaw=-math.pi/2,tilt=float(a),heading=0.)
    compensation=0.
    def rw(d,p,f,label,a=0.,h=0.):
        yaw=-.060 if len(R)>=3 else 0.
        pickup_bias=(-.45 if compensation<.8 else .65)*float(np.clip((-.220-ky)/.035,0.,1.)) if len(R)>=3 else 0.
        rotation=parts._rot((0.,1.,0.),math.radians(pickup_bias-compensation))@tilt.rotation(yaw,float(a),float(h))
        R.append(Waypoint(float(d),tuple(map(float,p)),float(f),label,tilt._TiltedYaw(yaw,rotation)))
    kx,ky,kz=lots.current().pos('gel_knife');gripz=kz+.013
    op=-.016;cl=-.002
    rw(2,(kx,ky,gripz+.065),op,'above knife handle');rw(1.2,(kx,ky,gripz),op,'at native handle');rw(.8,(kx,ky,gripz),cl,'grip handle');rw(1.5,(kx,ky,gripz+.090),cl,'lift knife')
    # One flat fingertip on the original lower well lip, other finger beyond the cassette.
    HF=-.015
    hold=(CX+.061,CY,BOTTOM_Z+PT/2+.0128-.006)
    L.extend([w(3,(hold[0],hold[1],hold[2]+.020),HF,'above exposed lower lip'),w(1.5,hold,HF,'counterhold lower plate on dish')])
    # Knife tip approach. Blade lies 19.4 mm below TCP (6.4 offset +13 mm TCP above handle).
    z=SEAM+.0194+.0014
    left_retired=False
    for side,y,x in [('front_a',-.010,CX-.05-TIP+.008),('front_b',.010,CX-.05-TIP+.008),('right',-.0675,CX-.015-TIP),('left',.0675,CX-.015-TIP)]:
        if side=='left':
            tr=sum(v.duration for v in R);tl=sum(v.duration for v in L)
            L.append(w(tr-tl,hold,HF,'counterhold through first two seams'))
            L.append(w(1.,(hold[0],hold[1],hold[2]+.020),HF,'release lower lip'))
            L.append(Waypoint(1.5,(.12,.43,TOP+.10),HF,'left arm clear before last seam',0.))
            rw(2.5,R[-1].pos,cl,'wait for left arm clear')
            left_retired=True
        isfront=side.startswith('front')
        z=SEAM+.0194+.0014+((.0008 if side=='front_b' else 0.) if isfront else -.0003)-(.0006*float(np.clip((-.220-ky)/.035,0.,1.)) if side=='front_a' else 0.)
        yout=y+(math.copysign(.020,y) if not isfront else 0.)
        rw(2,(x-.015,yout,z+.035),cl,'clear before '+side)
        rw(1.5,(x-.015,yout,z),cl,'lower outside '+side+' seam')
        rw(.6,(x-.015,yout,z),cl,'settle before insertion')
        if not isfront:rw(1.,(x,yout,z),cl,'alongside native margin')
        rw(3.,(x,y,z),cl,'insert bevel from outside '+side)
        rw(.6,(x,y,z),cl,'settle native bevel')
        # Lower lip is braced for the first two seams; the arm is clear during the last seam.
        angle=3. if isfront else 6.
        heading=0. if isfront else (-math.copysign(math.pi/2,y))
        rw(2.,(x,y,z+.001),cl,'lever loaded seam',a=angle,h=heading)
        if not isfront:
            for xx in (CX+.010-TIP,CX+.035-TIP,CX+.049-TIP):rw(1.5,(xx,y,z+.001),cl,'work along '+side,a=angle,h=heading)
        endx=x if isfront else CX+.049-TIP
        flatten_y=y+(math.copysign(.005,y)*float(np.clip((-.220-ky)/.035,0.,1.)) if not isfront else 0.)
        rw(.8,(endx,flatten_y,z+.001),cl,'flatten while starting lateral withdrawal')
        if isfront:
            rw(1.2,(x-.020,y,z+.001),cl,'back straight out of front margin')
            rw(.8,(x-.020,y,z+.025),cl,'lift only outside front margin')
        else:
            rw(1.3,(endx,yout,z+.001),cl,'withdraw sideways from side margin')
            rw(.8,(endx,yout,z+.025),cl,'lift only outside side margin')
        compensation+=.8 if isfront else 0.
    # Take the freed upper plate from its exposed -y border, outside the retained gel.
    # Last-side prying can shift the loose plate; the 90mm blade overlaps the whole side.
    x=CX-.110;z=SEAM+.0194-.0002
    rw(2.,(x,-.105,z+.030),cl,'above free negative-side border')
    rw(1.5,(x,-.105,z),cl,'lower outside retained gel')
    rw(2.,(x,-.070,z),cl,'wedge under free plate border')
    rw(.6,(x,-.070,z),cl,'settle at free border')
    rw(1.5,(x,-.070,z+.001),cl,'lever free plate border',a=3.,h=math.pi/2)
    rw(1.5,(x,-.070,z+.012),cl,'lift border clear of gel',a=3.,h=math.pi/2)
    rw(2.5,(x,-.020,z+.012),cl,'move support blade under free plate above gel',a=3.,h=math.pi/2)
    rw(1.,(x,-.020,z+.012),cl,'level plate on support blade')
    rw(2.,(x,-.020,z+.075),cl,'lift plate on blade')
    rw(3.,(x,-.200,z+.075),cl,'carry empty plate clear')
    rw(2.,(x,-.200,TOP+.0194+.001),cl,'lower plate to counter')
    rw(2.,(x-.100,-.200,TOP+.0194+.001),cl,'slide knife out from plate')
    rw(1.5,(.14,-.34,TOP+.020),cl,'knife down');rw(1.,(.14,-.34,TOP+.020),op,'release knife')
    tl=sum(v.duration for v in L);tr=sum(v.duration for v in R)
    if not left_retired:raise ValueError('left arm must clear before loose plate pickup')
    L.append(Waypoint(tr-tl,(.12,.43,TOP+.10),HF,'remain clear until tools are down',0.))
    L.append(park(2.,'left'));R.append(park(2.,'right'))
    total=max(sum(v.duration for v in L),sum(v.duration for v in R))+3
    for plan,arm in ((L,'left'),(R,'right')):plan.append(park(total-sum(v.duration for v in plan),arm))
    return {'left':L,'right':R}
parts._cassette_namespace=globals()
