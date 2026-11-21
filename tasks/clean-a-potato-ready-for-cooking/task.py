"""A loose russet potato with twelve brown soil marks lies in a shallow kitchen catch tray beside a green
vegetable brush. The brush is loose, bristles up, against the near-left tray corner. A green polypropylene plate is on the counter to the left. A faucet and backsplash stand
behind the tray. The brown marks show attached soil; the tan surface beneath is intact potato skin.
"""
from __future__ import annotations
import json
import math
import pathlib
import sys
import os
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts
import lots
import tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.constants import FINGER_OPEN
SCENE='station'
TOP=TABLE_TOP[SCENE]
SUCCESS_HOLD_S=1.0
EPS=1e-6
R=np.array([.0475,.0325,.030])
BROWN=(.22,.105,.035);SKIN=(.68,.47,.25);GREEN=(.28,.63,.15)
TRAY_Z=TOP+.004
PZ=TRAY_Z+.030
# An uneven russet's broad contact faces, within 3 mm of the nominal ellipsoid.
_TH=np.arange(13)*math.pi/12
_XR=R[0]*np.cos(_TH)[::-1]
_HR=np.sin(_TH)[::-1]
def skin_h(x):return float(np.interp(x,_XR,_HR))
def skin_scale(ph):return 1/max(math.cos(ph-k*math.pi/4) for k in range(8))
POTATO_VERTICES=np.array([(R[0]*math.cos(th),R[1]*math.sin(th)*math.cos(ph)/math.cos(math.pi/8),R[2]*math.sin(th)*math.sin(ph)/math.cos(math.pi/8)) for th in _TH for ph in (np.arange(8)+.5)*math.pi/4])
PATCHES=[(x, ph) for x in (-.024,0.,.024) for ph in np.arange(4)*math.pi/2]
# Each marked sector is a 12 x 12 mm curved patch, split into 25 visible soil cells.
CELL=[]; VIS=[]
for k,(x,ph) in enumerate(PATCHES):
    for i in range(5):
        for j in range(5):
            xx=x+(i-2)*.0024; pp=ph+(j-2)*.0024/.031
            h=skin_h(xx)*skin_scale(pp)
            q=np.array([xx,R[1]*h*math.cos(pp),R[2]*h*math.sin(pp)])
            CELL.append((k,q))
            pts=[]
            for dx,dp in ((-.0012,-.0012/.031),(.0012,-.0012/.031),(.0012,.0012/.031),(-.0012,.0012/.031)):
                sx=xx+dx;sp=pp+dp;sh=skin_h(sx)*skin_scale(sp)
                pts.append((sx,(R[1]*sh+.0004)*math.cos(sp),(R[2]*sh+.0004)*math.sin(sp)))
            VIS.append(parts.Visual(f'soil{len(CELL)-1}',tuple(pts),((0,1,2),(0,2,3)),color=BROWN))
for k in range(12):
    vv=VIS[k*25:(k+1)*25];pts=[];tris=[]
    for v in vv:
        offset=len(pts);pts.extend(tuple(np.asarray(q)*1.004) for q in v.points)
        tris.extend(tuple(offset+i for i in t) for t in v.triangles)
    VIS.append(parts.Visual(f'damage{k}',tuple(pts),tuple(tris),color=(.93,.82,.55),visible=False))
for name,x,ph,col in (('axis_dot',.040,0.,(.1,.3,.9)),('up_dot',-.040,math.pi/2,(.8,.12,.1))):
    pts=[]
    for k in range(17):
        a=2*math.pi*k/16;sx=x+.003*math.cos(a);sp=ph+.12*math.sin(a)
        h=skin_h(sx)*skin_scale(sp)
        pts.append((sx,(R[1]*h+.0006)*math.cos(sp),(R[2]*h+.0006)*math.sin(sp)))
    VIS.append(parts.Visual(name,tuple(pts),tuple((0,i,i+1) for i in range(1,16)),color=col))
POINTS=np.array([q for _,q in CELL]);IDS=np.array([k for k,_ in CELL])
POTATO=parts.Part('potato',pos=(.245,-.035,PZ),shapes=[parts.potato(2*R,color=SKIN)],mass=.200,
    friction=(.65,.5),contact_offset=.001,angular_damping=3.,visuals=VIS,report=('base',))
# Brush origin is its thick rubber body centre. Its bristle tips are 40.5 mm below this centre.
BRUSH=parts.Part('brush',pos=(.1809,.0906,TRAY_Z+.023),quat=(1.,0.,0.,0.),mass=.075,
    shapes=[parts.ellipsoid((.0838,.0508,.046),color=GREEN)],
    joints=[parts.Slider('bristles',shapes=[parts.box((.072,.042,.0175),at=(0,0,-.03175),color=(.82,.80,.53))],
        axis=(0,0,1),travel=.012,mass=.015,spring=1500.,damping=12.,max_effort=25.)],
    friction=(.8,.65),angular_damping=3.,link_friction=(('bristles',(.3,.25)),),contact_offset=.001,report=('bristles','base'))
TRAY=parts.Part('catch_tray',pos=(.285,0,TOP),shapes=[
    parts.box((.300,.240,.004),at=(0,0,.002),color=(.5,.55,.58)),
    parts.box((.300,.004,.035),at=(0,-.118,.0175)),parts.box((.300,.004,.035),at=(0,.118,.0175)),
    parts.box((.004,.240,.035),at=(-.148,0,.0175)),parts.box((.004,.240,.035),at=(.148,0,.0175))])
# Fixed plate: ordinary anti-slip securing of its underside, not a support for scrubbing.
PLATE=parts.Part('clean_plate',pos=(.255,.25,TOP),shapes=[parts.cylinder(.095,.004,at=(0,0,.002),color=(.35,.75,.38))]+[
    parts.box((.012,.025,.014),at=(.087*math.cos(a),.087*math.sin(a),.009),color=(.35,.75,.38),
    quat=(0,0,math.sin(a/2),math.cos(a/2))) for a in np.arange(24)*2*math.pi/24])
BACK=parts.Part('backsplash',pos=(.56,0,TOP+.15),shapes=[parts.box((.014,.60,.30),color=(.88,.88,.84))])
FAUCET=parts.Part('faucet',pos=(.54,-.18,TOP),shapes=[parts.cylinder(.012,.18,at=(0,0,.09)),
    parts.cylinder(.009,.08,at=(-.035,0,.18),axis='x'),parts.cylinder(.009,.025,at=(-.075,0,.17))])
OBJECTS=[POTATO,BRUSH,TRAY,PLATE,BACK,FAUCET]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'potato':POTATO.pos}),
    lots.Lot('B',place={'potato':(.245,-.035,PZ),'brush':(.1809,.0906,TRAY_Z+.023)},yaw={'potato':math.radians(70)}))
S=parts.__dict__.setdefault('potato_clean_state',{})
def reset_state():
    S.clear();S.update(travel=np.zeros(len(CELL)),prev=None,known=False,broken=False,damage=False,dropped=False,
        clear=False,hold=0,views=None,support_views=None,supported=False,physics_steps=0,hidden=set(),damaged=set(),occluded=np.zeros(len(CELL),bool),force=0.,t=0.,maxforce=0.,path=0.)
if not S:reset_state()
def clean_count():
    return int(sum(np.mean(S['travel'][IDS==k]>=.025-EPS)>=.9-EPS for k in range(12)))
def record_contact(local,rotation,force,dt,brush_pose=None,field_center=None):
    """25 mm of loaded sliding per soil cell, in the actual bristle rectangle, with real force.
    Sliding is the motion over the skin of the bristle material point now at the contact, so rocking or rolling the
    potato on the bristles without slip clears nothing. brush_pose is the bristle link (rotation, origin) in the
    potato frame; without it (synthetic controls) the brush is taken as fixed while the contact point moves."""
    local=np.array(local,float)
    prev=S['prev'];S['prev']=(local,float(force),brush_pose)
    if force>12.+EPS:
        S['damage']=True
        S['damaged'].add(int(IDS[np.argmin(np.linalg.norm(POINTS-local,axis=1))]))
    if prev is None or not (1-EPS<=force<=5+EPS and 1-EPS<=prev[1]<=5+EPS):return
    if brush_pose is None or prev[2] is None:delta=local-prev[0]
    else:
        rb,tb=brush_pose;rb0,tb0=prev[2]
        delta=local-(np.asarray(rb0)@(np.asarray(rb).T@(local-np.asarray(tb)))+np.asarray(tb0))
    normal=local/(R*R);normal/=max(np.linalg.norm(normal),EPS)
    step=float(np.linalg.norm(delta-normal*(delta@normal)))
    # 0.5 mm per 20 Hz sample is a 10 mm/s sliding floor; above 240 mm/s the sample is treated as a glitch.
    if step>.012 or step<.0005-EPS:return
    # Cells inside the actual 72 x 42 mm bristle footprint; surface must face the bristles.
    d=POINTS-(local if field_center is None else np.asarray(field_center))
    b=d@rotation
    normals=POINTS/(R*R);normals/=np.linalg.norm(normals,axis=1)[:,None]
    touched=(np.abs(b[:,0])<=.036+EPS)&(np.abs(b[:,1])<=.021+EPS)&(np.abs(b[:,2])<=.0045+EPS)&(normals@rotation[:,2]>.4)
    touched &= ~S['occluded']
    S['travel'][touched]+=step;S['path']+=step

def _arr(x):
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    if hasattr(x,'numpy'):return np.asarray(x.numpy())
    return np.asarray(x)
def _descend(p):
    for c in p.GetAllChildren():
        yield c;yield from _descend(c)
def _views():
    from isaaclab_physx.physics import PhysxManager
    sv=PhysxManager.get_physics_sim_view()
    return sv.create_rigid_contact_view(parts._PATHS['brush']+'/bristles',filter_patterns=[parts._PATHS['potato']],max_contact_data_count=128)

# Cache samples across hull faces; include the faces between distant vertices.
HAND=('gripper','tip_left','tip_right')
_CACHE={}
def hand_points(base):
    from yam_arena import analysis
    from scipy.spatial import ConvexHull
    origin=parts._np(base.scene.env_origins[0]);points=[]
    for arm in ('left_robot','right_robot'):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        for i,name in enumerate(asset.body_names):
            if name not in HAND:continue
            if name not in _CACHE:
                vv=analysis.world_hulls({name:np.eye(4)})
                if not vv:raise RuntimeError('missing hand hull '+name)
                v=np.concatenate([h.vertices for _,h in vv]);samples=[v]
                for tri in ConvexHull(v).simplices:
                    a,b,c=v[tri];n=max(1,int(np.ceil(max(np.linalg.norm(b-a),np.linalg.norm(c-a),np.linalg.norm(c-b))/.003)))
                    samples.append(np.array([a+(b-a)*u/n+(c-a)*w/n for u in range(n+1) for w in range(n+1-u)]))
                _CACHE[name]=np.unique(np.round(np.concatenate(samples)/.0025).astype(np.int32),axis=0)*.0025
            points.append(_CACHE[name]@parts._quat_to_matrix(poses[i,3:]).T+poses[i,:3]-origin)
    if len(points)!=6:raise RuntimeError('unknown hand state')
    return np.concatenate(points)
def hands_clear(points,st):
    cp=(points-st['potato'].pos)@parts._quat_to_matrix(st['potato'].quat_xyzw)
    # Conservative ellipsoid clearance: scaled radial distance bounds the Euclidean distance from below.
    r_clear=R+np.array([0,.003,.003])
    pd=(np.linalg.norm(cp/r_clear,axis=1)-1)*min(r_clear)
    b=st['brush'];bp=(points-b.pos)@parts._quat_to_matrix(b.quat_xyzw)
    body=np.maximum(np.abs(bp)-np.array([.0419,.0254,.023]),0)
    br=np.maximum(np.abs(bp-np.array([0,0,-.03175]))-np.array([.036,.021,.00875]),0)
    return bool(pd.min()>=.030-EPS and np.linalg.norm(body,axis=1).min()>=.030-EPS and np.linalg.norm(br,axis=1).min()>=.030-EPS)
def bottom(st):
    rot=parts._quat_to_matrix(st.quat_xyzw)
    return float(st.pos[2]+(POTATO_VERTICES@rot.T)[:,2].min())
def brush_bottom(b):
    rot=parts._quat_to_matrix(b.quat_xyzw)
    body=float(b.pos[2]-np.linalg.norm(rot[2]*np.array([.0419,.0254,.023])))
    pp,qq=b.links['bristles'];rr=parts._quat_to_matrix(qq)
    return min(body,float(pp[2]-np.abs(rr[2])@np.array([.036,.021,.00875])))
def brush_outside_plate(b):
    """Conservative transformed collider boxes, including the actual moving bristle link."""
    centre=np.asarray(PLATE.pos[:2]);boxes=[]
    for pos,rot,half in ((b.pos,parts._quat_to_matrix(b.quat_xyzw),np.array([.0419,.0254,.023])),
            (b.links['bristles'][0],parts._quat_to_matrix(b.links['bristles'][1]),np.array([.036,.021,.00875]))):
        xy=np.array(pos[:2]);extent=np.abs(rot[:2])@half
        distance=np.linalg.norm(np.maximum(np.abs(centre-xy)-extent,0.))
        boxes.append(distance>=.095-EPS)
    return all(boxes)

def latch_violations(force,lowest,local=None):
    if not np.isfinite(force) or not np.isfinite(lowest):S['broken']=True;return
    S['maxforce']=max(S['maxforce'],float(force))
    if lowest<TOP-.01-EPS:S['dropped']=True
    if force>12.+EPS:
        S['damage']=True
        k=int(IDS[np.argmin(np.linalg.norm(POINTS-np.array(local if local is not None else [0,0,-.03]),axis=1))])
        S['damaged'].add(k)

@parts.on_physics_step
def monitor_physics(sim):
    try:
        base=sim.env.unwrapped
        if S['views'] is None:S['views']=_views()
        fm=_arr(S['views'].get_contact_force_matrix(dt=base.physics_dt)).reshape(-1,3)
        if not np.isfinite(fm).all():raise RuntimeError('nonfinite substep force')
        p=parts._read(base,POTATO)
        latch_violations(float(np.linalg.norm(fm.sum(0))),bottom(p))
        S['physics_steps']+=1
    except Exception as exc:
        if not S['broken']:print('[potato] PHYSICS MONITOR FAILED '+repr(exc),flush=True)
        S['broken']=True

def counter_colliders(stage,collision_api,descendants):
    roots=[p for p in stage.Traverse() if p.GetName()=='table']
    if len(roots)!=1:raise RuntimeError('unknown counter root '+str(len(roots)))
    colliders=[str(p.GetPath()) for p in descendants(roots[0]) if p.HasAPI(collision_api)]
    if not colliders:raise RuntimeError('counter has no collision geometry')
    return colliders

def support_views():
    from isaaclab_physx.physics import PhysxManager
    from isaaclab.sim.utils import get_current_stage
    from pxr import Usd,UsdPhysics
    tables=counter_colliders(get_current_stage(),UsdPhysics.CollisionAPI,lambda root:Usd.PrimRange(root,Usd.TraverseInstanceProxies()))
    sv=PhysxManager.get_physics_sim_view()
    allowed=tables+[parts._PATHS['catch_tray']+'/s0']
    print('[support] allowed horizontal colliders '+repr(allowed),flush=True)
    return tuple(sv.create_rigid_contact_view(parts._PATHS['brush']+'/'+link,filter_patterns=allowed,max_contact_data_count=128) for link in ('base','bristles'))

def released(st):
    p,b=st['potato'],st['brush']
    on_plate=np.linalg.norm(p.pos[:2]-np.array(PLATE.pos[:2]))<=.040
    in_tray=abs(p.pos[0]-.285)<=.100 and abs(p.pos[1])<=.082
    surf=TOP+.004 if on_plate or in_tray else TOP
    brush_surface=TRAY_Z if abs(b.pos[0]-.285)<=.146 and abs(b.pos[1])<=.116 else TOP  # centre over the tray floor
    return S['clear'] and S['supported'] and p.resting() and b.resting() and abs(bottom(p)-surf)<=.005+EPS and abs(brush_bottom(b)-brush_surface)<=.005+EPS

def score(st):
    if not S['known'] or S['broken'] or S['damage'] or S['dropped'] or not released(st) or S['hold']<40:return 0.
    n=clean_count()
    p,b=st['potato'],st['brush']
    plate=np.linalg.norm(p.pos[:2]-PLATE.pos[:2])<=.040+EPS and abs(bottom(p)-(TOP+.004))<=.005+EPS
    beside=.105-EPS<=np.linalg.norm(b.pos[:2]-PLATE.pos[:2])<=.20+EPS and brush_outside_plate(b)
    if n==12 and plate and beside:return 1.
    return .75 if n>=9 else .5 if n>=6 else .25 if n>=3 else 0.
def success(st):return score(st)==1.
# Reference-only coarse RGB localization between released grasp segments.
# These functions receive pixels, published camera calibration and a clock, never object/force/soil state.
_VISION_JOBS=[]
def visual_pose(rgb,camera,xy_hint,plane_z):
    import cv2
    rgb=cv2.resize(np.asarray(rgb[:,:,:3],np.uint8),(320,176),interpolation=cv2.INTER_AREA)
    lens=camera.lens.resized(320,176);rot=parts._quat_to_matrix(camera.rot_xyzw);origin=np.array(camera.pos)
    q=rot.T@(np.array([*xy_hint,plane_z])-origin)
    if q[2]>=0:return None
    uv=np.array([lens.fx*q[0]/-q[2]+lens.cx,lens.cy-lens.fy*q[1]/-q[2]])
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);yy,xx=np.indices(hsv.shape[:2])
    mask=((hsv[:,:,0]>=7)&(hsv[:,:,0]<=32)&(hsv[:,:,1]>=105)&(hsv[:,:,2]>=55)&(hsv[:,:,2]<=240)&((xx-uv[0])**2+(yy-uv[1])**2<28**2)).astype(np.uint8)
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask,8)
    candidates=[]
    for i in range(1,count):
        if not 20<=stats[i,4]<=600 or max(stats[i,2:4])>40:continue
        v,u=np.where(labels==i)
        rays=np.c_[(u-lens.cx)/lens.fx,-(v-lens.cy)/lens.fy,-np.ones(len(u))]@rot.T
        if np.any(np.abs(rays[:,2])<EPS):continue
        pts=origin+rays*((plane_z-origin[2])/rays[:,2])[:,None]
        xy=pts[:,:2].mean(0)
        if np.linalg.norm(xy-np.asarray(xy_hint))>.060:continue
        vals,vecs=np.linalg.eigh(np.cov(pts[:,:2].T));axis=vecs[:,-1]
        yaw=(math.atan2(axis[1],axis[0])+math.pi/2)%math.pi-math.pi/2
        candidates.append((float(np.linalg.norm(centers[i]-uv)),xy,yaw,int(stats[i,4])))
    if not candidates:return None
    _,xy,yaw,area=min(candidates,key=lambda x:x[0])
    xy=np.round(xy/.005)*.005;yaw=math.radians(round(math.degrees(yaw)/5)*5)
    return tuple(map(float,xy)),float(yaw),area

def visual_tool(rgb,camera,xy_hint):
    """Fit the visible 72x42 mm bristle rectangle above its declared support plane."""
    import cv2
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    rgb=np.asarray(rgb[:,:,:3],np.uint8);height,width=rgb.shape[:2]
    lens=camera.lens.resized(width,height);origin=np.array(camera.pos)
    rcv=parts._quat_to_matrix(camera.rot_xyzw)@np.diag([1.,-1.,-1.])
    km=np.array([[lens.fx,0,lens.cx],[0,lens.fy,lens.cy],[0,0,1.]])
    hint=np.array([*xy_hint,TOP+.0644]);q=rcv.T@(hint-origin)
    uv=(km@q)[:2]/q[2];yy,xx=np.indices((height,width))
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=((hsv[:,:,0]>=25)&(hsv[:,:,0]<=42)&(hsv[:,:,1]>=20)&(hsv[:,:,1]<=110)&(hsv[:,:,2]>=190)&((xx-uv[0])**2+(yy-uv[1])**2<40**2)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    obj=np.array([[-.036,-.021,0],[.036,-.021,0],[.036,.021,0],[-.036,.021,0]])
    candidates=[]
    def project(v):
        rw=Rotation.from_euler('xyz',v[2:]).as_matrix()
        # Native thin-floor calibration: support settles 3 mm below nominal tray top.
        z=TOP+.001+np.linalg.norm(rw[2]*np.array([.0419,.0254,.023]))+.0404*rw[2,2]
        pw=np.array([v[0],v[1],z]);q=(obj@rw.T+pw-origin)@rcv;im=q@km.T
        return im[:,:2]/im[:,2:],pw
    for i in range(1,count):
        if not 40<=stats[i,4]<=1000 or max(stats[i,2:4])>55:continue
        contours,_=cv2.findContours((labels==i).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contour=max(contours,key=cv2.contourArea)
        quad=cv2.approxPolyDP(contour,.035*cv2.arcLength(contour,True),True).reshape(-1,2)
        if len(quad)!=4:continue
        for rev in (False,True):
            for shift in range(4):
                image=np.roll(quad[::-1] if rev else quad,shift,axis=0).astype(float)
                def residual(v):return np.r_[(project(v)[0]-image).ravel(),v[2:4]*.6]
                fit=least_squares(residual,[hint[0],hint[1],0,0,0],bounds=([hint[0]-.05,hint[1]-.05,-.5,-.5,-.65],[hint[0]+.05,hint[1]+.05,.5,.5,.65]),max_nfev=40)
                error=float(np.linalg.norm(residual(fit.x)))
                if error>6:continue
                _,pw=project(fit.x)
                candidates.append((error,pw,fit.x[2:].copy()))
    if not candidates:return None
    error,pw,angles=min(candidates,key=lambda v:v[0])
    pw[:2]=np.round(pw[:2]/.005)*.005;pw[2]=round(float(pw[2])/.002)*.002
    angles=np.deg2rad(np.round(np.rad2deg(angles)/5)*5)
    return pw,angles,error

def update_visual_plan(rgb,camera,t):
    for job in _VISION_JOBS:
        if job['done'] or t<job['start'] or t>job['end']+.1:continue
        pose=visual_pose(rgb,camera,job['hint'],job['plane'])
        if pose is not None:job['seen'].append(pose)
        if t<job['end']-.05:continue
        job['done']=True
        if job.get('tool_refs'):
            tool=visual_tool(rgb,camera,BRUSH.pos[:2])
            if tool is not None:
                centre,angles,error=tool
                # Follow only the visible field centre (x, y) and heading (capped at 10 degrees). The fitted roll and pitch
                # of this small rectangle are noisy (5 degrees of pitch on the level reset image); following them moved the
                # held potato sideways and tilted it toward one end. Height stays at the declared field height, which
                # keeps the bristle load in the same range as the first face.
                heading=float(np.clip(angles[2],-math.radians(10),math.radians(10)))
                rr=parts._rot((0,0,1),heading)
                fitted=centre;centre=np.array([centre[0],centre[1],BRUSH.pos[2]+.0405])
                for waypoint,offset in job['tool_refs']:
                    waypoint.pos=tuple(centre+rr@offset)
                    waypoint.yaw=tilt._TiltedYaw(heading,rr@tilt.rotation(0.,waypoint.tilt,0.))
                print('[vision-tool] '+json.dumps(dict(t=round(t,2),centre=list(fitted),angles_degrees=list(np.rad2deg(angles)),pixel_error=round(error,2),applied_heading_degrees=math.degrees(heading)),default=float),flush=True)
            else:print('[vision-tool] no visible rectangle; nominal passes retained',flush=True)
        if not job['seen']:
            print('[vision] no visible potato; nominal grasp retained',flush=True);continue
        xy=np.round(np.median([v[0] for v in job['seen']],axis=0)/.005)*.005
        yaw=math.radians(round(math.degrees(float(np.median([v[1] for v in job['seen']])))/5)*5)
        for waypoint,z,align in job['refs']:
            waypoint.pos=(float(xy[0]),float(xy[1]),float(z))
            waypoint.yaw=0. if align else yaw
        print('[vision] '+json.dumps(dict(t=round(t,2),xy=list(xy),yaw_degrees=math.degrees(yaw),pixels=job['seen'][-1][2]),default=float),flush=True)

@parts.on_reset
def on_reset(sim):
    hidden=S.get('hidden',set()).copy();v=S.get('views');sv=S.get('support_views');reset_state();S['views']=v;S['support_views']=sv
    for name in hidden:sim.set_visible('potato',name,True)
    for k in range(12):sim.set_visible('potato',f'damage{k}',False)
@parts.on_step
def on_step(sim):
    if any(not j['done'] and j['start']<=float(sim.t)<=j['end']+.1 for j in _VISION_JOBS):
        try:
            raw=sim.env.unwrapped.scene['top_cam'].data.output['rgb']
            raw=raw.torch if hasattr(raw,'torch') else raw
            update_visual_plan(_arr(raw)[0,:,:,:3],sim.embodiment.rig.top_cam,float(sim.t))
        except Exception as exc:
            print('[vision] camera estimate unavailable: '+repr(exc),flush=True)
    try:
        base=sim.env.unwrapped;st=sim.states();p,b=st['potato'],st['brush'];dt=base.physics_dt
        hp=hand_points(base)
        if S['support_views'] is None:S['support_views']=support_views()
        support=np.concatenate([_arr(v.get_contact_force_matrix(dt=base.physics_dt)).reshape(-1,3) for v in S['support_views']])
        if not np.isfinite(support).all():raise RuntimeError('unreadable support forces')
        S['supported']=bool(support[:,2].sum()>.25)
        if sim.t>.05 and S['physics_steps']<round(sim.t/base.physics_dt)-2:
            raise RuntimeError('missing native physics-step history')
        if S['views'] is None:S['views']=_views()
        view=S['views'];fm=_arr(view.get_contact_force_matrix(dt=dt)).reshape(-1,3)
        if not np.isfinite(fm).all():raise RuntimeError('non-finite bristle contact forces')
        force=float(np.linalg.norm(fm.sum(0)));S['force']=force;S['maxforce']=max(S['maxforce'],force)
        if force>.05:
            f,pts,_,_,cnt,start=(_arr(x) for x in view.get_contact_data(dt=dt))
            n,i=int(cnt.reshape(-1)[0]),int(start.reshape(-1)[0]);w=np.abs(f.reshape(-1)[i:i+n])
            pp=pts.reshape(-1,3)[i:i+n]-parts._np(base.scene.env_origins[0])
            if n<=0 or not np.isfinite(w).all() or not np.isfinite(pp).all() or w.sum()<=0:
                raise RuntimeError('unreadable loaded bristle contact points')
            if n:
                point=(w[:,None]*pp).sum(0)/max(w.sum(),EPS)
                rot=parts._quat_to_matrix(p.quat_xyzw);loc=rot.T@(point-p.pos)
                from scipy.spatial import cKDTree
                world_cells=POINTS@rot.T+p.pos
                S['occluded']=cKDTree(hp).query(world_cells,k=1)[0]<=.005
                br=parts._quat_to_matrix(b.quat_xyzw)
                field=rot.T@(np.array(b.links['bristles'][0])+br@np.array([0,0,-.00875])-p.pos)
                lp,lq=b.links['bristles'];lr=parts._quat_to_matrix(lq)
                record_contact(loc,rot.T@br,force,dt,(rot.T@lr,rot.T@(np.array(lp)-p.pos)),field)
        else:S['prev']=None
        S['clear']=hands_clear(hp,st);S['known']=True
        if bottom(p)<TOP-.01-EPS:S['dropped']=True
        S['hold']=S['hold']+1 if released(st) else 0
        for i in np.flatnonzero(S['travel']>=.025-EPS):
            name=f'soil{i}'
            if name not in S['hidden']:sim.set_visible('potato',name,False);S['hidden'].add(name)
        for k in S['damaged']:sim.set_visible('potato',f'damage{k}',True)
        if int(sim.t*20)%20==0:
            from yam_arena import ik, station
            tcp={}
            for side in ('left','right'):
                asset=base.scene[side+'_robot'];ix=sim.embodiment._ids[side+'_robot'][0]
                q=parts._np(asset.data.joint_pos)[0][ix]
                tcp[side]=[round(float(v),4) for v in ik.tcp_pose(q,station.BASES[side])[0]]
            print('[potato] '+json.dumps(dict(t=round(sim.t,2),clean=clean_count(),force=round(force,3),maxforce=round(S['maxforce'],2),path=round(S['path'],3),damage=S['damage'],dropped=S['dropped'],clear=S['clear'],supported=S['supported'],physics_steps=S['physics_steps'],hold=S['hold'],potato=[round(float(x),4) for x in p.pos],bristles=b.joints,tcp=tcp,quat=list(p.quat_xyzw),cells=[int(np.sum(S['travel'][IDS==k]>=.025-EPS)) for k in range(12)],brush=[round(float(x),4) for x in b.pos]),default=float),flush=True)
    except Exception as exc:
        S['broken']=True;print('[potato] FAILED '+repr(exc),flush=True)

# Reference segments use known layout and coarse RGB poses; no object-state, force or soil feedback.
def wp(dt,p,f,label,yaw=0,lean=0,heading=0):return tilt.Waypoint(dt,tuple(map(float,p)),float(f),label,yaw=yaw,tilt=lean,heading=heading)
def jaw_width(turn):
    """Declared hull width across the jaws after the potato has turned `turn` radians about its long axis."""
    d=np.array([0.,math.cos(turn),math.sin(turn)]);s=POTATO_VERTICES@d
    return float(s.max()-s.min())
def squeeze(turn):
    """Finger target 3.5 mm inside each flat, so every face gets the same squeeze (the 60 mm faces otherwise get 1 mm)."""
    return -(jaw_width(turn)-.007)/2
BIAS=.003  # nominal path offset toward the long tray wall behind the brush
def demo():
    _VISION_JOBS.clear()
    c=np.array(lots.current().pos('potato'));b=np.array(BRUSH.pos);up=np.array([0,0,1.])
    start_heading=float(lots.current().yaw_of('potato'))
    if start_heading>math.pi/4:start_heading-=math.pi  # equivalent parallel-jaw grasp of the visible axis
    pick=c+np.array([0,0,.008]);contact_z=b[2]+.0405+.030+.008-.003
    grip=squeeze(0.)
    left=[wp(2,pick+.08*up,FINGER_OPEN,'over potato',yaw=start_heading),wp(1.5,pick,FINGER_OPEN,'descend',yaw=start_heading),
          wp(1,pick,grip,'grip potato',yaw=start_heading),wp(2,pick+.07*up,grip,'lift',yaw=start_heading),
          wp(1.5,pick+.07*up,grip,'align visible long axis')]
    def scrub(phase):
        first=len(left)
        left.append(wp(2,(b[0],b[1]+BIAS,contact_z+.025),grip,f'over bristles phase {phase}'))
        # Faces 2 and 4 add one constant 18-degree stroke per end over 50 mm (passes 13 and 15).
        for k in range(16 if phase in (1,3) else 12):
            span=.025 if k>=12 else .020
            angle=18. if k>=12 else 12.
            dx=-span if k%2==0 else span
            lean=-angle if k%4<2 else angle
            base_rot=parts._rot((1,0,0),-phase*math.pi/2)
            tip_rot=parts._rot((0,1,0),-math.radians(lean))@base_rot
            radius0=-(POTATO_VERTICES@base_rot.T)[:,2].min()
            radius1=-(POTATO_VERTICES@tip_rot.T)[:,2].min()
            lift=radius1-radius0-.008*(1-math.cos(math.radians(lean)))
            left.append(wp(1.6 if k>=12 else 1.2,(b[0]+dx,b[1]+BIAS,float(contact_z+lift)),grip,f'loaded pass {phase}.{k}',lean=lean,heading=0.))
        left.append(wp(1.5,(b[0]+.020,b[1]+BIAS,contact_z+.040),grip,'lift vertically clear'))
        if phase:
            centre=np.array([b[0],b[1],b[2]+.0405])
            # A located field is scrubbed on its centre line; the nominal path keeps a 3 mm bias toward the wall it rests on.
            _VISION_JOBS[-1]['tool_refs']=[(w,np.array(w.pos)-centre-np.array([0,BIAS,0])) for w in left[first:]]
    def roll(deg,phase):
        nonlocal grip
        a=math.radians(deg)
        dz=math.sqrt((.0325*math.sin(a))**2+(.030*math.cos(a))**2)
        tcp=c.copy();tcp[1]+=.008*math.sin(a);tcp[2]=TRAY_Z+dz+.008*math.cos(a)+.003
        left.extend([wp(1.5,pick+.075*up,grip,'carry back to tray'),
            wp(2.0,tcp,grip,f'roll {deg} degrees {phase}',lean=deg,heading=-math.pi/2),
            wp(.7,tcp,-.034,'release rotated potato',lean=deg,heading=-math.pi/2),
            wp(1.0,tcp+np.array([0,.040,.050]),-.034,'withdraw from new side',lean=deg,heading=-math.pi/2),
            wp(1.5,(.20,.28,TOP+.15),FINGER_OPEN,'clear camera view')])
        t0=sum(w.duration for w in left)
        left.append(wp(.4,(.20,.28,TOP+.15),FINGER_OPEN,'photograph released potato'))
        nominal=parts._rot((1,0,0),-(phase+1)*math.pi/4)
        rad=-(POTATO_VERTICES@nominal.T)[:,2].min()
        z=TRAY_Z+float(rad)+.008
        refs=[];grip=squeeze((phase+1)*math.pi/4)
        for duration,h,opening,label,align in ((1.2,z+.065,FINGER_OPEN,'over observed potato',False),
                (1.2,z,FINGER_OPEN,'descend to observed potato',False),(.8,z,grip,'grip observed potato',False),
                (1.2,z+.065,grip,'lift observed potato',False),(1.,z+.065,grip,'align visible long axis',True)):
            w=wp(duration,(c[0],c[1],h),opening,label);left.append(w);refs.append((w,h,align))
        _VISION_JOBS.append(dict(start=t0+.05,end=t0+.4,hint=tuple(c[:2]),plane=TRAY_Z+float(rad),refs=refs,seen=[],done=False))
    for side in range(4):
        scrub(side)
        if side<3:
            roll(45,2*side);roll(45,2*side+1)
    place=np.array([PLATE.pos[0]+.010,PLATE.pos[1],TOP+.004+.033+.008+.006])
    left.extend([wp(2,place+.06*up,grip,'carry to clean plate'),wp(1.2,place,grip,'set down'),
        wp(.8,place,FINGER_OPEN,'release'),wp(2,(.20,.28,TOP+.15),FINGER_OPEN,'clear'),
        wp(4,(.20,.28,TOP+.15),FINGER_OPEN,'settle')])
    return {'left':left}
