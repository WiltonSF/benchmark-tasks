"""A lidless Hammond 1550G enclosure contains a custom two-screw cord saddle and a connected grey Prysmian
H05VV-F cable. The cable's sheath ends under the loose steel clamp bar. A green Wera stubby handle with a
7 mm socket stands at the left. The box and terminal block are bolted down. The circuit is de-energized.
"""
from __future__ import annotations
import json, math, os, pathlib, sys
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots, tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN, grip_rotation
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.1
X=.280; Y=.010; Z=TOP+.02745; R=.00545
GREY=(.64,.66,.69); STEEL=(.7,.73,.75); BLACK=(.10,.11,.12); GREEN=(.12,.51,.23)
PITCH=.0007; RESIST_TORQUE=.050; SHEATH_START=-.001; SHEATH_GOAL=.012
BAR_BOTTOM=Z+R; BAR_Z=BAR_BOTTOM+.0015; HEAD_BOTTOM=BAR_BOTTOM+.003+.0004
# Geometry follows the maker's true-size envelope; die-cast wall draft is approximated.
body=[parts.box((.222,.146,.004),(0,0,.002),GREY),parts.box((.004,.146,.051),(-.109,0,.0255),GREY),
      parts.box((.004,.146,.051),(.109,0,.0255),GREY),parts.box((.214,.004,.051),(0,.071,.0255),GREY)]
# Split the entry wall around a 16 x 16 mm opening, centred 27.45 mm above the floor reference.
body += [parts.box((.103,.004,.051),(-.0595,-.071,.0255),GREY),parts.box((.103,.004,.051),(.0595,-.071,.0255),GREY),
         parts.box((.016,.004,.01945),(0,-.071,.009725),GREY),parts.box((.016,.004,.01555),(0,-.071,.043225),GREY)]
BOX=parts.Part('enclosure',pos=(X,.015,TOP),shapes=body,friction=(.4,.35))
PANEL=parts.Part('mounting_panel',pos=(X,.015,TOP),shapes=[parts.box((.245,.170,.006),(0,0,.003),(.46,.48,.51))])
# The steel mounting pad is below the floor underside; permanent bolted connections are fixed.
# Keep actual table surface under all moving objects; panel is visual-only where it overlaps the box floor.
PANEL=parts.Part('mounting_panel',pos=(X,.015,TOP),shapes=[parts.box((.245,.170,.003),(0,0,-.0015),(.46,.48,.51))])
saddle_shapes=[parts.box((.0314,.016,.018),(0,0,.013),GREY)]
for sign in (-1,1):
    saddle_shapes += [parts.box((.0057,.016,.018),(sign*.02315,0,.013),GREY),
                     parts.box((.0046,.0057,.018),(sign*.018,-.00515,.013),GREY),
                     parts.box((.0046,.0057,.018),(sign*.018,.00515,.013),GREY)]
SADDLE=parts.Part('saddle',pos=(X,Y,TOP),shapes=saddle_shapes,friction=(.6,.55),contact_offset=.0001)
TERMINAL=parts.Part('terminal_anchor',pos=(X,.069,TOP),shapes=[parts.box((.050,.018,.025),(0,0,.0165),(.12,.34,.70)),
    *[parts.cylinder(.0025,.002,(dx,0,.030),( .7,.7,.7)) for dx in (-.016,0,.016)]])
# A guided axial coordinate proxies the connected cores' bow. No drive moves the cable inward.
# The rigid tail passes through the actual entry and can only be displaced by robot contact.
cord_shapes=[parts.cylinder(R,.230,(0,SHEATH_START-.115,.02745),GREY,axis='y')]
for k,col in enumerate(((.4,.18,.08),(.08,.25,.72),(.18,.65,.16))):
    cord_shapes.append(parts.cylinder(.0019,.060,((k-1)*.0039,.029,.02745),col,axis='y',collide=False))
CABLE=parts.Part('cable',pos=(X,Y,TOP),shapes=[parts.box((.012,.005,.010),(0,.058,.02745),BLACK)],
    joints=[parts.Slider('feed',tuple(cord_shapes),(0,1,0),.035,mass=.052,spring=250.,damping=2.,max_effort=40.,back=.015)],
    friction=(.6,.55),compliance=(100000.,100.),self_collision=False,contact_offset=.0001)
# Two holes, 36 mm apart, are open collision geometry, not painted circles.
bars=[parts.box((.0314,.014,.003),(0,0,0),STEEL)]
for sign in (-1,1):
    dx=sign*.018
    bars += [parts.box((.0047,.014,.003),(sign*.02265,0,0),STEEL),
             parts.box((.0046,.0047,.003),(dx,-.00465,0),STEEL),parts.box((.0046,.0047,.003),(dx,.00465,0),STEEL)]
BAR=parts.Part('clamp_bar',pos=(X,Y,BAR_Z+.0001),mass=.0165,shapes=bars,friction=(.6,.55),contact_offset=.0001)
def hex_shape(af,height,at,color=BLACK):
    return parts.Shape('hex',(af/math.sqrt(3),height,0.),tuple(at),color)
SCREWS=[]
for i,dx in enumerate((-.018,.018)):
    head=[hex_shape(.007,.0032,(0,0,HEAD_BOTTOM+.0016-TOP)),
          parts.cylinder(.002,.025,(0,0,HEAD_BOTTOM-.0125-TOP),STEEL)]
    pivot=(0,0,HEAD_BOTTOM-TOP)
    spin=parts.Hinge('spin',(parts.cylinder(.001,.001,pivot,BLACK,collide=False),),pivot,(0,0,1),-10*math.pi,.002,
                     mass=.005,damping=.002,max_effort=1.)
    slide=parts.Slider('advance',tuple(head),(0,0,-1),.0036,mass=.005,damping=.05,parent='spin',thread=('spin',-PITCH),back=.00001)
    SCREWS.append(parts.Part('screw_'+str(i),pos=(X+dx,Y,TOP),shapes=[parts.box((.006,.006,.004),(0,0,.010),GREY)],
        joints=[spin,slide],friction=(.2,.18),self_collision=False,contact_offset=.0001))
# Maker envelopes: 65 mm holder (54 mm handle), 50 mm bit, estimated 10 mm insertion = 105 mm.
# The reusable bit has a specified shop-machined 1.5 mm entrance taper, not a maker-certified chamfer.
def driver_shapes():
    out=[];outer=.0055*math.cos(math.pi/6)
    for k in range(6):
        a=k*math.pi/3+math.pi/6
        out.append(parts.Shape('hexwall',(.0036,outer,.0055),(0,0,.00425),STEEL,yaw=a))
        out.append(parts.Shape('hexwall',(.0036,outer,.0015),(0,0,.00075),STEEL,yaw=a,taper=.0008))
    out += [parts.cylinder(.0055,.002,(0,0,.008),STEEL),parts.cylinder(.0032,.041,(0,0,.0295),STEEL),
            parts.cylinder(.0055,.011,(0,0,.0455),STEEL),parts.cylinder(.012,.009,(0,0,.0555),BLACK),
            hex_shape(.0294,.040,(0,0,.080),GREEN),parts.cylinder(.014,.005,(0,0,.1025),BLACK)]
    return out
DRIVER=parts.Part('driver',pos=(.155,.185,TOP+.002),mass=.064,shapes=driver_shapes(),friction=(.9,.8),contact_offset=.0001)
# A shop-made tool rest has a through opening for the shaft; only the handle collar supports the tool.
def rest_shapes(x,y):
    return [parts.box((.040,.010,.051),(x,y+sign*.015,.0255),BLACK) for sign in (-1,1)]+[
        parts.box((.010,.020,.051),(x+sign*.015,y,.0255),BLACK) for sign in (-1,1)]
REST_A=parts.Part('rest_a',pos=(.155,.185,TOP),shapes=rest_shapes(0,0),friction=(.5,.4))
REST_B=parts.Part('rest_b',pos=(.215,.205,TOP),shapes=rest_shapes(0,0),friction=(.5,.4))
TRAY_X,TRAY_Y=.200,.360
TRAY=parts.Part('tool_tray',pos=(TRAY_X,TRAY_Y,TOP),shapes=[parts.box((.200,.200,.003),(0,0,.0015),BLACK),
    parts.box((.004,.200,.012),(-.098,0,.009),BLACK),parts.box((.004,.200,.012),(.098,0,.009),BLACK),
    parts.box((.192,.004,.012),(0,-.098,.009),BLACK),parts.box((.192,.004,.012),(0,.098,.009),BLACK)],friction=(.6,.5))
OBJECTS=[PANEL,BOX,SADDLE,TERMINAL,CABLE,BAR,*SCREWS,DRIVER,REST_A,REST_B,TRAY]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'driver':DRIVER.pos}),lots.Lot('B',place={'driver':(.215,.205,TOP+.002)}))
DEMO_TOUCHES={'enclosure','saddle','rest_a','rest_b','tool_tray'}
# Runtime: native bidirectional thread friction, measured contacts/proof pull and sampled release.
H={}; V=None

def array(x):
    if hasattr(x,'torch'):x=x.torch
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    if hasattr(x,'numpy'):return x.numpy()
    return np.asarray(x)

def arrays(values):
    """Snapshot native buffers with one device-to-host copy; retain each input dtype/shape."""
    values=list(values)
    tensors=[x.torch if hasattr(x,'torch') else x for x in values]
    if not tensors:return []
    if any(not hasattr(x,'detach') for x in tensors):
        # Raw PhysX views return Warp arrays; asset-data proxies return torch views.
        if any(not hasattr(x,'detach') and not hasattr(x,'numpy') for x in tensors):
            return [array(x) for x in values]
        import warp as wp
        if any(not hasattr(x,'detach') and not isinstance(x,wp.array) for x in tensors):
            return [array(x) for x in values]
        tensors=[wp.to_torch(x) if isinstance(x,wp.array) else x for x in tensors]
    import torch
    types={torch.float16:np.float16,torch.float32:np.float32,torch.float64:np.float64,
           torch.int16:np.int16,torch.int32:np.int32,torch.int64:np.int64,torch.bool:np.bool_}
    if not tensors or not all(isinstance(x,torch.Tensor) and x.dtype in types for x in tensors):
        return [array(x) for x in values]
    if len({x.device for x in tensors})!=1:return [array(x) for x in values]
    dtype=torch.float64 if any(x.dtype in (torch.float64,torch.int64) for x in tensors) else torch.float32
    data=torch.cat([x.detach().reshape(-1).to(dtype=dtype) for x in tensors]).cpu().numpy()
    out=[];offset=0
    for x in tensors:
        n=x.numel();out.append(data[offset:offset+n].reshape(tuple(x.shape)).astype(types[x.dtype],copy=False));offset+=n
    return out

def set_force(asset,force,body='feed'):
    import torch
    f=torch.zeros((1,len(asset.body_names),3),device=asset.device)
    f[0,asset.body_names.index(body)]=torch.tensor(force,device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=f,torques=torch.zeros_like(f),body_ids=list(range(len(asset.body_names))),is_global=True)

@lru_cache(None)
def samples(link):
    from scipy.spatial import ConvexHull
    from yam_arena import analysis
    rows=[]
    for hull in analysis.link_hulls().get(link,[]):
        vv=hull.vertices
        for ids in ConvexHull(vv).simplices:
            a,b,c=vv[ids];n=max(1,int(math.ceil(max(np.linalg.norm(b-a),np.linalg.norm(c-a),np.linalg.norm(c-b))/.003)))
            rows.extend(a+(i/n)*(b-a)+(j/n)*(c-a) for i in range(n+1) for j in range(n+1-i))
    if not rows:raise ValueError('missing hand surface')
    return np.unique(np.round(rows,5),axis=0)

def hand_clouds(sim):
    base=sim.env.unwrapped;origin=array(base.scene.env_origins[0]);out=[]
    for name in ('left_robot','right_robot'):
        arm=base.scene[name];poses=array(arm.data.body_link_pose_w)[0]
        for link in ('gripper','tip_left','tip_right'):
            p=poses[arm.body_names.index(link)]
            out.append(samples(link)@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
    if len(out)!=6 or any(not np.isfinite(p).all() for p in out):raise ValueError('unknown gripper')
    return out

def release_from(clouds,objects):
    if len(clouds)!=6:return False
    for spec in (CABLE,BAR,*SCREWS,DRIVER):
        s=objects[spec.name];r=parts._quat_to_matrix(s.quat_xyzw)
        poses={'base':(s.pos,r)}|{k:(p,parts._quat_to_matrix(q)) for k,(p,q) in getattr(s,'links',{}).items()}
        for _,c,rr,size in spec.world_boxes(poses):
            if any(np.min(np.linalg.norm(np.maximum(np.abs((p-c)@rr)-size/2,0),axis=1))<=.008 for p in clouds):return False
    return True

def release_clear(sim,objects):
    # A bounding-box distance is a lower bound on every surface-point distance.
    # Skip dense surface sampling only when all six hand bounds prove >8 mm.
    # Bounds include a 0.1 mm margin for sample rounding; near cases use the original test.
    base=sim.env.unwrapped;origin=array(base.scene.env_origins[0]);far=True
    for name in ('left_robot','right_robot'):
        arm=base.scene[name];poses=array(arm.data.body_link_pose_w)[0]
        for link in ('gripper','tip_left','tip_right'):
            p=poses[arm.body_names.index(link)];r=parts._quat_to_matrix(p[3:]);c,h=hand_bound(link)
            center=p[:3]-origin+r@c;half=np.abs(r)@h+.0001
            if not np.isfinite(np.r_[center,half]).all():return False
            for spec in (CABLE,BAR,*SCREWS,DRIVER):
                part=objects[spec.name]
                gap=np.maximum(np.maximum(center-half-part.high,part.low-center-half),0.)
                if not np.isfinite(gap).all():return False
                if np.linalg.norm(gap)<=.008:far=False
    return True if far else release_from(hand_clouds(sim),objects)

def tool_clear(objects):
    driver=objects['driver']
    for spec in (CABLE,BAR,*SCREWS):
        part=objects[spec.name]
        gap=np.maximum(np.maximum(driver.low-part.high,part.low-driver.high),0.)
        if not np.isfinite(gap).all() or np.linalg.norm(gap)<=.006:return False
    return True

def proof_state(objects):
    return {spec.name: {'pos':objects[spec.name].pos.tolist(),'quat':objects[spec.name].quat_xyzw.tolist()} for spec in (BAR,DRIVER)}

def proof_changed(objects):
    ref=H.get('pull_ref')
    if not isinstance(ref,dict):return True
    for spec in (BAR,DRIVER):
        if spec.name not in ref:return True
        old=ref[spec.name];state=objects[spec.name]
        if np.linalg.norm(state.pos-np.asarray(old['pos']))>.0001:return True
        q=np.array(state.quat_xyzw,dtype=float,copy=True);r=np.asarray(old['quat'],float)
        if not np.isfinite(np.r_[state.pos,q,r]).all() or min(np.linalg.norm(q),np.linalg.norm(r))<1e-6:return True
        q/=np.linalg.norm(q);r/=np.linalg.norm(r)
        if 2*math.acos(float(np.clip(abs(q@r),0.,1.)))>.002:return True
    return False

def invalidate_proof(clear,objects):
    if H.get('pull') not in ('passed','failed'):return
    changed=max(abs(objects[s.name].joints['spin']-q) for s,q in zip(SCREWS,H['pull_q']))>.001
    moved=H.get('pull')=='passed' and (abs(objects['cable'].joints['feed']-H.get('pull_result_feed',H['pull_start']))>.0001 or not observable_still(objects) or proof_changed(objects))
    if not clear or changed or moved:
        H['pull']='none';H['pull_q']=None

def interval_gap(a0,a1,b0,b1):
    return float(np.linalg.norm(np.maximum(np.maximum(a0-b1,b0-a1),0.)))

@lru_cache(None)
def hand_bound(link):
    from yam_arena import analysis
    vv=np.concatenate([h.vertices for h in analysis.link_hulls()[link]])
    return (vv.min(0)+vv.max(0))/2,(vv.max(0)-vv.min(0))/2

def pose_motion_rates(previous,current,dt):
    """Physical pose increments per physics step; no averaging or velocity-field bias."""
    c=np.asarray(current,dtype=float)
    if c.shape!=(15,) or not np.isfinite(c).all() or not dt>0:raise ValueError('unknown pose sample')
    for a in (3,10):
        if np.linalg.norm(c[a:a+4])<1e-6:raise ValueError('unknown orientation')
    if previous is None:return None
    p=np.asarray(previous,dtype=float)
    if p.shape!=(15,) or not np.isfinite(p).all():raise ValueError('unknown previous pose')
    rates=[]
    for a in (0,7):
        rates.append(float(np.linalg.norm(c[a:a+3]-p[a:a+3])/dt))
        q=c[a+3:a+7];r=p[a+3:a+7]
        if np.linalg.norm(r)<1e-6:raise ValueError('unknown previous orientation')
        q=q/np.linalg.norm(q);r=r/np.linalg.norm(r)
        chord=min(np.linalg.norm(q-r),np.linalg.norm(q+r))
        rates.append(float(4*math.asin(float(np.clip(chord/2,0.,1.)))/dt))
    rates.append(float(abs(c[14]-p[14])/dt))
    return rates

def observable_still(objects):
    if H.get('armed'):
        rates=H.get('motion_rates')
        return (rates is not None and bool(np.isfinite(rates).all()) and
                bool(np.all(np.asarray(rates)<np.array([.003,.05,.003,.05,.003]))) and
                H.get('physics_still',False) and H.get('feed_speed',math.inf)<.003)
    # Offline injected scorer states have no native pose history.
    return (objects['clamp_bar'].resting(.003,.05) and objects['driver'].resting(.003,.05) and
            H.get('feed_speed',math.inf)<.003 and H.get('physics_still',True))

def mark_bad(reason):
    H['bad']=True;H.setdefault('bad_reason',reason)

def physics_guard(sim):
    if not H.get('armed'):return
    try:
        base=sim.env.unwrapped;cable_asset=base.scene['cable']
        origin,bar_poses,driver_poses,cable_positions,bar_velocities,driver_velocities,cable_velocities=arrays([
            base.scene.env_origins[0],base.scene['clamp_bar'].root_view.get_transforms(),
            base.scene['driver'].root_view.get_transforms(),cable_asset.root_view.get_dof_positions(),
            base.scene['clamp_bar'].root_view.get_velocities(),base.scene['driver'].root_view.get_velocities(),
            cable_asset.root_view.get_dof_velocities()])
        barpose=bar_poses[0]
        bp=barpose[:3]-origin;br=parts._quat_to_matrix(barpose[3:])
        if abs(bp[0]-X)>.0015 or abs(bp[1]-Y)>.0015 or not TOP+.020<=bp[2]<=TOP+.050 or br[2,2]<math.cos(math.radians(30)):
            mark_bad('bar left guided envelope')
        dp=driver_poses[0]
        dr=parts._quat_to_matrix(dp[3:]);dc=dp[:3]-origin+dr@np.array([0,0,.0525]);dh=np.abs(dr)@np.array([.0174,.0174,.0525])
        if dc[2]-dh[2]<TOP-.020:mark_bad('driver fell below table')
        feed=float(cable_positions[0,cable_asset.joint_names.index('feed')])
        bv=bar_velocities[0];dv=driver_velocities[0]
        fv=float(cable_velocities[0,cable_asset.joint_names.index('feed')])
        if not np.isfinite(np.r_[bv,dv,fv]).all():raise ValueError('unknown physics-step velocity')
        speeds=np.array([np.linalg.norm(bv[:3]),np.linalg.norm(bv[3:]),np.linalg.norm(dv[:3]),np.linalg.norm(dv[3:]),abs(fv)])
        H['substep_velocity_peaks']=np.maximum(H.get('substep_velocity_peaks',np.zeros(5)),speeds).tolist()
        sample=np.r_[barpose,dp,feed].astype(float)
        rates=pose_motion_rates(H.get('motion_previous'),sample,float(base.physics_dt))
        H['motion_previous']=sample.tolist();H['motion_rates']=rates
        H['bar_pose']=barpose.tolist();H['driver_pose']=dp.tolist()
        if rates is not None:
            H['substep_motion_peaks']=np.maximum(H.get('substep_motion_peaks',np.zeros(5)),rates).tolist()
            updates=np.array([rates[0]>1e-8 or rates[1]>1e-8,rates[2]>1e-8 or rates[3]>1e-8,rates[4]>1e-8],int)
            H['substep_pose_updates']=(np.asarray(H.get('substep_pose_updates',[0,0,0]))+updates).tolist()
        if H.get('pull') in ('running','settling','passed'):
            H['slip']=max(H['slip'],abs(H['pull_start']-feed))
            if H['slip']>.001+1e-6:
                H['pull']='failed';H['pull_result_feed']=feed;set_force(cable_asset,(0.,0.,0.))
        moving=rates is None or bool(np.any(np.asarray(rates)>=np.array([.003,.05,.003,.05,.003])))
        H['substep_moving']=H.get('substep_moving',False) or moving
        if moving:
            H['held']=0.;H['pull_settle']=0.
            if H.get('pull')=='passed':
                H['pull']='none';H['pull_q']=None;set_force(cable_asset,(0.,0.,0.))

        bh=np.abs(br)@np.array([.025,.007,.0015])
        regions=[(bp-bh,bp+bh),
                 (np.array([X-.0222,Y-.0042,TOP+.008]),np.array([X+.0222,Y+.0042,TOP+.041])),
                 (np.array([X-.0062,Y+SHEATH_START-.230+feed,TOP+.022]),np.array([X+.0062,max(.0705,.069+feed),TOP+.0329]))]
        blocked=any(interval_gap(dc-dh,dc+dh,lo,hi)<=.006 for lo,hi in regions)
        from yam_arena import analysis,station
        checked=0;labels=('left_robot','right_robot');robot_bounds=[]
        robot_positions=arrays([base.scene[label].root_view.get_dof_positions() for label in labels])
        for label,positions in zip(labels,robot_positions):
            arm=base.scene[label];q=positions[0]
            joints={name:float(q[i]) for i,name in enumerate(arm.joint_names)}
            fk=analysis.fk_link_poses_world(joints,station.BASES[label.split('_')[0]])
            for link in analysis.link_hulls():
                pose=fk[link];rr=pose[:3,:3];c,h=hand_bound(link)
                center=pose[:3,3]+rr@c;half=np.abs(rr)@h
                if not np.isfinite(np.r_[center,half]).all():raise ValueError('unknown robot collision link')
                robot_bounds.append((center-half,center+half));checked+=1
        # Same AABB lower-bound distances, evaluated together instead of 72 tiny calls.
        robot_bounds=np.asarray(robot_bounds);obstacles=np.asarray(regions+[(dc-dh,dc+dh)])
        gaps=np.maximum(np.maximum(robot_bounds[:,0,None,:]-obstacles[None,:,1,:],
                                   obstacles[None,:,0,:]-robot_bounds[:,1,None,:]),0.)
        blocked |= bool(np.any(np.linalg.norm(gaps,axis=2)<=.006))
        H['robot_collision_links_checked']=checked
        if H.get('pull')=='passed' and abs(feed-H['pull_result_feed'])>.0001:
            H['pull']='none';H['pull_q']=None
        if H.get('pull')=='passed':
            ref=H.get('pull_ref',{})
            from types import SimpleNamespace
            pp={BAR.name:SimpleNamespace(pos=bp,quat_xyzw=barpose[3:]),DRIVER.name:SimpleNamespace(pos=dp[:3]-origin,quat_xyzw=dp[3:])}
            if proof_changed(pp):H['pull']='none';H['pull_q']=None
        H['substep_count']=H.get('substep_count',0)+1
        H['substep_blocked']=H.get('substep_blocked',False) or bool(blocked)
        if blocked:
            H['held']=0.
            if H.get('pull') in ('passed','failed'):H['pull']='none';H['pull_q']=None
        # Read each physics-step contact report, including peaks between control samples.
        from types import SimpleNamespace
        centers={}
        screw_poses=arrays([base.scene[spec.name].root_view.get_link_transforms() for spec in SCREWS])
        for spec,poses in zip(SCREWS,screw_poses):
            art=base.scene[spec.name];pose=poses[0,art.body_names.index('advance')]
            centers[spec.name]=SimpleNamespace(links={'advance':(pose[:3]-origin,None)})
        # Whole-trial guide retention uses the live screw spans, not only the bar's box envelope.
        for spec,dx in zip(SCREWS,(-.018,.018)):
            hole=bp+br@np.array([dx,0.,0.]);axis=centers[spec.name].links['advance'][0]
            joint=next(j for j in spec.joints if j.name=='advance')
            head_bottom=axis[2]+HEAD_BOTTOM-TOP-joint.origin[2]
            if (np.linalg.norm(hole[:2]-axis[:2])>.0015 or
                hole[2]-.0015*abs(br[2,2])>head_bottom+.0032+.0002 or
                hole[2]+.0015*abs(br[2,2])<head_bottom-.025-.0002):mark_bad('bar hole left live screw span')
        if H.get('pull') in ('running','settling','passed','failed') and H.get('pull_q') is not None:
            screw_positions=arrays([base.scene[spec.name].root_view.get_dof_positions() for spec in SCREWS])
            for spec,ref,positions in zip(SCREWS,H['pull_q'],screw_positions):
                asset=base.scene[spec.name]
                q=float(positions[0,asset.joint_names.index('spin')])
                if abs(q-ref)>.002:
                    H['pull']='none';H['pull_q']=None;set_force(cable_asset,(0.,0.,0.));break
        _,torque,_=contact_metrics(sim,centers,origin)
        H['peak_torque']=np.maximum(H['peak_torque'],abs(torque)).tolist()
    except Exception as e:
        H['bad']=True;H['known']=False;H['physics_error']=repr(e)
        if not H.get('physics_printed'):
            import traceback;traceback.print_exc();H['physics_printed']=True

def install_guard(sim):
    engine=sim.env.unwrapped.sim
    if getattr(engine,'_cord_guard_wrapped',False):return
    original=engine.step
    def guarded_step(*args,**kwargs):
        result=original(*args,**kwargs)
        physics_guard(sim)
        return result
    engine.step=guarded_step;engine._cord_guard_wrapped=True

def configure_thread(asset):
    # Native bidirectional Coulomb friction, not a moving one-way stop or a forward drive.
    # Isaac Sim >=5 defines static/dynamic coefficients here as maximum friction effort.
    import torch
    ids=asset.find_joints(['spin'])[0]
    asset.write_joint_position_limit_to_sim_index(limits=torch.tensor([[[-10*math.pi,.002]]],device=asset.device),joint_ids=ids,warn_limit_violation=False)
    asset.set_joint_effort_target_index(target=torch.zeros((1,1),device=asset.device),joint_ids=ids)
    effort=torch.full((1,len(ids)),RESIST_TORQUE,dtype=torch.float32,device=asset.device)
    asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=effort,
        joint_dynamic_friction_coeff=effort,joint_viscous_friction_coeff=torch.zeros_like(effort),joint_ids=ids)

@parts.on_pre_reset
def pre_reset(sim):
    H['armed']=False
    for screw in SCREWS:configure_thread(sim.env.unwrapped.scene[screw.name])
    set_force(sim.env.unwrapped.scene['cable'],(0.,0.,0.))

@parts.on_reset
def reset(sim):
    global H,V
    H=dict(known=False,bad=False,t=float(sim.t),initial_t=float(sim.t),held=0.,clear=False,pull='none',pull_start=None,pull_t=0.,slip=0.,
           pull_q=None,peak_torque=[0.,0.],end_forces=[0.,0.],cable_force=0.,finger_force_measurement=None,last_feed=0.,last_bar=None,armed=False,substep_blocked=False,substep_moving=False,physics_still=False,substep_count=0)
    V=None
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    try:
        from isaaclab_physx.physics import PhysxManager
        from pxr import PhysxSchema
        from isaaclab.sim.utils import get_current_stage
        cfg=getattr(sim.env.unwrapped.sim,'cfg',None)
        path=getattr(cfg,'physics_prim_path','/physicsScene')
        H['solver_type']=PhysxSchema.PhysxSceneAPI.Get(get_current_stage(),path).GetSolverTypeAttr().Get()
        if H['solver_type']!='TGS':raise ValueError('Expected TGS configured before scene creation')
        roots=parts._PATHS
        sources=[roots[s.name]+'/advance' for s in SCREWS]+[roots[BAR.name],roots[CABLE.name]+'/feed']
        filters=[[roots[DRIVER.name],roots[BAR.name],roots[CABLE.name]+'/feed',roots[SADDLE.name]]]*4
        V=PhysxManager.get_physics_sim_view().create_rigid_contact_view(sources,filter_patterns=filters,max_contact_data_count=512)
        install_guard(sim);H['armed']=True
        for screw in SCREWS:configure_thread(sim.env.unwrapped.scene[screw.name])
    except Exception as e:
        H['bad']=True;H['error']=repr(e)
        import traceback;traceback.print_exc()

def contact_metrics(sim,objects,origin=None):
    if V is None:raise ValueError('missing native contacts')
    f,p,n,sep,cnt,start=arrays(V.get_contact_data(dt=float(sim.env.unwrapped.physics_dt)))
    f=f.reshape(-1);p=p.reshape(-1,3);n=n.reshape(-1,3);cnt=cnt.reshape(4,4);start=start.reshape(4,4)
    if int(cnt.sum())>=len(f):raise ValueError('contact buffer overflow')
    if origin is None:origin=array(sim.env.unwrapped.scene.env_origins[0])
    end=np.zeros(2);torque=np.zeros(2);clamp=0.
    for row in range(4):
        for col in range(4):
            a=int(start[row,col]);nn=int(cnt[row,col])
            for k in range(a,a+nn):
                if not np.isfinite(np.r_[f[k],p[k],n[k]]).all():raise ValueError('nonfinite contact')
                force=f[k]*n[k];point=p[k]-origin
                if row<2 and col==0:
                    center=objects[SCREWS[row].name].links['advance'][0]
                    torque[row]+=float(np.cross(point-center,force)[2])
                if row<2 and col==1:end[row]+=abs(float(force[2]))
                if row==2 and col==2:clamp+=abs(float(force[2]))
    return end,torque,clamp

def measure(objects):
    cable=objects['cable'];bar=objects['clamp_bar'];r=parts._quat_to_matrix(bar.quat_xyzw)
    feed=cable.joints['feed'];sheath=SHEATH_START+feed
    ends=np.array([bar.pos+r@np.array([dx,0,0]) for dx in (-.018,.018)])
    heads=np.array([objects[s.name].links['advance'][0][2] for s in SCREWS])
    aligned=abs(bar.pos[0]-X)<=.0015 and abs(bar.pos[1]-Y)<=.0015 and r[2,2]>=math.cos(math.radians(30))
    covered=(sheath>=SHEATH_GOAL-1e-6 and Y+sheath>=bar.high[1]+.005-1e-6)
    head_bottoms=[]
    for spec in SCREWS:
        origin=next(j.origin for j in spec.joints if j.name=='advance')
        head_bottoms.append(objects[spec.name].links['advance'][0][2]+HEAD_BOTTOM-TOP-origin[2])
    gaps=np.asarray(head_bottoms)-(ends[:,2]+.0015)
    seated=(np.array(H.get('end_forces',[0.,0.]))>=.1)&(gaps<=.0002+1e-6)&(gaps>=-.0005)
    level=abs(ends[0,2]-ends[1,2])<=.001+1e-6
    driver=objects['driver']
    park=bool(driver.low[0]>=TRAY_X-.096-1e-6 and driver.high[0]<=TRAY_X+.096+1e-6 and driver.low[1]>=TRAY_Y-.096-1e-6 and driver.high[1]<=TRAY_Y+.096+1e-6 and TOP+.002-1e-6<=driver.low[2]<=TOP+.006+1e-6)
    still=observable_still(objects)
    return dict(bar_velocity=bar.lin_vel.tolist(),bar_angular_velocity=bar.ang_vel.tolist(),driver_velocity=driver.lin_vel.tolist(),driver_angular_velocity=driver.ang_vel.tolist(),bar_speed=float(np.linalg.norm(bar.lin_vel)),bar_spin=float(np.linalg.norm(bar.ang_vel)),driver_speed=float(np.linalg.norm(driver.lin_vel)),driver_spin=float(np.linalg.norm(driver.ang_vel)),feed=feed,sheath=sheath,ends=ends.tolist(),aligned=bool(aligned),covered=bool(covered),seated=seated.tolist(),level=bool(level),parked=bool(park),still=bool(still),head_gaps=gaps.tolist())

@parts.on_step
def step(sim):
    if not H:return
    t=float(sim.t);dt=t-H['t'];objects=sim.states();H['t']=t
    try:
        if not 0<dt<=.101:raise ValueError('bad sample time')
        end,tq,clamp=contact_metrics(sim,objects)
        H['known']=True;H['clear']=not H.get('substep_blocked',True) and tool_clear(objects) and release_clear(sim,objects);H['substep_blocked']=False;H['end_forces']=end.tolist();H['cable_force']=clamp
        H['peak_torque']=np.maximum(H['peak_torque'],abs(tq)).tolist()
        feed=objects['cable'].joints['feed'];H['feed_speed']=abs(feed-H['last_feed'])/dt;H['last_feed']=feed
        # Thread friction is solved natively in both directions; no per-step motor or limit edits.
        if t-H['initial_t']<=.5:
            initial_spin=[float(objects[s.name].joints['spin']) for s in SCREWS]
            H['initial_spin']=initial_spin
            H['initial_head_gaps']=measure(objects)['head_gaps']
            if max(abs(q) for q in initial_spin)>.05:
                H['bad']=True;H['reset_error']='screw spin outside initial loose-state tolerance'
        H['physics_still']=not H.get('substep_moving',True);H['substep_moving']=False
        H['velocity_peaks']=H.get('substep_velocity_peaks');H['substep_velocity_peaks']=[0.]*5
        H['motion_peaks']=H.get('substep_motion_peaks');H['substep_motion_peaks']=[0.]*5
        H['pose_updates']=H.get('substep_pose_updates');H['substep_pose_updates']=[0,0,0]
        m=measure(objects)
        if not m['aligned'] or objects['driver'].low[2]<TOP-.020:mark_bad('invalid assembly or driver geometry')
        eligible=H['clear'] and m['still']
        H['held']=H['held']+dt if eligible else 0.
        # Proof test physically loads the cable's actual free body coordinate, reacting at the fixed terminal/box.
        asset=sim.env.unwrapped.scene['cable'];force=0.
        if H['pull']=='none' and H['held']>=1. and m['covered'] and any(m['seated']):
            H['pull']='running';H['pull_t']=t;H['pull_start']=feed;H['slip']=0.
            H['pull_q']=[objects[s.name].joints['spin'] for s in SCREWS]
        if H['pull']=='running':
            elapsed=t-H['pull_t'];force=-30.*min(1.,elapsed/1.)
            H['slip']=max(H['slip'],abs(H['pull_start']-feed))
            if not H['clear']:H['pull']='none';force=0.
            elif H['slip']>.001+1e-6:H['pull']='failed';H['pull_result_feed']=feed;force=0.
            elif elapsed>=1.5:H['pull']='settling';H['pull_settle']=0.;force=0.
        elif H['pull']=='settling':
            H['slip']=max(H['slip'],abs(H['pull_start']-feed))
            if not H['clear']:H['pull']='none'
            elif H['slip']>.001+1e-6:H['pull']='failed';H['pull_result_feed']=feed
            else:
                H['pull_settle']=H['pull_settle']+dt if eligible else 0.
                if H['pull_settle']>=1.:
                    H['pull']='passed';H['pull_result_feed']=feed;H['pull_ref']=proof_state(objects)
                    H['pull_q']=[objects[s.name].joints['spin'] for s in SCREWS]
        elif H['pull'] in ('passed','failed'):
            invalidate_proof(H['clear'],objects)
        set_force(asset,(0.,force,0.))
        H['pull_force']=force
    except Exception as e:
        H['known']=False;H['bad']=True;H['error']=repr(e)
        if not H.get('printed'):
            import traceback;traceback.print_exc();H['printed']=True
    if os.environ.get('TRACE_OUT'):
        row={k:v for k,v in H.items() if k not in ('certificate','printed')}
        try:
            row.update(measure(objects),score=score(objects))
            from yam_arena.constants import TCP_OFFSET
            origin=array(sim.env.unwrapped.scene.env_origins[0]);row['hands']={}
            for label in ('left_robot','right_robot'):
                arm=sim.env.unwrapped.scene[label];pose=array(arm.data.body_link_pose_w)[0,arm.body_names.index('gripper')]
                row['hands'][label]=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist()
        except Exception:pass
        with pathlib.Path(os.environ['TRACE_OUT']).with_name('mechanism.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
    try:
        if not H.get('known') or H.get('bad') or not H.get('clear') or H.get('held',0)<1.-1e-9:return 0.
        m=measure(objects)
        if not m['aligned'] or not m['covered'] or not m['still'] or not any(m['seated']):return 0.
        value=.25
        if all(m['seated']):value=.5
        if all(m['seated']) and H.get('pull')=='passed' and not proof_changed(objects):
            value=.75
            if m['level'] and m['parked']:value=1.
        return value
    except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

def demo():
    # Only the published, camera-visible driver location determines the starting pick point.
    # No screw/cable state, joint angle or internal completion signal is consulted.
    pick=np.array(lots.current().pos('driver'));plan={'left':[],'right':[]}
    def w(side,t,p,f,label,yaw=0.,lean=0.,heading=0.):
        plan[side].append(tilt.Waypoint(t,tuple(float(v) for v in p),f,label,yaw=yaw,tilt=lean,heading=heading))
    grip=pick+np.array([0,0,.089]);travel=TOP+.165
    w('left',2.,(grip[0],grip[1],travel),FINGER_OPEN,'approach driver')
    w('left',1.5,grip,FINGER_OPEN,'lower around handle')
    w('left',1.,grip,-.010,'grip driver')
    w('left',1.5,(grip[0],grip[1],travel),-.010,'lift driver')
    # Cable grasp uses physical 10.9 mm sheath; tilted wrist stays outside enclosure.
    cp=np.array([X,-.165,Z+.006]);target=cp+np.array([0,.025,0])
    w('right',2.,cp+[0,0,.055],FINGER_OPEN,'approach cable',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('right',1.5,cp,FINGER_OPEN,'lower around sheath',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('right',1.,cp,-.002,'hold sheath',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('right',2.,target,-.002,'advance sheath',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('left',2.,(X-.018,Y,travel),-.010,'over first screw')
    # Thirteen 120-degree strokes per screw; lift and reset between strokes. All physical engagement.
    tool_offset=.089
    for round_ in range(13):
        for dx in (-.018,.018):
            zhead=HEAD_BOTTOM+.0032-(round_/3)*PITCH
            socket=np.array([X+dx+.0020,Y-.0015,zhead+tool_offset-.0055])
            # Fixed 2 mm forward / 1.5 mm right compensation measured in native camera-visible tool/head alignment; no pose feedback.
            w('left',1.,socket+[0,0,.016],-.010,'over socket',yaw=math.pi/3)
            w('left',.8,socket,-.010,'engage socket',yaw=math.pi/3)
            w('left',.5,socket,-.010,'seat socket on its chamfer',yaw=math.pi/3)
            w('left',1.5,socket+[0,0,-.001],-.010,'turn screw clockwise',yaw=-math.pi/3)
            w('left',.6,socket+[0,0,.016],-.010,'lift socket',yaw=-math.pi/3)
            w('left',.6,socket+[0,0,.016],-.010,'reset wrist',yaw=math.pi/3)
    duration=sum(p.duration for p in plan['left'])
    w('right',duration-6.5,target,-.002,'hold against spring',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('right',1.,target,FINGER_OPEN,'release cable',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('right',2.,target+[0,-.035,.080],FINGER_OPEN,'withdraw cable hand',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('right',15.,target+[0,-.035,.080],FINGER_OPEN,'hands clear',yaw=math.pi/2,lean=35.,heading=math.pi/2)
    w('left',1.,(X+.018,Y,travel),-.010,'withdraw driver')
    stow=np.array([TRAY_X,TRAY_Y,TOP+.093])
    w('left',2.,(TRAY_X,TRAY_Y,travel),-.010,'carry driver to tool tray')
    w('left',1.5,stow,-.010,'lower into tool tray')
    w('left',1.,stow,FINGER_OPEN,'release driver in tray')
    w('left',2.,(TRAY_X,TRAY_Y,travel),FINGER_OPEN,'withdraw driver hand')
    w('left',17.,(TRAY_X,TRAY_Y,travel),FINGER_OPEN,'proof pull and still hold')
    return plan
