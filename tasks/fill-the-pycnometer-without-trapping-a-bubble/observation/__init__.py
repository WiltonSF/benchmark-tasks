"""Native sampled hand loads and full face-sampled hand geometry."""
import math
from functools import lru_cache
import numpy as np
import parts
# Instrument capacity, not a force or scoring threshold. Saturation still invalidates.
CONTACT_BUFFER_CAPACITY=8192
TIP_BUFFER_CAPACITY=16384
_tip_contact_view=None;_tip_net_view=None;_tip_filter_count=0
last_tip_loads=np.zeros(4)
last_closing_axes=np.zeros((2,3))
last_hold_forces=np.zeros((4,2))
last_hold_normal_vectors=np.zeros((4,2,3));last_hold_friction_vectors=np.zeros((4,2,3));last_hold_total_vectors=np.zeros((4,2,3))
last_glass_contact=0.
last_glass_pair=np.zeros(2);last_glass_vectors=np.zeros((2,3))
_hold_contact_view=None

contact_format_counts={'signed_normal_rows':0,'zero_force_zero_normal_rows':0}

def _array(x):
    # Copy now: PhysX reuses buffers and normal/friction counts share storage.
    x=x.torch if hasattr(x,"torch") else x
    if hasattr(x,"detach"):x=x.detach().cpu().numpy()
    elif hasattr(x,"numpy"):x=x.numpy()
    return np.array(x,copy=True)

def _sum_pairs(values,counts,starts,shape,normals=None):
    values=np.asarray(values);counts=np.asarray(counts);starts=np.asarray(starts)
    if counts.shape!=shape or starts.shape!=shape or not np.issubdtype(counts.dtype,np.integer) or not np.issubdtype(starts.dtype,np.integer) or (counts<0).any():
        raise ValueError('unknown detailed contact indexing')
    if normals is None:
        if values.ndim!=2 or values.shape[1]!=3:raise ValueError('unknown friction vector buffer')
    else:
        values=values.reshape(-1)
        if normals.shape!=(len(values),3):raise ValueError('unknown normal contact buffer')
    result=np.zeros((*shape,3));capacity=len(values)
    for ij in np.ndindex(shape):
        n=int(counts[ij]);start=int(starts[ij])
        if not n:continue # Unused start indices and unused buffer entries are unspecified.
        if start<0 or start+n>=capacity:raise ValueError(f'invalid or saturated contact buffer: start={start}, count={n}, capacity={capacity}, pairs={shape}')
        contact_format_counts['max_active_end_index']=max(contact_format_counts.get('max_active_end_index',0),start+n)
        contact_format_counts['min_allocated_capacity']=min(contact_format_counts.get('min_allocated_capacity',capacity),capacity)
        v=values[start:start+n]
        if normals is not None:
            normal=normals[start:start+n]
            if not np.isfinite(normal).all():raise ValueError('nonfinite active contact normals')
            # Scalars are signed for the reverse sensor/filter ordering.
            # A zero-force row can have a zero normal; it contributes no force.
            live=np.abs(v)>1e-12;lengths=np.linalg.norm(normal,axis=1)
            contact_format_counts['signed_normal_rows']+=int((v<0).sum())
            contact_format_counts['zero_force_zero_normal_rows']+=int(((~live)&(lengths==0)).sum())
            if not np.allclose(lengths[live],1.,atol=1e-4):raise ValueError('nonunit active contact normal:'+repr(lengths[live].tolist()))
            v=v[:,None]*normal
        if not np.isfinite(v).all():raise ValueError('unknown detailed contact force')
        result[ij]=v.sum(axis=0)
    return result

def contact_components(view,dt,shape):
    """Reconstruct normal and friction independently; never assume matrix semantics.

    IsaacSim6.1 matrix is normal-only; newer backends may include friction.
    Normal counts MUST be copied before get_friction_data overwrites them.
    """
    aggregate=_array(view.get_contact_force_matrix(dt=dt))
    if aggregate.shape!=(*shape,3) or not np.isfinite(aggregate).all():raise ValueError('unknown aggregate contact force')
    raw=view.get_contact_data(dt=dt)
    normal=_sum_pairs(_array(raw[0]),_array(raw[4]),_array(raw[5]),shape,_array(raw[2]))
    raw=view.get_friction_data(dt=dt)
    friction=_sum_pairs(_array(raw[0]),_array(raw[2]),_array(raw[3]),shape)
    total=normal+friction
    if not (np.allclose(aggregate,normal,atol=5e-4,rtol=1e-4) or np.allclose(aggregate,total,atol=5e-4,rtol=1e-4)):
        raise ValueError('inconsistent/truncated native contact forces')
    return normal,friction,total

last_glass_normal_vectors=np.zeros((2,3));last_glass_friction_vectors=np.zeros((2,3))

@lru_cache(maxsize=1)
def hand_mesh():
    from yam_arena import analysis
    out={}
    for name in ('gripper','tip_left','tip_right'):
        cloud=[]
        for h in analysis.link_hulls().get(name,[]):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7),axis=0):
                face=h.vertices[np.abs(h.vertices@pl[:3]-pl[3])<2e-6]
                if len(face)<3:raise ValueError('unknown hull face')
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(pl[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for a,b in zip(face,np.roll(face,-1,axis=0)):
                    n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                    for i in range(n+1):
                        for j in range(n-i+1):cloud.append(c+(a-c)*(i/n)+(b-c)*(j/n))
        if not cloud:raise ValueError('missing hand geometry')
        cloud=np.asarray(cloud)
        _,keep=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True)
        out[name]=cloud[keep]
    return out

def fingertip_contact_forces(sim):
    global _tip_contact_view,_tip_net_view,_tip_filter_count
    base=sim.env.unwrapped
    if _tip_contact_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths();env_path=base.scene.env_prim_paths[0]
        paths=[env_path+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('tip_left','tip_right')]
        from pxr import UsdPhysics
        import omni.usd
        stage=omni.usd.get_context().get_stage();targets=[]
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):targets.append(str(prim.GetPath()))
            elif prim.HasAPI(UsdPhysics.CollisionAPI):
                parent=prim.GetParent();has_body=False
                while parent and not parent.IsPseudoRoot():
                    if parent.HasAPI(UsdPhysics.RigidBodyAPI):has_body=True;break
                    parent=parent.GetParent()
                if not has_body:targets.append(str(prim.GetPath()))
        if not targets or len(targets)!=len(set(targets)):raise ValueError('unknown global contact filters')
        manager=PhysxManager.get_physics_sim_view()
        _tip_net_view=manager.create_rigid_contact_view(paths,filter_patterns=[],max_contact_data_count=0)
        _tip_contact_view=manager.create_rigid_contact_view(paths,filter_patterns=[targets for _ in paths],max_contact_data_count=TIP_BUFFER_CAPACITY)
        _tip_filter_count=len(targets)
    normal,friction,total=contact_components(_tip_contact_view,base.sim.get_physics_dt(),(4,_tip_filter_count))
    n=normal.sum(axis=1);t=total.sum(axis=1)
    unfiltered=_array(_tip_net_view.get_net_contact_forces(dt=base.sim.get_physics_dt()))
    if unfiltered.shape!=(4,3) or not np.isfinite(unfiltered).all() or not (np.allclose(unfiltered,n,atol=5e-4,rtol=1e-4) or np.allclose(unfiltered,t,atol=5e-4,rtol=1e-4)):
        raise ValueError('missing or duplicated fingertip contact coverage')
    # Preserve the original normal-force guard and also guard total contact load.
    return np.where((np.linalg.norm(n,axis=1)>=np.linalg.norm(t,axis=1))[:,None],n,t)

def hand_state(sim):
    global last_tip_loads,last_closing_axes
    base=sim.env.unwrapped
    origin=parts._np(base.scene.env_origins[0]);clouds=[];finger=0.
    if origin.shape!=(3,) or not np.isfinite(origin).all():raise ValueError('unknown environment origin')
    for arm_index,arm in enumerate(('left_robot','right_robot')):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        if poses.ndim!=2 or poses.shape[1]!=7 or not np.isfinite(poses).all() or np.any(np.linalg.norm(poses[:,3:],axis=1)<1e-6):raise ValueError('unknown robot body poses')
        if not np.isfinite(parts._np(asset.data.joint_pos)).all():raise ValueError('unknown robot joint state')
        axis=poses[asset.body_names.index('tip_right'),:3]-poses[asset.body_names.index('tip_left'),:3]
        if np.linalg.norm(axis)<1e-5:raise ValueError('unknown pad closing axis')
        last_closing_axes[arm_index]=axis/np.linalg.norm(axis)
        for name,p in hand_mesh().items():
            pose=poses[asset.body_names.index(name)]
            if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all():raise ValueError('unknown hand mesh')
            cloud=p@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin
            if not np.isfinite(cloud).all():raise ValueError('unknown hand surface')
            clouds.append(cloud)
        # Native actuator/joint effort is recorded; this is not a calibration of
        # grip force. These20Hz samples flag simulator feasibility separately from the physical endpoint score.
        raw=asset.root_view.get_dof_projected_joint_forces()
        raw=raw.torch if hasattr(raw,'torch') else raw
        efforts=raw.detach().cpu().numpy()[0] if hasattr(raw,'detach') else np.asarray(raw.numpy())[0]
        values=[float(efforts[asset.joint_names.index(n)]) for n in ('joint7','joint8')]
        if not np.isfinite(values).all():raise ValueError('unknown finger forces')
        finger=max(finger,max(map(abs,values)))

    last_tip_loads=np.linalg.norm(fingertip_contact_forces(sim),axis=1)
    finger=max(finger,float(last_tip_loads.max()))
    return clouds,finger

def reset():
    contact_format_counts.clear();contact_format_counts.update(signed_normal_rows=0,zero_force_zero_normal_rows=0)
    global _tip_contact_view,_tip_net_view,_tip_filter_count,_arm_contact_view,_support_contact_view,_hold_contact_view,last_tip_loads,last_closing_axes,last_hold_forces,last_hold_normal_vectors,last_hold_friction_vectors,last_hold_total_vectors,last_glass_contact,last_glass_pair,last_glass_vectors,last_glass_normal_vectors,last_glass_friction_vectors
    _tip_contact_view=None;_tip_net_view=None;_tip_filter_count=0;_arm_contact_view=None;_support_contact_view=None
    _hold_contact_view=None;last_tip_loads=np.zeros(4);last_closing_axes=np.zeros((2,3));last_hold_forces=np.zeros((4,2))
    last_hold_normal_vectors=np.zeros((4,2,3));last_hold_friction_vectors=np.zeros((4,2,3));last_hold_total_vectors=np.zeros((4,2,3))
    last_glass_contact=0.;last_glass_pair=np.zeros(2);last_glass_vectors=np.zeros((2,3));last_glass_normal_vectors=np.zeros((2,3));last_glass_friction_vectors=np.zeros((2,3))

def counterhold_forces(sim):
 """Resolved pad forces against each glass piece,projected on each pair's closing axis."""
 global _hold_contact_view,last_hold_forces,last_hold_normal_vectors,last_hold_friction_vectors,last_hold_total_vectors
 base=sim.env.unwrapped
 if _hold_contact_view is None:
  from isaaclab_physx.physics import PhysxManager
  from yam_arena.robot import asset_link_paths
  root=base.scene.env_prim_paths[0];links=asset_link_paths()
  paths=[root+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('tip_left','tip_right')]
  _hold_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[root+'/density_bottle',root+'/perforated_stopper'] for _ in paths],max_contact_data_count=CONTACT_BUFFER_CAPACITY)
 value,friction,total=contact_components(_hold_contact_view,base.sim.get_physics_dt(),(4,2))
 last_hold_normal_vectors=value.copy();last_hold_friction_vectors=friction.copy();last_hold_total_vectors=total.copy()
 if not np.isfinite(last_closing_axes).all():raise ValueError('unknown counterholding axis')
 axes=np.repeat(last_closing_axes,2,axis=0)
 last_hold_forces=np.abs(np.einsum('ijk,ik->ij',value,axes))
 return last_hold_forces

_arm_contact_view=None
def nonfinger_contact_load(sim):
 """Sample actual housing/forearm-to-glass contacts as a conservative impact proxy."""
 global _arm_contact_view
 base=sim.env.unwrapped
 if _arm_contact_view is None:
  from isaaclab_physx.physics import PhysxManager
  from yam_arena.robot import asset_link_paths
  links=asset_link_paths();env_path=base.scene.env_prim_paths[0]
  paths=[env_path+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('gripper','link1','link2','link3','link4','link5')]
  _arm_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[env_path+'/density_bottle',env_path+'/perforated_stopper'] for _ in paths],max_contact_data_count=CONTACT_BUFFER_CAPACITY)
 normal,friction,total=contact_components(_arm_contact_view,base.sim.get_physics_dt(),(12,2))
 return float(max(np.linalg.norm(normal,axis=2).max(),np.linalg.norm(total,axis=2).max()))

_support_contact_view=None
def support_loads(sim):
 """Actual glass contacts with the kit or other glass; arms are excluded."""
 global _support_contact_view,last_glass_contact,last_glass_pair,last_glass_vectors,last_glass_normal_vectors,last_glass_friction_vectors
 targets=['water_bath','stopper_rack/s0','stopper_rack/s1','stopper_rack/s2','stopper_rack/s3','stopper_rack/s4','fill_start_shelf/s0','inspection_pad/s0','bottle_rest_pad/s0','bath_locators/s0','bath_locators/s1','bath_locators/s2','bath_locators/s3','density_bottle','perforated_stopper']
 base=sim.env.unwrapped
 if _support_contact_view is None:
  from isaaclab_physx.physics import PhysxManager
  root=base.scene.env_prim_paths[0]
  paths=[root+'/'+n for n in ('density_bottle','perforated_stopper')]
  _support_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[root+'/'+n for n in targets] for _ in paths],max_contact_data_count=CONTACT_BUFFER_CAPACITY)
 normal,friction,total=contact_components(_support_contact_view,base.sim.get_physics_dt(),(2,len(targets)))
 pair=(targets.index('perforated_stopper'),targets.index('density_bottle'))
 last_glass_normal_vectors=np.array((normal[0,pair[0]],normal[1,pair[1]]))
 last_glass_friction_vectors=np.array((friction[0,pair[0]],friction[1,pair[1]]))
 last_glass_vectors=last_glass_normal_vectors+last_glass_friction_vectors
 last_glass_pair=np.linalg.norm(last_glass_normal_vectors,axis=1)
 last_glass_contact=float(last_glass_pair.min()) # Preserve normal contact/seal threshold.
 return np.maximum(np.linalg.norm(normal,axis=2),np.linalg.norm(total,axis=2)).max(axis=1)
 # Maximum per filter prevents duplicate static-filter support inflation.


def compressive_axial(bottle_force,stopper_force,axis):
 """Conservative measured compression on BOTH parts,not pair-force magnitude."""
 b=np.asarray(bottle_force);s=np.asarray(stopper_force);a=np.asarray(axis)
 if b.shape!=(3,) or s.shape!=(3,) or a.shape!=(3,) or not np.isfinite(np.r_[b,s,a]).all():raise ValueError('unknown axial glass contact')
 return max(0.,min(-float(b@a),float(s@a)))

def stopper_hand_press(axis,lr,rl):
 """Actual downward pad input on stopper,not its weight or a glass-impact proxy.

 Contact vectors act on fingertip sensors;positive axial reaction on the
 stopper-holding fingertips is equal/opposite downward force on the stopper.
 """
 a=np.asarray(axis);v=np.asarray(last_hold_total_vectors)
 if a.shape!=(3,) or v.shape!=(4,2,3) or not np.isfinite(np.r_[a,v.ravel()]).all() or not np.isclose(np.linalg.norm(a),1.,atol=1e-6):raise ValueError('unknown stopper-hand pressure')
 return max(0.,float(v[2:4,1].sum(axis=0)@a) if lr else 0.,float(v[:2,1].sum(axis=0)@a) if rl else 0.)
