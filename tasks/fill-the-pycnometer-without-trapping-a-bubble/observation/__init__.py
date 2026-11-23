"""Native sampled hand loads and full face-sampled hand geometry."""
import math
from functools import lru_cache
import numpy as np
import parts
_tip_contact_view=None
last_tip_loads=np.zeros(4)
last_closing_axes=np.zeros((2,3))
last_hold_forces=np.zeros((4,2))
last_glass_contact=0.
last_glass_pair=np.zeros(2)
_hold_contact_view=None

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
    global _tip_contact_view
    base=sim.env.unwrapped
    if _tip_contact_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths();env_path=base.scene.env_prim_paths[0]
        paths=[env_path+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('tip_left','tip_right')]
        _tip_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            paths,filter_patterns=[],max_contact_data_count=256)
    raw=_tip_contact_view.get_net_contact_forces(dt=base.sim.get_physics_dt())
    if hasattr(raw,'torch'):raw=raw.torch
    force=raw.detach().cpu().numpy() if hasattr(raw,'detach') else np.asarray(raw.numpy())
    force=np.asarray(force).reshape(-1,3)
    if force.shape!=(4,3) or not np.isfinite(force).all():raise ValueError('unknown fingertip contact data')
    return force

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
    global _tip_contact_view,_arm_contact_view,_support_contact_view,_hold_contact_view,last_tip_loads,last_closing_axes,last_hold_forces,last_glass_contact,last_glass_pair
    _tip_contact_view=None;_arm_contact_view=None;_support_contact_view=None
    _hold_contact_view=None;last_tip_loads=np.zeros(4);last_closing_axes=np.zeros((2,3));last_hold_forces=np.zeros((4,2))
    last_glass_contact=0.;last_glass_pair=np.zeros(2)

def counterhold_forces(sim):
 """Resolved pad forces against each glass piece,projected on each pair's closing axis."""
 global _hold_contact_view,last_hold_forces
 base=sim.env.unwrapped
 if _hold_contact_view is None:
  from isaaclab_physx.physics import PhysxManager
  from yam_arena.robot import asset_link_paths
  root=base.scene.env_prim_paths[0];links=asset_link_paths()
  paths=[root+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('tip_left','tip_right')]
  _hold_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[root+'/density_bottle',root+'/perforated_stopper'] for _ in paths],max_contact_data_count=1024)
 raw=_hold_contact_view.get_contact_force_matrix(dt=base.sim.get_physics_dt())
 value=np.asarray(parts._np(raw)).reshape(4,2,3)
 if not np.isfinite(value).all() or not np.isfinite(last_closing_axes).all():raise ValueError('unknown counterholding contacts')
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
  _arm_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[env_path+'/density_bottle',env_path+'/perforated_stopper'] for _ in paths],max_contact_data_count=1024)
 raw=_arm_contact_view.get_contact_force_matrix(dt=base.sim.get_physics_dt());raw=raw.torch if hasattr(raw,'torch') else raw
 value=raw.detach().cpu().numpy() if hasattr(raw,'detach') else np.asarray(raw.numpy())
 value=np.asarray(value).reshape(12,2,3)
 if not np.isfinite(value).all():raise ValueError('unknown housing/forearm contacts')
 return float(np.linalg.norm(value,axis=2).max())

_support_contact_view=None
def support_loads(sim):
 """Actual glass contacts with the kit or other glass; arms are excluded."""
 global _support_contact_view,last_glass_contact,last_glass_pair
 base=sim.env.unwrapped
 if _support_contact_view is None:
  from isaaclab_physx.physics import PhysxManager
  root=base.scene.env_prim_paths[0]
  targets=['water_bath','stopper_rack/s0','stopper_rack/s1','stopper_rack/s2','fill_start_shelf/s0','inspection_pad/s0','bottle_rest_pad/s0','bath_locators/s0','bath_locators/s1','bath_locators/s2','bath_locators/s3','density_bottle','perforated_stopper']
  paths=[root+'/'+n for n in ('density_bottle','perforated_stopper')]
  _support_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[root+'/'+n for n in targets] for _ in paths],max_contact_data_count=1024)
 raw=_support_contact_view.get_contact_force_matrix(dt=base.sim.get_physics_dt())
 value=np.asarray(parts._np(raw)).reshape(2,13,3)
 if not np.isfinite(value).all():raise ValueError('unknown glass support contacts')
 last_glass_pair=np.array((np.linalg.norm(value[0,12]),np.linalg.norm(value[1,11])))
 last_glass_contact=float(last_glass_pair.min())
 return np.linalg.norm(value,axis=2).sum(axis=1)
