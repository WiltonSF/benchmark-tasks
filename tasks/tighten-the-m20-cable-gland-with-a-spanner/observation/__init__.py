"""Native sampled hand loads and full face-sampled hand geometry."""
import math
from functools import lru_cache
import numpy as np
import parts
_tip_contact_view=None

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
    base=sim.env.unwrapped
    origin=parts._np(base.scene.env_origins[0]);clouds=[];finger=0.
    if origin.shape!=(3,) or not np.isfinite(origin).all():raise ValueError('unknown environment origin')
    for arm in ('left_robot','right_robot'):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        if poses.ndim!=2 or poses.shape[1]!=7 or not np.isfinite(poses).all() or np.any(np.linalg.norm(poses[:,3:],axis=1)<1e-6):raise ValueError('unknown robot body poses')
        if not np.isfinite(parts._np(asset.data.joint_pos)).all():raise ValueError('unknown robot joint state')
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

    finger=max(finger,float(np.linalg.norm(fingertip_contact_forces(sim),axis=1).max()))
    return clouds,finger

def reset():
    global _tip_contact_view,_spanner_cap_view,_finger_spanner_view
    _tip_contact_view=None
    _spanner_cap_view=None
    _finger_spanner_view=None

_spanner_cap_view=None
last_spanner_cap_force=np.zeros(3)
last_spanner_cap_min_distance=None

def spanner_cap_load(sim,axis):
    """Native normal + friction contact torque on the cap nut, filtered to the spanner.

    Each contact has an equal opposite reaction on the spanner in PhysX. Forces
    are impulses divided by the physics step, not a configured brake force.
    """
    global _spanner_cap_view,last_spanner_cap_force,last_spanner_cap_min_distance
    base=sim.env.unwrapped
    if _spanner_cap_view is None:
        from isaaclab_physx.physics import PhysxManager
        env_path=base.scene.env_prim_paths[0]
        _spanner_cap_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            env_path+'/gland/nut',filter_patterns=[env_path+'/spanner'],max_contact_data_count=256)
    if _spanner_cap_view.sensor_count!=1 or _spanner_cap_view.filter_count!=1:
        raise ValueError('missing spanner-cap contact view')
    dt=base.sim.get_physics_dt()
    def arr(x):
        if hasattr(x,'torch'):x=x.torch
        return np.asarray(x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy())
    raw=_spanner_cap_view.get_contact_data(dt=dt)
    forces,points,normals,distances,counts,starts=map(arr,raw)
    n=int(counts.reshape(-1)[0]);i=int(starts.reshape(-1)[0])
    if n<0 or n>=256 or i<0 or i+n>256:raise ValueError('contact buffer missing or full')
    last_spanner_cap_min_distance=float(distances.reshape(-1)[i:i+n].min()) if n else None
    f=forces.reshape(-1,1)[i:i+n]*normals.reshape(-1,3)[i:i+n]
    p=points.reshape(-1,3)[i:i+n]
    center=np.asarray(axis)+parts._np(base.scene.env_origins[0])
    net_force=f.sum(axis=0)
    torque=float(np.cross(p-center,f)[:,0].sum())
    load=float(np.linalg.norm(f,axis=1).sum())
    friction,points,counts,starts=map(arr,_spanner_cap_view.get_friction_data(dt=dt))
    n=int(counts.reshape(-1)[0]);i=int(starts.reshape(-1)[0])
    if n<0 or n>=256 or i<0 or i+n>256:raise ValueError('friction buffer missing or full')
    f=friction.reshape(-1,3)[i:i+n];p=points.reshape(-1,3)[i:i+n]
    net_force+=f.sum(axis=0)
    torque+=float(np.cross(p-center,f)[:,0].sum())
    load+=float(np.linalg.norm(f,axis=1).sum())
    if not np.isfinite([torque,load]).all():raise ValueError('unknown wrench-cap contact load')
    last_spanner_cap_force=net_force
    return torque,load


_finger_spanner_view=None
def finger_spanner_wrench(sim,tcp,axis):
    """Read-only native normal/friction patch audit for the two tool fingers."""
    global _finger_spanner_view
    base=sim.env.unwrapped
    if _finger_spanner_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths();env=base.scene.env_prim_paths[0]
        paths=[env+'/LeftArm'+links[n] for n in ('tip_left','tip_right')]
        _finger_spanner_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[[env+'/spanner'] for _ in paths],max_contact_data_count=1024)
    view=_finger_spanner_view
    if view is None or view.sensor_count!=2 or view.filter_count!=1:raise ValueError('missing finger-spanner diagnostic view')
    def arr(x):
        if hasattr(x,'torch'):x=x.torch
        return np.asarray(x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy())
    dt=base.sim.get_physics_dt();forces,points,normals,distances,counts,starts=map(arr,view.get_contact_data(dt=dt))
    friction,fpoints,fcounts,fstarts=map(arr,view.get_friction_data(dt=dt))
    origin=parts._np(base.scene.env_origins[0]);tcp=np.asarray(tcp)+origin;axis=np.asarray(axis)+origin
    out=[]
    for finger in range(2):
        n=int(counts.reshape(-1)[finger]);i=int(starts.reshape(-1)[finger]);nf=int(fcounts.reshape(-1)[finger]);j=int(fstarts.reshape(-1)[finger])
        if min(n,i,nf,j)<0 or n>=1024 or nf>=1024 or i+n>1024 or j+nf>1024:raise ValueError('finger diagnostic buffer missing/full')
        normal=forces.reshape(-1,1)[i:i+n]*normals.reshape(-1,3)[i:i+n];p=points.reshape(-1,3)[i:i+n]
        tangential=friction.reshape(-1,3)[j:j+nf];fp=fpoints.reshape(-1,3)[j:j+nf]
        force=normal.sum(0)+tangential.sum(0)
        moment_tcp=np.cross(p-tcp,normal).sum(0)+np.cross(fp-tcp,tangential).sum(0)
        moment_axis=np.cross(p-axis,normal).sum(0)+np.cross(fp-axis,tangential).sum(0)
        scalars=[float(np.abs(forces.reshape(-1)[i:i+n]).sum()),float(np.linalg.norm(tangential,axis=1).sum())]
        if not np.isfinite(np.r_[force,moment_tcp,moment_axis,scalars]).all():raise ValueError('unknown finger diagnostic data')
        out.append(dict(finger=('tip_left','tip_right')[finger],normal_contacts=n,friction_patches=nf,force_on_finger_n=force.tolist(),normal_abs_sum_n=scalars[0],friction_abs_sum_n=scalars[1],moment_about_tcp_nm=moment_tcp.tolist(),moment_about_gland_nm=moment_axis.tolist()))
    return dict(fingers=out,note='Native filtered forces on fingers from spanner; equal opposite forces act on spanner. Optional audit, not a scorer input or hardware measurement.')
