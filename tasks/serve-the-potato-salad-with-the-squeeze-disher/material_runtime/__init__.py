"""Task-local constitutive coupling at native physics steps; no pose writes."""
import numpy as np
_sim=None
_task=None
_subscription=None
_indices=None
_coms={}

def _state(name):
    asset=_sim.env.unwrapped.scene[name];view=asset.root_view;origin=_task.release.array(_sim.env.unwrapped.scene.env_origins)[0]
    if name=='disher':
        pp=_task.release.array(view.get_link_transforms())[0];vv=_task.release.array(view.get_link_velocities())[0]
        qq=_task.release.array(view.get_dof_positions())[0];qd=_task.release.array(view.get_dof_velocities())[0]
        poses={n:(pp[i,:3]-origin,_task.parts._quat_to_matrix(pp[i,3:])) for i,n in enumerate(asset.body_names)}
        j={n:float(qq[i]) for i,n in enumerate(asset.joint_names)};bi=asset.body_names.index('base');lin=vv[bi,:3]-np.cross(vv[bi,3:],poses['base'][1]@_coms[name][bi,:3])
        return _task.DISHER.make_state(poses,j,lin,vv[bi,3:]),float(qd[asset.joint_names.index('wiper')]),pp
    pp=_task.release.array(view.get_transforms());vv=_task.release.array(view.get_velocities())[0];r=_task.parts._quat_to_matrix(pp[0,3:]);lin=vv[:3]-np.cross(vv[3:],r@_coms[name][0,:3]);spec=_task.BOWL if name=='bowl' else _task.PLATE
    return spec.make_state({'base':(pp[0,:3]-origin,r)},{},lin,vv[3:]),0.,pp

def _apply(name,forces,torques,poses):
    import warp as wp
    asset=_sim.env.unwrapped.scene[name];device=_sim.env.unwrapped.device
    com=np.array([p[:3]+_task.parts._quat_to_matrix(p[3:])@c[:3] for p,c in zip(poses,_coms[name])],np.float32)
    f=np.asarray(forces,np.float32);t=np.asarray(torques,np.float32)
    # Specify the actual COM explicitly; convert all moments to that reference.
    if name=='disher':f,t,com=f[None],t[None],com[None]
    asset.root_view.apply_forces_and_torques_at_position(wp.array(f,dtype=wp.float32,device=device),wp.array(t,dtype=wp.float32,device=device),wp.array(com,dtype=wp.float32,device=device),_indices,True)

def _friction(requested=None):
    """Read/alter the wiper only; preserve all other DOFs and viscosity."""
    import warp as wp
    asset=_sim.env.unwrapped.scene['disher'];view=asset.root_view;j=asset.joint_names.index('wiper');raw=view.get_dof_friction_properties();old=np.asarray(raw.numpy(),dtype=np.float32).copy()
    if old.shape!=(1,len(asset.joint_names),3) or not np.isfinite(old).all() or np.any(old<0):raise ValueError('unknown native friction properties')
    if requested is None:
        if abs(float(old[0,j,0])-float(old[0,j,1]))>1e-7:raise ValueError('mismatched native static/dynamic resistance')
        return float(old[0,j,1])
    if not np.isfinite(requested) or not 0<=requested<=.040001:raise ValueError('invalid friction request')
    changed=old.copy();changed[0,j,:2]=requested;device=raw.device
    view.set_dof_friction_properties(wp.array(changed,dtype=wp.float32,device=device),wp.array([0],dtype=wp.uint32,device=device))
    got=np.asarray(view.get_dof_friction_properties().numpy(),dtype=np.float32)
    if got.shape!=changed.shape or not np.isfinite(got).all() or not np.allclose(got,changed,rtol=1e-6,atol=1e-7):raise ValueError('native friction readback mismatch')
    return float(got[0,j,1])

def _physics(dt):
    if _sim is None:return
    try:
        tool,qd,poses=_state('disher');bowl,_,bp=_state('bowl');plate,_,pp=_state('plate')
        if not np.isfinite(np.r_[tool.pos,tool.quat_xyzw,tool.lin_vel,tool.ang_vel,bowl.pos,bowl.quat_xyzw,bowl.lin_vel,bowl.ang_vel,plate.pos,plate.quat_xyzw,plate.lin_vel,plate.ang_vel,qd]).all():raise ValueError('unknown physics state')
        previous=_friction()
        if abs(previous-_task._food.applied_mu)>1e-7:raise ValueError('native friction changed outside material coupling')
        _task._food.applied_mu=previous
        ft,tt,fb,tw,fp=_task._food.advance(float(dt),bowl,tool,plate,tool.joints['wiper'],qd)
        if not np.isfinite(np.r_[ft,tt,fb,tw,fp]).all():raise ValueError('unknown material wrench')
        asset=_sim.env.unwrapped.scene['disher'];forces=np.zeros((len(asset.body_names),3));torques=forces.copy();bi=asset.body_names.index('base');wi=asset.body_names.index('wiper');forces[bi]=ft;torques[bi]=tt-np.cross(_task.rotation(tool)@_coms['disher'][bi,:3],ft);torques[wi]=tw
        _apply('disher',forces,torques,poses);_apply('bowl',[fb],np.zeros((1,3)),bp);_apply('plate',[fp],np.zeros((1,3)),pp)
        wanted=.040*min(1.,float(_task._food.cap.sum())/_task.CAPACITY)
        _task._food.applied_mu=_friction(wanted)
        _task._food.ticks+=1
    except Exception as error:
        _task._food.invalid=True;_task._food.error=type(error).__name__+':'+str(error)[:160]
        # Applied external wrenches expire each step. Native joint friction
        # remains at its last written value after an error; the permanently
        # invalid material state cannot certify subsequent motion or success.

def reset(sim,task):
    global _sim,_task,_subscription,_indices,_coms
    import warp as wp
    from types import SimpleNamespace
    if isinstance(task,dict):task=SimpleNamespace(**task)
    _sim=sim;_task=task;_indices=wp.array([0],dtype=wp.uint32,device=sim.env.unwrapped.device);_coms={}
    for name in ('disher','bowl','plate'):_coms[name]=task.release.array(sim.env.unwrapped.scene[name].root_view.get_coms()).reshape(-1,7)
    _task._food.applied_mu=_friction(0.)
    _task.native_history.reset(sim,_task)
    if _subscription is None:
        import omni.physx
        _subscription=omni.physx.get_physx_interface().subscribe_physics_on_step_events(_physics,pre_step=True,order=0)
