"""Passive stock finger loads and native-step outward-pull displacement."""
import numpy as np
import parts
_sim=None;_sub=None;_view=None;_pull_view=None
peak=0.;invalid=False;error=None;pull_active=False;max_out=0.;max_move=0.;baseline=None;ticks=0;pull_N=0.;proof_hold=0.;proof_ticks=0;reaction=None;force_world=None;physics_dt=None
def array(x):
    if hasattr(x,'torch'):x=x.torch
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    if hasattr(x,'numpy'):return x.numpy()
    return np.asarray(x)
def step(dt):
    global peak,invalid,error,ticks,_view,_pull_view,max_out,max_move,baseline,proof_hold,proof_ticks,reaction,force_world,physics_dt
    if _sim is None:return
    try:
        base=_sim.env.unwrapped;fingers=[]
        for name in ('left_robot','right_robot'):
            a=base.scene[name];v=array(a.root_view.get_dof_projected_joint_forces())[0]
            fingers.extend(float(v[a.joint_names.index(n)]) for n in ('joint7','joint8'))
        if _view is None:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths
            links=asset_link_paths();root=base.scene.env_prim_paths[0]
            paths=[root+'/'+arm+links[n] for arm in ('LeftArm','RightArm') for n in ('tip_left','tip_right')]
            _view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[],max_contact_data_count=256)
        forces=array(_view.get_net_contact_forces(dt=base.sim.get_physics_dt())).reshape(-1,3)
        if forces.shape!=(4,3) or not np.isfinite(np.r_[fingers,forces.ravel()]).all():raise ValueError('unknown native finger loads')
        peak=max(peak,float(np.abs(fingers).max()),float(np.linalg.norm(forces,axis=1).max()))
        if pull_active:
            v=array(base.scene['bezel'].root_view.get_root_transforms())[0];origin=parts._np(base.scene.env_origins[0])
            if not np.isfinite(v).all() or np.linalg.norm(v[3:])<1e-8:raise ValueError('unknown native bezel pose')
            R=parts._quat_to_matrix(v[3:]);p=v[:3]-origin
            # Read the body-frame force buffer composed and passed to PhysX at this native step.
            composer=base.scene['bezel'].permanent_wrench_composer
            fb=array(composer.out_force_b).reshape(1,-1,3)[0,0]
            force_world=R@fb
            physics_dt=float(base.sim.get_physics_dt())
            if not np.isfinite(force_world).all() or not(0<physics_dt<.1):raise ValueError('unknown applied force/dt')
            if np.allclose(force_world,(-10.,0.,0.),rtol=0,atol=.001):
                proof_hold+=physics_dt;proof_ticks+=1
            else:proof_hold=0.;proof_ticks=0
            # Reaction is inspectable diagnostic evidence, not an extra force-tolerance requirement.
            try:
                if _pull_view is None:
                    from isaaclab_physx.physics import PhysxManager
                    root=base.scene.env_prim_paths[0]
                    _pull_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view([root+'/bezel/base',root+'/bezel/latch'],filter_patterns=[],max_contact_data_count=256)
                cf=array(_pull_view.get_net_contact_forces(dt=physics_dt)).reshape(-1,3)
                reaction=float(cf[:,0].sum()) if cf.shape==(2,3) and np.isfinite(cf).all() else None
            except Exception:reaction=None
            points=np.array([[-.0125,y,z] for y in (-.200,0,.200) for z in (-.018,.018)])@R.T+p
            if baseline is None:baseline=points.copy()
            max_out=max(max_out,float(np.abs(.3575-points[:,0]).max()))
            max_move=max(max_move,float(np.linalg.norm(points-baseline,axis=1).max()))
        ticks+=1
    except Exception as exc:invalid=True;error=type(exc).__name__+': '+str(exc)
def reset(sim):
    global _sim,_sub,_view,_pull_view,peak,invalid,error,pull_active,max_out,max_move,baseline,ticks,pull_N,proof_hold,proof_ticks,reaction,force_world,physics_dt
    _sim=sim;_view=None;_pull_view=None;peak=0.;invalid=False;error=None;pull_active=False;max_out=0.;max_move=0.;baseline=None;ticks=0;pull_N=0.;proof_hold=0.;proof_ticks=0;reaction=None;force_world=None;physics_dt=None
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    if _sub is None:
        import omni.physx
        _sub=omni.physx.get_physx_interface().subscribe_physics_on_step_events(step,pre_step=False,order=0)
