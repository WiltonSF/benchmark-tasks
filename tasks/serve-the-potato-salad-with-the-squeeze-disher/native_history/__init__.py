"""Latched native post-physics force and vessel history; no actuation."""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np

POSITION_TOL=1e-6  # declared numerical margin, metres
ANGLE_TOL=1e-9    # cosine comparison only

@dataclass
class History:
    expected:dict=field(default_factory=dict)
    ticks:int=0
    invalid:bool=False
    error:str|None=None
    peak_projected:float=0.
    peak_contact:float=0.
    max_excursion:float=0.
    min_vertical:float=1.
    peak_event:dict|None=None

    def fail(self,reason):
        self.invalid=True
        if self.error is None:self.error=str(reason)[:200]

    def observe(self,dt,vessels,projected,contact):
        try:
            effort=np.asarray(projected,float);force=np.asarray(contact,float)
            if not np.isfinite(dt) or not 0<float(dt)<=.100001:raise ValueError('unknown physics interval')
            if effort.shape!=(4,) or force.shape!=(4,3) or not np.isfinite(effort).all() or not np.isfinite(force).all():raise ValueError('unknown native finger forces')
            p=float(np.abs(effort).max());norm=np.linalg.norm(force,axis=1);c=float(norm.max())
            self.peak_projected=max(self.peak_projected,p)
            if c>self.peak_contact:
                self.peak_contact=c;self.peak_event=dict(tick=self.ticks,index=int(norm.argmax()),force=force[int(norm.argmax())].tolist())
            if p>40.01 or c>40.01:self.fail('native finger force above40.01N')
            for name in ('bowl','plate'):
                pos,rotation=vessels[name];pos=np.asarray(pos,float);rotation=np.asarray(rotation,float);start=np.asarray(self.expected[name],float)
                if pos.shape!=(3,) or start.shape!=(3,) or rotation.shape!=(3,3) or not np.isfinite(np.r_[pos,start,rotation.ravel()]).all():raise ValueError('unknown native vessel pose')
                if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6,rtol=0) or abs(np.linalg.det(rotation)-1)>1e-6:raise ValueError('unknown native vessel rotation')
                displacement=float(np.linalg.norm(pos-start));vertical=float(rotation[2,2]);self.max_excursion=max(self.max_excursion,displacement);self.min_vertical=min(self.min_vertical,vertical)
                if displacement>.030+POSITION_TOL:self.fail(name+' excursion above30mm')
                if vertical<math.cos(math.radians(10.))-ANGLE_TOL:self.fail(name+' tilt above10degrees')
            self.ticks+=1
        except Exception as error:self.fail(type(error).__name__+':'+str(error))

_sim=None
_task=None
_subscription=None
_contact_view=None

def _array(value):
    if hasattr(value,'torch'):value=value.torch
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    if hasattr(value,'numpy'):return value.numpy()
    return np.asarray(value)

def _forces():
    global _contact_view
    base=_sim.env.unwrapped;efforts=[]
    for name in ('left_robot','right_robot'):
        robot=base.scene[name];raw=_array(robot.root_view.get_dof_projected_joint_forces())[0]
        efforts.extend(float(raw[robot.joint_names.index(j)]) for j in ('joint7','joint8'))
    if _contact_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths();env_path=base.scene.env_prim_paths[0]
        paths=[env_path+'/'+arm+links[j] for arm in ('LeftArm','RightArm') for j in ('tip_left','tip_right')]
        _contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[],max_contact_data_count=256)
    contact=_array(_contact_view.get_net_contact_forces(dt=base.sim.get_physics_dt()))
    return np.asarray(efforts),np.asarray(contact).reshape(-1,3)

def _physics(dt):
    if _sim is None:return
    try:
        vessels={}
        for name in ('bowl','plate'):
            state,_,_=_task.material_runtime._state(name)
            vessels[name]=(state.pos,_task.rotation(state))
        effort,contact=_forces();_task._history.observe(dt,vessels,effort,contact)
    except Exception as error:_task._history.fail(type(error).__name__+':'+str(error))

def reset(sim,task):
    global _sim,_task,_subscription,_contact_view
    _sim=sim;_task=task;_contact_view=None
    if _subscription is None:
        import omni.physx
        _subscription=omni.physx.get_physx_interface().subscribe_physics_on_step_events(_physics,pre_step=False,order=0)
