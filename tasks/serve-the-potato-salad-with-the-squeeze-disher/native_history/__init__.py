"""Native post-physics scoring history, with independent safety diagnostics."""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np

POSITION_TOL=1e-6
ANGLE_TOL=1e-9

@dataclass
class History:
    expected:dict=field(default_factory=dict)
    ticks:int=0
    invalid:bool=False
    error:str|None=None
    max_excursion:float=0.
    min_vertical:float=1.
    hand_food_contact:bool=False

    def fail(self,reason):
        self.invalid=True
        if self.error is None:self.error=str(reason)[:200]

    def observe(self,dt,vessels,food_clear):
        try:
            if not np.isfinite(dt) or not 0<float(dt)<=.100001:raise ValueError('unknown physics interval')
            if food_clear is not True and food_clear is not False:raise ValueError('unknown native hand-food geometry')
            if not food_clear:
                self.hand_food_contact=True
                self.fail('direct hand-food contact')
            for name in ('bowl','plate'):
                pos,rotation=vessels[name];pos=np.asarray(pos,float);rotation=np.asarray(rotation,float);start=np.asarray(self.expected[name],float)
                if pos.shape!=(3,) or start.shape!=(3,) or rotation.shape!=(3,3) or not np.isfinite(np.r_[pos,start,rotation.ravel()]).all():raise ValueError('unknown native vessel pose')
                if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6,rtol=0) or abs(np.linalg.det(rotation)-1)>1e-6:raise ValueError('unknown native vessel rotation')
                displacement=float(np.linalg.norm(pos-start));vertical=float(rotation[2,2]);self.max_excursion=max(self.max_excursion,displacement);self.min_vertical=min(self.min_vertical,vertical)
                if displacement>.030+POSITION_TOL:self.fail(name+' excursion above30mm')
                if vertical<math.cos(math.radians(10.))-ANGLE_TOL:self.fail(name+' tilt above10degrees')
            self.ticks+=1
        except Exception as error:self.fail(type(error).__name__+':'+str(error))

@dataclass
class Safety:
    """Operational monitoring only: no field here is read by the scorer."""
    ticks:int=0
    error:str|None=None
    overload:bool=False
    peak_projected:float=0.
    peak_contact:float=0.
    peak_event:dict|None=None
    drive_caps:dict=field(default_factory=dict)

    def observe(self,projected,contact):
        try:
            effort=np.asarray(projected,float);force=np.asarray(contact,float)
            if effort.shape!=(4,) or force.shape!=(4,3) or not np.isfinite(effort).all() or not np.isfinite(force).all():raise ValueError('unknown native finger forces')
            p=float(np.abs(effort).max());norm=np.linalg.norm(force,axis=1);c=float(norm.max())
            self.peak_projected=max(self.peak_projected,p)
            if c>self.peak_contact:
                self.peak_contact=c;self.peak_event=dict(tick=self.ticks,index=int(norm.argmax()),force=force[int(norm.argmax())].tolist())
            self.overload|=p>40.01 or c>40.01
            self.ticks+=1
        except Exception as error:
            if self.error is None:self.error=type(error).__name__+':'+str(error)[:160]

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
        caps=_array(robot.root_view.get_dof_max_forces())
        ids=[robot.joint_names.index(j) for j in ('joint7','joint8')]
        if caps.shape!=(1,len(robot.joint_names)) or not np.isfinite(caps).all() or np.any(caps[0,ids]<=0) or np.any(caps[0,ids]>40.00001):raise ValueError('finger drive cap changed or unavailable')
        efforts.extend(float(raw[robot.joint_names.index(j)]) for j in ('joint7','joint8'))
    if _contact_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths();env_path=base.scene.env_prim_paths[0]
        paths=[env_path+'/'+arm+links[j] for arm in ('LeftArm','RightArm') for j in ('tip_left','tip_right')]
        _contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[],max_contact_data_count=256)
    contact=_array(_contact_view.get_net_contact_forces(dt=base.sim.get_physics_dt()))
    return np.asarray(efforts),np.asarray(contact).reshape(-1,3)

def _configure_drive_caps():
    """Clip finger drive effort independently of outcome grading; preserve all other DOFs."""
    import warp as wp
    for name in ('left_robot','right_robot'):
        robot=_sim.env.unwrapped.scene[name];view=robot.root_view
        raw=view.get_dof_max_forces();old=np.asarray(_array(raw),np.float32).copy()
        if old.shape!=(1,len(robot.joint_names)) or not np.isfinite(old).all() or np.any(old<=0):raise ValueError('unknown native drive effort limits')
        ids=[robot.joint_names.index(j) for j in ('joint7','joint8')]
        changed=old.copy();changed[:,ids]=np.minimum(changed[:,ids],40.)
        device=getattr(raw,'device',_sim.env.unwrapped.device)
        view.set_dof_max_forces(wp.array(changed,dtype=wp.float32,device=device),wp.array([0],dtype=wp.uint32,device=device))
        got=np.asarray(_array(view.get_dof_max_forces()))
        if got.shape!=changed.shape or not np.isfinite(got).all() or not np.allclose(got,changed,rtol=0,atol=1e-5):raise ValueError('native drive effort limit readback mismatch')
        _task._safety.drive_caps[name]=got[0,ids].tolist()

def _physics(dt):
    if _sim is None:return
    try:
        objects={}
        for name in ('disher','bowl','plate'):
            objects[name],_,_=_task.material_runtime._state(name)
        vessels={n:(objects[n].pos,_task.rotation(objects[n])) for n in ('bowl','plate')}
        clear=_task.release.clear_of_shapes(_task.native_hand_hulls(_sim),_task.food_boxes(objects),clearance=0.)
        _task._history.observe(dt,vessels,clear)
    except Exception as error:_task._history.fail(type(error).__name__+':'+str(error))
    # Availability or value of force telemetry does not enter primary history.
    try:_task._safety.observe(*_forces())
    except Exception as error:
        if _task._safety.error is None:_task._safety.error=type(error).__name__+':'+str(error)[:160]

def reset(sim,task):
    global _sim,_task,_subscription,_contact_view
    _sim=sim;_task=task;_contact_view=None
    _configure_drive_caps()
    if _subscription is None:
        import omni.physx
        _subscription=omni.physx.get_physx_interface().subscribe_physics_on_step_events(_physics,pre_step=False,order=0)
