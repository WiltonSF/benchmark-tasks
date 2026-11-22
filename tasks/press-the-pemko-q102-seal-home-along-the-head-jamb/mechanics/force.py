"""Per-physics-step contact/work certificate; never moves or seats a body."""
import json
import os
from pathlib import Path
import numpy as np
import parts


def array(value):
    value=value.torch if hasattr(value,'torch') else value
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    if hasattr(value,'numpy'):return np.asarray(value.numpy())
    return np.asarray(value)


class Certificate:
    def __init__(self,sim,task,required):
        from isaaclab_physx.physics import PhysxManager
        from isaaclab.sim.utils import get_current_stage
        from pxr import UsdPhysics
        self.base=sim.env.unwrapped;self.task=task;self.required=required
        stage=get_current_stage()
        paths={p.GetName():str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
        self.paths=[paths[s.name] for s in task.SEALS]
        jamb=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and '/head_jamb/' in str(p.GetPath()) and p.GetName() in ('s1','s2','s3')]
        if not jamb:raise ValueError('No physical jamb colliders')
        view=PhysxManager.get_physics_sim_view()
        self.bodies=view.create_rigid_body_view(self.paths)
        self.contacts=view.create_rigid_contact_view(self.paths,filter_patterns=[jamb for _ in self.paths],max_contact_data_count=8192)
        self.origin=array(self.base.scene.env_origins)[0]
        self.dt=float(self.base.sim.get_physics_dt())
        self.reset()

    def reset(self):
        self.trial=getattr(self,'trial',0)+1
        self.coverage=np.zeros((self.task.N,51),bool)
        self.work=np.zeros(self.task.N)
        self.loaded_travel=np.zeros(self.task.N)
        self.peaks=np.zeros(self.task.N)
        self.active_peaks=np.zeros(self.task.N)
        self.maximum_depth=None
        self.previous=None;self.previous_force=None
        self.observed=False;self.damaged=False;self.error=None;self.ticks=0

    @property
    def certified(self):
        if self.maximum_depth is None:return np.zeros(self.task.N,bool)
        return (self.active_peaks>=self.required)&(self.work>=.00425)&(self.loaded_travel>=.00025)&(self.maximum_depth>=.008)

    def collect(self):
        try:
            poses=array(self.bodies.get_transforms()).reshape(self.task.N,7)
            forces=array(self.contacts.get_contact_force_matrix(dt=self.dt)).reshape(self.task.N,-1,3).sum(axis=1)
            if not np.isfinite(poses).all() or not np.isfinite(forces).all():raise ValueError('Nonfinite force observation')
            positions=poses[:,:3]-self.origin
            rotations=np.array([parts._quat_to_matrix(q) for q in poses[:,3:]])
            tip=positions+np.einsum('nij,j->ni',rotations,np.array([.0079375+self.task.DEPTH,0.,0.]))
            depth=tip[:,0]-self.task.FACE
            axial=np.maximum(0.,-forces[:,0])
            self.peaks=np.maximum(self.peaks,axial)
            if self.previous is not None:
                withdrawn=(depth<0.)&(self.previous>=0.)
                self.work[withdrawn]=0.;self.loaded_travel[withdrawn]=0.;self.active_peaks[withdrawn]=0.
                self.coverage[withdrawn]=False;self.maximum_depth[withdrawn]=depth[withdrawn]
                dx=np.maximum(0.,depth-self.previous)
                contact=(axial>=self.required)&(self.previous_force>=self.required)
                aligned=np.abs(tip[:,2]-self.task.SEAL_Z)<self.task.KERF/2
                # The cited value is a peak insertion force, not constant drag.
                # Count only first forward travel through the actual barb-entry
                # region; a rear-stop impact or repeated pumping cannot qualify.
                forward=np.maximum(0.,np.minimum(depth,.0085)-np.maximum(self.maximum_depth,.002))
                active=aligned&(depth>=.002)&(depth<=.0085)&(dx>0.)&(dx<=.0005)
                resisting=np.minimum(axial,self.previous_force)
                self.active_peaks=np.maximum(self.active_peaks,np.where(active,resisting,0.))
                # Consecutive interval-mean contacts provide a conservative
                # work estimate; an isolated impact or >0.5 mm/tick jump
                # cannot certify insertion work.
                self.work+=resisting*forward*active
                self.loaded_travel+=forward*contact*active
                # Only full compression of the tapered barb (tip depth 5.5..8 mm)
                # is certified. A repeated final impact cannot fill earlier bins.
                for j in range(51):
                    point=.0055+j*.00005
                    swept=(self.previous<=point)&(depth>=point)&(dx>0.)
                    self.coverage[:,j] |= (axial>0.)&aligned&swept
            for i in range(self.task.N-1):
                relative=rotations[i].T@rotations[i+1]
                angle=float(np.arccos(np.clip((np.trace(relative)-1.)/2,-1,1)))
                self.damaged |= angle>np.deg2rad(30.)
            self.previous=depth.copy();self.previous_force=axial.copy();self.observed=True;self.ticks+=1
            self.maximum_depth=depth.copy() if self.maximum_depth is None else np.maximum(self.maximum_depth,depth)
            diagnostics=os.environ.get('SEAL_DIAGNOSTIC_DIR')
            if (os.environ.get('TRACE_OUT') or diagnostics) and self.ticks%12==0:
                file=Path(os.environ['TRACE_OUT']).with_name('force.jsonl') if os.environ.get('TRACE_OUT') else Path(diagnostics)/('force-'+str(os.getpid())+'.jsonl')
                file.parent.mkdir(parents=True,exist_ok=True)
                row=dict(trial=self.trial,ticks=self.ticks,depth=depth.tolist(),axial=axial.tolist(),coverage=self.coverage.sum(axis=1).tolist(),
                    work_J=self.work.tolist(),peak_N=self.peaks.tolist(),active_peak_N=self.active_peaks.tolist(),
                    loaded_travel_m=self.loaded_travel.tolist(),certified=self.certified.tolist(),damaged=bool(self.damaged),positions=positions.tolist(),quaternions=poses[:,3:].tolist())
                with file.open('a') as f:f.write(json.dumps(row)+'\n')
        except Exception as exc:
            self.error=type(exc).__name__+': '+str(exc);self.observed=False
            raise


def install(sim,task,required):
    base=sim.env.unwrapped
    sensor=getattr(base.sim,'_seal_certificate',None)
    if sensor is None:
        sensor=Certificate(sim,task,required)
        original=base.sim.step
        def step(*args,**kwargs):
            result=original(*args,**kwargs)
            base.sim._seal_certificate.collect()
            return result
        base.sim.step=step
        base.sim._seal_certificate=sensor
    sensor.reset()
    return sensor
