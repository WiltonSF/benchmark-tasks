"""Passive native velocity resistance. All six coordinates free; no position stiffness or target motion.

The cap is a model parameter, not a real-material measurement. Delivered forces are read separately from native structural fixed tip-frame reactions.
"""
import math
import numpy as np
import parts
DAMPING = 3e7  # N s/m, numerical Coulomb regularization; 0.03N/D = 1nm/s

class VelocityResistance:
    def __init__(self,pairs,cap):
        self.pairs=pairs;self.cap=float(cap);self.drives={};self.active={k:np.zeros(len(p['names']),bool) for k,p in pairs.items()};self.ids={};self.counts={k:np.zeros(len(p['names']),int) for k,p in pairs.items()}
        previous=parts._author
        def author(part,path,translation,orientation):
            result=previous(part,path,translation,orientation)
            from isaaclab.sim.utils import get_current_stage
            from pxr import UsdPhysics,UsdGeom,Gf,Sdf
            stage=get_current_stage()
            for key,p in self.pairs.items():
                required=[p['brush']]+([] if p.get('fixed_pose') is not None else [p['phone']])
                if not all(n in parts._PATHS and stage.GetPrimAtPath(parts._PATHS[n]).IsValid() for n in required):continue
                if key in self.drives:continue
                env=parts._PATHS[p['brush']].rsplit('/',1)[0];scope=env+'/lint_velocity_'+key
                UsdGeom.Scope.Define(stage,scope);rows=[]
                for i,(name,tip) in enumerate(zip(p['names'],p['offsets'])):
                    j=UsdPhysics.Joint.Define(stage,scope+'/tip_'+str(i))
                    if p.get('fixed_pose') is None:
                        j.CreateBody0Rel().SetTargets([Sdf.Path(parts._PATHS[p['phone']])]);pos0=np.zeros(3);q0=(0,0,0,1)
                    else:pos0,q0=p['fixed_pose']
                    j.CreateBody1Rel().SetTargets([Sdf.Path(parts._PATHS[p['brush']]+'/'+name)])
                    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float,pos0)));j.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float,tip)))
                    x,y,z,w=map(float,q0);j.CreateLocalRot0Attr().Set(Gf.Quatf(w,x,y,z));j.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
                    j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(True)
                    ds=[]
                    # No LimitAPI is applied: every translation and rotation remains free.
                    for axis in ('transX','transY','transZ'):
                        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force');d.CreateStiffnessAttr(0.)
                        d.CreateDampingAttr(0.);d.CreateTargetVelocityAttr(0.);d.CreateMaxForceAttr(0.);ds.append(d)
                    rows.append(ds)
                self.drives[key]=rows
                print('[resistance] '+str(dict(pair=key,tufts=len(rows),free_coordinates=6,stiffness=0.,target_velocity=0.,cap_per_axis_N=self.cap,collisions=True)),flush=True)
            return result
        parts._author=author

    def reset(self):
        for key,p in self.pairs.items():
            if key not in self.drives:raise RuntimeError('Missing native passive resistance '+key)
            # Force writes even if this is a reset after a reused stage.
            for ds in self.drives[key]:
                for d in ds:
                    if d.GetStiffnessAttr().Get()!=0. or d.GetTargetVelocityAttr().Get()!=0.:raise RuntimeError('Nonpassive lint drive')
                    d.GetDampingAttr().Set(0.);d.GetMaxForceAttr().Set(0.)
            self.active[key][:]=False;self.counts[key][:]=0

    def set_active(self,key,active):
        self.set_counts(key,np.asarray(active,dtype=int))

    def set_counts(self,key,counts):
        counts=np.asarray(counts,int)
        if counts.shape!=self.active[key].shape or np.any(counts<0):raise ValueError('Contact-count size mismatch')
        for i in np.flatnonzero(counts!=self.counts[key]):
            for d in self.drives[key][int(i)]:
                d.GetDampingAttr().Set(DAMPING*int(counts[i]));d.GetMaxForceAttr().Set(self.cap*int(counts[i]))
        self.counts[key]=counts.copy();self.active[key]=counts>0

