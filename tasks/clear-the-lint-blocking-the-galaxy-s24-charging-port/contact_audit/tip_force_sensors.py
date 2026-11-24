"""Native fixed structural tip-frame reactions; no contact-force subtraction or scene writes here."""
import numpy as np
import parts
from .native_shaft_audit import rotations
CREDIT_MARGIN_N=1e-5  # numerical sensor uncertainty; 172 latched a 2.38e-6 N cap excess. Subtracted from every credited force, so credit only gets stricter.

def array(v):
    if hasattr(v,'torch'):v=v.torch
    if hasattr(v,'detach'):return v.detach().cpu().numpy().astype(float)
    if hasattr(v,'numpy'):return v.numpy().astype(float)
    return np.asarray(v,float)
class TipForceSensors:
    def __init__(self,brush,names,joints):
        self.brush=brush;self.shafts=list(names);self.names=['force_'+n for n in names];self.joints={j.name:j for j in joints};self.reset()
    def reset(self):self.art=None;self.last={}
    def initialize(self,base):
        from isaaclab_physx.physics import PhysxManager
        self.art=base.scene[self.brush].root_physx_view;paths=self.art.link_paths[0]
        self.ids=[next(i for i,p in enumerate(paths) if p.endswith('/'+n)) for n in self.names]
        self.parent_ids=[next(i for i,p in enumerate(paths) if p.endswith('/'+n)) for n in self.shafts]
        self.mass=array(self.art.get_masses())[0,self.ids];self.gravity=np.asarray(tuple(PhysxManager.get_physics_sim_view().get_gravity()),float)
        if np.max(np.abs(self.mass-1e-9))>1e-12:raise RuntimeError('Unexpected structural sensor masses '+str(self.mass.tolist()))
        self.offsets=np.array([self.joints[n].origin-self.joints[p].origin for n,p in zip(self.names,self.shafts)])
        print('[tip_sensor_config] '+str(dict(brush=self.brush,sensors=len(self.ids),body_count=len(paths),mass_min_kg=float(self.mass.min()),mass_max_kg=float(self.mass.max()),credit_margin_N=CREDIT_MARGIN_N)),flush=True)
    def observe(self,base,dt,counts,cap,validate_inactive=False):
        if self.art is None:self.initialize(base)
        counts=np.asarray(counts,int);allposes=array(self.art.get_link_transforms())[0];poses=allposes[self.ids];parent=allposes[self.parent_ids]
        acc=array(self.art.get_link_accelerations())[0,self.ids];wrench=array(self.art.get_link_incoming_joint_force())[0,self.ids]
        rot=rotations(poses[:,3:]);pr=rotations(parent[:,3:])
        expected=parent[:,:3]+np.einsum('nij,nj->ni',pr,self.offsets)
        offset=float(np.linalg.norm(poses[:,:3]-expected,axis=1).max())
        raw=self.mass[:,None]*(acc[:,:3]-self.gravity)-np.einsum('nij,nj->ni',rot,wrench[:,:3])
        if not np.isfinite(raw).all() or offset>2e-6:raise RuntimeError('Invalid structural sensor closure '+str(offset))
        excess=float(np.maximum(np.abs(raw)-float(cap)*counts[:,None],0.).max())
        if excess>CREDIT_MARGIN_N:raise RuntimeError('Structural sensor force exceeds cap '+str(excess))
        pair=raw/np.maximum(counts,1)[:,None];norm=np.linalg.norm(pair,axis=1);credit=pair*np.maximum(0.,norm-CREDIT_MARGIN_N)[:,None]/np.maximum(norm,1e-30)[:,None];credit[counts==0]=0
        self.last=dict(balance_excess_N=excess,sensor_offset_m=offset,active=int(np.count_nonzero(counts)),raw_force_sum_N=float(np.linalg.norm(raw,axis=1).sum()))
        return raw,credit
