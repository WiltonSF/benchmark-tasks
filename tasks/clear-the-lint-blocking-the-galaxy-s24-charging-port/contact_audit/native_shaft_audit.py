"""Vectorized exact shaft/phone geometry guard. Observation only; no force or state writes."""
import numpy as np

def array(v):
    if hasattr(v,'torch'):v=v.torch
    if hasattr(v,'detach'):return v.detach().cpu().numpy().astype(float)
    if hasattr(v,'numpy'):return v.numpy().astype(float)
    return np.asarray(v,float)
def rotations(q):
    q=np.asarray(q,float);norm=np.linalg.norm(q,axis=1)
    if not np.isfinite(q).all() or np.any(np.abs(norm-1.)>.001):raise ValueError('Invalid native shaft quaternion')
    x,y,z,w=(q/norm[:,None]).T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=-1).reshape(-1,3,3)
class NativeShaftAudit:
    def __init__(self,joints,phone_shapes,sat):
        self.names=[j.name for j in joints];self.sat=sat;self.ids=None
        index=[];offset=[];orientation=[];half=[];labels=[]
        for i,j in enumerate(joints):
            for sh in j.shapes:
                if not sh.collide:continue
                if sh.kind!='box':raise ValueError('Native shaft audit requires box shafts')
                index.append(i);offset.append(np.asarray(sh.at)-j.origin);orientation.append(sh.R);half.append(sh.half);labels.append(j.name)
        self.index=np.asarray(index,int);self.offset=np.asarray(offset);self.orientation=np.asarray(orientation);self.half=np.asarray(half);self.labels=labels
        boxes=[sh for sh in phone_shapes if sh.collide]
        if any(sh.kind!='box' or not np.allclose(sh.R,np.eye(3)) for sh in boxes):raise ValueError('Phone boxes must align to the phone frame')
        self.cb=np.array([sh.at for sh in boxes]);self.hb=np.array([sh.half for sh in boxes])
    def evaluate(self,poses,pp,pr):
        poses=np.asarray(poses,float);r=rotations(poses[:,3:])[self.index];p=poses[self.index,:3]
        ca=(p+np.einsum('nij,nj->ni',r,self.offset)-pp)@pr
        ra=np.einsum('ij,njk,nkl->nil',pr.T,r,self.orientation)
        extent=np.einsum('nij,nj->ni',np.abs(ra),self.half)
        near=np.any(np.all(np.abs(ca[:,None,:]-self.cb[None,:,:])<extent[:,None,:]+self.hb[None,:,:],axis=-1),axis=1)
        if not near.any():return 0.,None,None
        selected=np.flatnonzero(near);hit=self.sat(ca[near],ra[near],self.half[near],self.cb,self.hb)
        i,j=np.unravel_index(int(np.argmax(hit)),hit.shape)
        return float(hit[i,j]),self.labels[selected[i]],int(j)
    def observe(self,art,pp,pr):
        if self.ids is None:
            paths=art.link_paths[0];self.ids=[next(i for i,p in enumerate(paths) if p.endswith('/'+n)) for n in self.names]
        return self.evaluate(array(art.get_link_transforms())[0,self.ids],np.asarray(pp),np.asarray(pr))
