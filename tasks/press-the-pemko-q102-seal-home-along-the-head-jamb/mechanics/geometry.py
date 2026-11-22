"""Distance between convex boxes, including face/edge closest approaches."""
import itertools
import numpy as np

SIGNS=np.array(list(itertools.product((-1.,1.),repeat=3)))
EDGES=np.array([(i,j) for i in range(8) for j in range(i) if np.count_nonzero(SIGNS[i]!=SIGNS[j])==1])


def box_distance(a, ra, sa, b, rb, sb):
    delta=b-a
    axes=[*ra.T,*rb.T,*[np.cross(x,y) for x in ra.T for y in rb.T]]
    separated=False
    for axis in axes:
        if np.linalg.norm(axis)<1e-9:continue
        if abs(delta@axis) > np.abs(ra.T@axis)@(sa/2)+np.abs(rb.T@axis)@(sb/2):
            separated=True
            break
    if not separated:return 0.
    va=(SIGNS*sa/2)@ra.T+a
    vb=(SIGNS*sb/2)@rb.T+b
    best=min(np.linalg.norm(np.maximum(np.abs((va-b)@rb)-sb/2,0.),axis=1).min(),
             np.linalg.norm(np.maximum(np.abs((vb-a)@ra)-sa/2,0.),axis=1).min())
    pa=va[EDGES[:,0]][:,None,:];pb=vb[EDGES[:,0]][None,:,:]
    u=(va[EDGES[:,1]]-va[EDGES[:,0]])[:,None,:]
    v=(vb[EDGES[:,1]]-vb[EDGES[:,0]])[None,:,:]
    w=pa-pb
    aa=(u*u).sum(-1);bb=(u*v).sum(-1);cc=(v*v).sum(-1)
    dd=(u*w).sum(-1);ee=(v*w).sum(-1)
    determinant=aa*cc-bb*bb
    safe=np.where(np.abs(determinant)>1e-20,determinant,1.)
    s=(bb*ee-cc*dd)/safe;t=(aa*ee-bb*dd)/safe
    interior=(np.abs(determinant)>1e-20)&(s>=0)&(s<=1)&(t>=0)&(t<=1)
    if interior.any():
        distances=np.linalg.norm(w+s[:,:,None]*u-t[:,:,None]*v,axis=-1)
        best=min(best,distances[interior].min())
    return float(best)
