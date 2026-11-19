import math,numpy as np
def faces(h,step=.003):
    pts=[h.vertices]
    for nx,ny,nz,d in np.unique(np.round(h.planes,6),axis=0):
        n=np.array((nx,ny,nz));v=h.vertices[np.abs(h.vertices@n-d)<1e-6]
        if len(v)<3:continue
        c=v.mean(0);u=v[0]-c
        if np.linalg.norm(u)<1e-9:continue
        u/=np.linalg.norm(u);w=np.cross(n,u);v=v[np.argsort(np.arctan2((v-c)@w,(v-c)@u))]
        for k in range(len(v)):
            a,b=v[k]-c,v[(k+1)%len(v)]-c;m=max(1,int(math.ceil(max(np.linalg.norm(a),np.linalg.norm(b),np.linalg.norm(a-b))/step)))
            pts.extend((c+a*i/m+b*j/m)[None,:] for i in range(m+1) for j in range(m+1-i))
    p=np.concatenate(pts);_,ix=np.unique(np.round(p/.0015).astype(int),axis=0,return_index=True);return p[ix]
