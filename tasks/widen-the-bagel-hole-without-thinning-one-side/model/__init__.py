"""Discrete cohesive dough law. Parameters are stated estimates, not measured rheology."""
import math
from dataclasses import dataclass,field
import numpy as np
N=16
R=.02625
A=.01375
B=.015
M=.102
K=60.
C=.3
RATE=.2
L0=2*R*math.sin(math.pi/N)
ANGLES=np.arange(N)*math.tau/N
P0=np.c_[R*np.cos(ANGLES),R*np.sin(ANGLES),np.zeros(N)]

def sections(p,rest,flatten=None):
    # One fixed material volume per sector. Plastic length redistribution reduces
    # both cross-section axes, preserving pi*a*b*rest exactly.
    lengths=(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)+np.linalg.norm(p-np.roll(p,1,axis=0),axis=1))/2
    scale=np.sqrt(L0/lengths)
    flatten=np.ones(N) if flatten is None else np.asarray(flatten)
    return A*scale/flatten,B*scale*flatten

def material_frame(p):
    # The ordered ring plane and tangents rotate with the free food. No world
    # axis or invisible fixture fixes its material cross-section orientation.
    c=p-p.mean(0);normal=np.cross(c,np.roll(c,-1,axis=0)).sum(0)
    if np.linalg.norm(normal)<1e-9:raise ValueError('folded material plane')
    normal/=np.linalg.norm(normal)
    tangent=np.roll(p,-1,axis=0)-np.roll(p,1,axis=0)
    tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-12)
    radial=np.cross(tangent,normal)
    radial/=np.maximum(np.linalg.norm(radial,axis=1)[:,None],1e-12)
    return tangent,radial,normal

def boundaries(p,rest,flatten=None):
    p=np.asarray(p,float);a,b=sections(p,rest,flatten)
    tangent,radial,vertical=material_frame(p)
    inner=[];outer=[]
    for i in range(N):
        j=(i+1)%N
        nr=radial[i]+radial[j];nr/=max(np.linalg.norm(nr),1e-12)
        mid=(p[i]+p[j])/2;aa=(a[i]+a[j])/2
        inner.extend([p[i]-radial[i]*a[i],mid-nr*aa])
        outer.extend([p[i]+radial[i]*a[i],mid+nr*aa])
    return np.asarray(inner),np.asarray(outer),a,b

def _cells(p,a,b):
    p=np.asarray(p,float)
    tangent,normal,vertical=material_frame(p)
    out=[]
    for i in range(N):
        points=[]
        for j,point in enumerate(((p[i-1]+p[i])/2,p[i],(p[i]+p[(i+1)%N])/2)):
            k=(i-1)%N if j==0 else ((i+1)%N if j==2 else i)
            nr=normal[i]+normal[k];nr/=max(np.linalg.norm(nr),1e-12)
            aa=(a[i]+a[k])/2;bb=(b[i]+b[k])/2
            for angle in np.arange(12)*math.tau/12:
                points.append(point+nr*aa*math.cos(angle)+vertical*bb*math.sin(angle))
        out.append(points)
    return np.asarray(out)

_cell_cache=None
_REFERENCE_VOLUME=None

def cell_points(p,rest,flatten=None):
    global _cell_cache,_REFERENCE_VOLUME
    from scipy.spatial import ConvexHull
    p=np.asarray(p,float)
    flatten=np.ones(N) if flatten is None else np.asarray(flatten,float)
    key=(p.tobytes(),flatten.tobytes())
    if _cell_cache is not None and key==_cell_cache[0]:return _cell_cache[1]
    if _REFERENCE_VOLUME is None:
        reference=_cells(P0,np.full(N,A),np.full(N,B))
        _REFERENCE_VOLUME=np.array([ConvexHull(cell).volume for cell in reference])
    a,b=sections(p,rest,flatten)
    correction=np.ones(N)
    for _ in range(40):
        cells=_cells(p,a*correction,b*correction)
        volume=np.array([ConvexHull(cell).volume for cell in cells])
        error=volume/_REFERENCE_VOLUME-1
        if np.max(abs(error))<1e-5:break
        correction*=np.sqrt(_REFERENCE_VOLUME/volume)
    if np.max(abs(error))>1e-3:raise ValueError('material volume solve did not converge')
    _cell_cache=(key,cells)
    return cells

def simple_ring(p):
    xy=p[:,:2];r=np.roll(xy,-1,axis=0)-xy
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    den=cross(r[:,None,:],r[None,:,:]);delta=xy[None,:,:]-xy[:,None,:]
    safe=np.where(abs(den)>1e-12,den,1.)
    t=cross(delta,r[None,:,:])/safe;u=cross(delta,r[:,None,:])/safe
    index=np.arange(N);apart=(index[:,None]-index[None,:])%N
    nonadj=(apart!=0)&(apart!=1)&(apart!=N-1)
    if np.any(nonadj&(abs(den)>1e-12)&(t>=-1e-7)&(t<=1+1e-7)&(u>=-1e-7)&(u<=1+1e-7)):return False
    rr=np.maximum(np.sum(r*r,axis=1),1e-15)[:,None]
    t0=np.sum(delta*r[:,None,:],axis=2)/rr
    t1=np.sum((delta+r[None,:,:])*r[:,None,:],axis=2)/rr
    overlap=np.minimum(np.maximum(t0,t1),1)-np.maximum(np.minimum(t0,t1),0)
    if np.any(nonadj&(abs(den)<=1e-12)&(abs(cross(delta,r[:,None,:]))<1e-10)&(overlap>1e-6)):return False
    c=xy-xy.mean(0);following=np.roll(c,-1,axis=0)
    winding=np.arctan2(cross(c,following),np.sum(c*following,axis=1)).sum()/math.tau
    return bool(abs(abs(winding)-1)<1e-5)

def minimum_profile_height(cells,center):
    """Conservative minimum of actual convex-union vertical radial sections.

    A surface-edge/vertical-plane intersection has linear-fractional height in
    tan(angle), hence is monotone between its angular endpoint events. Edges
    valid throughout an interval bound its complete top/bottom envelopes. Split
    the worst interval until the global lower/upper bracket is <=0.01mm. This
    checks between angular samples as well as all shared-cell interfaces.
    """
    from scipy.spatial import ConvexHull
    aa=[];bb=[]
    for cell in cells:
        tri=ConvexHull(cell).simplices
        pairs=np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]])
        pairs=np.unique(np.sort(pairs,axis=1),axis=0)
        aa.extend(cell[pairs[:,0]]);bb.extend(cell[pairs[:,1]])
    a=np.asarray(aa);b=np.asarray(bb);a[:,:2]-=center;b[:,:2]-=center
    delta=b-a;eps=1e-8
    cross=lambda u,v:u[...,0]*v[...,1]-u[...,1]*v[...,0]
    def intersections(angles):
        u=np.c_[np.cos(angles),np.sin(angles)]
        ca=cross(u[:,None,:],a[None,:,:2]);den=cross(u[:,None,:],delta[None,:,:2])
        safe=np.where(abs(den)>1e-14,den,1.)
        t=-ca/safe;q=a[None,:,:]+np.clip(t,0.,1.)[:,:,None]*delta[None,:,:]
        radial=np.sum(q[:,:,:2]*u[:,None,:],axis=2)
        regular=(abs(den)>1e-14)&(t>=-1e-7)&(t<=1+1e-7)&(radial>0)
        coplanar=(abs(ca)<=eps)&(abs(ca+den)<=eps)&(np.sum(a[None,:,:2]*u[:,None,:],axis=2)>0)&(np.sum(b[None,:,:2]*u[:,None,:],axis=2)>0)
        high=np.where(coplanar,np.maximum(a[:,2],b[:,2]),q[:,:,2])
        low=np.where(coplanar,np.minimum(a[:,2],b[:,2]),q[:,:,2])
        valid=regular|coplanar
        return high,low,valid,den
    def bounds(lo,hi):
        top0,bot0,v0,d0=intersections(lo);top1,bot1,v1,d1=intersections(hi)
        # No pole is allowed inside an edge's angular domain. Coplanar vertical
        # edges cover only the <=10nm numerical interface tolerance above.
        whole=v0&v1&((d0*d1>=0)|(abs(d0)+abs(d1)<eps))
        lower=np.max(np.where(whole,np.minimum(top0,top1),-np.inf),axis=1)-np.min(np.where(whole,np.maximum(bot0,bot1),np.inf),axis=1)
        tm,bm,vm,_=intersections((lo+hi)/2)
        actual=[]
        for top,bot,valid in ((top0,bot0,v0),(top1,bot1,v1),(tm,bm,vm)):
            actual.append(np.max(np.where(valid,top,-np.inf),axis=1)-np.min(np.where(valid,bot,np.inf),axis=1))
        upper=np.minimum.reduce(actual)
        lower=np.where(np.isfinite(lower),np.maximum(0.,lower),0.)
        if not np.isfinite(upper).all() or np.any(upper<0):raise ValueError('unmeasurable vertical profile')
        return lower,upper
    events=np.mod(np.arctan2(np.r_[a[:,1],b[:,1]],np.r_[a[:,0],b[:,0]]),math.tau)
    events=np.unique(np.r_[0.,math.tau,np.arange(128)*math.tau/128,np.round(events,14)])
    lo=events[:-1];hi=events[1:];keep=hi-lo>1e-14;lo=lo[keep];hi=hi[keep]
    lower,upper=bounds(lo,hi)
    for _ in range(256):
        k=int(np.argmin(lower));best=float(upper.min())
        if best-lower[k]<=1e-5:return float(max(0.,lower[k]-1e-7))
        mid=(lo[k]+hi[k])/2
        if hi[k]-lo[k]<1e-12:raise ValueError('uncertain sub-resolution height profile')
        newlo=np.array([lo[k],mid]);newhi=np.array([mid,hi[k]])
        nl,nu=bounds(newlo,newhi)
        lo=np.r_[np.delete(lo,k),newlo];hi=np.r_[np.delete(hi,k),newhi]
        lower=np.r_[np.delete(lower,k),nl];upper=np.r_[np.delete(upper,k),nu]
    raise ValueError('height profile bound did not converge')


_geo_cache=None

def geometry(p,rest,flatten=None,cells=None):
    global _geo_cache
    p=np.asarray(p,float);rest=np.asarray(rest,float)
    flatten=np.ones(N) if flatten is None else np.asarray(flatten,float)
    if p.shape!=(N,3) or rest.shape!=(N,) or flatten.shape!=(N,) or not np.isfinite(np.r_[p.ravel(),rest,flatten]).all() or min(rest)<=0 or min(flatten)<=0:raise ValueError('unknown geometry')
    if not simple_ring(p):raise ValueError('folded or overlapping ring')
    cells=cell_points(p,rest,flatten) if cells is None else np.asarray(cells,float)
    if cells.shape!=(N,36,3) or not np.isfinite(cells).all():raise ValueError('unknown authored cells')
    key=(p.tobytes(),rest.tobytes(),flatten.tobytes(),cells.tobytes())
    if _geo_cache is not None and key==_geo_cache[0]:return _geo_cache[1]
    from scipy.spatial import ConvexHull
    # PhysX uses each cell's convex hull, which fills concave inner arcs. Score
    # those actual projected hulls rather than the intended centreline surface.
    polys=[cell[ConvexHull(cell[:,:2]).vertices,:2] for cell in cells]
    for i in range(N):
        for j in range(i+2,N):
            if i==0 and j==N-1:continue
            first,second=polys[i],polys[j]
            edges0=np.roll(first,-1,axis=0)-first;edges1=np.roll(second,-1,axis=0)-second
            edges_both=np.vstack([edges0,edges1]);axes=np.c_[edges_both[:,1],-edges_both[:,0]]
            axes/=np.maximum(np.linalg.norm(axes,axis=1)[:,None],1e-12)
            aa=first@axes.T;bb=second@axes.T
            depth=np.minimum(aa.max(0),bb.max(0))-np.maximum(aa.min(0),bb.min(0))
            if np.all(depth>1e-5):raise ValueError('overlapping nonadjacent material cells')
    ends=np.concatenate(polys);starts=np.concatenate([np.roll(poly,1,axis=0) for poly in polys]);edges=ends-starts
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    center=p[:,:2].mean(0)
    groups=np.repeat(np.arange(N),[len(poly) for poly in polys])
    def rays(angles,c):
        u=np.c_[np.cos(angles),np.sin(angles)]
        v=starts-c;den=cross(u[:,None,:],edges[None,:,:])
        safe=np.where(abs(den)>1e-11,den,1.)
        t=cross(v,edges)[None,:]/safe
        f=cross(v[None,:,:],u[:,None,:])/safe
        valid=(abs(den)>1e-11)&(t>=0)&(f>=-1e-7)&(f<=1+1e-7)
        near=np.min(np.where(valid,t,np.inf),axis=1);far=np.max(np.where(valid,t,-np.inf),axis=1)
        if not np.isfinite(near).all() or min(near)<=0:raise ValueError('open or collapsed ring')
        # Material along a ray must form one contiguous band. Otherwise an
        # empty gap between folded walls would be counted as solid dough.
        enter=[];leave=[]
        for i in range(N):
            mask=groups==i
            enter.append(np.min(np.where(valid[:,mask],t[:,mask],np.inf),axis=1))
            leave.append(np.max(np.where(valid[:,mask],t[:,mask],-np.inf),axis=1))
        enter=np.stack(enter,axis=1);leave=np.stack(leave,axis=1)
        order=np.argsort(enter,axis=1);begin=np.take_along_axis(enter,order,axis=1);end=np.take_along_axis(leave,order,axis=1)
        covered=np.maximum.accumulate(end,axis=1)
        if np.any(np.isfinite(begin[:,1:])&(begin[:,1:]>covered[:,:-1]+1e-5)):raise ValueError('non star shaped material band')
        return near,far,u
    dense=np.arange(512)*math.tau/512
    for _ in range(3):
        near,far,u=rays(dense,center);outline=center+u*near[:,None]
        x,y=outline.T;cr=x*np.roll(y,-1)-np.roll(x,-1)*y;area=cr.sum()/2
        if area<=1e-7:raise ValueError('invalid hole polygon')
        center=np.array([((x+np.roll(x,-1))*cr).sum(),((y+np.roll(y,-1))*cr).sum()])/(6*area)
    # Centre must remain in every cell's exterior. Otherwise projected food
    # overlaps the proposed hole and an inner ray need not mean a clear opening.
    for poly in polys:
        edge=np.roll(poly,-1,axis=0)-poly
        if np.all(cross(edge,center-poly)>=-1e-9):raise ValueError('food inside hole')
    angles=np.arange(128)*math.tau/128
    near,far,u=rays(angles,center);diam=near[:64]+near[64:];walls=far-near
    # Include all mesh-vertex angles to catch a local neck between fixed rays.
    extra=np.arctan2(ends[:,1]-center[1],ends[:,0]-center[0])
    ni,fo,_=rays(extra,center);allwalls=np.r_[walls,fo-ni]
    outer=float(np.linalg.norm(ends[:,None,:]-ends[None,:,:],axis=2).max())
    height=minimum_profile_height(cells,center)
    g=dict(d=float(diam.min()),D=float(diam.max()),w_min=float(allwalls.min()),w_max=float(allwalls.max()),h_min=height,outer=outer,inner=center+u*near[:,None],outline=center+u*far[:,None],center=center)
    _geo_cache=(key,g)
    return g

def tier(g):
    e=1e-6
    # Flattening or a local neck is damage, including below the high tiers.
    if g['h_min']<.012-e or g['w_min']<.012-e or g['d']<.030-e or g['D']>.065+e:return 0.
    if g['d']<.040-e:return .25
    if g['d']<.045-e:return .5
    if g['D']>.055+e:return .5
    if g['w_max']/g['w_min']>1.35+e or g['outer']>.130+e:return .75
    return 1.

@dataclass
class Record:
    invalid:bool=False
    known:bool=False
    hold:float=0.
    t:float=0.
    certificate:tuple=()
    previous:object=None
    previous_edge:object=None
    hold_anchor:object=None
    props_anchor:object=None
    cells:object=None
    valid_reset:bool=False
    sub_clear:bool=True
    sub_tear:bool=False
    substeps:int=0
    rest:np.ndarray=field(default_factory=lambda:np.full(N,L0))
    anchors:list=field(default_factory=list)
    flatten:np.ndarray=field(default_factory=lambda:np.ones(N))
    error:str=''
    peak:float=0.
    def update(self,t,p,known,clear,supported,invalid):
        dt=t-self.t
        self.invalid|=bool(invalid or not known or dt<=0 or dt>.101)
        still=self.previous is not None and np.linalg.norm(p-self.previous,axis=1).max()<=.0005
        eligible=known and clear and supported and still and not self.invalid
        if not eligible:
            self.hold=0.;self.hold_anchor=None
        elif self.hold_anchor is None or np.linalg.norm(p-self.hold_anchor,axis=1).max()>.001:
            self.hold=0.;self.hold_anchor=p.copy()
        else:self.hold+=dt
        self.t=t;self.previous=p.copy();self.known=known
    def score(self,g):return tier(g) if self.valid_reset and self.known and not self.invalid and self.hold>=2.-1e-7 else 0.
