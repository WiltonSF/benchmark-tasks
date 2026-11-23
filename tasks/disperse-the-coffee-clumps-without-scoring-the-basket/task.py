"""A fresh dry coffee dose with six visible clumps sits in a loose 54 mm portafilter.
A nine-needle distribution tool stands in its supplied stand. Full-size passive
espresso-machine and grinder enclosures occupy the back of the kitchen counter.
The metal basket, native handle, needles, dose and stand are visible.
"""
import json, math, os, pathlib, sys
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts, lots
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.1
STEEL=(.62,.64,.66); BLACK=(.12,.12,.13); BROWN=(.25,.11,.035)
R=.025; FLOOR=.017; RIM=.041; CELL=.001; DENSITY=530.; MASS=.020
# Tool origin is the centre of its body. Nine full-length integral tines extend down.
NEEDLES=np.array([(0.,0.)]+[(.010*math.cos(a),.010*math.sin(a)) for a in np.linspace(0,2*math.pi,8,endpoint=False)])
NLOW=-.0765; NHIGH=-.0265; NR=.000125

def ring(radius,wall,height,z,color,n=64):
    # Tangent panels retain actual wall thickness instead of oversized axis-aligned cubes.
    return [parts.Shape('box',(wall,2*math.pi*radius/n+.0001,height),
            (radius*math.cos(a),radius*math.sin(a),z),color,
            orientation=tuple(parts._rot((0,0,1),a).reshape(-1))) for a in np.linspace(0,2*math.pi,n,endpoint=False)]
TOOL=parts.Part('wdt',pos=(.21,.16,TOP+.0845),mass=.065,friction=(.6,.5),shapes=[
    parts.cylinder(.0155,.0395,(0,0,.00675),BLACK),parts.cylinder(.0105,.0135,(0,0,-.01975),BLACK),
    *[parts.cylinder(NR,.050,(float(x),float(y),-.0515),STEEL) for x,y in NEEDLES]])
STAND=parts.Part('original_stand',pos=(.21,.16,TOP),mass=.10,friction=(.6,.5),shapes=[
    parts.cylinder(.025,.005,(0,0,.0025),BLACK),
    parts.box((.003,.014,.065),(-.022,0,.0365),BLACK),
    parts.box((.014,.032,.004),(-.0185,0,.0695),BLACK),
    parts.box((.022,.005,.004),(0,.0135,.0695),BLACK),
    parts.box((.022,.005,.004),(0,-.0135,.0695),BLACK)])
BASKET=parts.Part('portafilter',pos=(.27,-.015,TOP),mass=.450,friction=(.5,.4),shapes=[
    parts.cylinder(.0255,.0015,(0,0,FLOOR-.00075),STEEL),*ring(.0256,.0012,.024,(FLOOR+RIM)/2,STEEL),
    parts.cylinder(.006,.016,(-.012,0,.008),STEEL),parts.cylinder(.006,.016,(.012,0,.008),STEEL),
    parts.box((.023,.030,.018),(0,-.032,.028),STEEL),
    parts.cylinder(.014,.112,(0,-.097,.028),BLACK,axis='y')])
# Full 330W x310D x400H mm BES870 envelope; separate tray, head, hopper, wand.
MACHINE=parts.Part('espresso_machine',pos=(.66,.10,TOP),shapes=[
    parts.box((.235,.330,.320),(.0375,0,.200),STEEL),parts.box((.310,.330,.025),(0,0,.0125),BLACK),
    parts.box((.075,.275,.035),(-.1175,0,.250),STEEL),parts.cylinder(.032,.025,(-.120,0,.225),BLACK),
    parts.box((.090,.260,.014),(-.105,0,.033),STEEL),parts.cylinder(.065,.065,(.055,.080,.3675),BLACK),
    parts.cylinder(.006,.190,(-.115,-.145,.145),STEEL)])
GRINDER=parts.Part('coffee_grinder',pos=(.65,-.235,TOP),shapes=[
    parts.box((.160,.150,.230),(0,0,.115),STEEL),parts.cylinder(.075,.150,(0,0,.305),BLACK),
    parts.box((.065,.085,.040),(-.078,0,.155),BLACK),parts.box((.200,.160,.012),(-.020,0,.006),BLACK)])
OBJECTS=[TOOL,STAND,BASKET,MACHINE,GRINDER]
LOTS=lots.declare(OBJECTS,lots.Lot('A'),lots.Lot('B',place={'portafilter':(.29,.025,TOP)},yaw={'portafilter':math.pi/6}))
# No articulated parts are moved between layouts. The rigid tool and original stand stay fixed at reset.
DEMO_TOUCHES=set()
GRID=np.array([(x,y) for x in np.arange(-.024,.02401,CELL) for y in np.arange(-.024,.02401,CELL) if x*x+y*y<(.0245)**2])
AREA=math.pi*R*R/len(GRID)
CLUMP_CENTERS=np.array([(.016*math.cos(a),.016*math.sin(a),.037) for a in np.linspace(0,2*math.pi,6,endpoint=False)])
GRAIN=.0008
LOCAL=np.array([(x,y,z) for x in np.arange(-.0016,.00161,GRAIN) for y in np.arange(-.0016,.00161,GRAIN) for z in np.arange(-.0016,.00161,GRAIN) if x*x+y*y+z*z<=.002**2])
PAIRS=np.array([(i,j) for i in range(len(LOCAL)) for j in range(i) if np.linalg.norm(LOCAL[i]-LOCAL[j])<GRAIN*1.01])
# Each clump is a finite bonded grain graph. Only a tine's loaded swept shaft may sever a bond.

def span(points):
    p=np.asarray(points)
    return float(np.linalg.norm(p[:,None]-p[None,:],axis=2).max()+GRAIN) if len(p) else 0.

def shaft_damage(a,b):
    # Entire observed shaft, clipped to the metal floor/wall slabs; metres.
    a=np.asarray(a,float);b=np.asarray(b,float);d=b-a
    def clipped(lo,hi):
        if abs(d[2])<1e-12:return (a,b) if lo<=a[2]<=hi else None
        u,v=sorted(((lo-a[2])/d[2],(hi-a[2])/d[2]));u=max(0.,u);v=min(1.,v)
        return (a+u*d,a+v*d) if u<=v else None
    def radial(p,q):
        v=q[:2]-p[:2];t=np.clip(-p[:2]@v/max(v@v,1e-30),0,1)
        return np.linalg.norm(p[:2]+t*v),max(np.linalg.norm(p[:2]),np.linalg.norm(q[:2]))
    floor=clipped(-1.,FLOOR-.00025)
    if floor is not None and radial(*floor)[0]<R-.0005:return 'full shaft floor penetration'
    wall=clipped(FLOOR,RIM)
    if wall is not None:
        low,high=radial(*wall)
        if low<R+.002 and high>R+.00025:return 'full shaft wall penetration'
    return ''

class Coffee:
    def __init__(self):
        self.grains=np.concatenate([LOCAL+c for c in CLUMP_CENTERS]);self.edges=np.concatenate([PAIRS+k*len(LOCAL) for k in range(6)])
        self.active=np.ones(len(self.grains),bool);self.broken=np.zeros(len(self.edges),bool);self.shear=np.zeros(len(self.edges));self.work=np.zeros(len(self.edges))
        self.grain_mass=.020e-3/len(LOCAL);self.mass_grains=len(self.grains)*self.grain_mass
        self.volume=np.ones(len(GRID))*(MASS-self.mass_grains)/DENSITY/len(GRID)
        # Uneven fresh mound with full finite-mass accounting; no autonomous settling of clumps.
        delta=.0018*(1-(np.linalg.norm(GRID,axis=1)/R)**2);delta-=delta.mean();self.volume+=delta*AREA
        self.spilled=0.;self.load=np.zeros(3);self.torque=np.zeros(3);self.basket_torque=np.zeros(3)
        self.total_work=0.;self.last=None;self.peak_load=0.;self.damage=False;self.why='';self.surface_change=0.
        self.slide=np.zeros((len(GRID),2));self.motion_prev=None;self.peak_transport_accel=0.
        self.prev_force=np.zeros((9,3));self.next_force=np.zeros((9,3));self.hit=np.zeros(len(GRID),bool);self.step_hit=np.zeros(len(GRID),bool);self.last_motion=0.
        self.neighbor=np.argsort(np.sum((GRID[:,None]-GRID[None,:])**2,axis=2),axis=1)[:,1:13]
    def heights(self):return FLOOR+self.volume/AREA
    def count(self):
        parent=np.arange(len(self.grains))
        def root(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        for a,b in self.edges[~self.broken]:
            a,b=root(a),root(b)
            if a!=b:parent[a]=b
        groups={}
        for i in np.flatnonzero(self.active):groups.setdefault(root(i),[]).append(i)
        return int(sum(span(self.grains[g])>.003+1e-9 for g in groups.values()))
    def escaped_clump(self):
        # An intact aggregate cannot be discarded to earn deagglomeration credit.
        ids=np.flatnonzero(self.active)
        if not len(ids):return False
        p=self.grains[ids]
        if not np.isfinite(p).all():return True
        if not np.any((np.linalg.norm(p[:,:2],axis=1)>R)|(p[:,2]>RIM+.003)|(p[:,2]<FLOOR)):return False
        parent=np.arange(len(self.grains))
        def root(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        for a,b in self.edges[~self.broken]:
            a,b=root(a),root(b)
            if a!=b:parent[a]=b
        groups={}
        for i in ids:groups.setdefault(root(i),[]).append(i)
        for g in groups.values():
            center=self.grains[g].mean(0)
            if span(self.grains[g])>.003+1e-9 and (np.linalg.norm(center[:2])>R+1e-9 or center[2]>RIM+.003+1e-9 or center[2]<FLOOR-1e-9):return True
        return False
    def mass(self):return float(self.volume.sum()*DENSITY+self.mass_grains)
    def relief(self):
        h=self.heights().copy()
        # Include every surviving grain; a buried intact clump remains in count().
        for p in self.grains[self.active]:
            i=np.argmin(np.sum((GRID-p[:2])**2,axis=1));h[i]=max(h[i],p[2]+GRAIN/2)
        return float(h.max()-h.min())
    def transport(self,basket,dt):
        # Finite conservative upwind bulk slip from actual basket acceleration.
        # Coulomb coefficient0.6 is a declared uncalibrated estimate; no bond credit.
        velocity=np.asarray(basket.lin_vel);omega=np.asarray(basket.ang_vel)
        if self.motion_prev is None:self.motion_prev=(velocity.copy(),omega.copy());return 0.
        oldv,oldw=self.motion_prev;self.motion_prev=(velocity.copy(),omega.copy())
        rb=parts._quat_to_matrix(basket.quat_xyzw);acc=(velocity-oldv)/dt
        w=omega@rb;alpha=((omega-oldw)/dt)@rb
        centers=np.c_[GRID,(FLOOR+self.heights())/2]
        effective=(np.array([0.,0.,-9.81])-acc)@rb-np.cross(alpha,centers)-np.cross(w,np.cross(w,centers))
        self.peak_transport_accel=max(self.peak_transport_accel,float(np.linalg.norm(effective[:,:2],axis=1).max()))
        trial=self.slide+effective[:,:2]*dt;speed=np.linalg.norm(trial,axis=1)
        self.slide=trial*np.maximum(0.,1-.6*np.maximum(0.,-effective[:,2])*dt/np.maximum(speed,1e-30))[:,None]
        delta=self.slide*dt;distance=np.linalg.norm(delta,axis=1)
        if distance.max()<1e-12:return 0.
        n=max(1,int(np.ceil(distance.max()/.0005)))
        if n>100:self.damage=True;self.why='unresolved violent basket transport';return 0.
        direction=np.rint(delta/np.maximum(distance[:,None],1e-30)).astype(int)
        grid=np.rint(GRID/CELL).astype(int);lookup=np.full((49,49),-1,int);lookup[grid[:,0]+24,grid[:,1]+24]=np.arange(len(GRID))
        destgrid=grid+direction;inside=(np.abs(destgrid)<=24).all(1);dest=np.full(len(GRID),-1,int)
        dest[inside]=lookup[destgrid[inside,0]+24,destgrid[inside,1]+24];valid=dest>=0
        fraction=np.minimum(.5,distance/n/np.maximum(np.linalg.norm(direction,axis=1)*CELL,1e-30))
        before=self.volume.copy();edge=(~valid)&((self.slide*GRID).sum(1)>0)
        capacity=(RIM-FLOOR)*AREA
        for _ in range(n):
            amount=self.volume*fraction;move=np.where(valid,amount,0.);self.volume-=move
            np.add.at(self.volume,dest[valid],move[valid])
            loss=np.where(edge,np.maximum(0.,self.volume-capacity),0.)
            self.volume-=loss;self.spilled+=float(loss.sum()*DENSITY)
        return float(np.max(np.abs(self.volume-before))/AREA)

    def advance(self,tool,basket,dt,applied=False):
        carry_change=self.transport(basket,dt)
        rt=parts._quat_to_matrix(tool.quat_xyzw);rb=parts._quat_to_matrix(basket.quat_xyzw)
        tips=np.c_[NEEDLES,np.full(9,NLOW)]@rt.T+tool.pos
        tops=np.c_[NEEDLES,np.full(9,NHIGH)]@rt.T+tool.pos
        lower=(tips-basket.pos)@rb;upper=(tops-basket.pos)@rb
        self.next_force[:]=0;self.load[:]=0;self.torque[:]=0;self.basket_torque[:]=0;self.surface_change=0.
        if self.last is None:self.last=(lower.copy(),upper.copy());return
        old,oldtop=self.last;self.last=(lower.copy(),upper.copy())
        v=(lower-old)/dt
        h=self.heights();active=np.zeros(len(GRID),bool)
        for k in range(9):
            travel=float(np.linalg.norm(lower[k]-old[k]));vxy=float(np.linalg.norm(v[k,:2]));axis=upper[k]-lower[k];length=np.linalg.norm(axis)
            # Basket wall and floor are physical colliders, with a separate declared damage witness.
            submerged=0.;contact_centers=[]
            n=max(1,int(math.ceil(travel/.0004)))
            if n>200:self.damage=True;self.why='unresolved fast shaft motion';continue
            for f in np.linspace(0,1,n+1)[1:]:
                a=old[k]+f*(lower[k]-old[k]);b=oldtop[k]+f*(upper[k]-oldtop[k]);ab=b-a
                why=shaft_damage(a,b)
                if why:self.damage=True;self.why=why
                t=np.clip(((np.c_[GRID,h]-a)@ab)/(ab@ab),0,1)
                nearest=a+t[:,None]*ab
                dist=np.linalg.norm(nearest[:,:2]-GRID,axis=1)
                touched=(dist<NR+CELL*.71)&(nearest[:,2]>=FLOOR)&(nearest[:,2]<=h+.0005)&(a[2]<h)
                if touched.any():
                    loaded=max(0.,float(-self.prev_force[k]@(rb@(lower[k]-old[k])))) if applied else 0.
                    if travel>=.00001 and loaded>0:active|=touched
                    submerged=max(submerged,float(np.clip((h[touched]-a[2]).max()/.020,0,1)))
                    centers=(self.grains[self.edges[:,0]]+self.grains[self.edges[:,1]])/2
                    d=np.full(len(self.edges),np.inf)
                    # Contact with either constituent grain can strain their bond.
                    for endpoint in (self.grains[self.edges[:,0]],self.grains[self.edges[:,1]],centers):
                        tt=np.clip(((endpoint-a)@ab)/(ab@ab),0,1)
                        d=np.minimum(d,np.linalg.norm(endpoint-a-tt[:,None]*ab,axis=1))
                    hit=(d<NR+GRAIN/2)&~self.broken
                    ds=travel/n
                    # 4mN/tine at20mm depth; per-bond work shares the actual needle reaction.
                    loaded_work=max(0.,float(-self.prev_force[k]@(rb@(lower[k]-old[k]))))/n if applied else 0.
                    if loaded_work>0:self.shear[hit]+=ds
                    self.work[hit]+=loaded_work/max(1,int(hit.sum()))
                    sever=hit&(self.shear>=.0008)&(self.work>=5e-8)
                    self.broken|=sever
                    contact_centers.append(np.array([a[0],a[1],min(float(h[touched].max()),b[2])]))
            if submerged>0:
                speed=np.linalg.norm(v[k]);f_local=-v[k]/max(speed,1e-12)*(.004*submerged*min(1,speed/.001))
                self.next_force[k]=rb@f_local
                self.load+=rb@f_local
                point=rb@np.array([lower[k,0],lower[k,1],FLOOR+.010])+basket.pos
                self.torque+=np.cross(point-tool.pos,rb@f_local);self.basket_torque+=np.cross(point-basket.pos,-rb@f_local)
                self.total_work+=max(0.,float(-self.prev_force[k]@(rb@(lower[k]-old[k])))) if applied else 0.
                # A swift sweep ejects finite coffee, rather than deleting it or leveling freely.
                if vxy>.12:
                    loss=min(self.mass()/4,(vxy-.12)*dt*.001)
                    frac=loss/max(self.mass(),1e-12);self.volume*=1-frac;self.mass_grains*=1-frac;self.grain_mass*=1-frac;self.spilled+=loss
                    self.damage|=vxy>.4;self.why='violent sweep' if vxy>.4 else self.why
        self.peak_load=max(self.peak_load,float(np.linalg.norm(self.load)))
        # Mobilized grains exchange locally with nearest cells. Equal and opposite volumes conserve mass.
        before=self.volume.copy()
        for i in np.flatnonzero(active):
            for j in self.neighbor[i]:
                dh=(self.volume[i]-self.volume[j])/AREA
                if dh>.00005:
                    transfer=min(self.volume[i]*.1,dh*AREA*.18)
                    self.volume[i]-=transfer;self.volume[j]+=transfer
        self.hit|=active;self.step_hit=active.copy()
        # Broken aggregates smaller than the declared3mm criterion settle into local free columns.
        # This transfers their exact mass, not a visibility flag or whole-clump deletion.
        if self.broken.any():
            parent=np.arange(len(self.grains))
            def root(i):
                while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
                return i
            for a,b in self.edges[~self.broken]:
                a,b=root(a),root(b)
                if a!=b:parent[a]=b
            groups={}
            for i in np.flatnonzero(self.active):groups.setdefault(root(i),[]).append(i)
            for g in groups.values():
                if span(self.grains[g])<=.003+1e-9:
                    for i in g:
                        j=np.argmin(np.sum((GRID-self.grains[i,:2])**2,axis=1))
                        self.volume[j]+=self.grain_mass/DENSITY;self.mass_grains-=self.grain_mass;self.active[i]=False
                        self.grains[i,2]=FLOOR+.0001
        self.surface_change=max(carry_change,float(np.max(np.abs(self.volume-before))/AREA))
        self.last_motion=max(self.surface_change,float(travel if active.any() else 0))

COFFEE=Coffee();REC={};_visual=None

def fresh():return dict(known=False,bad=False,why='',hold=0.,t=0.,certificate=None,finger_peak=0.,clear=False,window=[],physics_steps=0,physics_hand_checks=0,physics_checked=0,physics_t=0.,physics_known=False,physics_dt=None,min_support_reaction_N=math.inf)
REC=fresh()

def stamp(objects):return tuple(np.concatenate([np.r_[objects[n].pos,objects[n].quat_xyzw,objects[n].lin_vel,objects[n].ang_vel] for n in ('wdt','portafilter','original_stand')]))

@lru_cache(maxsize=1)
def hand_mesh():
    from yam_arena import analysis
    out={}
    for name in ('gripper','tip_left','tip_right'):
        cloud=[]
        for hull in analysis.link_hulls().get(name,[]):
            cloud.extend(hull.vertices)
            for plane in np.unique(hull.planes.round(7),axis=0):
                face=hull.vertices[np.abs(hull.vertices@plane[:3]-plane[3])<2e-6]
                if len(face)<3:continue
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(plane[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for a,b in zip(face,np.roll(face,-1,axis=0)):
                    nn=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                    for i in range(nn+1):
                        for j in range(nn-i+1):cloud.append(c+(a-c)*i/nn+(b-c)*j/nn)
        cloud=np.asarray(cloud);_,keep=np.unique(np.floor(cloud/.0015).astype(int),axis=0,return_index=True);out[name]=cloud[keep]
    return out

@lru_cache(maxsize=1)
def hand_hulls():
    from yam_arena import analysis
    return {n:analysis.link_hulls().get(n,[]) for n in ('gripper','tip_left','tip_right')}

def segment_in_solid(a,b,planes,margin=.00025):
    normals=planes[:,:3];v=normals@a-planes[:,3]+margin;d=normals@(b-a)
    if np.any((np.abs(d)<1e-12)&(v>0)):return False
    enter=max(0.,float(np.max(-v[d< -1e-12]/d[d< -1e-12],initial=-np.inf)))
    leave=min(1.,float(np.min(-v[d>1e-12]/d[d>1e-12],initial=np.inf)))
    return enter<=leave

def box_planes(size):
    normal=np.r_[np.eye(3),-np.eye(3)]
    return np.c_[normal,np.r_[size/2,size/2]]

def wire_safety(objects):
    tool=objects['wdt'];rt=parts._quat_to_matrix(tool.quat_xyzw)
    current=np.stack([np.c_[NEEDLES,np.full(9,z)]@rt.T+tool.pos for z in (NLOW,NHIGH)],axis=1)
    previous=REC.get('wire_previous',current);oldsolids=REC.get('solid_previous',{})
    solids=dict(REC.get('hand_solids',{}));stand=objects['original_stand']
    for i,(_,center,r,size) in enumerate(STAND.world_boxes({'base':(stand.pos,parts._quat_to_matrix(stand.quat_xyzw))})):
        solids['stand'+str(i)]=(np.asarray(center),r,box_planes(size),-size/2,size/2)
    from yam_arena.station import TABLE_CENTER,TABLE_SIZE
    size=np.asarray(TABLE_SIZE);solids['counter']=(np.asarray(TABLE_CENTER),np.eye(3),box_planes(size),-size/2,size/2)
    for key,(center,r,planes,low,high) in solids.items():
        oldcenter,oldr,*_=oldsolids.get(key,solids[key])
        start=(previous-oldcenter)@oldr;end=(current-center)@r
        for a,b in zip(start,end):
            cloud=np.r_[a,b]
            if np.any(cloud.max(0)<low) or np.any(cloud.min(0)>high):continue
            n=max(1,int(np.ceil(np.linalg.norm(b-a,axis=1).max()/.0004)))
            if n>200:COFFEE.damage=True;COFFEE.why='unresolved rapid needle/solid motion';continue
            for t in np.linspace(0,1,n+1)[1:]:
                segment=(1-t)*a+t*b
                if segment_in_solid(*segment,planes):
                    COFFEE.damage=True;COFFEE.why='needle penetrated '+key;break
    REC['wire_previous']=current;REC['solid_previous']=solids

def hands(sim):
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[];peak=0.;diag={}
    from yam_arena.constants import TCP_OFFSET
    REC['hand_solids']={}
    for arm in ('left_robot','right_robot'):
        robot=base.scene[arm];poses=parts._np(robot.data.body_link_pose_w)[0];q=parts._np(robot.data.joint_pos)[0]
        if not np.isfinite(np.r_[poses.reshape(-1),q]).all():raise ValueError('unknown arm')
        for name,p in hand_mesh().items():
            pose=poses[robot.body_names.index(name)];rotation=parts._quat_to_matrix(pose[3:]);center=pose[:3]-origin
            clouds.append(p@rotation.T+center)
            for i,hull in enumerate(hand_hulls()[name]):REC['hand_solids'][arm+name+str(i)]=(center,rotation,np.asarray(hull.planes),hull.vertices.min(0),hull.vertices.max(0))
        pose=poses[robot.body_names.index('gripper')];diag[arm]=dict(tcp=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@TCP_OFFSET).tolist(),quat=pose[3:].tolist())
        effort=robot.root_view.get_dof_projected_joint_forces();effort=effort.torch if hasattr(effort,'torch') else effort;effort=(effort.detach().cpu().numpy() if hasattr(effort,'detach') else np.asarray(effort.numpy()))[0]
        peak=max(peak,max(abs(float(effort[robot.joint_names.index(n)])) for n in ('joint7','joint8')))
    if not all(len(c) and np.isfinite(c).all() for c in clouds):raise ValueError('unknown hand geometry')
    return clouds,peak,diag

def clear_of(clouds,spec,state,margin=.005):
    if not clouds or any(c.ndim!=2 or c.shape[1]!=3 or not len(c) or not np.isfinite(c).all() for c in clouds):return False
    for _,center,r,size in spec.world_boxes({'base':(state.pos,parts._quat_to_matrix(state.quat_xyzw))}):
        if any(np.linalg.norm(np.maximum(np.abs((p-center)@r)-size/2,0),axis=1).min()<=margin for p in clouds):return False
    return True

def released(clouds,objects):
    if not clouds or any(spec.name not in objects for spec in (TOOL,BASKET,STAND)):return False
    return all(clear_of(clouds,spec,objects[spec.name]) for spec in (TOOL,BASKET,STAND))

# AABBs are only a broad phase. Final support queries use the actual solids.
SUPPORT_TOL=.0002
_AUTO_CONTACT=object()

def solid(shape,state):
    rotation=parts._quat_to_matrix(state.quat_xyzw)
    frame=rotation@shape.frame;center=state.pos+rotation@np.asarray(shape.at)
    half=shape.half;extent=np.abs(frame)@half
    return (shape,center,frame,center-extent,center+extent)

def solid_point(s,direction):
    shape,center,frame,*_=s;d=frame.T@direction
    if shape.kind=='box':return center+frame@(np.sign(d)*shape.half)
    if shape.kind=='hull':return shape.vertices[np.argmax(shape.vertices@direction)]
    axis='xyz'.index(shape.axis);radial=d.copy();radial[axis]=0.
    p=radial/max(np.linalg.norm(radial),1e-30)*shape.size[0]
    p[axis]=np.sign(d[axis])*shape.size[1]/2
    return center+frame@p

def closest_simplex(points):
    # Project the origin onto a simplex. Enumerating its faces handles degeneracy.
    from itertools import combinations
    best=None;best_points=None
    points=np.asarray(points)
    for n in range(1,min(4,len(points))+1):
        for ids in combinations(range(len(points)),n):
            p=points[list(ids)]
            if n==1:weights=np.ones(1)
            else:
                delta=(p[1:]-p[0]).T
                tail=np.linalg.lstsq(delta,-p[0],rcond=None)[0]
                weights=np.r_[1-tail.sum(),tail]
            if np.any(weights < -1e-10):continue
            v=weights@p;norm=float(v@v)
            if best is None or norm<float(best@best):
                best=v;best_points=p[weights>1e-10]
    return best,best_points

def solids_touch(a,b,tol=SUPPORT_TOL):
    if np.any(a[3]>b[4]+tol) or np.any(b[3]>a[4]+tol):return False
    direction=a[1]-b[1]
    if np.linalg.norm(direction)<1e-12:direction=np.array([0.,0.,1.])
    points=np.array([solid_point(a,-direction)-solid_point(b,direction)])
    v=points[0]
    for _ in range(48):
        distance=np.linalg.norm(v)
        if distance<=tol+1e-10:return True  # a certified upper bound on separation
        w=solid_point(a,-v)-solid_point(b,v)
        if float(v@w)/distance>tol+1e-10:return False  # separating support plane
        if np.linalg.norm(points-w,axis=1).min()<1e-12:return False
        v,points=closest_simplex(np.r_[points,w[None,:]])
        if v is None:return False
    return False  # unresolved query never supplies a support certificate

def basket_contact(objects):
    b=objects['portafilter']
    # Native spouts are checked first; this is only an optimization.
    order=sorted(range(len(BASKET.shapes)),key=lambda i:BASKET.shapes[i].at[2]-BASKET.shapes[i].half[2])
    basket=[]
    from yam_arena.station import TABLE_CENTER,TABLE_SIZE
    tc=np.asarray(TABLE_CENTER);ts=np.asarray(TABLE_SIZE)
    table_state=parts.PartState(pos=tc,quat_xyzw=np.array([0.,0.,0.,1.]),low=tc-ts/2,high=tc+ts/2,lin_vel=np.zeros(3),ang_vel=np.zeros(3))
    table=solid(parts.box(tuple(ts)),table_state)
    for i in order:
        body=solid(BASKET.shapes[i],b);basket.append((i,body))
        point=solid_point(body,np.array([0.,0.,-1.]))
        if abs(point[2]-TOP)<=SUPPORT_TOL and np.all(point[:2]>=tc[:2]-ts[:2]/2) and np.all(point[:2]<=tc[:2]+ts[:2]/2):
            return {'pair':'counter/basket.s'+str(i),'gap_m':float(abs(point[2]-TOP))}
        # The lowest point may be outside a finite table edge while another point touches.
        if abs(point[2]-TOP)<=SUPPORT_TOL and solids_touch(body,table):
            return {'pair':'counter-edge/basket.s'+str(i),'gap_m':None}
    for spec in (STAND,MACHINE,GRINDER,TOOL):
        state=objects.get(spec.name,spec.state())
        for j,shape in enumerate(spec.shapes):
            if spec is TOOL and j>=2:continue  # fine needles are not basket supports
            support=solid(shape,state)
            for i,body in basket:
                if solids_touch(body,support):
                    return {'pair':spec.name+'.s'+str(j)+'/basket.s'+str(i),'gap_m':None}
    return None

def basket_supported(objects):return basket_contact(objects) is not None

def safety_history(objects,clouds,dt=.05,com_velocity=None,*,require_load=False,surface_contact=_AUTO_CONTACT,hand_contact=None):
    b=objects['portafilter'];r=parts._quat_to_matrix(b.quat_xyzw)
    velocity=None if com_velocity is None else np.asarray(com_velocity,float)
    if velocity is not None and (velocity.shape!=(3,) or not np.isfinite(velocity).all()):velocity=None
    previous=REC.get('support_velocity');REC['support_velocity']=None if velocity is None else velocity.copy()
    reaction=None if previous is None or velocity is None else BASKET.mass*((velocity[2]-previous[2])/dt+9.81)
    REC['support_reaction_N']=reaction
    if r[2,2]<math.cos(math.radians(15)) or b.pos[2]<TOP-.005:
        REC.update(bad=True,why='basket tipped or dropped');return
    contact=basket_contact(objects) if surface_contact is _AUTO_CONTACT else surface_contact
    REC['surface_contact']=contact
    touch=hand_contact
    if contact is None and touch is None:
        touch=bool(clouds) and all(c.ndim==2 and c.shape[1]==3 and len(c) and np.isfinite(c).all() for c in clouds) and not clear_of(clouds,BASKET,b,margin=.0005)
    if not contact and not touch:
        REC.update(bad=True,why='unsupported basket release/drop');return
    if require_load or contact is None:
        if reaction is None or not np.isfinite(reaction) or reaction<.8*BASKET.mass*9.81:
            REC.update(bad=True,why='insufficient measured support reaction')

def native_array(value):
    # Direct PhysX views use Warp arrays; cached asset fields can be Torch.
    if hasattr(value,'torch'):value=value.torch
    if hasattr(value,'detach'):value=value.detach().cpu()
    if hasattr(value,'numpy'):value=value.numpy()
    return np.asarray(value,dtype=np.float64)

def native_body(base,spec):
    view=base.scene[spec.name].root_physx_view
    pose=native_array(view.get_transforms())[0];velocity=native_array(view.get_velocities())[0]
    if not np.isfinite(np.r_[pose,velocity]).all():raise ValueError('unknown physics body')
    from types import SimpleNamespace
    state=SimpleNamespace(pos=pose[:3]-parts._np(base.scene.env_origins[0]),quat_xyzw=pose[3:],lin_vel=velocity[:3],ang_vel=velocity[3:])
    return state,velocity[:3]

def native_hand_solids(base):
    from types import SimpleNamespace
    origin=parts._np(base.scene.env_origins[0]);out=[]
    for arm in ('left_robot','right_robot'):
        robot=base.scene[arm];poses=native_array(robot.root_view.get_link_transforms())[0]
        for name,hulls in hand_hulls().items():
            if not hulls:raise ValueError('missing physics hand hull')
            pose=poses[robot.body_names.index(name)];frame=parts._quat_to_matrix(pose[3:])
            for hull in hulls:
                vertices=np.asarray(hull.vertices)@frame.T+pose[:3]-origin
                if not np.isfinite(vertices).all():raise ValueError('unknown physics hand')
                shape=SimpleNamespace(kind='hull',vertices=vertices)
                out.append((shape,vertices.mean(0),frame,vertices.min(0),vertices.max(0)))
    if not out:raise ValueError('missing physics hands')
    return out

def native_hand_contact(base,objects,solids=None):
    basket=[solid(shape,objects['portafilter']) for shape in BASKET.shapes]
    hands=native_hand_solids(base) if solids is None else solids
    return any(solids_touch(body,hand,tol=.0005) for hand in hands for body in basket)

def hand_entry_history(objects,hands):
    if not hands:raise ValueError('unknown physics hand geometry')
    # The forbidden region is the basket opening between floor and rim,
    # including empty space above the grounds. A tiny inward margin keeps
    # tangency to the metal rim from becoming interior entry by roundoff.
    interior=solid(parts.cylinder(R-1e-7,RIM-FLOOR-2e-7,(0,0,(RIM+FLOOR)/2)),objects['portafilter'])
    entered=any(solids_touch(hand,interior,tol=0.) for hand in hands)
    if entered:REC.update(bad=True,why='physics-step hand inside basket')
    REC['physics_hand_checks']+=1
    return entered

@parts.on_physics_step
def physics_history(sim,dt):
    try:
        if abs(dt-1/240)>1e-9:raise ValueError('physics witness requires240Hz')
        base=sim.env.unwrapped
        objects={}
        for spec in OBJECTS:
            if spec.mass is not None:
                state,body_velocity=native_body(base,spec);objects[spec.name]=state
                if spec is BASKET:basket,velocity=state,body_velocity
            else:objects[spec.name]=spec.state()
        hand_solids=native_hand_solids(base)
        entered=hand_entry_history(objects,hand_solids)
        contact=basket_contact(objects)
        hand=False if contact else native_hand_contact(base,objects,hand_solids)
        safety_history(objects,[],dt,velocity,require_load=True,surface_contact=contact,hand_contact=hand)
        REC['physics_steps']+=1;REC['physics_t']+=dt;REC['physics_dt']=dt;REC['physics_known']=True
        REC['min_support_reaction_N']=min(REC['min_support_reaction_N'],REC['support_reaction_N'] if REC['support_reaction_N'] is not None else -math.inf)
        if os.environ.get('TRACE_OUT'):
            row=dict(t=REC['physics_t'],step=REC['physics_steps'],dt=dt,lot=lots.current().name,bad=REC['bad'],why=REC['why'],contact=contact,hand_contact=hand,reaction_N=REC['support_reaction_N'],hand_entry=entered,hand_checks=REC['physics_hand_checks'],basket_pos=basket.pos.tolist(),basket_quat=basket.quat_xyzw.tolist(),basket_com_v=velocity.tolist())
            with pathlib.Path(os.environ['TRACE_OUT']).with_name('physics.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    except Exception as e:
        REC.update(known=False,bad=True,physics_known=False,why='physics support witness: '+repr(e));raise

def body_in_dose(position,rotation):
    # Analytic finite body cylinders against every finite-height coffee column.
    a=(np.c_[GRID,np.full(len(GRID),FLOOR)]-position)@rotation
    b=(np.c_[GRID,COFFEE.heights()]-position)@rotation
    for radius,half,z in ((.0155,.01975,.00675),(.0105,.00675,-.01975)):
        aa=a-np.array([0,0,z]);d=b-a;dz=d[:,2];parallel=np.abs(dz)<1e-12
        den=np.where(parallel,1.,dz);u=(-half-aa[:,2])/den;v=(half-aa[:,2])/den
        low=np.maximum(0.,np.minimum(u,v));high=np.minimum(1.,np.maximum(u,v))
        low[parallel]=0.;high[parallel]=1.
        valid=(low<=high)&(~parallel|(np.abs(aa[:,2])<=half))&(COFFEE.volume>1e-18)
        lo=aa+low[:,None]*d;hi=aa+high[:,None]*d;dv=hi[:,:2]-lo[:,:2]
        t=np.clip(-np.sum(lo[:,:2]*dv,axis=1)/np.maximum(np.sum(dv*dv,axis=1),1e-30),0,1)
        if np.any(valid&(np.linalg.norm(lo[:,:2]+t[:,None]*dv,axis=1)<=radius+CELL*.71)):return True
        grains=(COFFEE.grains[COFFEE.active]-position)@rotation-np.array([0,0,z])
        if len(grains):
            gap=np.c_[np.maximum(np.linalg.norm(grains[:,:2],axis=1)-radius,0),np.maximum(np.abs(grains[:,2])-half,0)]
            if np.any(np.linalg.norm(gap,axis=1)<=GRAIN/2):return True
    return False

def body_safety(objects,dt=.05):
    b=objects['portafilter'];tool=objects['wdt'];rb=parts._quat_to_matrix(b.quat_xyzw)
    position=(tool.pos-b.pos)@rb;rotation=rb.T@parts._quat_to_matrix(tool.quat_xyzw)
    if (np.linalg.norm(tool.ang_vel)+np.linalg.norm(b.ang_vel))*dt>math.pi/2:
        REC.update(bad=True,why='unresolved full-body rotation');return
    q=parts._matrix_to_quat(rotation);oldp,oldq=REC.get('body_previous',(position,q))
    REC['body_previous']=(position.copy(),q.copy())
    if oldq@q<0:q=-q
    angle=2*math.acos(float(np.clip(oldq@q,-1,1)))
    steps=max(1,int(math.ceil((np.linalg.norm(position-oldp)+.031*angle)/.0004)))
    if steps>500:REC.update(bad=True,why='unresolved full-body sweep');return
    top=max(float(COFFEE.heights().max()),float(COFFEE.grains[COFFEE.active,2].max()+GRAIN/2) if COFFEE.active.any() else FLOOR)
    if min(position[2],oldp[2])-.031>top:return
    for f in np.linspace(0,1,steps+1):
        half=angle/2
        qi=((math.sin((1-f)*half)*oldq+math.sin(f*half)*q)/math.sin(half)) if half>1e-8 else (1-f)*oldq+f*q
        qi/=np.linalg.norm(qi)
        if body_in_dose((1-f)*oldp+f*position,parts._quat_to_matrix(qi)):
            REC.update(bad=True,why='full tool body in coffee');return

def parked(objects):
    s=objects['wdt'];stand=objects['original_stand'];r=parts._quat_to_matrix(stand.quat_xyzw)
    p=(s.pos-stand.pos)@r
    return bool(-.001-1e-9<=p[0]<=.005+1e-9 and abs(p[1])<=.0015+1e-9 and abs(p[2]-.0845)<=.002 and parts._quat_to_matrix(s.quat_xyzw)[2,2]>=math.cos(math.radians(3)))

def tier(count,relief,mass,parked_,valid=True):
    if not valid or mass<.019-1e-9 or count>=6:return 0.
    if count>2 or relief>.004+1e-9:return .25
    if count>0 or relief>.002+1e-9:return .5
    return 1. if parked_ else .75

def score(objects):
    try:
        if not REC['known'] or REC['bad'] or COFFEE.damage or COFFEE.escaped_clump() or REC['hold']<1.-1e-9 or REC['certificate']!=stamp(objects):return 0.
        return tier(COFFEE.count(),COFFEE.relief(),COFFEE.mass(),parked(objects))
    except (KeyError,ValueError,TypeError,AttributeError):return 0.
def success(objects):return score(objects)==1.

# All runtime changes are task local: finite material rule, visible particles and physical reaction loads.
def force(sim,name,f,tau):
    import torch
    base=sim.env.unwrapped;asset=base.scene[name]
    f=torch.tensor(np.asarray(f).reshape(1,1,3),dtype=torch.float32,device=base.device)
    tau=torch.tensor(np.asarray(tau).reshape(1,1,3),dtype=torch.float32,device=base.device)
    asset.set_external_force_and_torque(forces=f,torques=tau,is_global=True)
    asset.write_data_to_sim()

def draw(sim,objects):
    global _visual
    from pxr import UsdGeom,Gf,Vt
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage();path='/World/coffee_visible'
    if _visual is None or not _visual.GetPrim().IsValid():
        _visual=UsdGeom.Points.Define(stage,path);_visual.CreateDisplayColorAttr([Gf.Vec3f(*BROWN)]);_visual.CreateWidthsAttr([.001]*len(GRID)+[GRAIN]*len(COFFEE.grains))
    basket=objects['portafilter'];r=parts._quat_to_matrix(basket.quat_xyzw)
    points=np.r_[np.c_[GRID,COFFEE.heights()],COFFEE.grains]@r.T+basket.pos
    _visual.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    # Colored grains denote physical coffee, never an alternate scoring-state indicator.

@parts.on_reset
def reset(sim):
    global COFFEE,REC,_visual
    COFFEE=Coffee();REC=fresh();_visual=None
    _,velocity=native_body(sim.env.unwrapped,BASKET);REC['support_velocity']=velocity.copy()
    draw(sim,sim.states())
    for _ in range(4):sim.env.unwrapped.sim.render()
    from inspect_robots_yam.config import DEFAULT_CAMERAS
    for name in DEFAULT_CAMERAS:sim.env.unwrapped.scene[name].update(0.,force_recompute=True)

def update_hold(t,objects,eligible):
    if not eligible:
        REC['window']=[];REC['hold']=0.;return
    poses=np.array([np.r_[objects[n].pos,objects[n].quat_xyzw] for n in ('wdt','portafilter','original_stand')])
    window=REC['window'];window.append((t,poses))
    while len(window)>1 and window[1][0]<=t-1.+1e-8:window.pop(0)
    pp=np.array([v for _,v in window]);still=True
    for i in range(3):
        pos=pp[:,i,:3];q=pp[:,i,3:];q=q/np.linalg.norm(q,axis=1)[:,None]
        travel=float(np.linalg.norm(pos[:,None]-pos[None,:],axis=2).max())
        turn=float((2*np.arccos(np.clip(np.abs(q@q.T),0,1))).max())
        still &= travel<=.001+1e-9 and turn<=math.radians(1)+1e-9
    if still:REC['hold']=t-window[0][0]
    else:REC['window']=[(t,poses)];REC['hold']=0.

@parts.on_step
def step(sim):
    global REC
    objects=sim.states();dt=float(sim.t)-REC['t']
    try:
        if not 0<dt<=.10001:raise ValueError('invalid clock')
        if not REC['physics_known'] or REC['physics_dt'] is None:raise ValueError('missing physics history')
        ticks=REC['physics_steps']-REC['physics_checked']
        if ticks!=round(dt/REC['physics_dt']) or REC['physics_hand_checks']!=REC['physics_steps']:raise ValueError('incomplete physics history')
        REC['physics_checked']=REC['physics_steps']
        clouds,peak,diag=hands(sim);REC['known']=True;REC['finger_peak']=max(REC['finger_peak'],peak)
        b=objects['portafilter'];tool=objects['wdt']
        if not np.isfinite(np.r_[stamp(objects)]).all():raise ValueError('unknown scored pose')
        r=parts._quat_to_matrix(b.quat_xyzw)
        com_velocity=parts._np(sim.env.unwrapped.scene['portafilter'].data.root_com_lin_vel_w)[0]
        # Direct hand entry, pressing with the tool body, or crushing the basket never earns material credit.
        for c in clouds:
            p=(c-b.pos)@r
            if np.any((np.linalg.norm(p[:,:2],axis=1)<R)&(p[:,2]>FLOOR)&(p[:,2]<RIM)):REC.update(bad=True,why='hand inside coffee')
        body_safety(objects,dt)
        wire_safety(objects)
        if COFFEE.escaped_clump():REC.update(bad=True,why='intact clump discarded')
        COFFEE.advance(tool,b,dt,applied=REC.get("load_applied",False))
        force(sim,'wdt',COFFEE.load,COFFEE.torque);force(sim,'portafilter',-COFFEE.load,COFFEE.basket_torque)
        COFFEE.prev_force=COFFEE.next_force.copy();REC['load_applied']=True
        REC['clear']=released(clouds,objects)
        eligible=REC['clear'] and COFFEE.surface_change<.00005 and not REC['bad'] and not COFFEE.damage
        update_hold(float(sim.t),objects,eligible)
        REC['certificate']=stamp(objects);REC['t']=float(sim.t)
        draw(sim,objects)
        if os.environ.get('TRACE_OUT'):
            for checkpoint in (90.,99.2,101.1):
                key='snapshot_'+str(checkpoint)
                if sim.t>=checkpoint and not REC.get(key):
                    np.savez(pathlib.Path(os.environ['TRACE_OUT']).with_name('coffee-'+lots.current().name+'-at-'+str(checkpoint)+'.npz'),volume=COFFEE.volume,grains=COFFEE.grains,active=COFFEE.active,broken=COFFEE.broken,hit=COFFEE.hit)
                    REC[key]=True
        if os.environ.get('TRACE_OUT') and (sim.t>=119.8 or success(objects)) and not REC.get('dumped'):
            np.savez(pathlib.Path(os.environ['TRACE_OUT']).with_name('coffee-final-'+lots.current().name+'.npz'),volume=COFFEE.volume,grains=COFFEE.grains,active=COFFEE.active,broken=COFFEE.broken,hit=COFFEE.hit)
            REC['dumped']=True
        if os.environ.get('TRACE_OUT') and (round(sim.t*20)%5==0 or sim.t>=90.):
            row=dict(t=sim.t,known=REC['known'],bad=REC['bad'],why=REC['why'],damage=COFFEE.damage,damage_why=COFFEE.why,
                clumps=COFFEE.count(),broken=int(COFFEE.broken.sum()),bonds=len(COFFEE.broken),relief=COFFEE.relief(),mass=COFFEE.mass(),spilled=COFFEE.spilled,
                load=COFFEE.load.tolist(),work=COFFEE.total_work,peak_load=COFFEE.peak_load,coverage=float(COFFEE.hit.mean()),transport_accel=COFFEE.peak_transport_accel,hold=REC['hold'],clear=REC['clear'],parked=parked(objects),score=score(objects),
                physics_steps=REC['physics_steps'],physics_hand_checks=REC['physics_hand_checks'],physics_ticks=ticks,physics_dt=REC['physics_dt'],min_support_reaction_N=REC['min_support_reaction_N'],surface_contact=REC.get('surface_contact'),ridge_step_hit=bool(COFFEE.step_hit[np.argmin(np.linalg.norm(GRID-[.008,-.015],axis=1))]),support_reaction_N=REC.get('support_reaction_N'),basket_com_velocity=com_velocity.tolist(),finger_peak=REC['finger_peak'],hands=diag,velocities={n:np.r_[objects[n].lin_vel,objects[n].ang_vel].tolist() for n in ('wdt','portafilter','original_stand')},poses={n:np.r_[s.pos,s.quat_xyzw].tolist() for n,s in objects.items() if n in ('wdt','portafilter','original_stand')})
            with pathlib.Path(os.environ['TRACE_OUT']).with_name('coffee.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    except Exception as e:
        REC.update(known=False,bad=True,hold=0.,why=repr(e));raise

def demo():
    # Published visible basket and stand positions select geometry; no material/force/hidden-state feedback.
    basket=np.asarray(lots.current().pos('portafilter'));home=np.asarray(TOOL.pos)+np.array([0,0,.015]);plan=[]
    def w(t,p,f,label):
        # Fixed calibration from the published layout's native support creep; no state feedback.
        p=np.asarray(p,float).copy()
        if label in ('loaded whole-dose tine raster','move to finishing sweep','loaded finishing redistribution'):
            elapsed=sum(v.duration for v in plan)+t
            velocity=np.array([-.00000083,-.000056,0.]) if abs(yaw)<.1 else np.array([.000049,-.000078,0.])
            p+=velocity*elapsed
            if label=='loaded finishing redistribution':
                p[0]-=.0045
                if abs(yaw)>.1 and angle>5.:p+=np.array([.0035,.001,0.])
        plan.append(Waypoint(t,tuple(p),f,label))
    grip=-.014;travel=TOP+.160;stand_clear=TOP+.175
    w(2,(home[0],home[1],travel),FINGER_OPEN,'approach native body')
    w(2,home,FINGER_OPEN,'descend around31mm body');w(1.5,home,grip,'grasp native body')
    w(.75,home+np.array([0,0,.006]),grip,'unseat shoulder with tines still below cradle')
    w(1.,home+np.array([.035,0,.006]),grip,'extract neck through open side before lifting tines')
    w(1.,(home[0]+.035,home[1],stand_clear),grip,'lift tines only outside cradle')
    w(.25,(home[0]+.035,home[1],travel),grip,'lower transit after clearing cradle')
    yaw=lots.current().yaw.get('portafilter',0.)
    correction=parts._rot((0,0,1),yaw)@np.array([0.,-.0042,0.])+np.array([.001,-.001,0.])
    center=np.array([basket[0],basket[1],TOP+.117])+correction
    w(3,(center[0],center[1],travel),grip,'carry above basket')
    w(3,center,grip,'enter coffee clear of metal floor')
    # Whole-dose crossed raster; open-loop and independent of hidden grain/bond state.
    for swap in (False,True):
        for i,y in enumerate(np.arange(-.009,.00901,.0006)):
            extent=math.sqrt(.0125**2-float(y)**2)
            xs=(-extent,extent) if i%2==0 else (extent,-extent)
            for x in xs:
                offset=np.array([y,x,0.] if swap else [x,y,0.])
                w(.18 if x==xs[0] else 1.05,center+offset,grip,'loaded whole-dose tine raster')
    # Address all six published reset sectors; no hidden clump/height feedback.
    for angle in np.linspace(0,2*math.pi,6,endpoint=False):
        target=(parts._rot((0,0,1),yaw)@np.array([.016*math.cos(angle),.016*math.sin(angle),0.]))[:2]
        peripheral=NEEDLES[1:][np.argmin(np.linalg.norm(NEEDLES[1:]-target,axis=1))]
        local=target-peripheral
        w(.20,center+np.r_[local+[-.003,-.003],0.],grip,'loaded finishing redistribution')
        for i,y in enumerate(np.linspace(-.003,.0034 if abs(yaw)>.1 else .003,7)):
            bounds=(-.003,.00475 if abs(yaw)>.1 else .003)
            xs=bounds if i%2==0 else bounds[::-1]
            for j,x in enumerate(xs):
                w(.055 if j==0 else .16,center+np.r_[local+[x,y],0.],grip,'loaded finishing redistribution')
    w(2,(center[0],center[1],travel),grip,'withdraw without rim contact')
    w(3,(home[0]+.035,home[1]-.001,stand_clear),grip,'return outside open side of supplied cradle')
    w(2.5,home+np.array([.035,-.001,.006]),grip,'lower beside cradle without threading tines through prongs')
    w(1.5,home+np.array([.001,-.001,.006]),grip,'slide native neck into open C cradle')
    w(1,home+np.array([.001,-.001,.006]),grip,'settle neck inside cradle')
    w(1.5,home+np.array([.001,-.001,.001]),grip,'seat shoulder onto native cradle')
    w(1.5,home+np.array([.001,-.001,.001]),FINGER_OPEN,'release tool')
    w(2,(home[0],home[1],travel),FINGER_OPEN,'withdraw hand')
    w(3,(.16,.28,TOP+.16),FINGER_OPEN,'hands clear still hold')
    if os.environ.get('TRACE_OUT'):print(f'[coffee-demo] lot={lots.current().name} yaw={yaw:.6f} final-sector-XY-mm={(3.5,1.) if abs(yaw)>.1 else (0.,0.)}',file=sys.stderr,flush=True)
    return {'left':plan}

# Refresh material after the layout helper has moved a rigid basket and settled it.
lots.AFTER_LAYOUT.append(reset)
