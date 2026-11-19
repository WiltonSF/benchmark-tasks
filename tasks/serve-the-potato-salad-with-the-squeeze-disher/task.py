"""A loose squeeze disher, a bowl of soft potato salad and an empty serving
plate sit on the counter. The tool is not preloaded. Both containers are loose.
"""
import itertools
import json
import math
import os
import pathlib
import sys
from dataclasses import dataclass, field
import numpy as np

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts
import lots
import tilt
import geom_helpers as release
import material_runtime,native_history
from yam_arena.demo import Waypoint,FINGER_OPEN
from yam_arena.task_file import TABLE_TOP

SCENE="station"
TOP=TABLE_TOP[SCENE]
SCORE_OVER="end"
SUCCESS_HOLD_S=.1
EPS=1e-9
STEEL=(.62,.65,.67)
FOOD=(.88,.78,.47)
R,D=.0231,.0231
HEAD=np.array((.115,0.,0.))
LEVER_MAX=.13
RATIO=math.pi/LEVER_MAX
CAPACITY=2/3*math.pi*R*R*D
DENSITY=1050.

def wedge(r0,r1,z0,z1,a0,a1,color,center=(0,0,0),inner_top=None,outer_top=None):
    rt0=r0 if inner_top is None else inner_top
    rt1=r1 if outer_top is None else outer_top
    v=np.array([(r*math.cos(a),r*math.sin(a),z)
      for z,rs in ((z0,(r0,r1)),(z1,(rt0,rt1)))
      for r,a in ((rs[0],a0),(rs[1],a0),(rs[1],a1),(rs[0],a1))])+center
    lo,hi=v.min(0),v.max(0)
    return parts.Shape("mesh",tuple(hi-lo),tuple((hi+lo)/2),color,
        vertices=tuple(tuple(float(x) for x in p) for p in v))

def ring(radius,wall,height,z,color,n=24):
    return [wedge(radius-wall/2,radius+wall/2,z-height/2,z+height/2,
        2*math.pi*i/n,2*math.pi*(i+1)/n,color) for i in range(n)]

def bar(a,b,width,height,color):
    a,b=np.array(a,float),np.array(b,float)
    axis=b-a;axis/=np.linalg.norm(axis)
    side=np.cross((0,0,1),axis);side/=np.linalg.norm(side)
    v=np.array([p+sgn*width/2*side+np.array((0,0,z*height/2))
        for p in (a,b) for sgn,z in ((-1,-1),(1,-1),(1,1),(-1,1))])
    lo,hi=v.min(0),v.max(0)
    return parts.Shape("mesh",tuple(hi-lo),tuple((hi+lo)/2),color,
        vertices=tuple(tuple(float(x) for x in p) for p in v))

# The tool starts inverted. Its concave bowl opens downward; the shell is above
# the rim plane. This is an orientation of the real tool, not a new tool shape.
shell=[]
for z0,z1 in zip(np.linspace(0,D,9)[:-1],np.linspace(0,D,9)[1:]):
    r0=R*math.sqrt(max(0,1-(z0/D)**2))
    r1=R*math.sqrt(max(0,1-(z1/D)**2))
    for i in range(24):
        shell.append(wedge(max(.0001,r0),r0+.001,float(z0),float(z1),
            i*math.pi/12,(i+1)*math.pi/12,STEEL,HEAD,
            inner_top=max(.0001,r1),outer_top=max(.0011,r1+.001)))
shell.append(parts.cylinder(.003,.0015,tuple(HEAD+(0,0,D)),STEEL))
fixed=[parts.box((.165,.018,.014),(-.0175,-.018,.0145),(.40,.20,.58)),
       parts.box((.020,.018,.008),(-.090,-.018,.004),(.40,.20,.58)),
       parts.box((.055,.014,.012),(.0875,-.009,.0145),STEEL),*shell]
lever=[bar((.06,-.018,.0145),(-.10,.0244,.0145),.014,.014,STEEL)]
# A thin semicircular steel wiper lies just inside the bowl. It has its own
# physical body and a native mimic relation, so its load reacts on the handle.
wiper=[]
for a,b in zip(np.linspace(0,math.pi,19)[:-1],np.linspace(0,math.pi,19)[1:]):
    pa=HEAD+np.array((R*.94*math.cos(a),R*.94*math.sin(a),0.))
    pb=HEAD+np.array((R*.94*math.cos(b),R*.94*math.sin(b),0.))
    wiper.append(bar(pa,pb,.0025,.0025,STEEL))
DISHER=parts.Part("disher",pos=(.255,0.,TOP+.00075),yaw=math.pi,mass=.123,
    friction=(1.2,1.),shapes=fixed,joints=[
    parts.Hinge("lever",tuple(lever),(.06,-.018,.0145),(0,0,1),0.,LEVER_MAX,
        mass=.024,spring=3.,damping=.03,max_effort=3.),
    parts.Hinge("wiper",tuple(wiper),tuple(HEAD),(1,0,0),0.,math.pi,
        mass=.008,spring=0.,damping=.0005,max_effort=.5,gear=("lever",RATIO))])
BOWL_R,BOWL_H,FLOOR=.075,.060,.005
BOWL=parts.Part("bowl",pos=(.16,-.14,TOP),mass=.18,friction=(1.,.8),
    shapes=[parts.cylinder(BOWL_R+.002,.005,(0,0,.0025),(.24,.42,.56)),
        *ring(BOWL_R+.0015,.003,BOWL_H-.005,(BOWL_H+.005)/2,(.24,.42,.56))])
PLATE_R=.085
PLATE=parts.Part("plate",pos=(.16,-.35,TOP),mass=.16,friction=(1.,.8),
    shapes=[parts.cylinder(PLATE_R,.005,(0,0,.0025),(.85,.87,.85)),
        *ring(PLATE_R-.0015,.003,.012,.011,(.85,.87,.85))])
OBJECTS=[DISHER,BOWL,PLATE]
LOTS=lots.declare(OBJECTS,
    lots.Lot("A",place={"disher":DISHER.pos}),
    lots.Lot("B",place={"disher":(.205,0.,TOP+.00075),
        "bowl":(.14,-.140,TOP),"plate":(.14,-.350,TOP)}))
DEMO_TOUCHES=set()

CELL=.008
GRID=np.array([(x,y) for x in np.arange(-.064,.065,CELL)
    for y in np.arange(-.064,.065,CELL) if x*x+y*y<.064**2])
AREA=CELL*CELL
VOX=[]
for x in np.linspace(-R+R/10,R-R/10,10):
    for y in np.linspace(-R+R/10,R-R/10,10):
        rr=(x*x+y*y)/(R*R)
        if rr>=1:continue
        h=D*math.sqrt(1-rr)
        for k in range(8):VOX.append((x,y,(k+.5)*h/8))
VOX=np.array(VOX)
VW=np.array([2*R/10*2*R/10*D*math.sqrt(1-(x*x+y*y)/(R*R))/8 for x,y,z in VOX])
VW*=CAPACITY/VW.sum()
INITIAL=np.full(len(GRID),.035*AREA)

def rotation(state):return parts._quat_to_matrix(state.quat_xyzw)
def points(state,local):return np.asarray(local)@rotation(state).T+state.pos
def local_points(state,world):return (np.asarray(world)-state.pos)@rotation(state)

@dataclass
class Food:
    source:np.ndarray=field(default_factory=lambda:INITIAL.copy())
    cap:np.ndarray=field(default_factory=lambda:np.zeros(len(VOX)))
    deposits:list=field(default_factory=list)
    flying:list=field(default_factory=list)
    spill:float=0.
    spilled:list=field(default_factory=list)
    wiped:float=0.
    captured:float=0.
    invalid:bool=False
    last_q:float|None=None
    last_vel:np.ndarray|None=None
    last_tool:np.ndarray|None=None
    last_bowl:np.ndarray|None=None
    last_contact:np.ndarray=field(default_factory=lambda:np.zeros(3))
    last_t:float|None=None
    accel_exposure:list=field(default_factory=list)
    response_accel:float=0.
    physics_elapsed:float=0.
    peak_effective_accel:float=0.
    inertial_releases:list=field(default_factory=list)
    ticks:int=0
    applied_mu:float=0.
    wipe_work:float=0.
    injected_work:float=0.
    contact_force:float=0.
    wipe_torque:float=0.
    input_work:float=0.
    error:str|None=None

    def total(self):
        return self.source.sum()+self.cap.sum()+sum(p[1] for p in self.deposits)+sum(p[2] for p in self.flying)+self.spill

    def served(self):return sum(v for p,v in self.deposits)

    def advance(self,dt,bowl,tool,plate,q,qdot):
        if not 0<dt<=.100001 or not np.isfinite(np.r_[dt,q,qdot]).all():
            self.invalid=True;return np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3)
        self.physics_elapsed+=dt
        if not all(np.isfinite(np.r_[o.pos,o.quat_xyzw,o.lin_vel,o.ang_vel]).all() and np.linalg.norm(o.quat_xyzw)>1e-8 for o in (bowl,tool,plate)):
            self.invalid=True;self.error='unknown kinematic state';return (np.zeros(3),)*5
        rt,rb,rp=rotation(tool),rotation(bowl),rotation(plate)
        wp=points(tool,VOX+HEAD)
        bp=local_points(bowl,wp)
        # The source is a finite height field. Only real cavity/source overlap
        # can load a voxel, and each occupied voxel has a fixed conserved volume.
        distances=np.linalg.norm(bp[:,:2,None]-GRID.T[None,:,:],axis=1)
        ids=np.argmin(distances,axis=1)
        high=FLOOR+self.source[ids]/AREA
        in_food=(distances[np.arange(len(ids)),ids]<CELL*.71)&(bp[:,2]>=FLOOR)&(bp[:,2]<=high)
        contact=bool(in_food.any())
        cut_up=np.zeros(3);drag=np.zeros(3)
        if contact:
            # Yield/cutting resistance and velocity drag are applied to the
            # physical tool with an equal/opposite load on the loose source.
            fraction=min(1.,float(in_food.sum())/len(VOX))
            cut_up=rb[:,2]*(.8+2.2*fraction)
            velocity=np.asarray(tool.lin_vel)
            drag=-.8*np.tanh(velocity/.025);drag[2]=0.
            # Loading consumes work measured from the physical relative motion
            # against the applied material wrench. Static overlap earns none.
            displacement=(np.zeros(3) if self.last_tool is None else
                points(tool,HEAD)-self.last_tool-(np.asarray(bowl.pos)-self.last_bowl))
            work=max(0.,-float(self.last_contact@displacement))
            self.input_work+=work
            available=work/2000.  # estimated 2 kPa plastic work density
            for k in np.flatnonzero(in_food & (self.cap<VW*.999)):
                amount=min(VW[k]-self.cap[k],self.source[ids[k]],available)
                self.source[ids[k]]-=amount;self.cap[k]+=amount;self.captured+=amount
                available-=amount
        cap_volume=float(self.cap.sum())
        # Native joint friction supplies wiper resistance and its joint reaction.
        # No external resisting/assisting wiper or opposite base torque exists.
        previous_mu=float(self.applied_mu)
        if not np.isfinite(previous_mu) or not 0<=previous_mu<=.040001:
            self.invalid=True;self.error='unknown applied friction';return (np.zeros(3),)*5
        self.wipe_torque=(-math.copysign(previous_mu,qdot) if abs(qdot)>1e-9 else 0.)
        if self.last_q is not None and abs(q-self.last_q)>1e-6:
            alpha=np.arctan2(VOX[:,2]/D,VOX[:,1]/R)
            low,high=sorted((q,self.last_q))
            # Actual swept blade intersects occupied food sectors. Each volume
            # leaves from its own world point and falls; no plate teleport.
            hit=(alpha>=low-.035)&(alpha<=high+.035)&(self.cap>0)
            # Native Coulomb friction supplies resistance. This is a
            # conservative dissipation estimate, not a measured joint impulse.
            dissipated=previous_mu*abs(q-self.last_q)
            self.wipe_work+=dissipated
            available=dissipated/1200.
            for k in np.flatnonzero(hit):
                amount=min(float(self.cap[k]),available);self.cap[k]-=amount;self.wiped+=amount
                available-=amount
                if amount<=0:continue
                vel=np.asarray(tool.lin_vel)+np.cross(tool.ang_vel,wp[k]-tool.pos)
                self.flying.append([wp[k].copy(),vel.copy(),amount])
        # Tool-level finite-exposure adhesion proxy. The estimated50ms
        # response window is independent of physics dt. It deliberately changes
        # short-impulse response; contact and wiper work remain native-rate.
        # History stays continuous while empty: capture creates no blind period.
        if self.last_vel is not None:
            effective=np.array((0,0,-9.81))-(np.asarray(tool.lin_vel)-self.last_vel)/dt
            accel=float(np.linalg.norm(effective));self.peak_effective_accel=max(self.peak_effective_accel,accel)
            self.accel_exposure.append([float(dt),accel]);span=sum(x[0] for x in self.accel_exposure)
            while span>.05+1e-12:
                excess=span-.05
                if self.accel_exposure[0][0]<=excess+1e-12:span-=self.accel_exposure.pop(0)[0]
                else:self.accel_exposure[0][0]-=excess;span=.05
            self.response_accel=(sum(t*a for t,a in self.accel_exposure)/.05 if span>=.05-1e-9 else None)
        else:self.response_accel=None
        if self.cap.sum()>0:
            if self.response_accel is None:self.invalid=True;self.error='unknown50ms inertial history'
            elif self.response_accel>35.:
                self.inertial_releases.append(dict(physics_s=self.physics_elapsed,accel=accel,response_accel=self.response_accel,cap_ml=float(self.cap.sum()*1e6)))
                for k in np.flatnonzero(self.cap>0):
                    amount=float(self.cap[k]);self.cap[k]=0.
                    self.flying.append([wp[k].copy(),np.asarray(tool.lin_vel).copy(),amount])
        remaining=[]
        for position,velocity,amount in self.flying:
            for _ in range(10):
                h=dt/10;old=position.copy();velocity[2]-=9.81*h;position+=velocity*h
                # The inner shell still blocks displaced food until it reaches
                # the downward opening; local ellipsoid projection is a proxy.
                tp=local_points(tool,position)-HEAD
                norm=np.linalg.norm(tp/np.array((R,R,D)))
                if tp[2]>0 and norm>1 and np.linalg.norm(tp[:2])<R+.003:
                    previous=local_points(tool,old)-HEAD
                    if np.linalg.norm(previous/np.array((R,R,D)))<1.05:
                        tp/=max(1.,norm);position=points(tool,HEAD+tp*.995)
                        normal=rt@(tp/np.array((R*R,R*R,D*D)))
                        normal/=np.linalg.norm(normal)
                        relative=velocity-np.asarray(tool.lin_vel)
                        relative-=max(0.,float(relative@normal))*normal
                        velocity=np.asarray(tool.lin_vel)+relative*math.exp(-2*h)
                pp=local_points(plate,position)
                if np.linalg.norm(pp[:2])<PLATE_R-.006 and pp[2]<=.006 and local_points(plate,old)[2]>=.005:
                    self.deposits.append([np.array((pp[0],pp[1],.006)),amount]);amount=0.;break
                bb=local_points(bowl,position)
                if np.linalg.norm(bb[:2])<BOWL_R-.004:
                    j=int(np.argmin(np.linalg.norm(GRID-bb[:2],axis=1)))
                    surface=FLOOR+self.source[j]/AREA
                    if bb[2]<=surface and local_points(bowl,old)[2]>=surface-.002:
                        self.source[j]+=amount;amount=0.;break
                if position[2]<TOP+.003:
                    self.spill+=amount;self.spilled.append([position.copy(),amount]);amount=0.;break
            if amount:remaining.append([position,velocity,amount])
        self.flying=remaining
        self.last_q=q;self.last_vel=np.asarray(tool.lin_vel).copy()
        self.last_tool=points(tool,HEAD).copy();self.last_bowl=np.asarray(bowl.pos).copy()
        self.last_contact=(cut_up+drag).copy()
        self.contact_force=float(np.linalg.norm(cut_up+drag))
        if abs(self.total()-INITIAL.sum())>1e-10 or np.min(self.source)<-1e-12 or np.min(self.cap)<-1e-12:self.invalid=True
        tool_force=cut_up+drag+np.array((0,0,-self.cap.sum()*DENSITY*9.81))
        bowl_force=-cut_up-drag+np.array((0,0,-self.source.sum()*DENSITY*9.81))
        body_torque=np.cross(rt@HEAD,tool_force)
        wiper_torque=np.zeros(3)  # native joint friction carries its own reaction
        plate_force=np.array((0,0,-self.served()*DENSITY*9.81))
        return tool_force,body_torque,bowl_force,wiper_torque,plate_force

_food=Food()
_history=native_history.History()
_safety=native_history.Safety()
_state={}
_supports={}
_support_signals={}
_visuals=None

def signature(objects):
    geometry=tuple(float(x) for name in ("disher","bowl","plate")
        for x in np.r_[objects[name].pos,objects[name].quat_xyzw,
            objects[name].lin_vel,objects[name].ang_vel,
            *[objects[name].joints[k] for k in sorted(objects[name].joints)],
            *[np.r_[p,q] for _,(p,q) in sorted(objects[name].links.items())]])
    material=tuple(float(x) for x in np.r_[_food.source,_food.cap,_food.spill,
        *[np.r_[p,v] for p,v in _food.deposits],
        *[np.r_[p,v,a] for p,v,a in _food.flying],
        *[np.r_[p,v] for p,v in _food.spilled]])
    return geometry+material

def force(sim,name,body,value,torque=(0,0,0),reset=True):
    import torch
    base=sim.env.unwrapped;asset=base.scene[name];dev=base.device
    ids=[asset.body_names.index(body)] if body else None
    f=torch.tensor([[list(map(float,value))]],device=dev,dtype=torch.float32)
    t=torch.tensor([[list(map(float,torque))]],device=dev,dtype=torch.float32)
    env_ids=torch.tensor([0],device=dev)
    # The deprecated per-body setter resets every body's wrench in the target
    # environment. Compose both loads after one reset instead.
    if reset:asset.permanent_wrench_composer.reset(env_ids=env_ids)
    asset.permanent_wrench_composer.add_forces_and_torques(f,t,body_ids=ids,
        env_ids=env_ids,is_global=True)

def all_boxes(objects):
    return [b for spec in OBJECTS for b in release.boxes(spec,objects[spec.name])]+food_boxes(objects)

def food_boxes(objects):
    result=[]
    for xy,v in zip(GRID,_food.source):
        if v<=0:continue
        h=float(v/AREA);o=objects['bowl']
        result.append(('food',points(o,(*xy,FLOOR+h/2)),rotation(o),np.array((CELL,CELL,h))))
    for key,v in plate_cells().items():
        o=objects['plate'];h=float(v/AREA);xy=(np.asarray(key)+.5)*CELL
        result.append(('food',points(o,(*xy,.005+h/2)),rotation(o),np.array((CELL,CELL,h))))
    if _food.cap.sum()>0:
        o=objects['disher']
        result.append(('food',points(o,HEAD+(0,0,D/2)),rotation(o),np.array((2*R,2*R,D))))
    for pos,vel,v in _food.flying:
        if not np.isfinite(v) or v<=0:raise ValueError('unknown flying food volume')
        result.append(('food',np.asarray(pos),np.eye(3),np.full(3,max(float(v),1e-15)**(1/3))))
    for pos,v in _food.spilled:
        if not np.isfinite(v) or v<=0:raise ValueError('unknown spilled food volume')
        side=max(float(v),1e-15)**(1/3)
        result.append(('food',np.array((pos[0],pos[1],TOP+side/2)),np.eye(3),np.full(3,side)))
    for _,c,r,size in result:
        if not np.isfinite(np.r_[c,r.ravel(),size]).all() or np.any(size<=0):raise ValueError('unknown native food geometry')
    return result

def plate_cells():
    cells={}
    for p,v in _food.deposits:
        key=tuple(np.floor(p[:2]/CELL).astype(int));cells[key]=cells.get(key,0.)+v
    return cells

def portions():
    """Two separate deposited components, rather than one blob across a line."""
    cells=plate_cells()
    components=[]
    while cells:
        start=next(iter(cells));todo=[start];amount=0.
        while todo:
            key=todo.pop()
            if key not in cells:continue
            amount+=cells.pop(key)
            todo.extend((key[0]+a,key[1]+b) for a,b in itertools.product((-1,0,1),repeat=2) if a or b)
        components.append(amount)
    return sorted(components,reverse=True)

def progress():
    heaps=portions()
    if not heaps or heaps[0]<14e-6-EPS:return 0.
    return 1. if len(heaps)>=2 and heaps[1]>=14e-6-EPS and _food.served()>=34e-6-EPS else .5

def native_hand_hulls(sim):
    """Actual post-physics transforms of every convex hand/fingertip collision hull."""
    from yam_arena import analysis
    base=sim.env.unwrapped;origin=release.array(base.scene.env_origins)[0];hulls=[]
    for arm in ('left_robot','right_robot'):
        robot=base.scene[arm];poses=release.array(robot.root_view.get_link_transforms())[0];frames={}
        for name in ('gripper','tip_left','tip_right'):
            p=poses[robot.body_names.index(name)]
            if p.shape!=(7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1)>1e-4:raise ValueError('unknown native hand pose')
            mat=np.eye(4);mat[:3,:3]=parts._quat_to_matrix(p[3:]);mat[:3,3]=p[:3]-origin;frames[name]=mat
        posed=analysis.world_hulls(frames)
        if any(not any(n==name for n,h in posed) for name in frames):raise ValueError('missing native hand collision hull')
        hulls.extend(posed)
    return hulls

def other_links_clear(sim,bs):
    from yam_arena import analysis
    base=sim.env.unwrapped;origin=release.array(base.scene.env_origins)[0];hulls=[]
    for arm in ("left_robot","right_robot"):
        robot=base.scene[arm];pose=release.array(robot.data.body_link_pose_w)[0];frames={}
        for i,name in enumerate(robot.body_names):
            if name in ("gripper","tip_left","tip_right"):continue
            m=np.eye(4);m[:3,:3]=parts._quat_to_matrix(pose[i,3:]);m[:3,3]=pose[i,:3]-origin
            if not np.isfinite(m).all():return False
            frames[name]=m
        hulls.extend(analysis.world_hulls(frames))
    return release.clear_of_shapes(hulls,bs)

def supported(sim,name):
    # Geometry plus actual table-specific contact; unknown contact never passes.
    try:
        if name not in _supports:
            from isaaclab_physx.physics import PhysxManager
            base=sim.env.unwrapped
            # The standard station table is identified from the actual stage,
            # not a guessed API result. No contact with another prop suffices.
            from isaaclab.sim.utils import get_current_stage
            stage=get_current_stage()
            candidates=[str(p.GetPath()) for p in stage.Traverse()
                if 'table' in str(p.GetPath()).lower() and p.HasAPI(__import__('pxr',fromlist=['UsdPhysics']).UsdPhysics.CollisionAPI)]
            if not candidates:raise ValueError('missing table collider')
            path=parts.ROOT_PATHS[name]+('/base' if name=='disher' else '')
            _supports[name]=PhysxManager.get_physics_sim_view().create_rigid_contact_view([path],filter_patterns=[candidates])
        f=release.array(_supports[name].get_contact_force_matrix(dt=sim.env.unwrapped.sim.get_physics_dt()))
        if f.shape!=(1,1,3) or not np.isfinite(f).all():raise ValueError('unknown support')
        _support_signals[name]=f[0,0].tolist()
        return bool(f[0,0,2]>.02)
    except Exception as e:
        _support_signals[name]=type(e).__name__;return False

def visual(sim,objects):
    """Non-colliding aggregate food displays follow the conserved material state."""
    global _visuals
    from pxr import UsdGeom,Gf,Sdf
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage();origin=release.array(sim.env.unwrapped.scene.env_origins)[0]
    if _visuals is None:
        _visuals=[]
        count=len(GRID)+1+len(GRID)+160
        for i in range(count):
            cube=UsdGeom.Cube.Define(stage,f'/World/disher_food_proxy/f{i}');cube.CreateSizeAttr(1.)
            cube.CreateDisplayColorAttr([Gf.Vec3f(*FOOD)])
            ops=cube.GetOrderedXformOps()
            tr=next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate),None)
            sc=next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeScale),None)
            _visuals.append((tr if tr is not None else cube.AddTranslateOp(),
                sc if sc is not None else cube.AddScaleOp()))
    boxes=[];b=objects['bowl'];p=objects['plate'];t=objects['disher']
    for xy,v in zip(GRID,_food.source):
        h=max(.00001,float(v/AREA));boxes.append((points(b,(*xy,FLOOR+h/2)),(.0078,.0078,h)))
    v=float(_food.cap.sum());r=max(.0001,(3*v/(4*math.pi))**(1/3))
    boxes.append((points(t,HEAD+(0,0,.006)),(2*r,2*r,min(.014,2*r))))
    cells=list(plate_cells().items())
    for key,v in cells:
        xy=(np.asarray(key)+.5)*CELL
        h=max(.00001,float(v/AREA));boxes.append((points(p,(*xy,.005+h/2)),(.0078,.0078,h)))
    for _ in range(max(0,len(GRID)-len(cells))):boxes.append(((0,0,TOP-1),(.00001,)*3))
    for pos,vel,v in _food.flying[:80]:
        a=max(.0001,float(v)**(1/3));boxes.append((pos,(a,a,a)))
    for pos,v in _food.spilled[:80]:
        a=max(.0001,float(v)**(1/3));boxes.append(((pos[0],pos[1],TOP+a/2),(a,a,a)))
    while len(boxes)<len(_visuals):boxes.append(((0,0,TOP-1),(.00001,)*3))
    for (tr,sc),(pos,size) in zip(_visuals,boxes):
        tr.Set(Gf.Vec3d(*(np.asarray(pos)+origin)));sc.Set(Gf.Vec3f(*map(float,size)))

@parts.on_reset
def reset(sim):
    global _food,_history,_safety,_state,_supports,_support_signals,_visuals
    _food=Food();_history=native_history.History(expected={n:lots.current().pos(n) for n in ('bowl','plate')});_safety=native_history.Safety();_supports={};_support_signals={};_visuals=None
    _state=dict(known=False,invalid=False,held=0.,t=None,hands=None,corners=None,stamp=None,
        tier=0.)
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool('/physics/disableContactProcessing',False)
    for name in ('disher','bowl','plate'):
        sim.env.unwrapped.scene[name].permanent_wrench_composer.reset()
    material_runtime.reset(sim,globals())
    sim.set_target('disher','lever',-.33)
    # Setting a spring rest point outside a hard limit supplies the estimated
    # real return-spring preload. No joint pose is assigned during the trial.

def initial_visuals(emb):
    sim=parts.Sim(emb)
    visual(sim,sim.states())
    for _ in range(2):sim.env.unwrapped.sim.render()
    # Render alone leaves a same-timestamp sensor buffer cached. Refresh real
    # camera pixels after adding food displays, before the first observation.
    for name in ('top_cam','left_cam','right_cam'):
        sim.env.unwrapped.scene[name].update(0.,force_recompute=True)

lots.after_layout=initial_visuals

@parts.on_step
def step(sim):
    s=_state;objects=sim.states()
    try:
        tool,bowl,plate=(objects[n] for n in ('disher','bowl','plate'))
        for name,o in (('bowl',bowl),('plate',plate)):
            if (rotation(o)[2,2]<math.cos(math.radians(10))-native_history.ANGLE_TOL
                or np.linalg.norm(o.pos-np.asarray(lots.current().pos(name)))>.030+native_history.POSITION_TOL):
                s['invalid']=True
        asset=sim.env.unwrapped.scene['disher'];j=asset.joint_names.index('wiper')
        q=float(tool.joints['wiper']);qdot=float(release.array(asset.data.joint_vel)[0,j])
        dt=.05 if s['t'] is None else float(sim.t-s['t'])
        if not 0<dt<=.100001:raise ValueError('invalid interval')
        base=sim.env.unwrapped
        if _food.ticks<=s.get('physics_ticks',0):raise ValueError('missing native material updates')
        if _history.ticks!=_food.ticks or _history.ticks<=s.get('history_ticks',0):raise ValueError('missing native history coverage')
        if _history.invalid:s['invalid']=True
        s['history_ticks']=_history.ticks
        s['physics_ticks']=_food.ticks
        bs=all_boxes(objects);clouds,hands=release.hand_geometry(sim)
        corners=np.array([c+np.asarray(sgn)*size/2@r.T for _,c,r,size in bs
            for sgn in itertools.product((-1.,1.),repeat=3)])
        if not np.isfinite(np.r_[hands.ravel(),corners.ravel(),signature(objects)]).all():raise ValueError('unknown geometry')
        clear=release.sampled_clear(clouds,bs)
        if clear:clear=other_links_clear(sim,bs)
        still=(s['hands'] is not None and s['corners'] is not None
            and corners.shape==s['corners'].shape
            and np.max(np.linalg.norm(hands-s['hands'],axis=1))/dt<.01
            and np.max(np.linalg.norm(corners-s['corners'],axis=1))/dt<.01
            and all(o.resting(max_speed=.01,max_spin=.1) for o in (tool,bowl,plate)))
        settled=all(abs(o.low[2]-TOP)<.004 and rotation(o)[2,2]>.995 for o in (tool,bowl,plate))
        tier=progress()
        ready=(not _food.invalid and tier>0 and tier==s.get('tier') and _food.spill<=3e-6+EPS
            and not _food.flying and clear and still and settled)
        if ready:ready=all(supported(sim,n) for n in ('disher','bowl','plate'))
        s['held']=s['held']+dt if ready else 0.
        s.update(known=True,t=float(sim.t),hands=hands.copy(),corners=corners.copy(),stamp=signature(objects),tier=tier)
        visual(sim,objects)
    except Exception as e:
        s['known']=False;s['invalid']=True;_food.error=type(e).__name__+':'+str(e)[:120]
    path=os.environ.get('TRACE_OUT')
    if path and round(sim.t*20)%5==0:
        row={k:v for k,v in s.items() if k not in ('hands','corners','stamp')}
        row.update(native_history=dict(ticks=_history.ticks,invalid=_history.invalid,error=_history.error,hand_food_contact=_history.hand_food_contact,max_excursion_m=_history.max_excursion,min_vertical=_history.min_vertical),safety=dict(ticks=_safety.ticks,error=_safety.error,overload=_safety.overload,peak_projected=_safety.peak_projected,peak_contact=_safety.peak_contact,peak_event=_safety.peak_event,drive_caps=_safety.drive_caps),t=float(sim.t),lot=lots.current().name,source_ml=float(_food.source.sum()*1e6),
            cap_ml=float(_food.cap.sum()*1e6),served_ml=float(_food.served()*1e6),spill_ml=_food.spill*1e6,
            captured_ml=_food.captured*1e6,wiped_ml=_food.wiped*1e6,food_invalid=_food.invalid,
            error=_food.error,contact_force=_food.contact_force,wipe_torque=_food.wipe_torque,
            response_accel=_food.response_accel,peak_effective_accel=_food.peak_effective_accel,inertial_release_count=len(_food.inertial_releases),last_inertial_release=(_food.inertial_releases[-1] if _food.inertial_releases else None),
            native_applied_mu=_food.applied_mu,friction_dissipation_estimate=_food.wipe_work,
            input_work=_food.input_work,physics_ticks=_food.ticks,wipe_work=_food.wipe_work,injected_work=_food.injected_work,
            joints=objects['disher'].joints,support=_support_signals)
        row['poses']={n:np.r_[o.pos,o.quat_xyzw].tolist() for n,o in objects.items()}
        robot=sim.env.unwrapped.scene['right_robot']
        row['right_gripper_pose']=release.array(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')].tolist()
        row['right_finger_q']=release.array(robot.data.joint_pos)[0,-2:].tolist()
        with pathlib.Path(path).with_name('rule.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')

def score(objects):
    try:return progress() if (_state['known'] and not _state['invalid'] and not _food.invalid and not _history.invalid and _history.ticks==_food.ticks
        and _food.spill<=3e-6+EPS and _state['held']>=2.-EPS
        and _state['stamp']==signature(objects)) else 0.
    except Exception:return 0.

def success(objects):return score(objects)==1.

def demo():
    """Frozen waypoints selected from the visibly displaced starting tool."""
    layout='B' if lots.current().pos('disher')[0]<.23 else 'A'
    data=json.loads(pathlib.Path(__file__).with_name('demo-path.json').read_text())[layout]
    return {arm:[Waypoint(w['duration'],tuple(w['pos']),w['finger'],w['label'],tilt._TiltedYaw(0.,np.array(w['rotation']))) for w in rows] for arm,rows in data.items()}
