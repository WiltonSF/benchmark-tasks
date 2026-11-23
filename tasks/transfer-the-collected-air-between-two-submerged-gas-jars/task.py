"""Two full-size, open glass gas jars in a clear water trough on a laboratory bench.
The amber-banded donor contains 80 mL of ordinary air; the blue-banded receiver
is completely flooded. One porcelain shelf supports the receiver; the donor rests on the trough floor. The
waterline, jar mouths, calibration bands and rising bubbles are visible.
"""
import json
import math
import os
import pathlib
import sys
from functools import lru_cache
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts
import lots
import tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import FINGER_OPEN
SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = .1
# Version 1. Geometry in metres, gas inventory in ambient-pressure mL.
RI, RO, HALF, INNER_HALF = .022, .025, .075, .072
MOUTH_OUTER = .0295
WATER = TOP + .089  # 85mm above the 4mm trough floor
TROUGH_POS = (.30, 0., TOP)
DONOR_POS = (.350, .060, TOP+.079)
RECEIVER_POS = (.270, -.060, TOP+.099)
AMBER, BLUE, GLASS = (.92,.48,.08), (.08,.32,.90), (.65,.82,.88)
WHITE, GREY = (.94,.94,.91), (.48,.52,.55)


def ring(radius, rib, height, z, color, n=64):
    return [parts.cylinder(rib,height,(float(radius*math.cos(a)),float(radius*math.sin(a)),z),color)
            for a in np.linspace(0,2*math.pi,n,endpoint=False)]


def jar(name,pos,color):
    shapes = ring(.0235,.0015,.144,0.,GLASS)
    shapes += [parts.cylinder(.0295,.005,(0.,0.,.0725),GLASS)]
    shapes += ring(.02575,.00375,.004,-.073,GLASS)
    # Calibration marks are markings, not added grip features.
    shapes += [parts.box((.015,.0003,.0012),(0.,-.02505,.072-v*1e-6/(math.pi*RI*RI)),color)
               for v in (20,40,60,72,80,100)]
    shapes += [parts.box((.014,.0003,.008),(0.,-.02505,.061),color),parts.cylinder(.012,.0002,(0.,0.,.0751),color)]
    return parts.Part(name,pos,mass=.150,friction=(.4,.3),shapes=shapes)

DONOR = jar('donor_amber',DONOR_POS,AMBER)
RECEIVER = jar('receiver_blue',RECEIVER_POS,BLUE)
# 400x300x100mm acrylic trough. Included drawing gives actual wall dimensions.
TROUGH = parts.Part('water_trough',TROUGH_POS,mass=10.0,friction=(.8,.7),shapes=[
    parts.box((.400,.300,.004),(0.,0.,.002),GLASS),
    parts.box((.004,.300,.096),(-.198,0.,.052),GLASS),
    parts.box((.004,.300,.096),(.198,0.,.052),GLASS),
    parts.box((.392,.004,.096),(0.,-.148,.052),GLASS),
    parts.box((.392,.004,.096),(0.,.148,.052),GLASS)])
# Porcelain beehive shelves are open underneath and retain their native 70mm diameter.
# Wall ribs and a three-sector top leave a 12mm gas inlet. No artificial guide or catch.
def shelf(name,p):
    return parts.Part(name,(p[0],p[1],TOP+.004),mass=.250,friction=(.5,.4),shapes=[
        *ring(.0315,.0035,.017,.0085,WHITE,40),
        *ring(.0205,.0145,.003,.0185,WHITE,32)])
SHELVES = [shelf('receiver_shelf',RECEIVER_POS)]
OBJECTS = [DONOR,RECEIVER,TROUGH,*SHELVES]
LOTS = lots.declare(OBJECTS,
    lots.Lot('A',place={DONOR.name:DONOR.pos}),
    lots.Lot('B',place={DONOR.name:(.350,.045,TOP+.079),RECEIVER.name:(.320,-.060,TOP+.099),
        'receiver_shelf':(.320,-.060,TOP+.004)}))
DEMO_TOUCHES=set()

# Equal-volume deterministic cubature, not a fluid solver. Ring quadrature integrates
# the actual 44mm bore and 144mm internal height. Capacity has no dwell/pose reward.
_nr,_na,_nz = 6,24,48
rr=RI*np.sqrt((np.arange(_nr)+.5)/_nr)
aa=(np.arange(_na)+.5)*2*math.pi/_na
_disc=np.array([(r*math.cos(a),r*math.sin(a)) for r in rr for a in aa])
GAS_POINTS=np.array([(x,y,z) for z in np.linspace(-INNER_HALF,INNER_HALF,_nz,endpoint=False)+INNER_HALF/_nz for x,y in _disc])
CELL_ML=math.pi*RI**2*(2*INNER_HALF)*1e6/len(GAS_POINTS)
# Smooth finite glass displacement is separate from the gas cavity buoyancy.
GLASS_POINTS=np.array([(r*math.cos(a),r*math.sin(a),z) for z in np.linspace(-.072,.072,12) for a in np.linspace(0,2*math.pi,32,endpoint=False) for r in (.0235,)])
GLASS_M3=.150/2230. # estimated borosilicate density kg/m3


def finite(s):
    try:
        return bool(np.isfinite(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]).all() and abs(np.linalg.norm(s.quat_xyzw)-1)<.01)
    except (AttributeError,ValueError,TypeError):
        return False


@lru_cache(maxsize=64)
def cached_geometry(key):
    pos=np.asarray(key[:3]);r=parts._quat_to_matrix(key[3:]);u=r[:,2]
    mouth=pos-HALF*u
    radial=(np.array((0.,0.,1.))-u*u[2])/max(math.sqrt(max(0.,1-u[2]**2)),1e-12)
    cap_center=pos+(-.073+.002*np.sign(u[2]))*u
    high=float(cap_center[2]+MOUTH_OUTER*math.sqrt(max(0.,1-u[2]**2)))
    lip=cap_center+RI*radial
    world=GAS_POINTS@r.T+pos
    order=np.argsort(world[:,2])[::-1];ordered=world[order];cumulative=np.cumsum(ordered,axis=0)
    return (r,u,mouth,high,lip,world),ordered,cumulative


def geometry(s):return cached_geometry(tuple(np.r_[s.pos,s.quat_xyzw]))[0]


def pose_between(a,b,f):
    qa=np.asarray(a.quat_xyzw);qb=np.asarray(b.quat_xyzw)
    if qa@qb<0:qb=-qb
    q=(1-f)*qa+f*qb;q/=np.linalg.norm(q)
    return pose_state(np.r_[(1-f)*a.pos+f*b.pos,q])


def mouth_frame(s):
    r=parts._quat_to_matrix(s.quat_xyzw);u=r[:,2];mouth=s.pos-HALF*u
    high=float(s.pos[2]-.073*u[2]+.002*abs(u[2])+MOUTH_OUTER*math.sqrt(max(0.,1-u[2]**2)))
    return r,u,mouth,high


def capacity(s):
    r,u,mouth,high,lip,world=geometry(s)
    return float(np.count_nonzero(world[:,2]>lip[2]+1e-9)*CELL_ML) if u[2]>0 else 0.


def gas_shape(s,ambient_ml,water=WATER):
    g,ordered,cumulative=cached_geometry(tuple(np.r_[s.pos,s.quat_xyzw]))
    actual=float(ambient_ml);interface=float(ordered[0,2])
    for _ in range(3):
        n=max(1,min(len(ordered),int(math.ceil(actual/CELL_ML))))
        interface=float(ordered[n-1,2])
        pressure=101325.+1000.*9.81*(water-interface)
        actual=ambient_ml*101325./max(95000.,pressure)
    if ambient_ml<=1e-9:return 0.,np.asarray(s.pos),interface
    return actual,cumulative[n-1]/n,interface


def pose_state(p):
    return SimpleNamespace(pos=np.asarray(p[:3]),quat_xyzw=np.asarray(p[3:]),lin_vel=np.zeros(3),ang_vel=np.zeros(3))


@lru_cache(maxsize=None)
def collider_arrays(name):
    spec=next(o for o in OBJECTS if o.name==name)
    physical=spec.shapes[:129] if spec.name in (DONOR.name,RECEIVER.name) else spec.shapes
    cylinders=[x for x in physical if x.kind=='cylinder']
    boxes=[x for x in physical if x.kind=='box']
    return (np.array([x.at for x in cylinders]).reshape(-1,3),np.array([x.size[0] for x in cylinders]),np.array([x.size[1]/2 for x in cylinders]),
            np.array([x.at for x in boxes]).reshape(-1,3),np.array([np.asarray(x.size)/2 for x in boxes]).reshape(-1,3))


def solid_blocked(points,objects,margin=.00075,ignore=()):
    if objects is None:return False
    for spec in OBJECTS:
        if spec.name in ignore:continue
        st=objects.get(spec.name)
        if st is None:raise ValueError('unknown solid geometry')
        local=(points-st.pos)@parts._quat_to_matrix(st.quat_xyzw)
        cc,rad,half,bc,bh=collider_arrays(spec.name)
        if len(cc):
            d=local[:,None,:]-cc
            gap=np.maximum(np.c_[np.linalg.norm(d[:,:,:2],axis=2).reshape(-1)-np.tile(rad,len(points)),
                                 (np.abs(d[:,:,2])-half).reshape(-1)],0.)
            if np.any(np.linalg.norm(gap,axis=1)<=margin):return True
        if len(bc):
            gap=np.maximum(np.abs(local[:,None,:]-bc)-bh,0.)
            if np.any(np.linalg.norm(gap,axis=2)<=margin):return True
    return False


@lru_cache(maxsize=1)
def robot_hulls():
    from yam_arena import analysis
    return analysis.link_hulls()


def robot_blocked(points,robot_poses,margin=.001):
    if robot_poses is None:return False
    for name,p in robot_poses:
        local=(points-p[:3])@parts._quat_to_matrix(p[3:])
        for h in robot_hulls().get(name,[]):
            if np.any((local@h.planes[:,:3].T-h.planes[:,3]).max(1)<=margin):return True
    return False


def blocked_segment(p,q,objects=None,hand=None,robot_poses=None,ignore=()):
    # Upward flight <=0.75mm at240Hz, plus both endpoints. A finite bubble can
    # be lost at a blocker; the conservative model does not move it through a solid.
    points=np.linspace(p,q,max(3,int(math.ceil(np.linalg.norm(q-p)/.00075))+1))
    if hand is not None:
        if not len(hand) or not np.isfinite(hand).all():raise ValueError('unknown air-path hand witness')
        if np.min(np.linalg.norm(points[:,None,:]-hand[None,:,:],axis=2))<=.0025:return True
    return solid_blocked(points,objects,ignore=ignore) or robot_blocked(points,robot_poses)


def stamp(objects):
    return tuple(np.concatenate([np.r_[objects[n].pos,objects[n].quat_xyzw,objects[n].lin_vel,objects[n].ang_vel] for n in (DONOR.name,RECEIVER.name,TROUGH.name,*[s.name for s in SHELVES])]))


def mouth_in_trough(s,trough):
    """The water occupies the actual trough, not an infinite horizontal plane."""
    tr=parts._quat_to_matrix(trough.quat_xyzw).T
    u=tr@parts._quat_to_matrix(s.quat_xyzw)[:,2]
    c=tr@(s.pos-trough.pos)-.073*u
    extent=.002*np.abs(u)+MOUTH_OUTER*np.sqrt(np.maximum(0.,1-u*u))
    lo,hi=c-extent,c+extent
    # Floor tolerance matches the declared1.5mm rigid contact witness.
    return bool(lo[0]>=-.196 and hi[0]<=.196 and lo[1]>=-.146 and hi[1]<=.146 and lo[2]>=.004-.0015)


class AirTrial:
    def __init__(self):
        self.ml=np.array((80.,0.));self.bubbles=[];self.lost=0.;self.zero='';self.invalid='not observed'
        self.force=np.zeros((2,3));self.torque=np.zeros((2,3));self.reaction=np.zeros(3);self.reaction_torque=np.zeros(3)
        self.force_ready=False;self.t=0.;self.since=None;self.hold=[];self.clear=False;self.ready=False;self.state_stamp=None
        self.captured=0.;self.emitted=0.;self.peak_force=0.;self.loaded_steps=0;self.peak_torque=0.;self.min_depth=math.inf
        self.last_states=None;self.surface=WATER;self.max_trough_shift=0.;self.max_trough_tilt=0.;self.applied_count=0;self.vented=[False,False];self.physics_checks=0;self.blocked_ml=0.;self.source_blocked_attempt_ml=0.;self.physics_path=[];self.active=False;self.wrench_world={}

    def vent(self,i):
        if not self.vented[i]:
            self.lost+=float(self.ml[i]);self.ml[i]=0.;self.vented[i]=True
        self.zero='mouth crossed waterline'

    def advance(self,states,dt,loaded=True,objects=None,hand=None,robot_poses=None):
        if not all(finite(s) for s in states):raise ValueError('missing finite jar state')
        for i,s in enumerate(states):
            trough=objects[TROUGH.name] if objects is not None else TROUGH.state()
            if not mouth_in_trough(s,trough):
                self.vent(i);self.zero='mouth left trough water'
            r,u,mouth,high,lip,world=geometry(s)
            self.min_depth=min(self.min_depth,self.surface-high)
            if high>=self.surface-1e-9:
                # Conservative whole-mouth rule: any part of the OUTER rim at the
                # surface vents all its inventory immediately, and permanently zeros.
                self.vent(i)
        if self.zero:return
        if not loaded:
            self.last_states=states
            return
        self.loaded_steps+=1
        old=self.last_states or states
        # Native calls use actual240Hz pose samples; standalone controls use10ms. 
        count=max(1,int(math.ceil(dt/.01)))
        for sub in range(count):
            ds=dt/count
            for i,s in enumerate(states):
                cap=capacity(s)
                actual,centroid,interface=gas_shape(s,self.ml[i],self.surface)
                excess=max(0.,actual-cap)
                if excess<=1e-8:continue
                # Estimated open-mouth discharge: rate <=20mL/s, smoothly tends to0.
                amount=min(self.ml[i],excess,max(0.,20.*math.sqrt(excess/80.)*ds))
                if amount<=0:continue
                _,_,_,_,lip,_=geometry(s)
                r,u,mouth,high,lip,world=geometry(s)
                radial=(np.array((0.,0.,1.))-u*u[2])/max(math.sqrt(max(0.,1-u[2]**2)),1e-12)
                outlet=np.asarray(s.pos)+(-.073+.002*np.sign(u[2]))*u+(MOUTH_OUTER+.0015)*radial
                inside=np.asarray(s.pos)-.069*u+(RI-.002)*radial
                exit=np.asarray(s.pos)-.078*u+(RI-.002)*radial
                around=np.asarray(s.pos)-.078*u+(MOUTH_OUTER+.0015)*radial
                outlet=outlet+np.array((0.,0.,.001))
                source_name=(DONOR.name,RECEIVER.name)[i]
                route=(inside,exit,around,outlet)
                # The source route actually crosses the native open disk before
                # rounding its flange. It cannot spawn air on the far side of a
                # finger sealing the mouth. Own glass is excluded only along
                # this analytically open-aperture route, not during free flight.
                if any(blocked_segment(a,b,objects,hand,robot_poses,ignore=(source_name,)) for a,b in zip(route,route[1:])):
                    self.source_blocked_attempt_ml+=amount
                    continue
                self.ml[i]-=amount;self.emitted+=amount
                self.bubbles.append([outlet,amount,i])
            remaining=[]
            for p,v,source in self.bubbles:
                q=p+np.array((0.,0.,.18*ds)) # stated terminal-rise estimate
                dest=1-source
                begin=pose_between(old[dest],states[dest],sub/count)
                end=pose_between(old[dest],states[dest],(sub+1)/count)
                r0,u0,m0,hi0=mouth_frame(begin);r1,u1,m1,hi1=mouth_frame(end)
                f0=float((p-m0)@u0);f1=float((q-m1)@u1)
                hit=False;at=q
                if f0<=1e-9 and f1>=-1e-9 and min(u0[2],u1[2])>.25:
                    low,upper=0.,1.
                    for _ in range(12):
                        fraction=(low+upper)/2;st=pose_between(begin,end,fraction)
                        rr,uu,mm,hh=mouth_frame(st);at=p+fraction*(q-p)
                        if (at-mm)@uu>=0:upper=fraction
                        else:low=fraction
                    fraction=(low+upper)/2;st=pose_between(begin,end,fraction)
                    r,u,mouth,high=mouth_frame(st);at=p+fraction*(q-p)
                    local=r.T@(at-mouth)
                    throat_point=at+(.004/max(u[2],1e-12))*np.array((0.,0.,1.))
                    throat_local=r.T@(throat_point-mouth-.004*u)
                    hit=bool(np.linalg.norm(local[:2])<=RI-.001 and np.linalg.norm(throat_local[:2])<=RI-.001 and max(hi0,hi1,high)<self.surface)
                # Capture requires the entire currently modeled4mm throat to be
                # clear, not merely the observed crossing of its outer plane.
                endpoint=throat_point if hit else q
                if blocked_segment(p,endpoint,objects,hand,robot_poses):
                    self.lost+=v;self.blocked_ml+=v
                elif hit:
                    self.ml[dest]+=v
                    if source==0:self.captured+=v
                elif q[2]>=self.surface:self.lost+=v
                else:remaining.append([q,v,source])
            self.bubbles=remaining
        if abs(float(self.ml.sum())+self.lost+sum(v for p,v,i in self.bubbles)-80.)>1e-6:raise ValueError('air inventory mismatch')
        self.last_states=states

    def loads(self,states,trough,dt):
        forces=[];moments=[];self.reaction[:]=0;self.reaction_torque[:]=0
        for i,s in enumerate(states):
            r,u,mouth,high,lip,world=geometry(s)
            actual,centroid,interface=gas_shape(s,self.ml[i],self.surface)
            # Integrate inner-minus-outer hydrostatic pressure over the actual
            # closed roof and sidewall. This also loads raised water in the
            # flooded receiver above the trough surface, rather than treating it
            # as weightless. Open mouth has no cap force.
            z=np.linspace(-INNER_HALF,INNER_HALF,24,endpoint=False)+INNER_HALF/24
            a=np.linspace(0,2*math.pi,48,endpoint=False)
            local=np.array([(RI*math.cos(k),RI*math.sin(k),zz) for zz in z for k in a])
            normals=np.array([(math.cos(k),math.sin(k),0.) for zz in z for k in a])
            area_side=2*math.pi*RI*(2*INNER_HALF)/len(local)
            local=np.vstack((local,np.c_[_disc,np.full(len(_disc),INNER_HALF)]))
            normals=np.vstack((normals,np.tile((0.,0.,1.),(len(_disc),1))))
            areas=np.r_[np.full(24*48,area_side),np.full(len(_disc),math.pi*RI**2/len(_disc))]
            positions=local@r.T+s.pos;directions=normals@r.T
            # Entirely flooded jar has interface at its closed roof: the
            # internal pressure follows the connected water head everywhere.
            if self.ml[i]<=1e-8:interface=float(positions[:,2].max()+1.)
            pg=1000.*9.81*(self.surface-interface)
            pin=pg+1000.*9.81*np.maximum(interface-positions[:,2],0.)
            pout=1000.*9.81*np.maximum(self.surface-positions[:,2],0.)
            vectors=(pin-pout)[:,None]*directions*areas[:,None]
            hydro=vectors.sum(0)
            moment=np.cross(positions-s.pos,vectors).sum(0)
            glassworld=GLASS_POINTS@r.T+s.pos
            fraction=np.clip((self.surface-glassworld[:,2])/.003+.5,0,1).mean()
            glassbuoy=np.array((0.,0.,1000.*9.81*GLASS_M3*fraction))
            rel=np.asarray(s.lin_vel)-np.asarray(trough.lin_vel)
            area=.050*.150*fraction
            drag=-.5*1000.*1.1*area*np.linalg.norm(rel)*rel
            target=hydro+glassbuoy+drag
            target*=min(1.,3.5/max(np.linalg.norm(target),1e-12))
            torque=moment-.004*fraction*np.asarray(s.ang_vel)
            torque*=min(1.,.08/max(np.linalg.norm(torque),1e-12))
            alpha=1-math.exp(-dt/.10)
            self.force[i]+=alpha*(target-self.force[i]);self.torque[i]+=alpha*(torque-self.torque[i])
            self.reaction-=self.force[i]
            self.reaction_torque-=self.torque[i]+np.cross(s.pos-trough.pos,self.force[i])
        self.peak_force=max(self.peak_force,float(np.linalg.norm(self.force,axis=1).max()))
        self.peak_torque=max(self.peak_torque,float(np.linalg.norm(self.torque,axis=1).max()))
        return self.force,self.torque

T=AirTrial()
_MESH={}


def hand_mesh():
    if _MESH:return _MESH
    from yam_arena import analysis
    for name in ('gripper','tip_left','tip_right'):
        pts=[]
        for h in analysis.link_hulls().get(name,[]):
            pts.extend(h.vertices)
            for pl in np.unique(h.planes.round(7),axis=0):
                face=h.vertices[np.abs(h.vertices@pl[:3]-pl[3])<2e-6]
                if len(face)<3:raise ValueError('unknown hand hull')
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(pl[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for p,q in zip(face,np.roll(face,-1,axis=0)):
                    n=max(1,math.ceil(max(np.linalg.norm(p-c),np.linalg.norm(q-c),np.linalg.norm(p-q))/.003))
                    for j in range(n+1):
                        for k in range(n-j+1):pts.append(c+(p-c)*(j/n)+(q-c)*(k/n))
        arr=np.asarray(pts);_,ids=np.unique(np.floor(arr/.002).astype(int),axis=0,return_index=True)
        _MESH[name]=arr[ids]
    return _MESH


def clouds(sim):
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);out=[]
    for arm in ('left','right'):
        a=base.scene[arm+'_robot'];poses=parts._np(a.data.body_link_pose_w)[0]
        for name,p in hand_mesh().items():
            pose=poses[a.body_names.index(name)]
            out.append(p@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin)
    if len(out)!=6 or not all(len(p) and np.isfinite(p).all() for p in out):raise ValueError('unknown hands')
    return np.vstack(out)


def hand_clear(cloud,states):
    if cloud is None or not len(cloud) or not np.isfinite(cloud).all():return False
    for spec,s in zip((DONOR,RECEIVER),states):
        r=parts._quat_to_matrix(s.quat_xyzw);local=(cloud-s.pos)@r
        # Exact finite-cylinder side and flange/end distances, rather than an AABB.
        radial=np.linalg.norm(local[:,:2],axis=1)
        for radius,z0,z1 in ((RO,-.073,.070),(.0295,.070,.075),(MOUTH_OUTER,-.075,-.071)):
            d=np.c_[np.maximum(radial-radius,0),np.maximum(np.maximum(z0-local[:,2],local[:,2]-z1),0)]
            # 5mm true clearance +3mm face-grid and sqrt(3)*2mm thinning bounds.
            if np.linalg.norm(d,axis=1).min()<=.012:return False
    return True


def physics_clear(robot_poses,states):
    """Full gripper/finger faces from each actual physics observation."""
    if robot_poses is None:return False
    out=[];counts={name:0 for name in hand_mesh()}
    for name,p in robot_poses:
        if name not in counts:continue
        if not np.isfinite(p).all():return False
        counts[name]+=1
        out.append(hand_mesh()[name]@parts._quat_to_matrix(p[3:]).T+p[:3])
    if any(n!=2 for n in counts.values()):return False
    return hand_clear(np.vstack(out),states)


def hold_sample(t,objects,clear):
    states=[objects[DONOR.name],objects[RECEIVER.name]]
    held=[*states,objects[TROUGH.name],*[objects[x.name] for x in SHELVES]]
    stable=bool(clear and not T.zero and all(finite(s) for s in held) and park_ok(states,objects))
    if not stable:T.since=None;T.hold=[];T.ready=False;return
    p=np.array([s.pos for s in held]);q=np.array([s.quat_xyzw for s in held])
    T.hold=[row for row in T.hold if row[0]>=t-1.05]
    if T.since is None:T.since=t
    for _,oldp,oldq in T.hold:
        if np.max(np.linalg.norm(oldp-p,axis=1))>.001 or np.min(np.abs((oldq*q).sum(1)))<math.cos(math.radians(1)):
            T.since=t;T.hold=[];break
    T.hold.append((t,p,q))
    T.ready=bool(t-T.since>=1.-1e-9)


def jar_bounds_in_trough(s,trough):
    r=parts._quat_to_matrix(trough.quat_xyzw).T@parts._quat_to_matrix(s.quat_xyzw)
    pos=parts._quat_to_matrix(trough.quat_xyzw).T@(s.pos-trough.pos)
    cc,rad,half,bc,bh=collider_arrays(DONOR.name)
    centers=cc@r.T+pos;u=r[:,2]
    extent=half[:,None]*np.abs(u)+rad[:,None]*np.sqrt(np.maximum(0.,1-u*u))
    return (centers-extent).min(0),(centers+extent).max(0)


def park_ok(states,objects):
    donor,receiver=states;trough=objects[TROUGH.name]
    # The emptied donor rests on the native trough floor. A specimen need not
    # be put back upright to demonstrate conserved air recovery in the receiver.
    lo,hi=jar_bounds_in_trough(donor,trough)
    if abs(lo[2]-.004)>.0015 or lo[0]<-.1955 or hi[0]>.1955 or lo[1]<-.1455 or hi[1]>.1455:return False
    support=objects.get('receiver_shelf')
    if support is None or not finite(support):return False
    planned=np.asarray(lots.current().pos('receiver_shelf'))
    if np.linalg.norm(support.pos-planned)>.003 or parts._quat_to_matrix(support.quat_xyzw)[2,2]<math.cos(math.radians(2)):return False
    _,u,mouth,high,lip,world=geometry(receiver)
    if u[2]<math.cos(math.radians(5)) or abs(mouth[2]-(support.pos[2]+.020))>.0015:return False
    if np.linalg.norm(mouth[:2]-support.pos[:2])>.010:return False
    if any(geometry(s)[3]>WATER-.010+1e-6 for s in states):return False
    return np.linalg.norm(donor.pos-receiver.pos)>.065


def self_abs(x):return abs(float(x))


def tier(ml):
    lower=float(ml)-2. # calibration uncertainty, conservative lower bound
    return next((score for threshold,score in ((72.,1.),(60.,.75),(40.,.5),(20.,.25)) if lower>=threshold-1e-6),0.)


def observe(objects,hand,t):
    global T
    states=[objects[DONOR.name],objects[RECEIVER.name]];trough=objects[TROUGH.name]
    if not all(finite(s) for s in [*states,trough]):raise ValueError('unknown object state')
    dt=float(t)-T.t
    if dt<=0 or dt>.101:raise ValueError('nonmonotonic or unobserved trial time')
    # Trough displacement/tilt is a reproducible spill/rig disturbance witness.
    shift=float(np.linalg.norm(trough.pos-np.array(TROUGH_POS)));up=parts._quat_to_matrix(trough.quat_xyzw)[2,2]
    T.max_trough_shift=max(T.max_trough_shift,shift);T.max_trough_tilt=max(T.max_trough_tilt,math.degrees(math.acos(np.clip(up,-1,1))))
    if shift>.005 or up<math.cos(math.radians(2)):T.zero='trough moved or tipped'
    T.surface=WATER+float(trough.pos[2]-TOP)
    if T.physics_path:
        path=T.physics_path;T.physics_path=[];observed_t=T.t
        if abs(sum(row[0] for row in path)-dt)>1e-5:raise ValueError('unobserved flight interval')
        for h,poses,robots in path:
            snapshot={name:pose_state(p) for name,p in poses.items()}
            T.surface=WATER+float(snapshot[TROUGH.name].pos[2]-TOP)
            sample_states=[snapshot[DONOR.name],snapshot[RECEIVER.name]]
            T.advance(sample_states,h,T.force_ready,snapshot,None,robots)
            observed_t+=h
            clear=bool(not T.zero and park_ok(sample_states,snapshot) and physics_clear(robots,sample_states))
            hold_sample(observed_t,snapshot,clear)
    elif not T.active:
        # Pure offline controls may explicitly supply a complete hand witness.
        T.advance(states,dt,T.force_ready,objects,hand)
    else:raise ValueError('missing physics observations')
    T.clear=hand_clear(hand,states)
    T.t=float(t);T.invalid='';T.state_stamp=stamp(objects)
    # Include the observed control endpoint too. Native hold samples above use
    # every240Hz pose and hand face, including between-action excursions.
    hold_sample(float(t),objects,T.clear)


def score(objects):
    try:
        if T.invalid or T.zero or not T.ready or not T.force_ready or T.state_stamp!=stamp(objects):return 0.
        return tier(T.ml[1])
    except (KeyError,ValueError,TypeError):return 0.


def success(objects):return score(objects)==1.


def apply_wrench(sim,name,force,torque):
    import torch
    a=sim.env.unwrapped.scene[name]
    com=parts._np(a.data.root_com_pose_w)[0,:3]
    root=parts._np(a.data.root_link_pose_w)[0,:3]
    torque_com=np.asarray(torque)-np.cross(com-root,force)
    T.wrench_world[name]=(np.asarray(force).copy(),torque_com.copy()+np.cross(com,force))
    f=torch.tensor(force,dtype=torch.float32,device=a.device).reshape(1,1,3)
    m=torch.tensor(torque_com,dtype=torch.float32,device=a.device).reshape(1,1,3)
    a.set_external_force_and_torque(f,m,is_global=True)

# Rendering of declared state. These visuals have no colliders and cannot be manipulated.
_VIS={}

def make_visuals(sim):
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdGeom,UsdShade,Gf,Sdf
    stage=get_current_stage();base=sim.env.unwrapped.scene.env_prim_paths[0]
    def material(path,color,opacity):
        mat=UsdShade.Material.Define(stage,path);sh=UsdShade.Shader.Define(stage,path+'/shader');sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));sh.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return mat
    gasmat=material(base+'/air_material',(.15,.55,.90),.50)
    watermat=material(base+'/water_material',(.1,.55,.82),.12)
    # Physical glass colliders remain, with transparent visual materials.
    for name in (DONOR.name,RECEIVER.name,TROUGH.name):
        root=stage.GetPrimAtPath(base+'/'+name)
        for child in root.GetChildren():
            if child.GetName()=='looks':
                for m in child.GetChildren():
                    for sh in m.GetChildren():
                        shader=UsdShade.Shader(sh)
                        if shader:shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.32 if m.GetName()=='c0' else 1.)
    _VIS.clear();_VIS['stage']=stage;_VIS['base']=base
    # Gas volume is shown as a horizontal cap point cloud, matching cubature state.
    for name in (DONOR.name,RECEIVER.name):
        p=UsdGeom.Points.Define(stage,base+'/'+name+'/air_fill');p.CreateWidthsAttr([.003]);p.SetWidthsInterpolation('constant');p.CreateDisplayColorAttr([Gf.Vec3f(.12,.50,.88)])
        _VIS[name]=p
    bub=UsdGeom.Points.Define(stage,base+'/rising_bubbles');bub.CreateWidthsAttr([.004]);bub.SetWidthsInterpolation('constant');bub.CreateDisplayColorAttr([Gf.Vec3f(.9,.94,1.)]);_VIS['bubbles']=bub
    for i,(x,y,sx,sy) in enumerate(((.104,0.,.001,.292),(.496,0.,.001,.292),(.30,-.146,.392,.001),(.30,.146,.392,.001))):
        water=UsdGeom.Cube.Define(stage,base+'/waterline_'+str(i));water.CreateSizeAttr(1.);water.ClearXformOpOrder();water.AddTranslateOp().Set(Gf.Vec3d(x,y,WATER));water.AddScaleOp().Set(Gf.Vec3f(sx,sy,.001));water.CreateDisplayColorAttr([Gf.Vec3f(.05,.35,.75)])



def show_air(sim,states):
    from pxr import Gf
    origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
    for i,(name,s) in enumerate(zip((DONOR.name,RECEIVER.name),states)):
        actual,c,interface=gas_shape(s,T.ml[i],T.surface);world=geometry(s)[-1]
        pts=world[world[:,2]>=interface][::32] if actual>0 else np.empty((0,3))
        local=(pts-s.pos)@parts._quat_to_matrix(s.quat_xyzw)
        _VIS[name].GetPointsAttr().Set([Gf.Vec3f(*p) for p in local])
    _VIS['bubbles'].GetPointsAttr().Set([Gf.Vec3f(*(p+origin)) for p,v,i in T.bubbles])


_MONITOR={}

def rim_high(pos,quat):
    u=parts._quat_to_matrix(quat)[:,2]
    return float(pos[2]-.073*u[2]+.002*abs(u[2])+MOUTH_OUTER*math.sqrt(max(0.,1-u[2]**2)))

def install_physics_monitor(sim):
    """Observe the actual240Hz states after each scene update, including brief
    between-action rim excursions. It never writes a body pose or awards gas."""
    scene=sim.env.unwrapped.scene
    if _MONITOR.get('scene') is scene:return
    original=scene.update
    def update(dt,*args,**kwargs):
        original(dt,*args,**kwargs)
        try:
            origin=parts._np(scene.env_origins[0])
            trough_pose=parts._np(scene[TROUGH.name].data.root_link_pose_w)[0]
            surface=WATER+float(trough_pose[2]-origin[2]-TOP)
            poses={}
            for name in (DONOR.name,RECEIVER.name,TROUGH.name,*[x.name for x in SHELVES]):
                raw=parts._np(scene[name].data.root_link_pose_w)[0]
                if not np.isfinite(raw).all():raise ValueError('unknown physics solid pose')
                poses[name]=np.r_[raw[:3]-origin,raw[3:]].copy()
            tp=poses[TROUGH.name]
            if np.linalg.norm(tp[:3]-np.asarray(TROUGH_POS))>.005 or parts._quat_to_matrix(tp[3:])[2,2]<math.cos(math.radians(2)):T.zero='trough moved or tipped'
            for i,name in enumerate((DONOR.name,RECEIVER.name)):
                pose=poses[name]
                if not np.isfinite(pose).all():raise ValueError('unknown physics pose')
                depth=surface-rim_high(pose[:3],pose[3:])
                T.min_depth=min(T.min_depth,depth)
                if depth<=1e-9:T.vent(i)
                if not mouth_in_trough(pose_state(pose),pose_state(tp)):
                    T.vent(i);T.zero='mouth left trough water'
            if T.active:
                robots=[]
                for arm in ('left','right'):
                    asset=scene[arm+'_robot'];body=parts._np(asset.data.body_link_pose_w)[0]
                    if not np.isfinite(body).all():raise ValueError('unknown physics robot pose')
                    robots.extend((n,np.r_[p[:3]-origin,p[3:]].copy()) for n,p in zip(asset.body_names,body))
                T.physics_path.append((float(dt),poses,robots))
                if len(T.physics_path)>32:raise ValueError('unobserved control interval')
            T.physics_checks+=1
        except Exception as e:
            T.zero='unobserved physics interval';T.invalid=type(e).__name__+': '+str(e);T.ready=False
    scene.update=update
    _MONITOR.update(scene=scene,original=original)


@parts.on_reset
def reset(sim):
    global T
    T=AirTrial()
    install_physics_monitor(sim)
    for name in (DONOR.name,RECEIVER.name,TROUGH.name):apply_wrench(sim,name,np.zeros(3),np.zeros(3))
    states=sim.states();T.last_states=[states[DONOR.name],states[RECEIVER.name]]
    # Native contact offsets:2mm rather than stock5mm, important around mouths.
    make_visuals(sim)
    show_air(sim,T.last_states)


@parts.on_step
def step(sim):
    try:
        objects=sim.states();hand=clouds(sim);dt=float(sim.t)-T.t
        observe(objects,hand,float(sim.t))
        states=[objects[DONOR.name],objects[RECEIVER.name]];trough=objects[TROUGH.name]
        T.loads(states,trough,dt)
        for i,n in enumerate((DONOR.name,RECEIVER.name)):apply_wrench(sim,n,T.force[i],T.torque[i])
        apply_wrench(sim,TROUGH.name,T.reaction,T.reaction_torque)
        net_force=sum((T.wrench_world[n][0] for n in (DONOR.name,RECEIVER.name,TROUGH.name)),np.zeros(3))
        net_moment=sum((T.wrench_world[n][1] for n in (DONOR.name,RECEIVER.name,TROUGH.name)),np.zeros(3))
        if not np.isfinite(np.r_[net_force,net_moment]).all() or np.linalg.norm(net_force)>1e-6 or np.linalg.norm(net_moment)>1e-6:raise ValueError('unbalanced native reaction')
        T.force_ready=True;T.applied_count+=1;T.active=True
        show_air(sim,states)
        if os.getenv('GAS_AUDIT') and (round(sim.t*20)%5==0 or (T.ready and T.ml[1]>=22.)):
            row={'t':sim.t,'lot':lots.current().name,'ml':T.ml.tolist(),'lost':T.lost,'captured':T.captured,'emitted':T.emitted,'bubble_ml':sum(v for p,v,i in T.bubbles),'zero':T.zero,'invalid':T.invalid,'clear':T.clear,'ready':T.ready,'since':T.since,'min_depth':T.min_depth,'peak_force':T.peak_force,'peak_torque':T.peak_torque,'physics_checks':T.physics_checks,'blocked_ml':T.blocked_ml,'source_blocked_attempt_ml':T.source_blocked_attempt_ml,'net_force':net_force.tolist(),'net_moment':net_moment.tolist(),'loads':T.force.tolist(),'reaction':T.reaction.tolist(),'score':score(objects),'poses':{n:np.r_[objects[n].pos,objects[n].quat_xyzw].tolist() for n in (DONOR.name,RECEIVER.name,TROUGH.name,'receiver_shelf')},'velocities':{n:np.r_[objects[n].lin_vel,objects[n].ang_vel].tolist() for n in (DONOR.name,RECEIVER.name,TROUGH.name,'receiver_shelf')},'park_ok':bool(park_ok(states,objects))}
            row['hold_duration_s']=float(sim.t-T.since) if T.since is not None else 0.
            row['hold_samples']=len(T.hold)
            row['hands']={}
            origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
            for arm in ('left','right'):
                asset=sim.env.unwrapped.scene[arm+'_robot'];poses=parts._np(asset.data.body_link_pose_w)[0];pose=poses[asset.body_names.index('gripper')]
                tcp=pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array((0.,0.,.134))
                joint=parts._np(asset.data.joint_pos)[0]
                row['hands'][arm]={'tcp':tcp.tolist(),'quat':pose[3:].tolist(),'fingers':[float(joint[asset.joint_names.index(n)]) for n in ('joint7','joint8')]}
            with open(os.environ['GAS_AUDIT'],'a') as f:f.write(json.dumps(row)+'\n')
    except Exception as e:
        T.invalid=type(e).__name__+': '+str(e);T.zero='unobserved trial state';T.ready=False
        if not getattr(T,'printed',False):print('[gasjar] invalid: '+T.invalid,flush=True);T.printed=True

def initialize_fluid(sim):
    """Arm physical loads after the final layout reset, before the first action.
    The initial jars are prepared at rest; reset/settling is not scored work."""
    objects=sim.states();states=[objects[DONOR.name],objects[RECEIVER.name]]
    T.surface=WATER+float(objects[TROUGH.name].pos[2]-TOP)
    T.loads(states,objects[TROUGH.name],.5)
    for i,name in enumerate((DONOR.name,RECEIVER.name)):apply_wrench(sim,name,T.force[i],T.torque[i])
    apply_wrench(sim,TROUGH.name,T.reaction,T.reaction_torque)
    force=sum((T.wrench_world[n][0] for n in (DONOR.name,RECEIVER.name,TROUGH.name)),np.zeros(3))
    moment=sum((T.wrench_world[n][1] for n in (DONOR.name,RECEIVER.name,TROUGH.name)),np.zeros(3))
    if not np.isfinite(np.r_[force,moment]).all() or np.linalg.norm(force)>1e-6 or np.linalg.norm(moment)>1e-6:raise ValueError('unbalanced initial fluid load')
    T.last_states=states;T.physics_path=[];T.force_ready=True;T.active=True;T.applied_count=1
    show_air(sim,states)


_LAYOUT_APPLY=lots._apply

def _apply_layout_with_fluid(embodiment):
    result=_LAYOUT_APPLY(embodiment)
    initialize_fluid(parts.Sim(embodiment))
    # Update only visuals; no hidden robot motion or scored fluid transfer.
    embodiment._base.sim.render()
    embodiment._base.sim.render()
    return result

lots._apply=_apply_layout_with_fluid


# Ink bands and identity stickers are visual markings, not physical grasp features.
_BASE_AUTHOR=parts._author

def _author_native(part,*args,**kwargs):
    root=_BASE_AUTHOR(part,*args,**kwargs)
    if part.name in (DONOR.name,RECEIVER.name):
        from pxr import UsdPhysics
        for i in range(129,len(part.shapes)):
            child=root.GetStage().GetPrimAtPath(str(root.GetPath())+'/s'+str(i))
            if child.IsValid():UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
    return root

parts._author=_author_native

# Reduced contact offset is local to this submitted task folder.
parts.CONTACT_OFFSET=.0015


def demo():
    """Open-loop visible fixed layout, with native sidewall grasps."""
    d=np.array(lots.current().pos(DONOR.name));r=np.array(lots.current().pos(RECEIVER.name))
    heading=0.;theta=88.
    # Side approach (fingers point away from the bases); donor roof then tilts
    # away, which turns that same grip almost downward. No housing passes
    # through the closed end of an upright150mm vessel.
    center=np.array((.335,.020,TOP+.035))
    u=np.array((math.sin(math.radians(theta)),0.,math.cos(math.radians(theta))))
    mouth=center-HALF*u
    recipient=np.array((mouth[0]+.005,mouth[1]-.015,TOP+.075+.075+.020))
    donor_tcp=center+.025*u
    def wp(dt,p,f,label,a=75,h=0.):return tilt.Waypoint(dt,tuple(float(v) for v in p),f,label,tilt=a,heading=h)
    grip=-.019;gd=d+np.array((0.,0.,.025));gr=r+np.array((0.,0.,.025))
    pre_angle=60.;pre_u=np.array((math.sin(math.radians(pre_angle)),0.,math.cos(math.radians(pre_angle))))
    pre=np.array((.335,.090,TOP+.077))+.025*pre_u
    mid_angle=72.;mid_u=np.array((math.sin(math.radians(mid_angle)),0.,math.cos(math.radians(mid_angle))))
    middle=np.array((.335,.020,TOP+.058))+.025*mid_u
    captured_recipient=recipient+np.array((0.,.012,-.007))
    left=[wp(2,gd+[-.085,0,0],FINGER_OPEN,'approach donor from side'),wp(2,gd,FINGER_OPEN,'around glass sidewall'),wp(1,gd,grip,'grip donor'),
          wp(2,gd+[0,0,.020],grip,'clear the trough floor'),wp(6,pre,grip,'lower and pre-tilt with air retained',75-pre_angle),
          wp(5,pre,grip,'receiver positions next',75-pre_angle),wp(3,middle,grip,'lower with sample still retained',75-mid_angle),wp(4,donor_tcp,grip,'finish transfer tilt',75-theta),wp(15,donor_tcp,grip,'bubble transfer',75-theta),
          wp(2,donor_tcp,grip,'settle donor above trough floor',75-theta),wp(1,donor_tcp,FINGER_OPEN,'release donor underwater',75-theta),
          wp(2,(.36,.02,TOP+.10),FINGER_OPEN,'lift clear of donor',0),wp(2,(.36,.14,TOP+.14),FINGER_OPEN,'over the trough wall',0),wp(2,(.25,.22,TOP+.14),FINGER_OPEN,'withdraw left',0),wp(15,(.25,.22,TOP+.14),FINGER_OPEN,'still',0)]
    right=[wp(2,gr+[0,-.085,0],FINGER_OPEN,'approach receiver from side'),wp(2,gr,FINGER_OPEN,'around receiver'),wp(1,gr,grip,'grip receiver'),
           wp(2,gr,grip,'donor clears its shelf'),wp(6,gr,grip,'donor pre-tilts first'),wp(5,recipient+[0,0,.025],grip,'position receiver above donor'),
           wp(3,recipient+[0,0,.025],grip,'donor lowers first'),wp(4,recipient+[0,.012,.025],grip,'wait for final tilt'),wp(15,captured_recipient+[0,0,.025],grip,'receive bubbles'),
           wp(5,captured_recipient+[0,0,.025],grip,'donor parks underwater'),wp(4,captured_recipient+[0,0,.025],grip,'left hand clears first'),
           wp(4,(gr[0],gr[1],captured_recipient[2]+.025),grip,'move above receiver shelf before lowering'),wp(4,gr,grip,'receiver onto porcelain'),wp(1,gr,FINGER_OPEN,'release receiver'),wp(2,gr+[0,-.085,0],FINGER_OPEN,'withdraw'),wp(4,gr+[0,-.085,0],FINGER_OPEN,'still')]

    # Receiver approaches from the right; fingers close across x, not z.
    right=[tilt.Waypoint(w.duration,w.pos,w.finger,w.label,yaw=math.pi/2,tilt=75,heading=math.pi/2) for w in right]
    return {'left':left,'right':right}
