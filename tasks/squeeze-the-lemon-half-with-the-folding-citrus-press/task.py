"""An open yellow folding citrus press lies on a grey kitchen mat, its two original handles spread sideways.
A fresh lemon half rests cut-side down on the mat. A white saucer is at the left front and a blue collecting bowl is beside the press.
The pressing basket has linked side panels and a perforated pip catcher.
"""
from __future__ import annotations
import json,math,sys
from pathlib import Path
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import parts,lots,tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=0.;SCORE_OVER='end'
parts.CONTACT_OFFSET=.0005;parts.REST_OFFSET=0.
YELLOW=(.98,.76,.06);FLESH=(.98,.90,.46);WHITE=(.91,.91,.87)
PRESS_HOME=np.array([.27,.10,TOP+.007]);R_ARM=.050;L_PANEL=.100
D0=math.sqrt(L_PANEL**2-R_ARM**2);PHI_MAX=1.39;MAX_STROKE=2.8
A0=np.array([0.,R_ARM,.009]);B0=np.array([0.,-R_ARM,.009]);D_POS=np.array([D0,0.,.009]);C0=np.array([0.,0.,.009])
# The actual handle/basket topology is a four-bar C-A-D-B-C. All four axes are vertical in the cut-down pose.
# Detailed hinge offsets and2.5mm polypropylene walls are estimates; folded envelope250x104mm,270g.
def handle(sign):
    return [parts.box((.018,.183,.018),(0,sign*.100,.009),YELLOW),
            parts.cylinder(.009,.018,(0,sign*.191,.009),YELLOW)]
def side_panel(sign):
    AA=A0 if sign==1 else B0;mid=(AA+D_POS)/2
    angle=math.atan2(D_POS[1]-AA[1],D_POS[0]-AA[0])
    return [parts.box((.090,.0025,.040),(float(mid[0]),float(mid[1]),.040),YELLOW,yaw=angle),
            parts.cylinder(.0045,.024,(float(AA[0]),float(AA[1]),.012),YELLOW),parts.cylinder(.0045,.024,(float(D_POS[0]),float(D_POS[1]),.012),YELLOW)]
# The strainer's PP fold is a declared bounded passive follower, not an extra powered device in the real trial.
floor=[]
for y in np.arange(-.049,.050,.007):
    rx=.040*math.sqrt(max(.01,1-(y/.052)**2))
    floor.append(parts.box((2*rx,.005,.004),(D0/2,float(y),.002),YELLOW))
for a in np.arange(0,2*math.pi,math.pi/16):
    floor.append(parts.box((.0075,.0025,.007),(D0/2+.040*math.cos(a),.0505*math.sin(a),.0055),YELLOW,yaw=a+math.pi/2))
PRESS=parts.Part('citrus_press',pos=tuple(float(v) for v in PRESS_HOME),mass=.050,friction=(.75,.6),
 shapes=[*handle(1),parts.cylinder(.009,.018,tuple(C0),(.70,.70,.67)),parts.box((.018,.025,.004),(.006,0,.002),YELLOW)],
 joints=[parts.Hinge('handle',tuple(handle(-1)),tuple(C0),(0,0,1),-MAX_STROKE,.03,mass=.050,damping=.02),
         parts.Hinge('left_panel',tuple(side_panel(1)),tuple(A0),(0,0,1),-2.,.5,mass=.065,damping=.01),
         parts.Hinge('right_panel',tuple(side_panel(-1)),tuple(B0),(0,0,1),-.5,2.,mass=.065,damping=.01,parent='handle'),
         parts.Hinge('strainer_pivot',(parts.cylinder(.006,.010,tuple(C0),YELLOW),),tuple(C0),(0,0,1),-1.42,.03,mass=.010,spring=4.,damping=.08,max_effort=.30),
         parts.Slider('strainer',tuple(floor),(-1,0,0),.026,mass=.030,spring=500.,damping=3.,max_effort=3.,parent='strainer_pivot')])
fruit=[]
for z in np.arange(.0025,.030,.005):
    for y in np.arange(-.0275,.030,.005):
        rr=.030**2-float(y)**2-float(z)**2
        if rr>0:fruit.append(parts.box((2*math.sqrt(rr),.005,.005),(0,float(y),float(z)),YELLOW))
fruit.append(parts.box((.060,.060,.030),(0,0,.015),YELLOW,collide=False))
FRUIT_HOME=(.19,.25,TOP+.0065)
LEMON=parts.Part('lemon_half',pos=FRUIT_HOME,mass=.070,shapes=fruit,friction=(.8,.65))
MAT=parts.Part('press_mat',pos=(.28,.10,TOP),shapes=[parts.box((.40,.56,.006),(0,-.06,.003),(.24,.27,.28)),
 parts.box((.4,.001,.0002),(0,.2195,.0061),WHITE,collide=False),parts.box((.4,.001,.0002),(0,-.2195,.0061),WHITE,collide=False),
 parts.box((.001,.44,.0002),(.1995,0,.0061),WHITE,collide=False),parts.box((.001,.44,.0002),(-.1995,0,.0061),WHITE,collide=False)])
def vessel(name,pos,r,h,color,mass):
    shapes=[parts.cylinder(r-.003,.004,(0,0,.002),color)]
    for a in np.arange(0,2*math.pi,math.pi/16):
        x,y=(r-.0015)*math.cos(a),(r-.0015)*math.sin(a)
        shapes.append(parts.box((2*r*math.tan(math.pi/32)+.0005,.003,h),(x,y,h/2),color,yaw=a+math.pi/2))
    return parts.Part(name,pos=pos,mass=mass,shapes=shapes,friction=(.8,.65))
BOWL=vessel('collecting_bowl',(.355,-.130,TOP+.006),.080,.055,(.22,.48,.72),.130)
SAUCER=vessel('rind_saucer',(.13,0.,TOP+.006),.065,.008,WHITE,.120)
OBJECTS=[MAT,PRESS,LEMON,BOWL,SAUCER]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={BOWL.name:BOWL.pos}),lots.Lot('B',place={BOWL.name:(.375,-.130,TOP+.006),SAUCER.name:(.13,-.02,TOP+.006)}))
DEMO_TOUCHES={'press_mat'}

def rot(st):return parts._quat_to_matrix(st.quat_xyzw)
def world(st,p):return np.asarray(p)@rot(st).T+st.pos
def local(st,p):return (np.asarray(p)-st.pos)@rot(st)
def link_pose(st,n):
    if n=='base':return st.pos,rot(st)
    p,q=st.links[n];return np.asarray(p),parts._quat_to_matrix(q)
def frame(st):return rot(st)@parts._rot((0,0,1),float(st.joints['handle'])/2)
def pin_d(st):
    a,R=link_pose(st,'left_panel');b,S=link_pose(st,'right_panel')
    return (a+R@(D_POS-A0)+b+S@(D_POS-B0))/2

def state_for_closure(closure=0.,pos=None):
    phi=closure/2;beta=math.asin(R_ARM*math.cos(phi)/L_PANEL);beta0=math.asin(R_ARM/L_PANEL)
    R=parts._rot((0,0,1),phi);qx=parts._matrix_to_quat(R)
    dd=-R_ARM*math.sin(phi)+math.sqrt(L_PANEL**2-(R_ARM*math.cos(phi))**2)
    return PRESS.state(pos=PRESS_HOME if pos is None else pos,quat_xyzw=qx,joints={'handle':-closure,
        'left_panel':-beta+beta0-phi,'right_panel':beta-beta0+phi,
        'strainer_pivot':-phi,'strainer':D0/2-dd/2})

def handle_point(st,n,r):
    if n=='left':return world(st,[0,r,.009])
    p,R=link_pose(st,'handle');return p+R@np.array([0,-r,0.])

# A20Hz sampled controller is valid through3rad/s; future step bound includes cadence tolerance.
MODEL_SPEED=3.
MAX_MODEL_STEP=.151
ENERGY_RESERVE=8.*.180*MAX_MODEL_STEP
NATIVE_DAMPING=PRESS.joints[0].damping
def fresh():return dict(model_valid=True,known=False,bad=False,why='',t=None,q=None,open_angle=None,force=np.zeros(2),peak_force=0.,dose=0.,juice=0.,spill=0.,closure=0.,loaded=False,pressed=False,work=0.,external_work=0.,native_damping_work=0.,returned=0.,energy=0.,energy_error=0.,material_loss=0.,max_q=0.,clear=False,hold=0.,anchor=None,stamp=None,qualified_start=None,qualified_previous=False,material_closure=0.,seated=False,cut_down=False,handle_grasps=False,force_ok=False)
REC=fresh()
def finite_history(ob,hands,t):
    """Reject an unobserved frame permanently rather than resuming credit later."""
    try:
        if not math.isfinite(float(t)) or hands is None or set(hands)!={'left','right'}:return False
        for arm in ('left','right'):
            if arm not in hands.pads or arm not in hands.fingers or len(hands.pads[arm])!=2:return False
            for cloud in (hands[arm],*hands.pads[arm]):
                cloud=np.asarray(cloud)
                if cloud.ndim!=2 or cloud.shape[1]!=3 or not len(cloud) or not np.isfinite(cloud).all():return False
            fingers=np.asarray(hands.fingers[arm])
            if fingers.shape!=(2,) or not np.isfinite(fingers).all():return False
        for spec in OBJECTS:
            st=ob[spec.name]
            for field,shape in (('pos',(3,)),('quat_xyzw',(4,)),('lin_vel',(3,)),('ang_vel',(3,)),('low',(3,)),('high',(3,))):
                value=np.asarray(getattr(st,field))
                if value.shape!=shape or not np.isfinite(value).all():return False
            if spec.joints:
                if not np.isfinite(list(st.joints.values())).all():return False
                for name,_,_ in spec.links:
                    if name=='base':continue
                    pos,quat=st.links[name]
                    if np.asarray(pos).shape!=(3,) or np.asarray(quat).shape!=(4,) or not np.isfinite(pos).all() or not np.isfinite(quat).all():return False
        for name,spec in [('citrus_press',PRESS),('lemon_half',LEMON),('collecting_bowl',BOWL),('rind_saucer',SAUCER)]:
            for body,_,_ in spec.links:
                v,w=ob[name].link_velocities[body]
                if np.asarray(v).shape!=(3,) or np.asarray(w).shape!=(3,) or not np.isfinite(np.r_[v,w]).all():return False
        return True
    except (KeyError,ValueError,TypeError,AttributeError):return False

def geometric_seat(ob):
    st=ob['citrus_press'];l=ob['lemon_half'];R=frame(st);p=(l.pos-st.pos)@R;d=(pin_d(st)-st.pos)@R
    seat=abs(p[0]-d[0]/2)<=.014+1e-6 and abs(p[1])<=.012+1e-6 and .002-1e-6<=p[2]<=.014+1e-6 and R[2,2]>=math.cos(math.radians(15))
    return bool(seat),bool(rot(l)[2,2]>=math.cos(math.radians(20)))
def collector(ob):
    st,b=ob['citrus_press'],ob['collecting_bowl'];R=frame(st);spout=st.pos
    h=spout[2]-(b.pos[2]+.055)
    if R[2,2]<math.cos(math.radians(15)) or rot(b)[2,2]<math.cos(math.radians(5)) or not .005<=h<=.110:return False
    hit=spout[:2]+np.asarray(st.lin_vel)[:2]*math.sqrt(2*h/9.81)
    return bool(np.linalg.norm(hit-b.pos[:2])+.008+h*math.tan(math.radians(8))<=.069)
def tier(closure,juice,park):
    if closure+1e-6<math.radians(45) or juice<2:return 0.
    if closure+1e-6<math.radians(90) or juice<8:return .25
    if closure+1e-6<math.radians(120) or juice<15:return .5
    if closure+1e-6<math.radians(150) or juice<20 or not park:return .75
    return 1.
def body_boxes(ob,spec):
    st=ob[spec.name];ps={'base':(st.pos,rot(st))}
    ps.update({n:(p,parts._quat_to_matrix(q)) for n,(p,q) in getattr(st,'links',{}).items()})
    return spec.world_boxes(ps)
@lru_cache(maxsize=1)
def physical_witnesses():
    result={};corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    for spec in (PRESS,LEMON,BOWL,SAUCER):
        result[spec.name]={}
        for name,origin,shapes in spec.links:
            clouds=[]
            for sh in shapes:
                if not sh.collide:continue
                if sh.kind=='box':pts=corners*np.asarray(sh.size)/2
                else:
                    axis='xyz'.index(sh.axis);radial=[i for i in range(3) if i!=axis];pts=[]
                    for end in (-sh.size[1]/2,sh.size[1]/2):
                        for angle in np.linspace(0,2*math.pi,64,endpoint=False):
                            v=np.zeros(3);v[axis]=end;v[radial]=sh.size[0]*np.array([math.cos(angle),math.sin(angle)]);pts.append(v)
                    pts=np.array(pts)
                clouds.append(pts@parts._rot((0,0,1),sh.yaw).T+np.asarray(sh.at)-origin)
            result[spec.name][name]=np.concatenate(clouds)
    return result

def instantaneous_still(ob):
    # Native body-link velocities catch fast small oscillations hidden by position sampling.
    for name,links in physical_witnesses().items():
        st=ob[name]
        for body,cloud in links.items():
            p,R=link_pose(st,body);v,w=st.link_velocities[body]
            if not np.isfinite(np.r_[v,w]).all() or np.linalg.norm(w)>.15:return False
            if np.linalg.norm(v+np.cross(w,cloud@R.T),axis=1).max()>.012:return False
    return True

def snapshot(ob):
    out=[]
    for name,links in physical_witnesses().items():
        st=ob[name]
        for body,cloud in links.items():
            p,R=link_pose(st,body);out.append(cloud@R.T+p)
    return np.concatenate(out)

def shape_distance(points,shape,c,R):
    """Signed distance to a physical primitive, including actual cylinder caps."""
    q=(np.asarray(points)-c)@R
    if shape.kind=='box':
        d=abs(q)-np.asarray(shape.size)/2
        return np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0)
    axis='xyz'.index(shape.axis);radial=np.linalg.norm(np.delete(q,axis,axis=1),axis=1)-shape.size[0]
    axial=abs(q[:,axis])-shape.size[1]/2;d=np.column_stack([radial,axial])
    return np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0)

def released(ob,hands):
    for cloud in hands.values():
        if not np.isfinite(cloud).all():return False
        for spec in (PRESS,LEMON,BOWL,SAUCER):
            st=ob[spec.name];near=cloud[np.all((cloud>=st.low-.005001)&(cloud<=st.high+.005001),axis=1)]
            if not len(near):continue
            shapes=[sh for _,_,ss in spec.links for sh in ss]
            for sh,(_,c,R,size) in zip(shapes,body_boxes(ob,spec)):
                if not sh.collide:continue
                sample=near[np.all(abs(near-c)<=np.abs(R)@(size/2)+.005001,axis=1)]
                if len(sample) and shape_distance(sample,sh,c,R).min()<=.005:return False
    return True

def native_handles(ob,hands):
    st=ob['citrus_press'];matches=[]
    for n,sign in [('base',1),('handle',-1)]:
        p,R=link_pose(st,n);good=[];zc=.009 if n=='base' else 0.
        primitives=[parts.box((.018,.183,.018),(0,sign*.100,zc)),parts.cylinder(.009,.018,(0,sign*.191,zc))]
        for arm in hands:
            pads=hands.pads.get(arm);q=hands.fingers.get(arm)
            if pads is None or q is None or abs(q).sum()>.075:continue
            sides=[]
            for pad in pads:
                xy=(pad-p)@R
                distances=np.array([shape_distance(xy,sh,np.asarray(sh.at),np.eye(3)) for sh in primitives])
                dist=np.min(distances,axis=0);chosen=np.argmin(distances,axis=0)
                radial=np.linalg.norm(xy[:,:2]-np.array([0.,sign*.191]),axis=1)
                nx=np.where(chosen==0,np.sign(xy[:,0]),xy[:,0]/np.maximum(radial,1e-9))
                # Side-face witnesses exclude both horizontal caps and non-opposed end normals.
                hit=(sign*xy[:,1]>=.070)&(sign*xy[:,1]<=.201)&(abs(nx)>=math.sqrt(.5))&(abs(dist)<=.001001)&(abs(xy[:,2]-zc)<=.0075)
                if not hit.any():sides=[];break
                sides.append(float(np.median(xy[hit,0])))
            if len(sides)==2 and sides[0]*sides[1]<0:good.append(arm)
        matches.append(good)
    REC['grasp_matches']=matches
    return any(a!=b for a in matches[0] for b in matches[1])

def parked(ob):
    st,l,s=ob['citrus_press'],ob['lemon_half'],ob['rind_saucer'];mat=ob['press_mat']
    signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    pp=np.concatenate([c+(signs*size/2)@R.T for _,c,R,size in body_boxes(ob,PRESS)])
    # The400x440mm white parking outline is unchanged inside the larger support sheet.
    onmat=np.all(abs(pp[:,0]-mat.pos[0])<=.201)&np.all(abs(pp[:,1]-mat.pos[1])<=.221)&(abs(st.low[2]-(TOP+.006))<=.002)
    lp=local(s,l.pos)
    # The spent half may rest on either face; support, rather than irrelevant final orientation, is judged.
    solids=[c+(signs*size/2)@R.T for sh,(_,c,R,size) in zip(LEMON.shapes,body_boxes(ob,LEMON)) if sh.collide]
    lowest=float(local(s,np.concatenate(solids))[:,2].min())
    return bool(onmat and frame(st)[2,2]>.94 and abs(lp[0])<=.025 and abs(lp[1])<=.025 and .002-1e-6<=lowest<=.006+1e-6 and rot(s)[2,2]>.94)
def observe(ob,hands,t,applied=True):
    r=REC
    if not r['model_valid']:return
    if not finite_history(ob,hands,t):
        r.update(known=False,bad=True,why='unknown trial history',force_ok=False,force=np.zeros(2),hold=0.);return
    if r['t'] is None and not 0.<=t<=.050001:
        r.update(known=False,bad=True,why='missing initial history',force_ok=False,force=np.zeros(2),hold=0.);return
    if r['t'] is not None:
        if t==r['t']:return  # idempotent reread of one frame
        if t<r['t'] or t-r['t']>.050001:
            r.update(known=False,bad=True,why='missing temporal history',force_ok=False,force=np.zeros(2),hold=0.);return
    dt=.05 if r['t'] is None else t-r['t'];r['t']=float(t)
    if hands is None:r.update(known=False,bad=True,why='unknown trial history',hold=0.);return
    try:
        st=ob['citrus_press'];q=-float(st.joints['handle']);previous_q=r['q'];dq=0. if previous_q is None else (q-previous_q)/dt;r['q']=q
        seated,down=geometric_seat(ob);r['seated']=seated;r['cut_down']=down
        old_force=r['force'].copy()
        # Previously applied wrench times measured hinge motion: no energy from a static hold.
        delta=0. if previous_q is None else q-previous_q
        if q<-.035 or q>MAX_STROKE+.005:
            r.update(bad=True,why='hinge outside intact limits',known=True,force=np.zeros(2));return
        if abs(delta)>MODEL_SPEED*dt+1e-6:
            r.update(model_valid=False,known=False,force_ok=False,force=np.zeros(2),hold=0.,why='hinge motion outside sampled-load bandwidth');return
        dw=float(np.mean(old_force)*.180*delta)
        # Existing implicit native hinge damper dissipates c*integral(qdot²)dt.
        # c*delta²/dt is a conservative lower bound. Count it only for a seated loaded half.
        native_dw=NATIVE_DAMPING*delta*delta/dt if seated and down and r['loaded'] else 0.
        r['external_work']+=max(0.,dw);r['native_damping_work']+=native_dw
        r['work']+=max(0.,dw)+native_dw;r['returned']+=max(0.,-dw);r['energy']+=dw+native_dw
        r['energy_error']+=max(0.,-r['energy'])
        if r['energy_error'] > 1e-6:
            # Numerical model invalidity is not an observable physical task failure.
            # Fail closed for the scalar API, clear the load, and label this trial unscored in reports.
            r.update(model_valid=False,known=False,force_ok=False,force=np.zeros(2),hold=0.,why='invalid load-model energy history');return
        r['energy']=max(0.,r['energy']);r['max_q']=max(r['max_q'],q)
        target=min(8.,3.+3.*np.clip(q/MAX_STROKE,0,1)+2.*max(dq/2,0.)) if seated and q>.15 else 0.
        r['force']+=min(1.,dt/.15)*(target-r['force'])
        r['force_ok']=bool(applied);r['handle_grasps']=native_handles(ob,hands)
        if seated and down:r['loaded']=True;r['open_angle']=max(0,q) if r['open_angle'] is None else min(r['open_angle'],max(0,q))
        pa,ra=link_pose(st,'left_panel');pb,rb=link_pose(st,'right_panel')
        if np.linalg.norm((pa+ra@(D_POS-A0))-(pb+rb@(D_POS-B0)))>.003:r.update(bad=True,why='back hinge separated')
        if r['loaded'] and q>.35 and not seated:r.update(bad=True,why='rind escaped loaded basket')
        if seated and q>.35 and not down:r.update(bad=True,why='cut face or seed retention lost')
        stroke=0. if r['open_angle'] is None else max(0,q-r['open_angle'])
        # Material production depends on physical compression, including ineligible hand histories.
        # A later grasp cannot erase a spill or retroactively qualify an earlier closure.
        r['material_closure']=max(r['material_closure'],stroke if seated and down else 0.)
        loaded_compression=seated and down and applied and stroke>.10
        free_held=st.low[2]>max(TOP+.006,ob['collecting_bowl'].high[2],ob['rind_saucer'].high[2])+.005
        qualified=bool(seated and down and applied and r['handle_grasps'] and free_held)
        if qualified:
            if not r['qualified_previous']:r['qualified_start']=q
            r['qualified_start']=min(r['qualified_start'],q)
            r['closure']=max(r['closure'],max(0.,q-r['qualified_start']))
        else:r['qualified_start']=None
        r['qualified_previous']=qualified
        r['free_held']=bool(free_held)
        if loaded_compression:
            r['pressed']=True
            pressure=float(np.mean(old_force)*3.6/(math.pi*.030**2));dd=max(0.,pressure-5000.)*dt;r['dose']+=dd
            # Estimated irreversible rind/fluid loss is0.06J/mL; pressure-time remains rate-limiting.
            # Retain the maximum one-step return budget before spending energy on material.
            # Production consumes measured work; its loss cannot be reused for unloading.
            made=min(25.-r['juice']-r['spill'],25.*dd/120000.,max(0.,r['energy']-ENERGY_RESERVE)/.060)
            r['energy']-=made*.060;r['material_loss']+=made*.060
            if made>0:
                if collector(ob):r['juice']+=made
                else:r['spill']+=made;r.update(bad=True,why='expressed juice outside bowl')
        # Bound EVERY new load before publication, including first activation and static holding.
        # For any valid next delta>=-MAX_MODEL_STEP, old_force*.180*delta cannot overdraw the tank.
        r['force']=np.minimum(r['force'],r['energy']/(.180*MAX_MODEL_STEP))
        # Passivity budget: an unloading command may return at most the remaining stored energy,
        # even if the next sample traverses the entire remaining intact opening range.
        if dq < -.05 or q < r['max_q']-.02:
            r['force']=np.minimum(r['force'],r['energy']/(.180*max(q+.035,1e-6)))
        r['peak_force']=max(r['peak_force'],float(max(r['force'])))
        if r['pressed'] and rot(ob['collecting_bowl'])[2,2]<math.cos(math.radians(10)):r.update(bad=True,why='bowl tipped')
        if any(ob[spec.name].low[2]<TOP-.02 for spec in (PRESS,LEMON,BOWL,SAUCER)):r.update(bad=True,why='scored object dropped')
        r['known']=True;r['clear']=released(ob,hands);now=snapshot(ob)
        still=r['clear'] and instantaneous_still(ob) and all(ob[n].resting(.012,.15) for n in ('citrus_press','lemon_half','collecting_bowl','rind_saucer'))
        if r['stamp'] is not None:still=bool(still and np.linalg.norm(now-r['stamp'],axis=1).max()/dt<=.012 and abs(dq)<=.15)
        if still:
            if r['anchor'] is None:r['anchor']=now.copy();r['hold']=0.
            elif np.linalg.norm(now-r['anchor'],axis=1).max()<=.002:r['hold']+=dt
            else:r['anchor']=now.copy();r['hold']=0.
        else:r['anchor']=None;r['hold']=0.
        r['stamp']=now.copy()
    except (KeyError,ValueError,TypeError,AttributeError):r.update(known=False,bad=True,why='unknown object history',hold=0.)
def score(ob):
    try:
        r=REC
        if not r['model_valid'] or not r['known'] or r['bad'] or not r['force_ok'] or not r['clear'] or r['hold']<1.-1e-8 or r['stamp'] is None:return 0.
        if not np.allclose(snapshot(ob),r['stamp'],atol=1e-8):return 0.
        return tier(r['closure'],r['juice'],parked(ob))
    except (KeyError,ValueError,TypeError,AttributeError):return 0.
def success(ob):return score(ob)==1.
@lru_cache(maxsize=1)
def face_mesh():
    from yam_arena import analysis
    out={}
    for name in ('gripper','tip_left','tip_right'):
        pts=[]
        for hull in analysis.link_hulls().get(name,[]):
            pts.extend(hull.vertices)
            for plane in np.unique(hull.planes.round(7),axis=0):
                face=hull.vertices[np.abs(hull.vertices@plane[:3]-plane[3])<2e-6]
                if len(face)<3:continue
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(plane[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for a,b in zip(face,np.roll(face,-1,axis=0)):
                    n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                    for i in range(n+1):
                        for j in range(n-i+1):pts.append(c+(a-c)*i/n+(b-c)*j/n)
        cloud=np.asarray(pts);_,ix=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True);out[name]=cloud[ix]
    return out
class Hands(dict):
    def __init__(self):super().__init__();self.pads={};self.fingers={}
def get_hands(sim):
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);out=Hands()
    for arm in ('left','right'):
        robot=base.scene[arm+'_robot'];poses=parts._np(robot.data.body_link_pose_w)[0];named={}
        for n,cloud in face_mesh().items():
            p=poses[list(robot.body_names).index(n)];named[n]=cloud@parts._quat_to_matrix(p[3:]).T+p[:3]-origin
        out[arm]=np.concatenate(list(named.values()));out.pads[arm]=(named['tip_left'],named['tip_right'])
        q=parts._np(robot.data.joint_pos)[0];out.fingers[arm]=np.array([q[list(robot.joint_names).index(n)] for n in ('joint7','joint8')])
    return out

_PATHS={};_author_base=parts._author
def author(part,path,translation,orientation):
    root=_author_base(part,path,translation,orientation);_PATHS[part.name]=path
    from pxr import Gf,Sdf,UsdPhysics
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage()
    if part.name=='citrus_press':
        joint=UsdPhysics.RevoluteJoint.Define(stage,path+'/joints/back_pin')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(path+'/left_panel')]);joint.CreateBody1Rel().SetTargets([Sdf.Path(path+'/right_panel')])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(D_POS-A0)));joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*(D_POS-B0)))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.));joint.CreateAxisAttr('Z')
        joint.CreateLowerLimitAttr(-180.);joint.CreateUpperLimitAttr(180.);joint.CreateCollisionEnabledAttr(False);joint.CreateExcludeFromArticulationAttr(True)
        for n in ('strainer_pivot','strainer'):
            api=UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path+'/'+n))
            api.CreateFilteredPairsRel().SetTargets([Sdf.Path(path+'/'+nn) for nn in ('base','handle','left_panel','right_panel')])
    if part.name=='lemon_half':
        from pxr import Gf,UsdGeom
        from isaaclab.sim.utils import get_current_stage
        stage=get_current_stage()
        # Smooth true60mm hemispherical rind over the declared central physical contact strip.
        for i in range(len(LEMON.shapes)):UsdGeom.Imageable(stage.GetPrimAtPath(path+'/s'+str(i))).MakeInvisible()
        points=[]
        for theta in np.linspace(0,math.pi/2,10):
            for phi in np.linspace(0,2*math.pi,40,endpoint=False):points.append((.030*math.sin(theta)*math.cos(phi),.030*math.sin(theta)*math.sin(phi),.030*math.cos(theta)))
        mesh=UsdGeom.Mesh.Define(stage,path+'/rind_skin');mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4]*360)
        mesh.CreateFaceVertexIndicesAttr([v for i in range(9) for j in range(40) for v in (i*40+j,i*40+(j+1)%40,(i+1)*40+(j+1)%40,(i+1)*40+j)])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*YELLOW)]);mesh.AddScaleOp().Set(Gf.Vec3f(1.))
        cap=UsdGeom.Mesh.Define(stage,path+'/cut_face');cap.CreatePointsAttr([(0.,0.,.0003)]+[(.029*math.cos(a),.029*math.sin(a),.0003) for a in np.linspace(0,2*math.pi,40,endpoint=False)])
        cap.CreateFaceVertexCountsAttr([3]*40);cap.CreateFaceVertexIndicesAttr([v for j in range(40) for v in (0,1+(j+1)%40,1+j)])
        cap.CreateDisplayColorAttr([Gf.Vec3f(*FLESH)]);cap.AddScaleOp().Set(Gf.Vec3f(1.))
    if 'citrus_press' in _PATHS and 'lemon_half' in _PATHS:
        for n in ('left_panel','right_panel'):
            api=UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(_PATHS['citrus_press']+'/'+n))
            api.CreateFilteredPairsRel().SetTargets([Sdf.Path(_PATHS['lemon_half'])])
    return root
parts._author=author

def com_local(shapes,origin):
    w=np.array([np.prod(s.size) if s.kind=='box' else math.pi*s.size[0]**2*s.size[1] for s in shapes])
    return np.average(np.array([s.at for s in shapes]),weights=w,axis=0)-origin

def clear_resistance(sim):
    import torch
    asset=sim.env.unwrapped.scene['citrus_press']
    zero=torch.zeros((1,len(asset.body_names),3),dtype=torch.float32,device=asset.device)
    asset.set_external_force_and_torque(forces=zero,torques=zero,is_global=True);asset.write_data_to_sim()

def resistance(sim,ob):
    import torch
    asset=sim.env.unwrapped.scene['citrus_press'];dev=asset.device;nb=len(asset.body_names);st=ob['citrus_press']
    forces=np.zeros((nb,3));moments=np.zeros_like(forces);names=list(asset.body_names);ib=names.index('base');ir=names.index('handle')
    cp=world(st,C0);z=rot(st)[:,2];pR,RR=link_pose(st,'handle')
    cmB=st.pos+rot(st)@com_local(PRESS.shapes,np.zeros(3));cmR=pR+RR@com_local(PRESS.joints[0].shapes,C0)
    for i,n,sign,cm in [(ib,'left',-1,cmB),(ir,'right',1,cmR)]:
        h=handle_point(st,n,.18);rad=h-cp;rad-=z*(rad@z)
        force=sign*REC['force'][0 if n=='left' else 1]*np.cross(z,rad)/max(np.linalg.norm(rad),1e-9)
        forces[i]+=force;moments[i]+=np.cross(h-cm,force)
        forces[ib]-=force;moments[ib]+=np.cross(cp-cmB,-force)
    f=torch.tensor(forces[None],dtype=torch.float32,device=dev);tau=torch.tensor(moments[None],dtype=torch.float32,device=dev)
    asset.set_external_force_and_torque(forces=f,torques=tau,is_global=True);asset.write_data_to_sim()

def follow_strainer(sim,ob):
    st=ob['citrus_press'];R=frame(st);d=(pin_d(st)-st.pos)@R
    sim.set_target('citrus_press','strainer_pivot',float(np.clip(st.joints['handle']/2,-1.42,.03)))
    sim.set_target('citrus_press','strainer',float(np.clip(D0/2-d[0]/2,0,.026)))

def draw_material(sim,objects):
    from pxr import Gf,UsdGeom,Vt,Sdf
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage()
    # The compliant lobes narrow under actual loaded closure and remain crushed after reopening.
    factor=max(.30,1.-.70*REC['material_closure']/2.72)
    for n in ('rind_skin','cut_face'):
        prim=stage.GetPrimAtPath(_PATHS['lemon_half']+'/'+n)
        prim.GetAttribute('xformOp:scale').Set(Gf.Vec3f(1.,factor,1.))
    # Finite collected volume as a visible shallow surface in the actual open bowl, not a score badge.
    path='/World/citrus_collected_juice';mesh=UsdGeom.Mesh.Define(stage,path)
    r=.068;height=.004+REC['juice']*1e-6/(math.pi*r*r)
    local_points=np.array([[0,0,height]]+[[r*math.cos(a),r*math.sin(a),height] for a in np.linspace(0,2*math.pi,33)[:-1]])
    pts=world(objects['collecting_bowl'],local_points)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr().Set([3]*32)
    mesh.CreateFaceVertexIndicesAttr().Set([j for i in range(32) for j in (0,1+i,1+(i+1)%32)])
    mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(.96,.83,.35)])
    mesh.GetPrim().CreateAttribute('visibility',Sdf.ValueTypeNames.Token).Set('inherited' if REC['juice']>.05 else 'invisible')


@parts.on_reset
def reset(sim):
    global REC
    REC=fresh();ob=sim.states();follow_strainer(sim,ob);resistance(sim,ob);draw_material(sim,ob)
@parts.on_step
def step(sim):
    ob=sim.states();hands=None
    try:
        hands=get_hands(sim);observe(ob,hands,sim.t)
        if REC['known']:follow_strainer(sim,ob);resistance(sim,ob)
        else:clear_resistance(sim)
        if round(sim.t*20)%10==0:draw_material(sim,ob)
    except Exception as e:
        REC.update(known=False,bad=True,why='unknown runtime history',force_ok=False,hold=0.)
        if not REC.get('error'):print('[citrus-error] '+repr(e),flush=True);REC['error']=str(e)
    if round(sim.t*20)==1:
        st=ob['citrus_press'];print('[citrus-first] '+json.dumps({'pos':st.pos.tolist(),'quat':st.quat_xyzw.tolist(),'joints':st.joints,'fruit':ob['lemon_half'].pos.tolist()}),flush=True)
    if round(sim.t*20)%20==0 and (15<=sim.t<=30 or 72<=sim.t<=105) and hands is not None:
        print('[citrus-pads] '+json.dumps({'t':sim.t,'pads':{a:[{'low':p.min(0).tolist(),'high':p.max(0).tolist()} for p in hands.pads[a]] for a in ('left','right')},'fingers':{a:hands.fingers[a].tolist() for a in ('left','right')},'fruit':ob['lemon_half'].pos.tolist()}),flush=True)
    if round(sim.t*20)%20==0:
        st=ob['citrus_press'];ll=ob['lemon_half'];ff=frame(st)
        REC['geometry']={'press':st.pos.tolist(),'press_low':st.low.tolist(),'quat':st.quat_xyzw.tolist(),'rind':ll.pos.tolist(),'rind_in_basket':((ll.pos-st.pos)@ff).tolist(),'rear_in_basket':((pin_d(st)-st.pos)@ff).tolist(),'bowl':ob['collecting_bowl'].pos.tolist(),'saucer':ob['rind_saucer'].pos.tolist(),'parked':parked(ob),'fruit_quat':ll.quat_xyzw.tolist(),'floor':st.links['strainer'][0].tolist()}
    if round(sim.t*20)%20==0:print('[citrus] '+json.dumps({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in REC.items() if k not in ('anchor','stamp')}),flush=True)

def demo():
    b=np.array(lots.current().pos('collecting_bowl'));s=np.array(lots.current().pos('rind_saucer'))
    def W(t,p,g,label,yaw=0.):return Waypoint(t,tuple(float(v) for v in p),g,label,yaw)
    def Z(p,h):return np.array([p[0],p[1],TOP+h])
    def HP(arm,phi,shift=(0,0,0),r=None):
        rr=(.185 if arm=='left' else .085) if r is None else r
        sign=1 if arm=='left' else -1;yaw=sign*(math.pi/2-phi)
        # Left's line is equivalent modulo180deg; approach opposite its projecting native handle.
        grip_yaw=phi-math.pi/2 if arm=='left' else math.pi/2-phi
        th=math.radians(45 if arm=='left' else 0)
        off=np.array([-.012*math.sin(th)*math.cos(grip_yaw),-.012*math.sin(th)*math.sin(grip_yaw),.012*math.cos(th)])
        # Fixed tangential lead compensates arm deflection measured in native run3.
        # Desired wrist orientation stays within the real hinge stop; no observed state is read.
        lead=.001*(phi/PHI_MAX)**2
        tangent=np.array([-math.cos(phi),-sign*math.sin(phi),0.])
        return PRESS_HOME+np.array([-rr*math.sin(phi),sign*rr*math.cos(phi),.009])+off+np.array(shift)+lead*tangent
    def H(arm,t,p,g,label,phi=0.):
        yaw=phi-math.pi/2 if arm=='left' else math.pi/2-phi
        return tilt.Waypoint(t,tuple(float(v) for v in p),g,label,yaw,tilt=45 if arm=='left' else 0,heading=yaw)
    # The mat supports the open press during loading; the right arm waits clear.
    rf=HP('right',0,r=.110)
    right=[W(4.,(.16,-.25,TOP+.17),FINGER_OPEN,'wait clear during loading')]
    left=[W(4.,(.16,.25,TOP+.16),FINGER_OPEN,'wait clear')]
    fruit=np.array(FRUIT_HOME);pick=fruit+np.array([0,0,.025]);drop=PRESS_HOME+np.array([D0/2,0,.075])
    left += [W(1.5,Z(pick,.14),FINGER_OPEN,'over half',1.45),W(1.5,pick,FINGER_OPEN,'down to rind',1.45),W(1.,pick,-.027,'grasp rind',1.45),
             W(1.2,Z(pick,.14),-.027,'lift half',1.45),W(2.,Z(drop,.14),-.027,'over open basket',1.45),W(1.5,drop,-.027,'lower above basket',1.45),
             W(.8,drop,FINGER_OPEN,'release cut-down half above basket',1.45),W(1.2,Z(drop,.15),FINGER_OPEN,'clear basket',1.45)]
    lp=HP('left',0)
    left += [H('left',2.,Z(lp,.14),FINGER_OPEN,'over left handle'),H('left',1.5,lp,FINGER_OPEN,'descend to left handle'),H('left',1.,lp,0.,'grasp left handle')]
    tleft=sum(w.duration for w in left);right.append(W(tleft-4.,(.16,-.25,TOP+.17),FINGER_OPEN,'wait for left base-handle grasp'))
    rp=HP('right',0)
    right += [H('right',1.5,Z(rp,.14),FINGER_OPEN,'over near purchase'),H('right',1.,rp,FINGER_OPEN,'down to near purchase'),H('right',.8,rp,0.,'grasp original near handle')]
    left.append(H('left',3.3,lp,0.,'counterhold on mat while right approaches'))
    shift=np.array([b[0]-PRESS_HOME[0],b[1]-PRESS_HOME[1],.105])
    for arm,plan in [('left',left),('right',right)]:
        start=HP(arm,0)
        plan += [H(arm,2.,HP(arm,0,(0,0,.105)),0.,'lift loaded press'),H(arm,3.,HP(arm,0,shift),0.,'carry over bowl')]
        for phi in np.linspace(.15,PHI_MAX,10):plan.append(H(arm,.75,HP(arm,phi,shift),0.,'loaded four-bar closure',phi))
        plan.append(H(arm,40.,HP(arm,PHI_MAX,shift),0.,'hold compression',PHI_MAX))
        for phi in np.linspace(PHI_MAX-.15,0,8):plan.append(H(arm,.5,HP(arm,phi,shift),0.,'reopen linked basket',phi))
        plan += [H(arm,3.,HP(arm,0,(0,0,.105)),0.,'return press to mat'),H(arm,2.,HP(arm,0,(0,0,.105)),0.,'hold reopened press above mat')]
    # Both original grips remain closed; turn the left wrist about its closing axis.
    def P(arm,t,root,deg,g,label,left_tilt=0.):
        rr=.185 if arm=='left' else .085;sign=1 if arm=='left' else -1
        yaw=-math.pi/2 if arm=='left' else math.pi/2;angle=left_tilt if arm=='left' else 0.;th=math.radians(angle)
        off=np.array([-.012*math.sin(th)*math.cos(yaw),-.012*math.sin(th)*math.sin(yaw),.009+.012*math.cos(th)])
        Q=parts._rot((1,0,0),math.radians(10.*abs(deg)/80.))@parts._rot((0,1,0),math.radians(deg));target=np.asarray(root)+Q@(np.array([0,sign*rr,0.])+off)
        R=Q@tilt.rotation(yaw,angle,yaw)
        return Waypoint(t,tuple(float(v) for v in target),g,label,tilt._TiltedYaw(yaw,R))
    air=PRESS_HOME+np.array([0,0,.105]);destination=np.array([s[0]+.045,s[1],TOP+.080])
    for arm,plan in [('left',left),('right',right)]:
        plan.append(P(arm,3.,destination,0,0.,'carry open press over saucer',45.))
        plan.append(P(arm,3.,destination,0,0.,'pivot left wrist while both grips stay closed',0.))
        for angle in (-20,-40,-60,-80):plan.append(P(arm,1.,destination,angle,0.,'tip diagonally past common hinge'))
        plan.append(P(arm,3.,destination,-80,0.,'let spent half roll to saucer'))
        for angle in (-60,-40,-20,0):plan.append(P(arm,.8,destination,angle,0.,'level empty press'))
        plan.append(P(arm,3.,destination,0,0.,'restore left wrist with both grips closed',45.))
        plan += [P(arm,3.,air,0,0.,'return empty press',45.),P(arm,3.,PRESS_HOME,0,0.,'set press on mat',45.),P(arm,.7,PRESS_HOME,0,FINGER_OPEN,'release press',45.),W(2.,(.16,.25 if arm=='left' else -.25,TOP+.17),FINGER_OPEN,'hands clear'),W(5.,(.16,.25 if arm=='left' else -.25,TOP+.17),FINGER_OPEN,'still hold')]
    return {'left':left,'right':right}
