"""A lab counter beside a switched-off Gilson SS-3 sieve shaker. A Gilson V3SF No. 16 test sieve (76 mm stainless
frame, 1.18 mm openings) lies upside down with 20 near-size limestone grains wedged in its wire openings. Its empty
76 mm stainless pan stands upright nearby, and a Gilson TSA-170 fine sieve brush (wood handle, natural bristles)
stands on its bristles.
"""
import json,math,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts
import lots,tilt,observation
from residue import Residue,SITES,PITCH,WIRE,RADIUS,tip_flex
from yam_arena.task_file import TABLE_TOP
from yam_arena.station import TABLE_CENTER,TABLE_SIZE
from yam_arena.demo import Waypoint,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=.1
SILVER=(.68,.71,.75);DARK=(.20,.22,.25);WOOD=(.56,.32,.13);BRISTLE=(.72,.62,.43)

def ring(radius,height,z,wire=.0006,n=32):
    return [parts.box((2*radius*math.tan(math.pi/n),2*wire,height),(radius*math.cos(2*math.pi*i/n),radius*math.sin(2*math.pi*i/n),z),SILVER,yaw=2*math.pi*i/n+math.pi/2) for i in range(n)]
# Sieve frame origin at its centre; 0.63 mm wires at 1.81 mm pitch (1.18 mm openings), 15 mm below the centre.
mesh=[]
for a in (0,1):
    for i in range(-19,20):
        u=(i+.5)*PITCH
        length=2*math.sqrt(.0358**2-u*u) if abs(u)<.0358 else 0
        if length<=0:continue
        mesh.append(parts.cylinder(WIRE,length,(0,u,-.015) if a==0 else (u,0,-.015),SILVER,'x' if a==0 else 'y'))
SIEVE=parts.Part('sieve',pos=(.26,.13,TOP+.022225),mass=.09,pitch=math.pi,friction=(.45,.35),shapes=ring(.0374,.04445,0)+mesh)
PAN=parts.Part('pan',pos=(.25,-.03,TOP),mass=.10,friction=(.45,.35),shapes=ring(.0374,.04445,.022225)+[parts.cylinder(.038,.001,(0,0,.0005),SILVER)])
# Gilson TSA-170: 146.1 mm long, 31.75 mm bristle head on an 8 mm spring slide; 16 mm handle and 22 mm ferrule are estimates.
BRUSH=parts.Part('brush',pos=(.30,-.19,TOP+.0961),mass=.032,friction=(.5,.4),shapes=[parts.cylinder(.008,.100,(0,0,0),WOOD),parts.cylinder(.011,.020,(0,0,-.060),SILVER)],joints=[parts.Slider('bristles',shapes=(parts.cylinder(.015875,.0261,(0,0,-.08305),BRISTLE),),axis=(0,0,1),travel=.008,mass=.008,spring=500,damping=1.,max_effort=8.)])
# Gilson SS-3 sieve shaker, switched off: 203 x 279 x 559 mm outline with its rods, cover clamp and control panel.
MACHINE=parts.Part('ss3_complete_off',pos=(.57,.24,TOP),shapes=[parts.box((.203,.279,.070),(0,0,.035),DARK),parts.box((.150,.160,.015),(0,0,.078),SILVER),parts.cylinder(.008,.45,(-.075,0,.325),SILVER),parts.cylinder(.008,.45,(.075,0,.325),SILVER),parts.box((.188,.040,.025),(0,0,.5465),DARK),parts.cylinder(.047,.008,(0,0,.515),SILVER),parts.box((.024,.008,.020),(-.045,-.1355,.037),(.10,.10,.10)),parts.box((.04,.008,.022),(.025,-.1355,.037),(.13,.15,.14))],friction=(.4,.3))
SHELF=parts.Part('lab_counter_shelf',pos=(.63,.0,TOP),shapes=[parts.box((.020,.76,.60),(.15,0,.30),(.72,.73,.74)),parts.box((.25,.76,.020),(.035,0,.59),(.65,.65,.65))])
OBJECTS=[SIEVE,PAN,BRUSH,MACHINE,SHELF]
# Lot B moves the sieve and pan; the brush, a jointed part, starts at the same place in both lots.
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={PAN.name:PAN.pos}),lots.Lot('B',place={PAN.name:(.27,-.05,TOP),SIEVE.name:(.22,.17,TOP+.022225)}))
def _hard_points():
    # Wood handle (16 mm, 100 mm) and steel ferrule (22 mm, 20 mm) surfaces in the brush frame; the bristle head is soft.
    pts=[];a=np.linspace(0,2*np.pi,24,endpoint=False)
    for r0,z0,z1 in ((.011,-.070,-.050),(.008,-.050,.050)):
        for z in np.linspace(z0,z1,int(round((z1-z0)/.003))+1):pts+=[(r0*math.cos(t),r0*math.sin(t),z) for t in a]
        for rr in (0.,r0/2):pts+=[(rr*math.cos(t),rr*math.sin(t),z) for t in a for z in (z0,z1)]
    return np.asarray(pts)
HARD=_hard_points()
def hard_contact(brush,mesh,r):
    hard=(HARD@parts._quat_to_matrix(brush.quat_xyzw).T+np.asarray(brush.pos)-mesh)@r;near=(np.hypot(hard[:,0],hard[:,1])<RADIUS)&(np.abs(hard[:,2])<.02)
    return bool(near.any() and (np.abs(hard[near,2]).min()<WIRE+.0005 or hard[near,2].max()>0>hard[near,2].min()))
_model=Residue();_known=False;_bad=False;_hold=0.;_finish_hold=0.;_time=0.;_cert=None;_previous_eligible=False;_visual=[];_damage_attrs=[];_damage_shown=False;_soft_tip=None;_prev_pose=None

def signature(objects):
    return tuple(float(v) for name in ('sieve','pan','brush') for v in np.r_[objects[name].pos,objects[name].quat_xyzw,objects[name].lin_vel,objects[name].ang_vel])
def turn_angle(a,b):
    d=a.T@b;v=np.array((d[2,1]-d[1,2],d[0,2]-d[2,0],d[1,0]-d[0,1]))/2
    return float(math.atan2(np.linalg.norm(v),(np.trace(d)-1)/2))
def frame(s):
    r=parts._quat_to_matrix(s.quat_xyzw);return np.asarray(s.pos)+r@np.array((0,0,-.015)),r

def clear_hands(clouds,objects):
    if not clouds or any(p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all() for p in clouds):return False
    # Conservative filled cylinders enclose all scored physical surfaces.
    for name,radius,zlo,zhi in [('sieve',.038,-.022225,.022225),('pan',.038,0,.04445)]:
        s=objects[name];rot=parts._quat_to_matrix(s.quat_xyzw)
        for cloud in clouds:
            q=(cloud-s.pos)@rot
            d=np.hypot(np.maximum(np.linalg.norm(q[:,:2],axis=1)-radius,0),np.maximum(np.maximum(zlo-q[:,2],q[:,2]-zhi),0))
            if d.min()<=.030:return False
    s=objects['brush'];rot=parts._quat_to_matrix(s.quat_xyzw)
    for cloud in clouds:
        q=(cloud-s.pos)@rot
        d=np.hypot(np.maximum(np.linalg.norm(q[:,:2],axis=1)-.015875,0),np.maximum(np.maximum(-.0961-q[:,2],q[:,2]-.05),0))
        if d.min()<=.030:return False
    return True

@parts.on_reset
def reset(sim):
    global _model,_known,_bad,_hold,_finish_hold,_time,_cert,_previous_eligible,_visual,_damage_attrs,_damage_shown,_soft_tip,_prev_pose
    _model=Residue();_known=False;_bad=False;_hold=0.;_finish_hold=0.;_time=float(sim.t);_cert=None;_previous_eligible=False;_visual=[];_damage_attrs=[];_damage_shown=False;_soft_tip=None;_prev_pose=None
    states=sim.states();m,r=frame(states['sieve']);_model.initialize(m,r)
    # A declared granular material solver owns these visual grains; no tiny sprung rigid bodies.
    from pxr import UsdGeom,Gf
    from isaaclab.sim.utils import get_current_stage
    stage=get_current_stage();path=sim.env.unwrapped.scene.env_prim_paths[0]+'/sample_grains'
    origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
    for i,rad in enumerate(__import__('residue').RADII):
        geom=UsdGeom.Sphere.Define(stage,path+'/grain_'+str(i));geom.CreateRadiusAttr(float(rad));geom.CreateDisplayColorAttr([Gf.Vec3f(.58,.53,.44)])
        ops=geom.GetOrderedXformOps();op=ops[0] if ops else geom.AddTranslateOp();op.Set(Gf.Vec3d(*(_model.pos[i]+origin)));_visual.append(op)
    soft=UsdGeom.Cylinder.Define(stage,sim.env.unwrapped.scene.env_prim_paths[0]+'/soft_brush_tip')
    soft.CreateDisplayColorAttr([Gf.Vec3f(*BRISTLE)])
    ops=soft.GetOrderedXformOps()
    translate=ops[0] if ops else soft.AddTranslateOp()
    orient=ops[1] if len(ops)>1 else soft.AddOrientOp()
    _soft_tip=(translate,orient,soft.CreateRadiusAttr(.015875),soft.CreateHeightAttr(.0261))
    bs=states['brush'];br=parts._quat_to_matrix(bs.quat_xyzw);load=max(0.,500*bs.joints['bristles']);flex=tip_flex(load);axis=-br[:,2]
    tip=bs.pos+br@np.array((0,0,-.0961+bs.joints['bristles']));c=tip-axis*(.0261-flex)/2+origin;q=bs.quat_xyzw
    translate.Set(Gf.Vec3d(*map(float,c)));orient.Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
    _soft_tip[2].Set(float(.015875+flex));_soft_tip[3].Set(float(.0261+flex))
    from yam_arena.station import linear_rgb
    for prim in stage.Traverse():
        if '/sieve/' in str(prim.GetPath()).lower():
            attr=prim.GetAttribute('inputs:diffuseColor')
            if attr:
                attr.Set(Gf.Vec3f(*map(float,linear_rgb(SILVER))));_damage_attrs.append(attr)


@parts.on_step
def step(sim):
    global _known,_bad,_hold,_finish_hold,_time,_cert,_previous_eligible,_damage_shown,_prev_pose
    try:
        st=sim.states();dt=float(sim.t)-_time
        if not 0<dt<=.100001:raise ValueError('nonmonotonic time')
        for name in ('sieve','pan','brush'):
            s=st[name]
            if not np.isfinite(np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]).all() or np.linalg.norm(s.quat_xyzw)<1e-8:raise ValueError('unknown object state')
            if s.low[2]<TOP-.002 or any(s.low[a]<TABLE_CENTER[a]-TABLE_SIZE[a]/2 or s.high[a]>TABLE_CENTER[a]+TABLE_SIZE[a]/2 for a in (0,1)):_bad=True
        mesh,r=frame(st['sieve']);brush=st['brush'];br=parts._quat_to_matrix(brush.quat_xyzw)
        compression=float(brush.joints['bristles'])
        if not math.isfinite(compression):raise ValueError('unknown bristle compression')
        tip=brush.pos+br@np.array((0,0,-.0961+compression));load=max(0.,500*compression)
        pr=parts._quat_to_matrix(st['pan'].quat_xyzw)
        _model.advance(dt,mesh,r,tip,-br[:,2],load,np.asarray(st['pan'].pos),pr)
        clouds=observation.hand_state(sim);_known=True
        clear=clear_hands(clouds,st)
        # Hard tool/finger entry through wire plane is a declared damage approximation.
        for c in clouds:
            local=(c-mesh)@r
            inside=(np.linalg.norm(local[:,:2],axis=1)<.035)&(np.abs(local[:,2])<.0004)
            if inside.any():_model.damaged=True
        # Only the soft bristle head may touch the wires: the wood handle or steel ferrule at the wire plane is damage.
        if hard_contact(brush,mesh,r):_model.damaged=True
        flat=abs(float(r[2,2]))>=math.cos(math.radians(10))-1e-9
        supported=abs(float(st['sieve'].low[2])-TOP)<=.002 and abs(float(st['brush'].low[2])-TOP)<=.002 and abs(float(st['pan'].low[2])-TOP)<=.002
        # Stillness from pose change between control steps (as a video shows it): PhysX reports velocity jitter of a
        # few mm/s for a brush standing at rest on its bristles while its pose does not change.
        poses={n:(np.asarray(st[n].pos,float).copy(),parts._quat_to_matrix(st[n].quat_xyzw)) for n in ('sieve','pan','brush')}
        still=_prev_pose is not None and all(np.linalg.norm(poses[n][0]-_prev_pose[n][0])/dt<.003 and turn_angle(_prev_pose[n][1],poses[n][1])/dt<.03 for n in poses) and bool(np.all(np.linalg.norm(_model.vel,axis=1)<.003))
        _prev_pose=poses
        eligible=clear and supported and still and flat
        _hold=_hold+dt if eligible and _previous_eligible else 0.;_finish_hold=_hold;_previous_eligible=eligible
        _cert=signature(st);_time=float(sim.t)
        from pxr import Gf
        origin=parts._np(sim.env.unwrapped.scene.env_origins[0])
        if _model.damaged and not _damage_shown:
            for attr in _damage_attrs:attr.Set(Gf.Vec3f(1.,.015,.005))
            _damage_shown=True
        for i,op in enumerate(_visual):op.Set(Gf.Vec3d(*(_model.pos[i]+origin)))
        if _soft_tip is not None:
            trans,orient,radius,height=_soft_tip;flex=tip_flex(load);axis=-br[:,2]
            centre=tip-axis*(.0261-flex)/2+origin;q=brush.quat_xyzw
            trans.Set(Gf.Vec3d(*map(float,centre)));orient.Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
            radius.Set(float(.015875+flex));height.Set(float(.0261+flex))

        if os.environ.get('TRACE_OUT'):
            row=dict(speeds={n:[round(float(np.linalg.norm(st[n].lin_vel)),5),round(float(np.linalg.norm(st[n].ang_vel)),5)] for n in ('sieve','pan','brush')},flags=[bool(clear),bool(supported),bool(still),bool(flat)],t=_time,retained=int(_model.retained.sum()),counts=_model.counts(mesh,r,np.asarray(st['pan'].pos),pr),spring=compression,load=load,contacts=_model.contact_sites,clear=clear,hold=_hold,bad=_bad,damaged=_model.damaged,damage_visual_available=bool(_damage_attrs),grains=_model.pos.tolist(),score=score(st))
            from yam_arena.constants import TCP_OFFSET
            base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);row['tcp']={}
            for arm in ('left_robot','right_robot'):
                robot=base.scene[arm];pose=parts._np(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
                row['tcp'][arm]=(pose[:3]-origin+parts._quat_to_matrix(pose[3:])@np.array(TCP_OFFSET)).tolist()
            with pathlib.Path(os.environ['TRACE_OUT']).with_name('sieve.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    except (ValueError,KeyError,AttributeError,IndexError,TypeError) as exc:
        _known=False;_bad=True
        if os.environ.get('TRACE_OUT'):
            with pathlib.Path(os.environ['TRACE_OUT']).with_name('sieve-errors.txt').open('a') as f:f.write(repr(exc)+'\n')

def visible_grains(objects,points,radii):
    # Conservative overhead macro witness: a centre-top ray must clear every physical part.
    origins=np.asarray(points)+np.array((0,0,1.))*np.asarray(radii)[:,None]+np.array((0,0,.00001))
    visible=np.ones(len(origins),dtype=bool)
    for spec in OBJECTS:
        st=objects[spec.name];poses={'base':(np.asarray(st.pos),parts._quat_to_matrix(st.quat_xyzw))}
        for name,(p,q) in getattr(st,'links',{}).items():poses[name]=(np.asarray(p),parts._quat_to_matrix(q))
        boxes=spec.world_boxes(poses)
        if spec.name=='brush':
            compression=float(st.joints['bristles'])
            if not math.isfinite(compression):return np.zeros(len(origins),dtype=bool)
            rot=poses['base'][1];axis=-rot[:,2];flex=tip_flex(max(0.,500*compression))
            tip=st.pos+rot@np.array((0,0,-.0961+compression));centre=tip-axis*(.0261-flex)/2
            boxes.append(('fine_tips',centre,rot,np.array((2*(.015875+flex),2*(.015875+flex),.0261+flex))))
        for _,centre,rotation,size in boxes:
            if spec.name=='brush':
                near=(np.asarray(points)-centre)@rotation
                distance=np.linalg.norm(np.maximum(np.abs(near)-size/2,0.),axis=1)
                visible &= distance>np.asarray(radii)+.00001
            local=(origins-centre)@rotation;direction=np.array((0.,0.,1.))@rotation
            lo=np.full(len(origins),-np.inf);hi=np.full(len(origins),np.inf);possible=np.ones(len(origins),dtype=bool)
            for axis in range(3):
                if abs(direction[axis])<1e-12:possible &= np.abs(local[:,axis])<=size[axis]/2
                else:
                    a=(-size[axis]/2-local[:,axis])/direction[axis];b=(size[axis]/2-local[:,axis])/direction[axis]
                    lo=np.maximum(lo,np.minimum(a,b));hi=np.minimum(hi,np.maximum(a,b))
            visible &= ~(possible&(hi>=np.maximum(lo,.00001)))
    for i in range(len(origins)):
        for j in range(len(origins)):
            if i==j:continue
            radial=np.linalg.norm(origins[i,:2]-points[j,:2])
            if radial<radii[j] and points[j,2]+math.sqrt(radii[j]**2-radial**2)>origins[i,2]:visible[i]=False
    return visible

def endpoint(collected,retained,reblocked,unaccounted,invalid,known,released_still,finished=True):
    if not known or invalid or unaccounted or reblocked or not released_still:return 0.
    if collected>=20 and retained==0:return 1.
    if collected>=18:return .75
    if collected>=15:return .5
    if collected>=10:return .25
    return 0.
def score(objects):
    try:
        if not _known or _bad or _model.bad or _model.damaged or _hold<1.-1e-9 or _cert!=signature(objects):return 0.
        mesh,r=frame(objects['sieve']);pan=objects['pan'];pr=parts._quat_to_matrix(pan.quat_xyzw);cm,bm,em=_model.classify(mesh,r,np.asarray(pan.pos),pr)
        from residue import RADII
        if cm.any() and np.any(cm & ~visible_grains(objects,_model.pos,RADII)):return 0.
        c,b,e=(int(a.sum()) for a in (cm,bm,em))
        return endpoint(c,int(_model.retained.sum()),b,e,False,True,_hold>=1.-1e-9)
    except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

def demo():
    # Fixed public layout positions and native dimensions only; no hidden state or progress branches.
    pan=np.asarray(lots.current().pos('pan'));brush=np.asarray(lots.current().pos('brush'));sieve=np.asarray(lots.current().pos('sieve'))
    sign=1 if sieve[1]>0 else -1;support='left' if sign>0 else 'right';tool='right' if sign>0 else 'left'
    heading=-sign*math.radians(60)
    def w(t,p,f,label,yaw=0.):return Waypoint(t,tuple(map(float,p)),f,label,yaw)
    # Support hand: fingers either side of the inverted frame across its 76 mm diameter, hand leaning so the palm
    # stays outside the frame and the mesh is open from above.
    def sw(t,p,f,label):return tilt.Waypoint(t,tuple(map(float,p)),f,label,yaw=heading,tilt=65.,heading=heading)
    grasp=.025;squeeze=-.033;carry=TOP+.15;rim=TOP+.04445+.0015
    pick=np.array((sieve[0],sieve[1],TOP+grasp));over=np.array((pan[0],pan[1],rim+grasp))
    park=np.array((.14,sign*.27,TOP+.12));tool_park=np.array((.14,-sign*.27,TOP+.12))
    # Brush hand: straight-down grip on the wood handle 14.5 mm below its centre (palm 6 mm above the handle end),
    # bristle face flat on the mesh, fingers closing along y so strokes along y cannot roll the handle in the grip.
    # The frame rests on the pan rim, so the wires are 81.7 mm up. The hand is commanded 4.5 mm past first touch;
    # the arm yields about 30% of that (measured), leaving about 3 mm of bristle compression (about 1.5 N).
    yaw=0.
    u=np.array((0.,1.,0.));v=np.array((1.,0.,0.))  # strokes along y; rows 7 to 8 mm apart along x
    below=.0145;hold=np.array((brush[0],brush[1],brush[2]-.0006-below))
    mesh=TOP+.04445+.037225;touch=mesh+.000315+.0961-below;press=.0045;contact=touch-press;lift=touch+.008;high=TOP+.18
    centre=np.array((pan[0],pan[1],0.));wait=np.array((pan[0],pan[1]-sign*.11,high))
    rows=[(-.011,-1,.0115),(-.004,1,.014),(.004,-1,.014),(.011,1,.0115)]  # x, start side, half stroke
    def at(x,d,z,half):p=centre+v*x+u*d*half;return (p[0],p[1],z)
    bp=[w(2.5,hold+(0,0,.07),FINGER_OPEN,'above standing brush'),w(2,hold,FINGER_OPEN,'fingers around wood handle'),w(1,hold,0.,'close on handle'),w(2,(hold[0],hold[1],high),0.,'lift brush'),w(2.5,wait,0.,'wait beside pan',yaw)]
    t_tool=sum(x.duration for x in bp);t_place=2.5+2+1+2+3+2.5
    bp.append(w(max(.5,t_place+.5-t_tool),wait,0.,'wait for sieve over pan',yaw))
    x0,d0,h0=rows[0];bp.append(w(2.5,at(x0,d0,high,h0),0.,'above first row',yaw))
    for x,d,h in rows:
        bp += [w(1.5,at(x,d,lift,h),0.,'lower toward mesh',yaw),w(1.5,at(x,d,contact,h),0.,'bristles pressed on mesh',yaw),w(4,at(x,-d,contact,h),0.,'stroke across openings',yaw),w(1,at(x,-d,lift,h),0.,'lift bristles',yaw)]
    bp += [w(1.5,at(rows[-1][0],-rows[-1][1],high,rows[-1][2]),0.,'brush clear above sieve',yaw)]
    bp += [w(3,(hold[0],hold[1],high),0.,'brush back over its place'),w(2.5,hold+(0,0,.0005),0.,'stand brush on bristles'),w(1.5,hold+(0,0,.0005),FINGER_OPEN,'release brush'),w(2.5,(hold[0],hold[1],high),FINGER_OPEN,'fingers up past handle'),w(2.5,tool_park,FINGER_OPEN,'brush hand clear')]
    t_away=sum(x.duration for x in bp[:-4])  # brush over its own place, well clear of the sieve
    sp=[sw(2.5,pick+(0,0,.07),FINGER_OPEN,'above inverted sieve'),sw(2,pick,FINGER_OPEN,'fingers either side of frame'),sw(1,pick,squeeze,'close across frame'),sw(2,(pick[0],pick[1],carry),squeeze,'lift sieve'),sw(3,(over[0],over[1],carry),squeeze,'carry over pan'),sw(2.5,over,squeeze,'frame rim onto pan rim')]
    sp += [sw(max(.5,t_away-t_place),over,squeeze,'hold sieve over pan while brushing'),sw(2,(over[0],over[1],carry),squeeze,'lift sieve off pan'),sw(3,(pick[0],pick[1],carry),squeeze,'carry sieve back'),sw(2.5,pick+(0,0,.002),squeeze,'set sieve down'),sw(1,pick+(0,0,.002),FINGER_OPEN,'open fingers'),sw(2,pick+(0,0,.09),FINGER_OPEN,'open fingers rise clear of frame'),w(2.5,park,FINGER_OPEN,'support hand clear')]
    end=max(sum(x.duration for x in sp),sum(x.duration for x in bp))+8
    sp.append(w(max(.5,end-sum(x.duration for x in sp)),park,FINGER_OPEN,'remain clear'));bp.append(w(max(.5,end-sum(x.duration for x in bp)),tool_park,FINGER_OPEN,'remain clear'))
    return {support:sp,tool:bp}
