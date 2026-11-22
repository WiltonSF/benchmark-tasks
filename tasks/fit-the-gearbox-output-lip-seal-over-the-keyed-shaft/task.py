"""A complete upright gearbox is held in a bench vise. A new radial lip seal rests on the yellow parts
stand. A blue protective sleeve and an aluminium annular driver stand on their labelled tool rests.
The white flat face of the seal is its outside; the recessed black spring face faces the lubricant.
"""
from pathlib import Path
import sys, math, json, os
import numpy as np
from dataclasses import replace
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import parts
import lots
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint, FINGER_OPEN
SCENE='station'; TOP=TABLE_TOP[SCENE]; MM=.001; EPS=1e-6
SUCCESS_HOLD_S=1.; SCORE_OVER='end'
# Maker PLE080: 80mm housing,20x40mm shaft,total133.5mm. The seal carrier is explicitly custom.
GX,GY=.290,.105
FACE=TOP+.0975
SHAFT_TOP=FACE+.040
SHAFT_KEY_BOTTOM=FACE+.015
# A rigid metal case with a visual lip; lip expansion/cutting is the declared material rule.
SEAL_R=.0175; CASE_RI=.012; SEAL_H=.007; LIP_Z=-.0018
SLEEVE_RI=.0102; SLEEVE_RO=.0106; SLEEVE_H=.051
SLEEVE_HOME_Z=FACE-.008+SLEEVE_H/2
DRIVER_H=.050; DRIVER_R=.019; DRIVER_RI=.0145
K=5000.; STOP_MASS=.05; START_FRONT=.0006; FINAL_FRONT=-.0070
MIN_DRIVE=12.; MAX_DRIVE=30.; COCK_LOAD=4.; DAMAGE_TIME=.15
DARK=(.15,.17,.19); STEEL=(.63,.65,.67); WHITE=(.92,.92,.87); BLUE=(.10,.35,.80)
GOLD=(.88,.69,.20); BLACK=(.035,.035,.04); RED=(.85,.12,.10)


def ring(ri,ro,z0,z1,color,n=24,collide=True):
    out=[]
    if not collide:
        # Draw the same convex wedges, but disable their colliders.
        from dataclasses import replace
    half=math.pi/n
    for i in range(n):
        a=2*math.pi*i/n
        pts=[(r*math.cos(b),r*math.sin(b),z) for z in (z0,z1) for b in (a-half,a+half) for r in (ri,ro)]
        faces=((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))
        s=parts.hull(pts,faces,color,friction=(.35,.28))
        if not collide:s=replace(s,collide=False)
        out.append(s)
    return out

def entry_chamfer():
    out=[];n=64
    for i in range(n):
        a=2*math.pi*i/n
        pts=[(r*math.cos(b),r*math.sin(b),z) for z,ri in ((-.001,.0177),(0.,.0182)) for b in (a-math.pi/n,a+math.pi/n) for r in (ri,.030)]
        out.append(parts.hull(pts,((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)),STEEL,friction=(.35,.28)))
    return out

# Keyway is an actual 6mm-wide,3mm-deep recess on the +x side of the exposed shaft.
def keyed_shaft():
    out=[parts.cylinder(.010,.023,(0,0,-.008+.023/2),STEEL)]
    # Disk minus an exact6mm-wide rectangular recess whose floor is x=7mm.
    a=math.acos(.7);b=math.asin(.3)
    core=[(.010*math.cos(t),.010*math.sin(t)) for t in np.linspace(a,2*math.pi-a,32)]
    upper=[(.010*math.cos(t),.010*math.sin(t)) for t in np.linspace(b,a,12)]+[(.007,.003)]
    lower=[(x,-y) for x,y in upper[::-1]]
    for poly in (core,upper,lower):
        n=len(poly);pts=[(x,y,z) for z in (.015,.040) for x,y in poly]
        faces=[tuple(range(n))[::-1],tuple(range(n,2*n))]+[(j,(j+1)%n,n+(j+1)%n,n+j) for j in range(n)]
        out.append(parts.hull(pts,faces,STEEL))
    return out

GEARBOX=parts.Part('gearbox',pos=(GX,GY,FACE),contact_offset=.00002,shapes=[
    # Complete motor adapter flange, enclosed gear housing, output spigot and fasteners.
    parts.box((.090,.090,.012),(0,0,-.0875),DARK),
    parts.cylinder(.040,.0695,(0,0,-.04675),DARK),
    *ring(.0177,.030,-.012,-.001,STEEL,n=64),*entry_chamfer(),
    *ring(.0115,.0177,-.012,-.011,STEEL),
    parts.cylinder(.011,.004,(0,0,-.010),STEEL),
    *keyed_shaft(),
    *[parts.cylinder(.0038,.0022,(.026*math.cos(a),.026*math.sin(a),.0011),BLACK) for a in np.arange(4)*math.pi/2+math.pi/4],
])
VISE=parts.Part('vise',pos=(GX,GY,TOP),shapes=[
    parts.box((.140,.140,.004),(0,0,.002),DARK),
    parts.box((.090,.016,.045),(0,.053,.0265),STEEL),
    parts.box((.090,.016,.045),(0,-.053,.0265),STEEL),
    parts.cylinder(.009,.060,(0,-.088,.025),STEEL,axis='y'),
    parts.cylinder(.003,.080,(0,-.118,.025),STEEL,axis='x'),
])
STOP=parts.Part('press_response',pos=(GX,GY,FACE),contact_offset=.0002,shapes=[
    parts.box((.010,.010,.004),(.035,.035,-.060),DARK,collide=False)],
    joints=(parts.Slider('seat',tuple(ring(.012,.0170,START_FRONT-.003,START_FRONT,STEEL,n=64)),axis=(0,0,-1),
                         travel=.011,mass=STOP_MASS,spring=K,damping=30,max_effort=100),))
SEAL_POS=(.185,.220,TOP+.0305)
SEAL=parts.Part('seal',pos=SEAL_POS,mass=.014,contact_offset=.00002,angular_damping=1.,shapes=[
    *ring(CASE_RI,SEAL_R,-SEAL_H/2,SEAL_H/2,BLACK,n=64),
    *ring(.010,CASE_RI,-.0025,-.001,BLACK,collide=False),
    *ring(.013,.0174,.0035,.00352,WHITE,collide=False),
    *ring(.0112,.0120,-.0024,-.0016,STEEL,collide=False),
])
SLEEVE_POS=(.170,.075,TOP+.004+SLEEVE_H/2)
def sleeve_shapes():
    # Real entry chamfer:0.3mm radial lead over the first2mm; entire protective tube remains rigid/free.
    out=[];n=32;profile=[(SLEEVE_RI+.0003,-SLEEVE_H/2),(SLEEVE_RI,-SLEEVE_H/2+.002),
                       (SLEEVE_RI,SLEEVE_H/2),(SLEEVE_RO,SLEEVE_H/2),(SLEEVE_RO,-SLEEVE_H/2)]
    for i in range(n):
        pts=[]
        for a in (2*math.pi*(i-.5)/n,2*math.pi*(i+.5)/n):
            pts.extend((r*math.cos(a),r*math.sin(a),z) for r,z in profile)
        m=len(profile);faces=[tuple(range(m))[::-1],tuple(range(m,2*m))]+[(j,(j+1)%m,m+(j+1)%m,m+j) for j in range(m)]
        out.append(parts.hull(pts,faces,BLUE,friction=(.15,.12)))
    return out
def sleeve_panels():
    # Primitive boxes preserve a thin wall without convex-mesh cooking. Max outer radial error13um.
    visual=[replace(v,collide=False) for v in sleeve_shapes()]
    out=[];n=64
    for i in range(n):
        angle=2*math.pi*i/n;q=(0.,0.,math.sin(angle/2),math.cos(angle/2))
        layers=[(SLEEVE_RI,-SLEEVE_H/2+.002,SLEEVE_H/2)]
        for j in range(4):layers.append((SLEEVE_RI+.0003*(1-(j+.5)/4),-SLEEVE_H/2+j*.0005,-SLEEVE_H/2+(j+1)*.0005))
        for ri,z0,z1 in layers:
            r=(ri+SLEEVE_RO)/2
            out.append(parts.box((SLEEVE_RO-ri,2*ri*math.tan(math.pi/n),z1-z0),
                (r*math.cos(angle),r*math.sin(angle),(z0+z1)/2),BLUE,quat=q,hidden=True,friction=(.15,.12)))
    return visual+out
SLEEVE=parts.Part('sleeve',pos=SLEEVE_POS,mass=.0104,contact_offset=.000005,angular_damping=1.,report=('base',),shapes=sleeve_panels())

def guide_lands():
    # Two real polymer inserts:24mm entry ->21.3mm guide bore; no simulated axis constraint.
    out=[];n=128
    for base in (.007,.031):
        profile=[(.012,base),(.01065,base+.003),(.01065,base+.007),(.0145,base+.007),(.0145,base)]
        for i in range(n):
            pts=[]
            for a in (2*math.pi*(i-.5)/n,2*math.pi*(i+.5)/n):
                pts.extend((r*math.cos(a),r*math.sin(a),z-DRIVER_H/2) for r,z in profile)
            m=len(profile);faces=[tuple(range(m))[::-1],tuple(range(m,2*m))]+[(j,(j+1)%m,m+(j+1)%m,m+j) for j in range(m)]
            out.append(parts.hull(pts,faces,DARK,friction=(.15,.12)))
    return out

DRIVER_POS=(.345,.225,TOP+.004+DRIVER_H/2)
DRIVER=parts.Part('driver',pos=DRIVER_POS,mass=.070,contact_offset=.000005,angular_damping=1.,report=('base',),shapes=[
    *ring(DRIVER_RI,DRIVER_R,-DRIVER_H/2,DRIVER_H/2-.008,STEEL,n=64),
    *ring(DRIVER_RI,DRIVER_R,DRIVER_H/2-.008,DRIVER_H/2,BLACK,n=64),*guide_lands(),
])
# Narrow riser leaves the 7mm case rim accessible, without a fabricated handle.
STANDS=parts.Part('stands',pos=(0,0,TOP),shapes=[
    parts.cylinder(.0135,.027,(SEAL_POS[0],SEAL_POS[1],.0135),GOLD),
    parts.box((.060,.060,.004),(SLEEVE_POS[0],SLEEVE_POS[1],.002),BLUE),
    parts.box((.060,.060,.004),(DRIVER_POS[0],DRIVER_POS[1],.002),DARK),
])
# Readable labels sit beside each object, without replacing functional geometry.
FONT={'S':['111','100','111','001','111'],'L':['100','100','100','100','111'],'E':['111','100','110','100','111'],
      'V':['101','101','101','101','010'],'D':['110','101','101','101','110'],'R':['110','101','110','101','101'],
      'I':['111','010','010','010','111'],'A':['010','101','111','101','101'],'O':['111','101','101','101','111'],
      'U':['101','101','101','101','111'],'T':['111','010','010','010','010'],' ':['000']*5}
def text_shapes(word,x,y):
    out=[];s=.003
    for k,c in enumerate(word):
        for row,line in enumerate(FONT[c]):
            for col,v in enumerate(line):
                if v=='1':out.append(parts.box((s*.8,s*.8,.0003),(x+(4-row)*s,y+(len(word)-1-k)*4*s+(2-col)*s,.0045),WHITE,collide=False))
    return out
_letters=[*text_shapes('SLEEVE',.125,.040),*text_shapes('DRIVER',.380,.190),*text_shapes('SEAL OUT',.145,.200)]
# One visual mesh rather than hundreds of independent letter cubes; conservative box covers all letters.
_vertices=[];_tri=[]
for c in _letters:
    base=len(_vertices);v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.array(c.size)/2+np.array(c.at)
    _vertices.extend(v.tolist())
    for q in ((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)):
        _tri.extend([(base+q[0],base+q[1],base+q[2]),(base+q[0],base+q[2],base+q[3])])
_lo=np.min(_vertices,axis=0)-np.array([.003,.003,0]);_hi=np.max(_vertices,axis=0)+np.array([.003,.003,0])
_platepoints=[];_platetri=[]
for word,x,y in [('SLEEVE',.125,.040),('DRIVER',.380,.190),('SEAL OUT',.145,.200)]:
    b=len(_platepoints);x0=x-.003;x1=x+.016;y0=y-.003;y1=y+len(word)*.012+.003
    _platepoints.extend([(x0,y0,.0042),(x1,y0,.0042),(x1,y1,.0042),(x0,y1,.0042)])
    _platetri.extend([(b,b+1,b+2),(b,b+2,b+3)])
LABELS=parts.Part('labels',pos=(0,0,TOP),shapes=[parts.box(tuple(_hi-_lo),tuple((_hi+_lo)/2),WHITE,hidden=True,collide=False)],
                  visuals=[parts.Visual('lettering',tuple(map(tuple,_vertices)),tuple(_tri),WHITE),parts.Visual('plates',tuple(_platepoints),tuple(_platetri),DARK)])

OBJECTS=[GEARBOX,VISE,STOP,SEAL,SLEEVE,DRIVER,STANDS,LABELS]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'seal':SEAL_POS,'sleeve':SLEEVE_POS,'driver':DRIVER_POS},yaw={'seal':0.,'sleeve':0.,'driver':0.}),
    lots.Lot('B',place={'seal':(.215,.220,SEAL_POS[2]),'sleeve':SLEEVE_POS,'driver':DRIVER_POS},yaw={'seal':math.pi/2,'sleeve':0.,'driver':0.}))
# A second slim riser is needed at B's published seal position.
from dataclasses import replace
STANDS=replace(STANDS,shapes=STANDS.shapes+(parts.cylinder(.0135,.027,(.215,.220,.0135),GOLD),))
OBJECTS[6]=STANDS
DEMO_TOUCHES={'gearbox','stands'}

def rot(st):return parts._quat_to_matrix(st.quat_xyzw)
def fresh():
    return dict(known=False,fail='',prev_t=None,front=START_FRONT,load=0.,loads=[],support_load=0.,support_loads=[],driver_load=0.,driver_loads=[],tool_load=0.,tool_loads=[],peak=0.,over_s=0.,bad_s=0.,
                passage=False,previous=None,previous_q=None,previous_sleeve=None,previous_sleeve_q=None,previous_hands=None,hold=0.,anchor=None,anchor_q=None,released=False,tools_home=False,depth=-1.,square=99.,e=99.,
                sleeve_cover=False,driver_gap=99.,drive_work=0.,max_drive_depth=0.,max_seen_depth=0.,retained=0.,stage=0.)
REC=fresh()

def annulus_dist(points,st,ri,ro,h):
    p=(np.asarray(points)-st.pos)@rot(st)
    radial=np.maximum(np.maximum(ri-np.hypot(p[:,0],p[:,1]),np.hypot(p[:,0],p[:,1])-ro),0)
    axial=np.maximum(np.abs(p[:,2])-h/2,0)
    return float(np.hypot(radial,axial).min()) if len(p) else np.inf

def driver_distance(points,st):
    # Distance to the outer tube plus both actual axisymmetric guide profiles.
    best=annulus_dist(points,st,DRIVER_RI,DRIVER_R,DRIVER_H)
    v=(np.asarray(points)-st.pos)@rot(st);p=np.column_stack((np.hypot(v[:,0],v[:,1]),v[:,2]))
    for base in (.007,.031):
        z=base-DRIVER_H/2;c=math.cos(math.pi/128)
        poly=np.array([(.012*c,z),(.01065*c,z+.003),(.01065*c,z+.007),(.0145,z+.007),(.0145,z)])
        distance=np.full(len(p),np.inf);cross=[]
        for a,b in zip(poly,np.roll(poly,-1,axis=0)):
            d=b-a;u=np.clip((p-a)@d/(d@d),0.,1.)
            distance=np.minimum(distance,np.linalg.norm(p-a-u[:,None]*d,axis=1))
            cross.append(d[0]*(p[:,1]-a[1])-d[1]*(p[:,0]-a[0]))
        cross=np.array(cross);inside=(cross>=-EPS).all(axis=0)|(cross<=EPS).all(axis=0)
        distance[inside]=0.;best=min(best,float(distance.min()))
    return best

def sleeve_covers(pos,q):
    axis=parts._quat_to_matrix(q)[:,2]
    rim=SLEEVE_RO*math.sqrt(max(0.,1-axis[2]**2))
    bottom=pos[2]-SLEEVE_H/2*abs(axis[2])+rim
    top=pos[2]+SLEEVE_H/2*abs(axis[2])-rim
    if abs(axis[2])<1e-8:return False
    centres=np.asarray(pos[:2])+(np.array([SHAFT_KEY_BOTTOM,SHAFT_TOP])-pos[2])[:,None]*axis[:2]/axis[2]
    clearance=bool(np.linalg.norm(centres-[GX,GY],axis=1).max()<=SLEEVE_RI-.010+EPS)
    return bool(clearance and abs(axis[2])>=math.cos(math.radians(2))
                and bottom<=SHAFT_KEY_BOTTOM-.001+EPS and top>=SHAFT_TOP+.001-EPS)

def lip_intrusion(points,seal,old_points=None,old_pos=None,old_q=None):
    # Continuous line segments between corresponding sampled hand surfaces in the seal frame.
    end=(np.asarray(points)-seal.pos)@rot(seal)
    start=end if old_points is None or len(old_points)!=len(points) or old_pos is None else (np.asarray(old_points)-old_pos)@parts._quat_to_matrix(old_q)
    delta=end-start;dz=delta[:,2];moving=np.abs(dz)>1e-12
    u0=np.zeros(len(points));u1=np.ones(len(points))
    v0=np.divide(-.0025-start[:,2],dz,out=np.zeros(len(points)),where=moving)
    v1=np.divide(-.001-start[:,2],dz,out=np.ones(len(points)),where=moving)
    u0[moving]=np.maximum(0.,np.minimum(v0,v1)[moving]);u1[moving]=np.minimum(1.,np.maximum(v0,v1)[moving])
    valid=(u0<=u1)&(np.maximum(start[:,2],end[:,2])>=-.0025-EPS)&(np.minimum(start[:,2],end[:,2])<=-.001+EPS)
    a=start[:,:2]+delta[:,:2]*u0[:,None];b=start[:,:2]+delta[:,:2]*u1[:,None];d=b-a
    w=np.clip(np.divide(-np.sum(a*d,axis=1),np.sum(d*d,axis=1),out=np.zeros(len(points)),where=np.sum(d*d,axis=1)>1e-20),0.,1.)
    rmin=np.linalg.norm(a+d*w[:,None],axis=1);rmax=np.maximum(np.linalg.norm(a,axis=1),np.linalg.norm(b,axis=1))
    return bool(np.any(valid&(rmin<=.012+EPS)&(rmax>=.010-EPS)))

def qlerp(a,b,u):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if np.dot(a,b)<0:b=-b
    q=a+(b-a)*u
    return q/np.linalg.norm(q)

# Nominal lip contact band:64 points on20mm circle,0.3mm axial width.
_LIP_POINTS=np.array([[.010*math.cos(a),.010*math.sin(a),LIP_Z] for a in np.arange(64)*2*math.pi/64])
def lip_hazard(pos,R):
    points=_LIP_POINTS@R.T+pos
    return bool(np.any((np.linalg.norm(points[:,:2]-[GX,GY],axis=1)<=.010+EPS)
                       &(points[:,2]>=SHAFT_KEY_BOTTOM-.00015-EPS)
                       &(points[:,2]<=SHAFT_TOP+.00015+EPS)))

def collar_intrusion(pos,R):
    points=_LIP_POINTS@R.T+pos
    return bool(np.any((np.linalg.norm(points[:,:2]-[GX,GY],axis=1)<=.011+EPS)
                       &(points[:,2]<=FACE-.008+.00015+EPS)))

def geometry(objects):
    s=objects['seal'];v=objects['sleeve'];d=objects['driver'];R=rot(s)
    h=float(s.pos[2]);zaxis=R[:,2]
    square=float(2*SEAL_R*np.sqrt(max(0.,1-zaxis[2]**2)))
    e=float(np.linalg.norm(np.asarray(s.pos[:2])-[GX,GY]))
    # Plane of the lip at its +/-10mm ring radius; swept range makes the shoulder crossing conservative.
    lip=h+LIP_Z*zaxis[2]
    vr=rot(v);va=vr[:,2]
    cover=sleeve_covers(np.asarray(v.pos),v.quat_xyzw)
    bottom=h-SEAL_H/2*abs(zaxis[2]);top=h+SEAL_H/2*abs(zaxis[2])
    driver_axis=rot(d)[:,2];driver_face=np.asarray(d.pos)-DRIVER_H/2*driver_axis*(1. if driver_axis[2]>=0 else -1.)
    driver_bottom=float(driver_face[2])
    driver_ok=bool(np.linalg.norm(driver_face[:2]-np.asarray(s.pos[:2]))<=.001+EPS
                   and abs(rot(d)[2,2])>=math.cos(math.radians(1)) and abs(driver_bottom-top)<=.0005+EPS)
    return dict(e=e,square=square,lip=lip,bottom=bottom,top=top,cover=cover,axis=float(zaxis[2]),
                driver_ok=driver_ok,driver_bottom=driver_bottom,driver_gap=driver_bottom-top,driver_face_e=float(np.linalg.norm(driver_face[:2]-s.pos[:2])),driver_tilt=math.degrees(math.acos(min(1.,abs(rot(d)[2,2])))),work_clear=bool(driver_bottom-DRIVER_R*math.sqrt(max(0.,1-driver_axis[2]**2))>FACE+.0002+EPS),depth=FACE-bottom)

def observe(objects,hands,t,q,driver_force=0.,tool_force=0.,hand_lip_contact=False):
    r=REC
    try:t=float(t)
    except (ValueError,TypeError):t=float('nan')
    if not np.isfinite(t):
        r['known']=False;r['fail']=r['fail'] or 'timestamp unavailable';r['hold']=0.;return
    if r['prev_t'] is not None and abs(t-r['prev_t'])<=1e-8:return
    dt=.05 if r['prev_t'] is None else t-r['prev_t']
    if dt<0 or dt>.050001 or (r['prev_t'] is None and (t<0 or t>.050001)):
        r['known']=False;r['fail']=r['fail'] or 'missed20Hz observation';r['hold']=0.;r['prev_t']=t;return
    r['prev_t']=t
    if hands is None or not all(n in hands for n in ('left','right')) or q is None or not all(n in objects for n in ('seal','sleeve','driver')):
        r['known']=False;r['fail']=r['fail'] or 'observation unavailable during trial';r['hold']=0.;return
    if not np.isfinite(q) or not np.isfinite(driver_force) or not np.isfinite(tool_force) or any(not np.isfinite(np.concatenate((objects[n].pos,objects[n].quat_xyzw))).all() for n in ('seal','sleeve','driver')):
        r['known']=False;r['fail']=r['fail'] or 'nonfinite part or force observation';r['hold']=0.;return
    r['known']=True
    r['drive_qualified']=False;r['work_delta']=0.
    if hand_lip_contact:r['fail']=r['fail'] or 'hand hull entered conservative lip envelope'
    r['tool_loads']=(r['tool_loads']+[max(0.,float(tool_force))])[-3:]
    r['tool_load']=float(np.mean(r['tool_loads']))
    r['driver_loads']=(r['driver_loads']+[max(0.,float(driver_force))])[-3:]
    r['driver_load']=float(np.mean(r['driver_loads']))
    g=geometry(objects);r.update(e=g['e'],square=g['square'],depth=g['depth'],sleeve_cover=g['cover'],driver_gap=g['driver_gap'],driver_face_e=g['driver_face_e'],driver_tilt=g['driver_tilt'],driver_ok=g['driver_ok'],stop_clear=g['work_clear'])
    s=objects['seal'];v=objects['sleeve'];d=objects['driver']
    load=max(0.,K*(q-(START_FRONT-r['front']))-STOP_MASS*9.81) if q>r['retained']-.00008 else 0.
    engaged=g['e']<=.001 and g['bottom']<=FACE+START_FRONT+.002 and g['bottom']>=FACE-.010 and abs(g['bottom']-(FACE+START_FRONT-q))<=.0015+EPS
    r.update(raw_support=float(load),raw_driver=float(driver_force),raw_tool=float(tool_force),spring_reference_front=r['front'],spring_reference_retained=r['retained'],support_engaged=bool(engaged))
    r['support_loads']=(r['support_loads']+[float(load if engaged else 0.)])[-3:]
    r['support_load']=float(np.mean(r['support_loads']))
    near_housing=g['e']<=.025 and FACE-.015<=g['bottom']<=FACE+.020
    r['load']=float(max(load,driver_force,tool_force) if near_housing else 0.);r['loads']=(r['loads']+[r['load']])[-3:]
    mean=float(np.mean(r['loads']));r['peak']=max(r['peak'],mean)
    # Swept lip passage catches skipping the discontinuity in a single control step.
    previous=r['previous'];prior_q=r['previous_q'];now=np.asarray(s.pos,float).copy()
    crossing=False
    if previous is not None:
        distance=max(np.linalg.norm(now-previous),np.linalg.norm(np.asarray(v.pos)-r['previous_sleeve']))
        angular=max(math.acos(min(1.,abs(float(np.dot(s.quat_xyzw,r['previous_q']))))),
                    math.acos(min(1.,abs(float(np.dot(v.quat_xyzw,r['previous_sleeve_q']))))))*2
        count=max(2,int(math.ceil(distance/.001))+1,int(math.ceil(angular/math.radians(1)))+1)
        for u in np.linspace(0,1,count):
            p=previous+(now-previous)*u;R=parts._quat_to_matrix(qlerp(r['previous_q'],s.quat_xyzw,u))
            on_hazard=lip_hazard(p,R)
            vp=r['previous_sleeve']+(np.asarray(v.pos)-r['previous_sleeve'])*u
            vq=qlerp(r['previous_sleeve_q'],v.quat_xyzw,u)
            if collar_intrusion(p,R) or (on_hazard and not sleeve_covers(vp,vq)):crossing=True;break
    if crossing:r['fail']=r['fail'] or 'lip crossed keyway or shaft shoulder without sleeve'
    r['previous']=now;r['previous_q']=np.asarray(s.quat_xyzw).copy()
    r['previous_sleeve']=np.asarray(v.pos).copy();r['previous_sleeve_q']=np.asarray(v.quat_xyzw).copy()
    if g['cover'] and g['lip']<SHAFT_KEY_BOTTOM-.003 and g['e']<=.001 and g['axis']>0:
        r['passage']=True
    # Removing sleeve only after the lip is below the entire keyway is allowed.
    if collar_intrusion(np.asarray(s.pos),rot(s)) or (lip_hazard(np.asarray(s.pos),rot(s)) and not g['cover']):
        r['fail']=r['fail'] or 'lip exposed at keyway or shoulder'
    r['over_s']=r['over_s']+dt if mean>MAX_DRIVE+EPS else 0.
    if r['over_s']>=DAMAGE_TIME-EPS:r['fail']=r['fail'] or 'applied axial load exceeded30N for150ms'
    wrong=near_housing and mean>=COCK_LOAD-EPS and (g['e']>.0005+EPS or g['square']>.0005+EPS or g['axis']<=0)
    r['bad_s']=r['bad_s']+dt if wrong else 0.
    if r['bad_s']>=DAMAGE_TIME-EPS:r['fail']=r['fail'] or 'loaded eccentric, cocked or backward insertion damaged seal'
    seen_before=r['max_seen_depth'];r['depth_before']=seen_before
    if g['e']<=.001+EPS and g['axis']>0:
        r['max_seen_depth']=max(seen_before,min(.007,max(0.,g['depth'])))
    # Physical spring reaction: current measured driver contact must persist; a trailing mean alone cannot qualify new travel.
    # The stop follows only while loaded12-30N, square, lip-in and protected.
    if engaged and g['driver_ok'] and driver_force>=MIN_DRIVE-EPS and r['driver_load']>=MIN_DRIVE-EPS and r['passage'] and not r['fail'] and g['axis']>0 and g['square']<=.0005+EPS and MIN_DRIVE-EPS<=r['support_load']<=MAX_DRIVE+EPS and r['driver_load']<=MAX_DRIVE+EPS and MIN_DRIVE-EPS<=r['tool_load']<=MAX_DRIVE+EPS:
        r['drive_qualified']=True
        advance=min(.00025,max(0.,r['support_load']-MIN_DRIVE)*.0004*dt)
        r['front']=max(FINAL_FRONT,r['front']-advance)
        r['retained']=max(r['retained'],min(START_FRONT-FINAL_FRONT,q))
        new_depth=min(.007,max(0.,g['depth']))
        if g['work_clear']:
            r['work_delta']=r['support_load']*max(0.,new_depth-seen_before)
            r['drive_work']+=r['work_delta']
        r['max_drive_depth']=max(r['max_drive_depth'],new_depth)
    # Sleeve no longer touches lip/shaft after removal; tools must end supported on rests, not held.
    clear=True
    for arm,pts in hands.items():
        if not len(pts) or not np.isfinite(pts).all():
            r['known']=False;r['fail']=r['fail'] or 'hand surface observation unavailable';clear=False;break
        if near_housing and mean>=COCK_LOAD-EPS and annulus_dist(pts,s,CASE_RI,SEAL_R,SEAL_H)<.005-EPS:
            r['fail']=r['fail'] or 'hand approached case during loaded seating'
        previous_pts=None if r['previous_hands'] is None else r['previous_hands'].get(arm)
        if lip_intrusion(pts,s,previous_pts,previous,prior_q):
            r['fail']=r['fail'] or 'hand surface swept through sealing lip'
        if driver_distance(pts,d)<.005-EPS:clear=False
        for st,ri,ro,h in ((s,CASE_RI,SEAL_R,SEAL_H),(v,SLEEVE_RI,SLEEVE_RO,SLEEVE_H),(d,DRIVER_RI,DRIVER_R,DRIVER_H)):
            if annulus_dist(pts,st,ri,ro,h)<.005-EPS:clear=False
    r['previous_hands']={a:np.asarray(p).copy() for a,p in hands.items()}
    r['released']=bool(clear)
    def home(st,p,h):return np.linalg.norm(np.asarray(st.pos[:2])-np.asarray(p[:2]))<=.015+EPS and abs(st.pos[2]-(TOP+.004+h/2))<=.001+EPS and abs(rot(st)[2,2])>=math.cos(math.radians(5))
    r['tools_home']=bool(home(v,SLEEVE_POS,SLEEVE_H) and home(d,DRIVER_POS,DRIVER_H))
    # All partial levels require actual released supported progress, no tool parking points.
    supported=bool(g['e']<=.0005+EPS and g['axis']>0 and g['square']<=.0005+EPS
                   and abs(g['bottom']-(FACE+START_FRONT-q))<=.001+EPS)
    still=all(objects[n].resting(.01,.2) for n in ('seal','sleeve','driver'))
    flush=bool(max(abs(g['top']-FACE)+g['square']/2,abs(g['top']-FACE))<=.0003+EPS)
    stage=0.
    if r['passage'] and supported:
        stage=.25
        if g['depth']>=.002-EPS and r['drive_work']>=.015-EPS:stage=.5
        if flush and r['drive_work']>=.060-EPS:stage=.75
    r['stage']=stage
    marks=np.concatenate([objects[n].pos for n in ('seal','sleeve','driver')])
    quats=np.array([objects[n].quat_xyzw for n in ('seal','sleeve','driver')])
    moved=True
    if r['anchor'] is not None:
        shifts=np.linalg.norm((marks-r['anchor']).reshape(-1,3),axis=1)
        sine=np.sqrt(np.maximum(0.,1.-np.sum(quats*r['anchor_q'],axis=1)**2))
        radii=np.array([math.hypot(SEAL_R,SEAL_H/2),math.hypot(SLEEVE_RO,SLEEVE_H/2),math.hypot(DRIVER_R,DRIVER_H/2)])
        moved=bool(np.max(shifts+2*radii*sine)>.0005+EPS)
    if not (stage>0 and clear and still and not r['fail']):r['anchor']=None;r['anchor_q']=None;r['hold']=0.
    elif moved:r['anchor']=marks.copy();r['anchor_q']=quats.copy();r['hold']=0.
    else:r['hold']+=dt

def score(objects):
    r=REC
    if not r['known'] or r['fail'] or not r['released'] or r['hold']<1.-EPS:return 0.
    if r['stage']==.75 and r['tools_home']:return 1.
    return r['stage']
def success(objects):return score(objects)==1.

# Lip volume grid has covering radius<0.9mm; swept pose steps have<=1mm relative point motion.
# Expanding each actual hand hull by1.5mm covers grid error plus nearest swept-pose error.
_LIP_VOLUME=np.array([[r*math.cos(a),r*math.sin(a),z] for r in (.010,.011,.012) for z in (-.0025,-.00175,-.001) for a in np.arange(64)*2*math.pi/64])
_HAND_HULLS=None
def hand_hulls():
    global _HAND_HULLS
    if _HAND_HULLS is None:
        from yam_arena import analysis
        from scipy.spatial import ConvexHull
        from types import SimpleNamespace
        _HAND_HULLS={}
        for name in ('gripper','tip_left','tip_right'):
            _HAND_HULLS[name]=[]
            for h in analysis.link_hulls()[name]:
                eq=ConvexHull(h.vertices).equations
                _HAND_HULLS[name].append(SimpleNamespace(vertices=h.vertices,planes=np.column_stack((eq[:,:3],-eq[:,3]))))
    return _HAND_HULLS

def swept_hand_lip(seal,poses,old_seal=None,old_poses=None):
    old_poses=old_poses or poses
    sp,sq=(seal.pos,seal.quat_xyzw) if old_seal is None else old_seal
    old_lip=_LIP_VOLUME@parts._quat_to_matrix(sq).T+sp
    new_lip=_LIP_VOLUME@rot(seal).T+seal.pos
    sl=2*math.acos(min(1.,abs(float(np.dot(sq,seal.quat_xyzw)))))
    lip_radius=float(np.linalg.norm(_LIP_VOLUME,axis=1).max());margin=.0015
    for key,(hp,hq) in poses.items():
        op,oq=old_poses.get(key,(hp,hq));ha=2*math.acos(min(1.,abs(float(np.dot(oq,hq)))))
        for hull in hand_hulls()[key[1]]:
            radius=float(np.linalg.norm(hull.vertices,axis=1).max())
            before=hull.vertices@parts._quat_to_matrix(oq).T+op;after=hull.vertices@parts._quat_to_matrix(hq).T+hp
            hlo=np.minimum(before.min(axis=0),after.min(axis=0))-2*radius*ha-margin
            hhi=np.maximum(before.max(axis=0),after.max(axis=0))+2*radius*ha+margin
            llo=np.minimum(old_lip.min(axis=0),new_lip.min(axis=0))-2*lip_radius*sl
            lhi=np.maximum(old_lip.max(axis=0),new_lip.max(axis=0))+2*lip_radius*sl
            if np.any(hhi<llo) or np.any(lhi<hlo):continue
            motion=np.linalg.norm(hp-op)+np.linalg.norm(seal.pos-sp)+2*(radius*ha+lip_radius*sl)
            count=max(1,int(math.ceil(motion/.001)))
            normals=hull.planes[:,:3];offsets=hull.planes[:,3];scale=np.linalg.norm(normals,axis=1)
            for u in np.linspace(0,1,count+1):
                lip=_LIP_VOLUME@parts._quat_to_matrix(qlerp(sq,seal.quat_xyzw,u)).T+sp+(seal.pos-sp)*u
                local=(lip-(op+(hp-op)*u))@parts._quat_to_matrix(qlerp(oq,hq,u))
                if np.any(np.all(local@normals.T-offsets<=margin*scale+EPS,axis=1)):return True
    return False

_FACE_CACHE={}
def face_points(name):
    if name in _FACE_CACHE:return _FACE_CACHE[name]
    from yam_arena import analysis
    pts=[]
    for h in analysis.link_hulls().get(name,[]):
        pts.append(h.vertices)
        for plane in np.unique(np.round(h.planes,7),axis=0):
            n,d=plane[:3],plane[3];on=h.vertices[np.abs(h.vertices@n-d)<1e-6]
            if len(on)<3:continue
            c=on.mean(axis=0);u=on[0]-c;u/=np.linalg.norm(u);v=np.cross(n,u)
            edge=on[np.argsort(np.arctan2((on-c)@v,(on-c)@u))]
            for a,b in zip(edge,np.roll(edge,-1,axis=0)):
                count=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                ii,jj=np.meshgrid(np.arange(count+1),np.arange(count+1));valid=ii+jj<=count
                pts.append(c+(a-c)*ii[valid,None]/count+(b-c)*jj[valid,None]/count)
    if not pts:raise ValueError('missing hand hull '+name)
    p=np.concatenate(pts);_,ix=np.unique(np.round(p/.003).astype(int),axis=0,return_index=True)
    _FACE_CACHE[name]=p[np.sort(ix)];return _FACE_CACHE[name]
def hand_clouds(sim,seal):
    global _HAND_HISTORY
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);out={};tcp={};hand_poses={}
    for arm in ('left','right'):
        a=base.scene[arm+'_robot'];poses=parts._np(a.data.body_link_pose_w)[0];pts=[]
        for n in ('gripper','tip_left','tip_right'):
            pose=poses[list(a.body_names).index(n)];R=parts._quat_to_matrix(pose[3:])
            hand_poses[(arm,n)]=(pose[:3]-origin,pose[3:].copy())
            pts.append(face_points(n)@R.T+pose[:3]-origin)
            if n=='gripper':tcp[arm]=(R@np.array([0,0,.134])+pose[:3]-origin).tolist()
        out[arm]=np.concatenate(pts)
    previous=None if REC['previous'] is None else (REC['previous'],REC['previous_q'])
    hit=swept_hand_lip(seal,hand_poses,previous,_HAND_HISTORY)
    _HAND_HISTORY=hand_poses
    return out,tcp,hit
_FORCE_VIEW=None
_HAND_FORCE_VIEW=None
_HAND_HISTORY={}
def evidence_path():
    if os.environ.get('TRACE_OUT'):return Path(os.environ['TRACE_OUT']).with_name('lipseal.jsonl')
    if os.environ.get('LIPSEAL_RECORD'):return Path(os.environ['LIPSEAL_RECORD'])
    return None

def set_retention(sim,minimum):
    import torch
    asset=sim.env.unwrapped.scene['press_response']
    ids=asset.find_joints(['seat'],preserve_order=True)[0]
    limits=asset.data.joint_pos_limits[:,ids].clone()
    limits[:,:,0]=max(0.,float(minimum)-.0001)
    asset.write_joint_position_limit_to_sim_index(limits=limits,joint_ids=ids,warn_limit_violation=False)

@parts.on_reset
def reset(sim):
    global REC,_FORCE_VIEW,_HAND_FORCE_VIEW,_HAND_HISTORY
    _HAND_HISTORY={}
    from isaaclab_physx.physics import PhysxManager
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdPhysics
    stage=get_current_stage()
    # Enrollment reset only: both lots start all rigid parts at their declared base poses.
    # lots.py then applies the visible seal-start difference and settles every placed part.
    import torch
    base=sim.env.unwrapped;origin=base.scene.env_origins
    origin=origin.torch if hasattr(origin,'torch') else origin
    enrolled=[]
    for spec in (SEAL,SLEEVE,DRIVER):
        asset=base.scene[spec.name];pose=asset.data.root_link_pose_w
        pose=(pose.torch if hasattr(pose,'torch') else pose).clone()
        pose[:,:3]=origin+torch.tensor(spec.pos,dtype=pose.dtype,device=pose.device)
        pose[:,3:]=torch.tensor([0.,0.,0.,1.],dtype=pose.dtype,device=pose.device)
        asset.write_root_pose_to_sim_index(root_pose=pose)
        vel=asset.data.default_root_vel;vel=vel.torch if hasattr(vel,'torch') else vel
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(vel))
        enrolled.append(dict(part=spec.name,base_pos=list(spec.pos),quat=[0.,0.,0.,1.]))
    paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName() in ('gripper','tip_left','tip_right') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(paths)!=6:raise ValueError('expected both palms and four fingertips for applied-force measurement: '+str(paths))
    _FORCE_VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        [parts._PATHS['driver']],filter_patterns=[[parts._PATHS['seal']]],max_contact_data_count=4096)
    _HAND_FORCE_VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        [parts._PATHS['driver']],filter_patterns=[paths],max_contact_data_count=4096)
    REC=fresh();set_retention(sim,0.);sim.set_target('press_response','seat',0.)
    record=evidence_path()
    if record is not None:
        record.parent.mkdir(parents=True,exist_ok=True)
        record.with_name('reset-'+lots.current().name+'.json').write_text(json.dumps(dict(scope='declared rigid-part enrollment before lot placement and settle; no mid-trial pose reset',parts=enrolled),indent=2))
        record.with_name('force-paths.json').write_text(json.dumps(dict(seal=parts._PATHS['seal'],hands=paths),indent=2))
        from isaaclab.sim.utils import get_current_stage
        from pxr import Usd, UsdGeom, UsdPhysics
        stage=get_current_stage();root=stage.GetPrimAtPath(parts._PATHS['sleeve']);values=[]
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdGeom.Mesh):values.append(dict(path=str(prim.GetPath()),minimum_thickness=prim.GetAttribute('physxConvexHullCollision:minThickness').Get(),collision_enabled=bool(prim.HasAPI(UsdPhysics.CollisionAPI))))
        if not values or any(v['minimum_thickness'] is None or abs(v['minimum_thickness']-.00005)>1e-8 for v in values):raise ValueError('thin sleeve cooking rule unavailable')
        if any(v['collision_enabled'] for v in values):raise ValueError('sleeve visual mesh unexpectedly collides')
        record.with_name('cooking.json').write_text(json.dumps(values,indent=2))
        collision_checks=[]
        for spec in (SEAL,SLEEVE,DRIVER):
            for i,shape in enumerate(spec.shapes):
                prim=stage.GetPrimAtPath(parts._PATHS[spec.name]+f'/s{i}')
                actual=bool(prim.HasAPI(UsdPhysics.CollisionAPI))
                collision_checks.append(dict(part=spec.name,index=i,expected=shape.collide,actual=actual))
                if actual!=shape.collide:raise ValueError('visual/physical collision mismatch '+spec.name+str(i))
        record.with_name('collisions.json').write_text(json.dumps(collision_checks,indent=2))
        from pxr import PhysxSchema
        solver=[]
        for spec in (SEAL,SLEEVE,DRIVER,STOP):
            for prim in Usd.PrimRange(stage.GetPrimAtPath(parts._PATHS[spec.name])):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    rb=PhysxSchema.PhysxRigidBodyAPI(prim)
                    counts=[rb.GetSolverPositionIterationCountAttr().Get(),rb.GetSolverVelocityIterationCountAttr().Get()]
                    if counts!=list(parts.SOLVER_ITERATIONS):raise ValueError('body solver settings unavailable '+str(prim.GetPath())+str(counts))
                    solver.append(dict(path=str(prim.GetPath()),kind='body',iterations=counts))
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    api=PhysxSchema.PhysxArticulationAPI(prim)
                    counts=[api.GetSolverPositionIterationCountAttr().Get(),api.GetSolverVelocityIterationCountAttr().Get()]
                    if counts!=list(parts.SOLVER_ITERATIONS):raise ValueError('articulation solver settings unavailable '+str(counts))
                    solver.append(dict(path=str(prim.GetPath()),kind='articulation',iterations=counts))
        record.with_name('solver.json').write_text(json.dumps(solver,indent=2))

@parts.on_step
def step(sim):
    try:
        objects=sim.states();hands,tcp,lip_hit=hand_clouds(sim,objects['seal'])
        q=objects['press_response'].joints['seat']
        forces=parts._np(_FORCE_VIEW.get_contact_force_matrix(dt=1/240)).reshape(-1,3)
        force=float(forces[0,2]);hand_forces=parts._np(_HAND_FORCE_VIEW.get_contact_force_matrix(dt=1/240)).reshape(-1,3)
        total=max(0.,-float(hand_forces[:,2].sum()))+(DRIVER.mass+SEAL.mass)*9.81
        observe(objects,hands,float(sim.t),float(q),force,total,lip_hit)
        sim.set_target('press_response','seat',START_FRONT-REC['front'])
        set_retention(sim,REC['retained'])
        record=evidence_path()
        if record is not None:
            row={k:v for k,v in REC.items() if k not in ('previous','previous_q','previous_sleeve','previous_sleeve_q','anchor','anchor_q','previous_hands','loads','support_loads','driver_loads','tool_loads')}
            row.update(t=float(sim.t),lot=lots.current().name,q=float(q),tcp=tcp,raw_case_contact_vector=forces[0].tolist(),raw_hand_contact_vectors=hand_forces.tolist(),velocities={n:dict(linear=objects[n].lin_vel.tolist(),angular=objects[n].ang_vel.tolist(),still=objects[n].resting(.01,.2)) for n in ('seal','sleeve','driver')},
                objects={n:dict(pos=objects[n].pos.tolist(),q=objects[n].quat_xyzw.tolist()) for n in ('seal','sleeve','driver')},score=score(objects))
            with record.open('a') as f:f.write(json.dumps(row)+'\n')

    except Exception as e:
        REC['known']=False;REC['fail']=REC['fail'] or repr(e);raise

# Open-loop demo from published visible layout, no exact-pose or force feedback.
def demo():
    w=[]
    def W(d,x,y,z,f,label):w.append(Waypoint(d,(float(x),float(y),float(z)),f,label))
    # First phase fits sleeve then clears the assembly completely.
    right=[]
    def R(d,x,y,z,f,label):right.append(Waypoint(d,(float(x),float(y),float(z)),f,label))
    OPEN=-.035;SLGRIP=-.008;SEGRIP=-.0145;DRGRIP=-.016
    sx,sy,sz=SLEEVE_POS
    grip_offset=0.0
    R(1.5,sx,sy,TOP+.13,OPEN,'over sleeve');R(1.5,sx,sy,sz+.020,OPEN,'descend sleeve')
    R(1.,sx,sy,sz+.020,SLGRIP,'grip sleeve');R(1.5,sx,sy,TOP+.192,SLGRIP,'lift sleeve')
    R(2.,GX-.00050,GY-.00039,FACE+.092,SLGRIP,'over shaft');R(2.,GX-.00050,GY-.00039,FACE+.092,SLGRIP,'settle above shaft');R(3.,GX-.00050,GY-.00039,FACE+.084,SLGRIP,'start sleeve shallow');R(1.5,GX-.00050,GY-.00039,FACE+.084,SLGRIP,'settle sleeve shallow');R(3.,GX-.00050,GY-.00039,FACE+.058,SLGRIP,'held sleeve engagement')
    R(1.,GX-.00050,GY-.00039,FACE+.058,OPEN,'release sleeve to gravity');R(1.5,GX,GY,TOP+.18,OPEN,'clear sleeve')
    R(1.5,.17,-.22,TOP+.15,OPEN,'park right');R(80.,.17,-.22,TOP+.15,OPEN,'wait')
    p=lots.current().pos('seal');px,py,pz=p
    W(13.,.17,.25,TOP+.14,OPEN,'wait for sleeve')
    W(1.5,px,py,TOP+.13,OPEN,'over seal');W(1.5,px,py,pz+.008,OPEN,'descend seal')
    W(1.,px,py,pz+.008,SEGRIP,'grip seal');W(1.5,px,py,TOP+.17,SEGRIP,'lift seal')
    W(2.,GX,GY,FACE+.064,SEGRIP,'above sleeve');W(3.,GX,GY,FACE+.016,SEGRIP,'protected slide')
    W(1.,GX,GY,FACE+.016,OPEN,'release seal');W(1.5,GX,GY,TOP+.18,OPEN,'clear seal')
    # Annular driver over sleeve; its continuous lower rim loads the steel case evenly.
    dx,dy,dz=DRIVER_POS
    W(1.5,dx,dy,TOP+.13,OPEN,'over driver');W(1.5,dx,dy,dz+.012,OPEN,'descend driver')
    W(1.,dx,dy,dz+.012,DRGRIP,'grip driver');W(1.5,dx,dy,TOP+.18,DRGRIP,'lift driver')
    # Measured free-hover bias plus a stopped entry above the first pilot land.
    W(2.,GX+.0003,GY-.0005,FACE+.090,DRGRIP,'over seal');W(1.,GX+.0003,GY-.0005,FACE+.090,DRGRIP,'settle driver')
    W(2.,GX+.0003,GY-.0005,FACE+.076,DRGRIP,'above pilot entry')
    W(1.5,GX+.0003,GY-.0005,FACE+.076,DRGRIP,'settle pilot entry')
    W(2.5,GX-.0004,GY,FACE+.043,DRGRIP,'driver onto case');W(1.,GX-.0004,GY,FACE+.043,OPEN,'release driver')
    # Positive axial contact: finger end faces push the driver's upper rim; side friction carries only the lift.
    W(1.5,GX,GY,FACE+.095,OPEN,'clear above driver');W(1.,GX,GY,FACE+.095,-.016,'set press fingers')
    W(3.,GX,GY,FACE+.071,-.016,'finger ends onto upper rim')
    W(10.,GX,GY,FACE+.008,-.016,'drive square');W(4.,GX,GY,FACE+.0075,-.016,'finish drive')
    W(1.5,GX,GY,FACE+.100,-.016,'lift press fingers');W(1.,GX,GY,FACE+.100,OPEN,'open for retrieval')
    W(1.5,GX,GY,FACE+.037,OPEN,'descend driver retrieval');W(1.,GX,GY,FACE+.037,DRGRIP,'grip driver retrieval')
    W(1.5,GX,GY,TOP+.19,DRGRIP,'lift driver');W(1.5,dx,dy,TOP+.17,DRGRIP,'return driver')
    W(1.5,dx,dy,dz+.012,DRGRIP,'lower driver');W(1.,dx,dy,dz+.012,OPEN,'release driver')
    W(2.,.17,.25,TOP+.15,OPEN,'clear assembly');W(20.,.17,.25,TOP+.15,OPEN,'wait sleeve removal')
    # Final phase returns the sleeve after lifting the driver; all phases use the left arm.
    right=right[:-1]
    R(48.,.17,-.22,TOP+.15,OPEN,'wait for driver')
    R(2.,GX,GY,TOP+.18,OPEN,'over sleeve return');R(2.,GX,GY,SLEEVE_HOME_Z+.016,OPEN,'descend sleeve return')
    R(1.,GX,GY,SLEEVE_HOME_Z+.016,SLGRIP,'grip sleeve return');R(2.,GX,GY,TOP+.19,SLGRIP,'remove sleeve')
    R(2.,sx,sy,TOP+.17,SLGRIP,'return sleeve');R(2.,sx,sy,sz+.016,SLGRIP,'lower sleeve')
    R(1.,sx,sy,sz+.016,OPEN,'release sleeve');R(2.,.17,.25,TOP+.15,OPEN,'clear');R(1.,.17,.25,TOP+.15,OPEN,'still')
    plan=right[:next(i for i,v in enumerate(right) if v.label=='park right')]+w[1:-1]+right[next(i for i,v in enumerate(right) if v.label=='over sleeve return'):]
    for wp in plan:
        if wp.label in ('set press fingers','finger ends onto upper rim','drive square','finish drive','lift press fingers'):wp.yaw=math.radians(80)
    return {'left':plan}
