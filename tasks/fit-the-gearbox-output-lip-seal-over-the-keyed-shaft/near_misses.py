"""Scorer controls replay physical observations, including time, hand surfaces and spring deflection.
The factory CASES adapter primes a fresh synthetic trial before presenting each final ObjectState.
No scored state is installed by directly setting success, stage, passage or damage flags.
"""
import math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import task as T

FAR={'left':np.array([[.1,.5,1.1]]),'right':np.array([[.1,-.5,1.1]])}
def states():return {o.name:o.state() for o in T.OBJECTS}
def moved(st,p,q=(0,0,0,1),vel=(0,0,0)):
    from yam_arena.task_file import ObjectState
    R=T.parts._quat_to_matrix(q);h=np.abs(R)@((st.high-st.low)/2)
    return ObjectState(np.asarray(p,float),np.asarray(q,float),np.asarray(vel,float),np.zeros(3),np.asarray(p)-h,np.asarray(p)+h)

def replay(kind,driver_force_fraction=1.,support_fraction=1.,tools_inverted=False):
    T.REC=T.fresh();o=states();t=0.;q=0.;hands=FAR
    def tick(n=1,load=0.):
        nonlocal t,q
        for _ in range(n):
            t+=.05;q=T.START_FRONT-T.REC['front']+(load*support_fraction+T.STOP_MASS*9.81)/T.K
            if load>0 and kind in ('backward loaded','cocked loaded','overload'):
                st=o['seal'];o['seal']=moved(st,(T.GX,T.GY,T.FACE+T.START_FRONT-q+T.SEAL_H/2*abs(T.rot(st)[2,2])),tuple(st.quat_xyzw))
            o['driver']=moved(o['driver'],(T.GX,T.GY,float(o['seal'].pos[2]+T.SEAL_H/2+T.DRIVER_H/2)),(1,0,0,0) if tools_inverted else (0,0,0,1))
            T.observe(o,hands,t,q,load*driver_force_fraction,load)
    # Undisturbed start must fail closed until hands/force known, then still yields no progress.
    if kind=='unknown':return o
    tick(3)
    if kind=='touched lip':
        hands={'left':np.array([o['seal'].pos+[.011,0,T.LIP_Z]]),'right':FAR['right']};tick();hands=FAR
    if kind=='noop':tick(30);return o
    cover=kind not in ('no sleeve','swept bypass')
    if cover:
        o['sleeve']=moved(o['sleeve'],(T.GX,T.GY,T.SLEEVE_HOME_Z),(1,0,0,0) if tools_inverted else (0,0,0,1));tick()
    o['seal']=moved(o['seal'],(T.GX,T.GY,T.SHAFT_TOP+.010))
    tick()
    if kind=='swept bypass':
        o['seal']=moved(o['seal'],(T.GX,T.GY,T.FACE-.0035));tick()
    else:
        for z in np.linspace(T.SHAFT_TOP+.010,T.FACE+T.START_FRONT+T.SEAL_H/2,80):
            o['seal']=moved(o['seal'],(T.GX,T.GY,float(z)));tick()
    if kind in ('no sleeve','swept bypass'):
        o['sleeve']=moved(o['sleeve'],(T.GX,T.GY,T.SLEEVE_HOME_Z));tick(30);return o
    if kind=='remove sleeve early':
        o['seal']=moved(o['seal'],(T.GX,T.GY,T.SHAFT_KEY_BOTTOM+.003));tick()
        o['sleeve']=moved(o['sleeve'],T.SLEEVE_POS);tick(30);return o
    if kind in ('backward loaded','cocked loaded'):
        angle=math.pi if kind=='backward loaded' else math.asin(.0007/(2*T.SEAL_R))
        quat=(math.sin(angle/2),0,0,math.cos(angle/2))
        o['seal']=moved(o['seal'],(T.GX,T.GY,T.FACE+T.START_FRONT+T.SEAL_H/2),quat)
        tick(8,8);return o
    target=.0 if kind=='shaft tier' else (.0040 if kind=='started tier' else .007)
    if kind=='overload':tick(8,35);return o
    # Each loaded sample keeps the actual seal bottom at the physical spring-supported front.
    for _ in range(160):
        depth=min(target,max(0.,-T.REC['front']+.0024))
        o['seal']=moved(o['seal'],(T.GX,T.GY,T.FACE-depth+T.SEAL_H/2))
        tick(load=18. if depth<target+.00001 else 0.)
        if T.REC['front']<=-target-.0001:break
    # Unloaded released rest: the ring remains on the seating front. Set the spring reaction to weight only.
    final_bottom=T.FACE-target
    T.REC['front']=T.START_FRONT if kind=='shaft tier' else T.REC['front']
    o['seal']=moved(o['seal'],(T.GX,T.GY,final_bottom+T.SEAL_H/2))
    # q is independently provided from geometry at rest; observe recomputes load from the tracked material front.
    def rest(n=30):
        nonlocal t
        for _ in range(n):
            t+=.05;T.observe(o,hands,t,T.START_FRONT+target)
    if kind!='tools not returned':
        o['sleeve']=moved(o['sleeve'],T.SLEEVE_POS)
        o['driver']=moved(o['driver'],T.DRIVER_POS)
    if kind=='held':hands={'left':np.array([[T.GX+T.SEAL_R,T.GY,T.FACE]]),'right':FAR['right']}
    if kind=='moving':o['seal']=moved(o['seal'],o['seal'].pos,vel=(.02,0,0))
    if kind=='hover':o['seal']=moved(o['seal'],(T.GX,T.GY,T.FACE+.015))
    if kind=='off axis':o['seal']=moved(o['seal'],(T.GX+.003,T.GY,T.FACE-.0035))
    rest(10 if kind=='short hold' else 30)
    return o

class Case(dict):
    def __init__(self,kind):self.kind=kind
    def items(self):return replay(self.kind).items()

REPLAY_CASES=[(k,Case(k),score,success) for k,score,success in [
    ('touched lip',0,False),('unknown',0,False),('noop',0,False),('no sleeve',0,False),('swept bypass',0,False),
    ('remove sleeve early',0,False),('backward loaded',0,False),('cocked loaded',0,False),
    ('overload',0,False),('held',0,False),('moving',0,False),('hover',0,False),('off axis',0,False),
    ('shaft tier',.25,False),('started tier',.5,False),('tools not returned',.75,False),
    ('short hold',0,False),('full',1,True)]]
CASES=[('unobserved start',{},0.,False),
       ('unobserved flush',{'seal':((T.GX,T.GY,T.FACE-.0035),None,None)},0.,False),
       ('unobserved hovering',{'seal':((T.GX,T.GY,T.FACE+.015),None,None)},0.,False),
       ('unobserved reversed',{'seal':((T.GX,T.GY,T.FACE-.0035),None,None,(1,0,0,0))},0.,False)]
if __name__=='__main__':
    bad=0
    for k,_,s,yes in REPLAY_CASES:
        o=replay(k);got=T.score(o);ok=got==s and T.success(o)==yes
        print('PASS' if ok else 'FAIL',k,got,'expected',s,T.REC['fail'], 'stage',T.REC['stage'],'work',round(T.REC['drive_work'],3),'hold',round(T.REC['hold'],2))
        if k in ('backward loaded','cocked loaded','overload') and not T.REC['fail']:
            print('FAIL damage control did not exercise violation');ok=False
        bad+=not ok
    # A safe approach above the shaft cannot be falsely labelled a cut.
    T.REC=T.fresh();o=states();T.observe(o,FAR,.05,0.)
    o['seal']=moved(o['seal'],(T.SEAL_POS[0],T.SEAL_POS[1],T.SHAFT_TOP+.030));T.observe(o,FAR,.10,0.)
    o['seal']=moved(o['seal'],(T.GX,T.GY,T.SHAFT_TOP-T.LIP_Z+.001));T.observe(o,FAR,.15,0.)
    ok=not T.REC['fail'];bad+=not ok;print('PASS' if ok else 'FAIL','lip1mm above shaft is safe without sleeve')
    o['seal']=moved(o['seal'],(T.GX+.019,T.GY,T.SHAFT_TOP-T.LIP_Z-.001));T.observe(o,FAR,.20,0.)
    ok=bool(T.REC['fail']);bad+=not ok;print('PASS' if ok else 'FAIL','eccentric lip edge crossing is caught')
    o=replay('full',driver_force_fraction=0.)
    ok=T.score(o)<.5 and T.REC['drive_work']==0.;bad+=not ok
    print('PASS' if ok else 'FAIL','spring load without driver contact earns no drive',T.score(o))
    o=replay('full')
    for _ in range(30):T.observe(o,{},T.REC['prev_t']+.05,T.START_FRONT+.007)
    ok=T.score(o)==0. and bool(T.REC['fail']);bad+=not ok
    print('PASS' if ok else 'FAIL','missing both hand clouds permanently invalidates',T.score(o))
    # Simultaneous sleeve arrival cannot protect earlier points on the same sampled sweep.
    T.REC=T.fresh();o=states();T.observe(o,FAR,.05,0.)
    o['seal']=moved(o['seal'],(T.GX,T.GY,T.SHAFT_TOP+.010));T.observe(o,FAR,.10,0.)
    o['seal']=moved(o['seal'],(T.GX,T.GY,T.FACE+T.START_FRONT+T.SEAL_H/2))
    o['sleeve']=moved(o['sleeve'],(T.GX,T.GY,T.SLEEVE_HOME_Z));T.observe(o,FAR,.15,0.)
    ok=bool(T.REC['fail']);bad+=not ok;print('PASS' if ok else 'FAIL','simultaneous sleeve arrival sweep')
    # Earlier unqualified travel cannot become credited work under stationary later pressure.
    o=replay('shaft tier');o['seal']=moved(o['seal'],(T.GX,T.GY,T.FACE-.005+T.SEAL_H/2))
    o['driver']=moved(o['driver'],T.DRIVER_POS)
    T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.005)
    o['driver']=moved(o['driver'],(T.GX,T.GY,T.FACE-.005+T.SEAL_H+T.DRIVER_H/2))
    for _ in range(12):T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.005)
    ok=T.REC['drive_work']<1e-10;bad+=not ok;print('PASS' if ok else 'FAIL','stationary pressure after unqualified travel adds zero work',T.REC['drive_work'])
    variants=[
        ('concentric at0.5mm',(0.0005,0,0),0.,1.),
        ('concentric beyond0.5mm',(0.00051,0,0),0.,0.),
        ('flush at+0.3mm',(0,0,0.0003),0.,1.),
        ('flush beyond+0.3mm',(0,0,0.00031),0.,.5),
        ('flush beyond-0.3mm',(0,0,-0.00031),0.,.5),
        ('square at0.5mm',(0,0,0),math.asin(.0005/(2*T.SEAL_R)),1.),
        ('square beyond0.5mm',(0,0,0),math.asin(.00051/(2*T.SEAL_R)),0.),
        ('unsupported hover',(0,0,.0011),0.,0.),
    ]
    for label,offset,angle,want in variants:
        o=replay('full');st=o['seal'];q=(math.sin(angle/2),0,0,math.cos(angle/2))
        o['seal']=moved(st,st.pos+np.array(offset),q)
        for _ in range(30):T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.007)
        got=T.score(o);ok=got==want;bad+=not ok
        print('PASS' if ok else 'FAIL',label,got,'expected',want)
    o=replay('full');T.observe(o,None,T.REC['prev_t']+.05,T.START_FRONT+.007)
    for _ in range(30):T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.007)
    ok=T.score(o)==0. and bool(T.REC['fail']);bad+=not ok
    print('PASS' if ok else 'FAIL','observation gap cannot erase damage history',T.score(o))
    for label,hands,q in [('empty hand',{'left':np.empty((0,3)),'right':FAR['right']},.0076),
                          ('nonfinite hand',{'left':np.array([[np.nan,0,0]]),'right':FAR['right']},.0076),
                          ('nonfinite force',FAR,np.nan)]:
        o=replay('full');T.observe(o,hands,T.REC['prev_t']+.05,q)
        for _ in range(30):T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.007)
        ok=T.score(o)==0. and bool(T.REC['fail']);bad+=not ok
        print('PASS' if ok else 'FAIL',label+' permanently invalidates trial',T.score(o))
    for e,load in [(.0011,40.),(.0011,8.)]:
        o=replay('shaft tier');st=o['seal'];angle=math.asin(.0007/(2*T.SEAL_R))
        o['seal']=moved(st,st.pos+np.array([e,0,0]),(math.sin(angle/2),0,0,math.cos(angle/2)))
        for _ in range(20):T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT-T.REC['front']+(load+T.STOP_MASS*9.81)/T.K)
        ok=bool(T.REC['fail']);bad+=not ok;print('PASS' if ok else 'FAIL','eccentric loaded entry damage',e,load,T.REC['fail'])
    for e,angle,want in [(.00065,0.,False),(.00019,0.,True),(0.,math.radians(2),False)]:
        ok=T.sleeve_covers(np.array([T.GX+e,T.GY,T.SLEEVE_HOME_Z]),(math.sin(angle/2),0,0,math.cos(angle/2)))==want
        bad+=not ok;print('PASS' if ok else 'FAIL','actual radial sleeve containment',e,angle)
    o=replay('full');st=o['seal']
    for k in range(30):
        a=(k+1)*.05*.19;o['seal']=moved(st,st.pos,(0,0,math.sin(a/2),math.cos(a/2)))
        T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.007)
    ok=T.score(o)==0.;bad+=not ok;print('PASS' if ok else 'FAIL','spinning rim cannot satisfy still hold',T.score(o))
    o=replay('shaft tier');hand={'left':np.array([o['seal'].pos+[T.SEAL_R,0,0]]),'right':FAR['right']}
    T.observe(o,hand,T.REC['prev_t']+.05,T.START_FRONT-T.REC['front']+(8.+T.STOP_MASS*9.81)/T.K,8.)
    ok=bool(T.REC['fail']);bad+=not ok;print('PASS' if ok else 'FAIL','direct hand loading the case is invalid',T.REC['fail'])
    o=replay('shaft tier');st=o['seal']
    o['seal']=moved(st,(T.GX,T.GY,T.FACE-.009-T.LIP_Z))
    T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT+.0107)
    ok=bool(T.REC['fail']);bad+=not ok;print('PASS' if ok else 'FAIL','lip driven past lower collar shoulder is damaged')
    o=replay('full',support_fraction=0.)
    ok=T.REC['drive_work']==0.;bad+=not ok;print('PASS' if ok else 'FAIL','driver force without supporting reaction earns no drive',T.REC['drive_work'])
    o=replay('full',tools_inverted=True)
    ok=T.score(o)==1.;bad+=not ok;print('PASS' if ok else 'FAIL','either annular tool end may perform the lawful fit',T.score(o))
    for stamp in (float('nan'),None):
        o=replay('full');T.observe(o,FAR,stamp,T.START_FRONT+.007)
        ok=bool(T.REC['fail']);bad+=not ok;print('PASS' if ok else 'FAIL','invalid timestamp',stamp)
    o=replay('shaft tier');T.observe(o,FAR,T.REC['prev_t']+10.,0.)
    ok=bool(T.REC['fail']) and T.score(o)==0.;bad+=not ok;print('PASS' if ok else 'FAIL','missing ten seconds invalidates trial')
    # One18N point pulse out of three has6N mean and cannot qualify sustained drive.
    o=replay('shaft tier');before=T.REC['front']
    for k in range(12):
        f=18. if k%3==0 else 0.
        T.observe(o,FAR,T.REC['prev_t']+.05,T.START_FRONT-T.REC['front']+(18.+T.STOP_MASS*9.81)/T.K,f,f)
    ok=T.REC['front']==before and T.REC['drive_work']==0.;bad+=not ok;print('PASS' if ok else 'FAIL','sparse contact pulses cannot qualify drive')
    from dataclasses import replace
    seal=replace(T.SEAL.state(),pos=np.zeros(3))
    hp=np.array([-.01020486264669579,-.009906007000000007,-.0015980565720733662])
    hq=np.array([.7071067811865477,-.7071067811865475,0.,0.])
    local=(np.array([.011,0,T.LIP_Z])-hp)@T.parts._quat_to_matrix(hq)
    hull=T.hand_hulls()['tip_left'][0]
    inside=np.max(local@hull.planes[:,:3].T-hull.planes[:,3])<-.00001
    sparse=T.lip_intrusion(T.face_points('tip_left')@T.parts._quat_to_matrix(hq).T+hp,seal)
    hit=T.swept_hand_lip(seal,{('test','tip_left'):(hp,hq)})
    ok=inside and not sparse and hit;bad+=not ok;print('PASS' if ok else 'FAIL','actual hand penetration between retained samples',inside,sparse,hit)
    left={('test','tip_left'):(hp-np.array([.1,0,0]),hq)};right={('test','tip_left'):(hp+np.array([.1,0,0]),hq)}
    ok=not T.swept_hand_lip(seal,left) and not T.swept_hand_lip(seal,right) and T.swept_hand_lip(seal,right,(seal.pos,seal.quat_xyzw),left)
    bad+=not ok;print('PASS' if ok else 'FAIL','actual hand hull sweep catches between-frame passage')
    for axis in ([1,0,0],[0,1,0],[0,0,1],[1,-1,0],[1,2,-3]):
        for angle in (0.,.7,math.pi-1e-8,math.pi):
            R=T.parts._rot(axis,angle);q=T.parts._matrix_to_quat(R)
            ok=np.max(np.abs(T.parts._quat_to_matrix(q)-R))<1e-10;bad+=not ok
    print('PASS' if bad==0 else 'CHECK FAILURES ABOVE','rigid rotation round trips including mixed-axis180deg')
    bore=[x for x in T.GEARBOX.shapes if x.kind=='hull' and abs(min(p[2] for p in x.points)+.012)<1e-9 and abs(max(p[2] for p in x.points)+.001)<1e-9]
    actual_inner=min(math.hypot((x.points[0][0]+x.points[2][0])/2,(x.points[0][1]+x.points[2][1])/2) for x in bore)
    ok=len(bore)==64 and actual_inner-T.SEAL_R>.00017;bad+=not ok
    print('PASS' if ok else 'FAIL','bore relief survives actual facet chords',len(bore),(actual_inner-T.SEAL_R)*1000)
    nominal=(21.3/2)*math.cos(math.pi/128)-10.613
    ok=nominal>0.03 and T.SLEEVE.contact_offset+T.DRIVER.contact_offset<nominal*.001;bad+=not ok
    print('PASS' if ok else 'FAIL','pilot contact offsets below effective clearance',nominal)
    # A trailing18N contact mean must not credit travel after current contact ceases.
    o=replay('shaft tier')
    q=T.START_FRONT-T.REC['front']+(18.+T.STOP_MASS*9.81)/T.K
    bottom=T.FACE+T.START_FRONT-q
    o['seal']=moved(o['seal'],(T.GX,T.GY,bottom+T.SEAL_H/2))
    o['driver']=moved(o['driver'],(T.GX,T.GY,bottom+T.SEAL_H+T.DRIVER_H/2))
    for _ in range(3):T.observe(o,FAR,T.REC['prev_t']+.05,q,18.,18.)
    before=(T.REC['front'],T.REC['retained'],T.REC['drive_work'])
    o['seal']=moved(o['seal'],o['seal'].pos-np.array([0.,0.,.0001]))
    o['driver']=moved(o['driver'],o['driver'].pos+np.array([0.,0.,.0002]))
    T.observe(o,FAR,T.REC['prev_t']+.05,q+.0001,0.,0.)
    after=(T.REC['front'],T.REC['retained'],T.REC['drive_work'])
    ok=T.REC['driver_load']>=12.-T.EPS and T.REC['driver_gap']>0 and before==after and not T.REC['drive_qualified'] and T.REC['work_delta']==0.
    bad+=not ok;print('PASS' if ok else 'FAIL','zero current contact after sustained load cannot advance response, retention or work',T.REC['driver_load'],T.REC['driver_gap'],T.REC['work_delta'])
    raise SystemExit(int(bad))
