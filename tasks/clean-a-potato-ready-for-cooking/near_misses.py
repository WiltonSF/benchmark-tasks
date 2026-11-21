"""Endpoint, native-contact rule and history controls; no simulator results are fabricated."""
import numpy as np
import task as t

def controls():
    # Force and motion independent controls. Cleaning needs both continuous load and tangential displacement.
    for f in (0.,.99,5.01):
        t.reset_state()
        for x in np.linspace(-.03,.03,100):t.record_contact(np.array([x,0,.03]),np.eye(3),f,.05)
        assert t.clean_count()==0,('out-of-range contact',f)
    t.reset_state();t.S['occluded'][:]=True
    for x in np.linspace(-.03,.03,100):t.record_contact(np.array([x,0,.03]),np.eye(3),3.,.05)
    assert not t.S['travel'].any(),'hand-shielded cells cannot clear'
    t.reset_state()
    for _ in range(100):t.record_contact(np.array([0,0,.03]),np.eye(3),3.,.05)
    assert t.clean_count()==0,'stationary press gives no cleaning'
    # Damage is whole-trial and survives a later valid contact sequence.
    t.record_contact(np.array([0,0,.03]),np.eye(3),12.01,.05)
    for x in np.linspace(-.03,.03,100):t.record_contact(np.array([x,0,.03]),np.eye(3),3.,.05)
    assert t.S['damage']
    # A broad gripper face 0.5 mm from the potato cannot pass a release check.
    st={'potato':t.POTATO.state(),'brush':t.BRUSH.state()}
    assert not t.hands_clear(np.array([t.POTATO.pos])+[0,0,.0305],st)
    st['brush']=t.BRUSH.state(pos=(.60,-.30,t.TOP+.023))
    corners=np.array(t.POTATO.pos)+np.array([[x,y,.0305] for x in (-.08,.08) for y in (-.08,.08)])
    assert t.hands_clear(corners,st),'all broad-face vertices are far away'
    face=np.vstack([corners,np.array(t.POTATO.pos)+[0,0,.0305]])
    assert not t.hands_clear(face,st),'sampled face0.5mm away must block release'
    # Test the complete tier ladder, release, support and whole-trial failures with synthetic sensor history.
    from yam_arena.task_file import ObjectState
    def endpoint(n, plate=False):
        t.reset_state();t.S.update(known=True,clear=True,supported=True,hold=40)
        for k in range(n):t.S['travel'][t.IDS==k]=.025
        pos=(.255,.250,t.TOP+.034) if plate else t.POTATO.pos
        brush=t.BRUSH.state(pos=(.400,.250,t.TOP+.023))
        return {'potato':t.POTATO.state(pos=pos),'brush':brush}
    for n,want in ((0,0),(2,0),(3,.25),(5,.25),(6,.5),(8,.5),(9,.75),(11,.75),(12,.75)):
        st=endpoint(n);assert t.score(st)==want,(n,t.score(st),want)
    st=endpoint(12,True);assert t.score(st)==1 and t.success(st)
    for flag in ('damage','dropped','broken'):
        st=endpoint(12,True);t.S[flag]=True;assert t.score(st)==0
    # Verified counterexample: centre distance alone allowed overlap at x=.360.
    st=endpoint(12,True);st['brush']=t.BRUSH.state(pos=(.360,.250,t.TOP+.023));assert t.score(st)==.75
    st=endpoint(12,True);t.S['supported']=False;assert t.score(st)==0,'unintended/unknown support'
    # Native-substep events persist even if recovered before the next control observation.
    st=endpoint(12,True);t.latch_violations(12.01,t.TOP);t.latch_violations(0.,t.TOP);assert t.score(st)==0
    st=endpoint(12,True);t.latch_violations(0.,t.TOP-.011);t.latch_violations(0.,t.TOP);assert t.score(st)==0
    for flag in ('known','clear'):
        st=endpoint(12,True);t.S[flag]=False;assert t.score(st)==0
    st=endpoint(12,True);t.S['hold']=39;assert t.score(st)==0
    st=endpoint(12,True);st['potato']=t.POTATO.state(pos=(.255,.250,t.TOP+.040));assert t.score(st)==0
    t.reset_state();t.S['travel'][np.where(t.IDS==0)[0][:22]]=.025;assert t.clean_count()==0
    t.S['travel'][np.where(t.IDS==0)[0][22]]=.025;assert t.clean_count()==1
    t.reset_state()
    sliding_controls()

def sliding_controls():
    # A brush fixed on the counter, bristles up; the potato pressed 1 mm into the bristle band at 2 N.
    import math
    br=t.parts._quat_to_matrix((1.,0.,0.,0.));link=np.zeros(3)+br@np.array([0,0,-.03175])
    field=link+br@np.array([0,0,-.00875]);C=np.array([0,0,field[2]-.001])
    def Ry(a):c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    def Rx(a):c,s=math.cos(a),math.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    def run(poses,force=2.):
        t.reset_state()
        for rot,pos,contact in poses:
            t.record_contact(rot.T@(contact-pos),rot.T@br,force,.05,(rot.T@br,rot.T@(link-pos)),rot.T@(field-pos))
        return t.clean_count()
    # Rocking +/-12 degrees about the contact point (zero slip) for three cycles clears nothing.
    angles=[a for _ in range(3) for a in list(np.linspace(-12,12,41))+list(np.linspace(12,-12,41))]
    assert run([(Ry(math.radians(a)),C+Ry(math.radians(a))@np.array([0,0,.030]),C) for a in angles])==0,'rocking without slip'
    # Rolling without slip across the field width clears nothing.
    th=[x for _ in range(2) for x in list(np.linspace(-.3,.3,61))+list(np.linspace(.3,-.3,61))]
    assert run([(Rx(a),np.array([0,-.031*a,C[2]+.031]),np.array([0,-.031*a,C[2]])) for a in th])==0,'rolling without slip'
    # Sliding slower than 10 mm/s (0.06 mm jitter per sample, 45 s) clears nothing.
    xs=[(-1)**k*.00003 for k in range(900)]
    assert run([(np.eye(3),np.array([x,0,C[2]+.030]),np.array([x,0,C[2]])) for x in xs])==0,'sub-threshold jitter'
    # Positive control: the same press sliding 25 mm back and forth at 10-20 mm/s clears the underside sector.
    xs=list(np.linspace(-.0125,.0125,26))+list(np.linspace(.0125,-.0125,26))
    assert run([(np.eye(3),np.array([x,0,C[2]+.030]),np.array([x,0,C[2]])) for x in xs])==1,'sliding clears'
    # The brush at its own start pose (y = 90.6 mm) is judged against the tray floor, not the counter.
    t.reset_state();t.S.update(known=True,clear=True,supported=True,hold=40);t.S['travel'][:]=.025
    st={'potato':t.POTATO.state(pos=(.255,.250,t.TOP+.034)),'brush':t.BRUSH.state()}
    assert t.score(st)==1.,'untouched brush beside the plate on the tray floor'
    st['brush']=t.BRUSH.state(pos=(.1809,.0906,t.TRAY_Z+.023+.0012));assert t.score(st)==1.,'tray-floor tolerance'
    st['brush']=t.BRUSH.state(pos=(.1809,.0906,t.TRAY_Z+.023+.035));assert t.score(st)==0.,'brush lifted onto the tray wall height'
    t.reset_state()
controls()
CASES=[('do nothing',{},0.,False),('place dirty potato on plate',{'potato':((.255,.250,t.TOP+.034),None,None)},0.,False),('hold above plate',{'potato':((.255,.250,t.TOP+.055),None,None)},0.,False),('unknown hands fail closed',{},0.,False)]
