"""Scalar material, four-bar geometry and whole-trial bypass controls. No Isaac or policy claims."""
from pathlib import Path
import math,sys,json,copy
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
import task
CASES=[('do nothing',{},0.,False),('unknown hands fail closed',{},0.,False)]
def checks():
    from yam_arena.task_file import load
    m=load(Path(__file__).parent/'task.py').module;n=0
    def sample(ob,h,t_hint,**kwargs):
        # State transitions in these synthetic controls are observed at the native20Hz cadence.
        t=.05 if m.REC['t'] is None else m.REC['t']+.05
        return m.observe(ob,h,t,**kwargs)

    tests=[(0,25,True,0),(math.radians(45)-1e-5,25,True,0),(math.radians(45),1.999,True,0),(math.radians(45),2,True,.25),
      (math.radians(90)-1e-5,25,True,.25),(math.radians(90),8,True,.5),(math.radians(120)-1e-5,25,True,.5),(math.radians(120),15,True,.75),
      (math.radians(150),19.999,True,.75),(math.radians(150),20,False,.75),(math.radians(150),20,True,1),(2.72,25,True,1)]
    for c,j,p,s in tests:assert m.tier(c,j,p)==s;n+=1
    def scene(c=0.,seat=True,above=True):
        ob={o.name:o.state() for o in m.OBJECTS};b=ob['collecting_bowl'];pos=np.array([b.pos[0],b.pos[1],m.TOP+.10]) if above else m.PRESS_HOME
        ob['citrus_press']=m.state_for_closure(c,pos=pos)
        if seat:
            st=ob['citrus_press'];R=m.frame(st);d=(m.pin_d(st)-st.pos)@R
            ob['lemon_half']=m.LEMON.state(pos=st.pos+R@np.array([d[0]/2,0,.004]))
        return ob
    def hands(ob,one=False):
        out=m.Hands();st=ob['citrus_press']
        for arm,body,sign in [('left','base',1),('right','handle',-1)]:
            pp,R=m.link_pose(st,body);zz=.009 if body=='base' else 0
            pads=[np.array([pp+R@np.array([sg*.0094,sign*.15,zz])]) for sg in (-1,1)]
            if one and arm=='right':pads=[p+np.array([1,1,0]) for p in pads]
            out[arm]=np.concatenate(pads);out.pads[arm]=pads;out.fingers[arm]=np.array([-.009,-.009])
        return out
    for c in (0,.5,1.,1.5,2.,2.72):
        ob=scene(c);st=ob['citrus_press'];a,R=m.link_pose(st,'left_panel');b,S=m.link_pose(st,'right_panel')
        assert np.linalg.norm((a+R@(m.D_POS-m.A0))-(b+S@(m.D_POS-m.B0)))<1e-8;n+=1
        assert m.geometric_seat(ob)==(True,True);n+=1
        assert m.native_handles(ob,hands(ob));n+=1
        assert m.collector(ob);n+=1
    ob=scene();ob['lemon_half']=m.LEMON.state(pos=ob['lemon_half'].pos,quat_xyzw=(1,0,0,0));assert m.geometric_seat(ob)==(True,False);n+=1
    for label,seat,one,above,want in [('loaded',True,False,True,25.),('empty',False,False,True,0.),('one-hand',True,True,True,25.),('away',True,False,False,0.)]:
        m.REC=m.fresh()
        for k in range(1400):
            ob=scene(2.72*min(1,k/25),seat,above);sample(ob,hands(ob,one),.1*(k+1))
        assert abs(m.REC['juice']-want)<1e-6,(label,m.REC);n+=1
        json.dumps({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in m.REC.items()});n+=1
        assert m.REC['juice']+m.REC['spill']<=25.+1e-8;n+=1
        if label=='one-hand':assert m.REC['closure']==0.;n+=1
        if label=='away':assert m.REC['bad'] and m.REC['spill']>0;n+=1
        if label=='loaded':
            assert m.REC['peak_force']<=8. and m.REC['closure']>=math.radians(150);n+=1
            for q in np.linspace(2.72,0,81):
                ob=scene(q);sample(ob,hands(ob),71.)
            ob['citrus_press']=m.state_for_closure(0)
            ob['lemon_half']=m.LEMON.state(pos=ob['rind_saucer'].pos+np.array([0,0,.004]))
            far=m.Hands();far['left']=np.array([[3,3,3.]]);far['right']=np.array([[4,4,4.]])
            for arm in far:far.pads[arm]=(far[arm],far[arm]);far.fingers[arm]=np.array([m.FINGER_OPEN,m.FINGER_OPEN])
            for k in range(28):sample(ob,far,71.1+k*.1)
            assert m.score(ob)==1. and m.success(ob),(m.REC,m.parked(ob));n+=1
            assert m.REC['returned']+m.REC['material_loss']<=m.REC['work']+1e-6;n+=1
            completed=(copy.deepcopy(m.REC),copy.deepcopy(ob),copy.deepcopy(far))
            for badkey,value in [('clear',False),('known',False),('force_ok',False),('bad',True),('hold',.999)]:
                saved=m.REC[badkey];m.REC[badkey]=value;assert m.score(ob)==0.;m.REC[badkey]=saved;n+=1
    # Qualifying late must not credit prior work; material history remains physical.
    for above in (True,False):
        m.REC=m.fresh()
        for k in range(70):
            ob=scene(2.72*min(1,k/25),True,above);sample(ob,hands(ob,True),.1*(k+1))
        before=m.REC['juice']+m.REC['spill']
        for k in range(40):
            ob=scene(2.72);sample(ob,hands(ob),7.1+.1*k)
        assert m.REC['closure']==0.,m.REC;n+=1
        assert m.REC['juice']+m.REC['spill']>=before;n+=1
        if not above:assert m.REC['bad'] and m.REC['spill']>0;n+=1
    # Hovering pads and table-braced loaded closure never qualify as handle work.
    ob=scene(1.);h=hands(ob)
    for arm,body in [('left','base'),('right','handle')]:
        pp,R=m.link_pose(ob['citrus_press'],body)
        h.pads[arm]=[p+sg*.004*R[:,0] for p,sg in zip(h.pads[arm],(-1,1))]
    assert not m.native_handles(ob,h);n+=1
    m.REC=m.fresh()
    for k in range(20):
        ob=scene(2.72*k/19,True,False);sample(ob,hands(ob),.1*(k+1))
    assert m.REC['closure']==0. and not m.REC['free_held'];n+=1
    # Closing empty before loading,then holding,is not a loaded stroke.
    m.REC=m.fresh();ob=scene(2.72)
    for k in range(40):sample(ob,hands(ob),.1*(k+1))
    assert m.REC['juice']==0 and m.REC['closure']==0;n+=1
    # Whole-trial failure does not disappear when the part is put back.
    for kind in ('unknown','cut-up','escaped','bowl-tipped'):
        m.REC=m.fresh()
        for q in np.linspace(0,2.,41):
            ob=scene(q);sample(ob,hands(ob),.1)
        ob=scene(2.)
        if kind=='unknown':sample(ob,None,.2)
        elif kind=='cut-up':
            ob['lemon_half']=m.LEMON.state(pos=ob['lemon_half'].pos,quat_xyzw=(1,0,0,0));sample(ob,hands(ob),.2)
        elif kind=='escaped':
            ob['lemon_half']=m.LEMON.state(pos=ob['lemon_half'].pos+np.array([0,.04,0]));sample(ob,hands(ob),.2)
        else:
            sample(ob,hands(ob),.2);ob['collecting_bowl']=m.BOWL.state(quat_xyzw=(.2,0,0,.979796));sample(ob,hands(ob),.3)
        assert m.REC['bad'],kind;n+=1
        ob=scene(2.);sample(ob,hands(ob),.4);assert m.REC['bad'] and m.score(ob)==0.;n+=1
    # Invalid sensor history cannot be erased by a later healthy frame.
    for kind in ('nan-hand','missing-arm','nan-fingers','nan-object'):
        m.REC=m.fresh();ob=scene();h=hands(ob);sample(ob,h,.1)
        if kind=='nan-hand':h['left'][0,0]=np.nan
        elif kind=='missing-arm':del h['right']
        elif kind=='nan-fingers':h.fingers['left'][0]=np.nan
        else:ob['lemon_half'].pos[0]=np.nan
        sample(ob,h,.2);assert m.REC['bad'] and not m.REC['known'];n+=1
        ob=scene(1.);sample(ob,hands(ob),.3)
        assert m.REC['bad'] and m.score(ob)==0.;n+=1
    # Either stable face on the saucer is valid; a floating spent half is not supported.
    ob=scene(0,True,False);sau=ob['rind_saucer']
    ob['lemon_half']=m.LEMON.state(pos=sau.pos+np.array([0,0,.034]),quat_xyzw=(1,0,0,0))
    assert m.parked(ob);n+=1
    ob['lemon_half']=m.LEMON.state(pos=sau.pos+np.array([0,0,.044]),quat_xyzw=(1,0,0,0))
    assert not m.parked(ob);n+=1
    # Surface-cloud contact0.5mm from a scored handle vetoes release,even with no nearby vertex elsewhere.
    ob=scene(0,True,False);h=m.Hands();h['left']=np.array([[3,3,3.]]);h['right']=np.array([[4,4,4.]])
    assert m.released(ob,h);n+=1
    h['left']=np.array([m.world(ob['citrus_press'],[.0095,.15,.009])]);assert not m.released(ob,h);n+=1
    # Independent review end-cap counterexample:3.945mm clear is not1mm contact.
    ob=scene();h=hands(ob)
    for arm,body,sign in [('left','base',1),('right','handle',-1)]:
        pp,R=m.link_pose(ob['citrus_press'],body);z=.009 if body=='base' else 0.
        h.pads[arm]=[np.array([pp+R@np.array([sg*.0094,sign*.1999,z])]) for sg in (-1,1)]
    assert not m.native_handles(ob,h);n+=1
    # A legitimate opening then closing within one qualified interval credits the full stroke.
    m.REC=m.fresh()
    for q in np.concatenate([np.linspace(0,1,21),np.linspace(1,0,21)[1:],np.linspace(0,2.72,56)[1:]]):
        ob=scene(q);sample(ob,hands(ob),0)
    assert abs(m.REC['closure']-2.72)<1e-9;n+=1
    # Invisible decoration cannot veto a clear physical surface gap.
    ob=scene(0,True,False);h=m.Hands();h['left']=np.array([m.world(ob['lemon_half'],[.025,.025,.025])]);h['right']=np.array([[4.,4.,4.]])
    assert m.released(ob,h);n+=1
    for late in (60.,.01):
        m.REC=m.fresh();ob=scene();m.observe(ob,hands(ob),.05)
        ob=scene(2.72);m.observe(ob,hands(ob),late)
        assert m.REC['bad'] and not m.REC['known'] and m.REC['why']=='missing temporal history';n+=1
    ob=scene();h=hands(ob)
    for arm,body,sign in [('left','base',1),('right','handle',-1)]:
        pp,R=m.link_pose(ob['citrus_press'],body);z=.009 if body=='base' else 0.
        h.pads[arm]=[np.array([pp+R@np.array([sg*.0094,sign*.15,z+.009])]) for sg in (-1,1)]
        h[arm]=np.concatenate(h.pads[arm])
    assert not m.native_handles(ob,h);n+=1
    m.REC=m.fresh();ob=scene();m.observe(ob,hands(ob),60.)
    assert m.REC['bad'] and not m.REC['known'] and m.REC['why']=='missing initial history';n+=1
    # Native trajectories must stop short of the declared mechanical joint limit.
    assert 2*m.PHI_MAX<=m.MAX_STROKE;n+=1
    # Actual rim points, not a40mm surrogate, exceed2mm during the review's slow rotation.
    m.REC,base,far=copy.deepcopy(completed)
    for k in range(1,22):
        ob=copy.deepcopy(base);b=ob['collecting_bowl'];angle=.0018*k
        ob['collecting_bowl']=m.BOWL.state(pos=b.pos,quat_xyzw=(0,0,math.sin(angle/2),math.cos(angle/2)))
        sample(ob,far,0)
    assert not m.REC['bad'] and m.REC['hold']<1. and m.score(ob)==0.;n+=1
    for name,spec in [('collecting_bowl',m.BOWL),('rind_saucer',m.SAUCER)]:
        m.REC,ob,far=copy.deepcopy(completed);restored=copy.deepcopy(ob)
        ob[name]=spec.state(pos=(.65,0,m.TOP-.1));sample(ob,far,0)
        assert m.REC['bad'] and m.REC['why']=='scored object dropped'
        for k in range(30):sample(restored,far,0)
        assert m.REC['bad'] and m.score(restored)==0.;n+=1
    # Two samples on the same rounded front face do not form opposed lateral contacts.
    ob=scene();h=hands(ob)
    for arm,body,sign in [('left','base',1),('right','handle',-1)]:
        pp,R=m.link_pose(ob['citrus_press'],body);z=.009 if body=='base' else 0.
        h.pads[arm]=[np.array([pp+R@np.array([sg*.0041,sign*.199,z])]) for sg in (-1,1)]
        h[arm]=np.concatenate(h.pads[arm])
    assert not m.native_handles(ob,h);n+=1
    # Review regression: overtravel fails permanently even after valid recovery.
    for q in (2.85,-.05):
        m.REC,base,far=copy.deepcopy(completed)
        ob=scene(q);sample(ob,hands(ob),0)
        assert m.REC['bad'] and m.REC['why']=='hinge outside intact limits';n+=1
        for k in range(30):sample(base,far,0)
        assert m.score(base)==0.;n+=1
    # The one-frame zero-work closure cannot turn a subsequent static hold into juice.
    m.REC=m.fresh();ob=scene();sample(ob,hands(ob),0)
    ob=scene(2.78)
    for k in range(180):sample(ob,hands(ob),0)
    assert m.REC['work']==0. and m.REC['juice']==0. and m.REC['spill']==0.;n+=1
    # Rapid and slow complete ramps cannot return their material loss as unloading work.
    for count in (20,150):
        m.REC=m.fresh()
        for q in np.linspace(0,2.78,count+1):
            ob=scene(q);sample(ob,hands(ob),0)
        for k in range(30):sample(ob,hands(ob),0)
        for q in np.linspace(2.78,0,81):
            ob=scene(q);sample(ob,hands(ob),0)
        r=m.REC
        assert r['model_valid'] and not r['bad'],r
        assert r['returned']+r['material_loss']<=r['work']+1e-6,r;n+=1
        assert abs(r['work']-r['returned']-r['material_loss']-r['energy'])<1e-6,r;n+=1
    # A legitimate partial hold exhausting its paid yield must still allow a gentle release.
    m.REC=m.fresh()
    for q in np.linspace(0,1.8,151):
        ob=scene(q);sample(ob,hands(ob),0)
    while m.REC['model_valid'] and m.REC['t']<112. and m.REC['energy']>m.ENERGY_RESERVE+1e-10:sample(ob,hands(ob),0)
    assert m.REC['energy']>=m.ENERGY_RESERVE-1e-9 and m.REC['juice']>=8.;n+=1
    ob=scene(1.79);sample(ob,hands(ob),0)
    assert not m.REC['bad'],m.REC;n+=1
    for q in np.linspace(1.79,0,81):
        ob=scene(q);sample(ob,hands(ob),0)
    ob['citrus_press']=m.state_for_closure(0)
    ob['lemon_half']=m.LEMON.state(pos=ob['rind_saucer'].pos+np.array([0,0,.004]))
    far=copy.deepcopy(completed[2])
    for k in range(28):sample(ob,far,0)
    assert m.score(ob)==.5 and not m.REC['bad'],m.REC;n+=1
    assert m.REC['returned']+m.REC['material_loss']<=m.REC['work']+1e-6;n+=1
    # Nonpassive numerical histories are invalid/unscored, not a physical task-failure claim.
    m.REC=m.fresh()
    for q in np.linspace(0,2.78,151):
        ob=scene(q);sample(ob,hands(ob),0)
    for k in range(10):sample(ob,hands(ob),0)
    ob=scene(0);sample(ob,hands(ob),0)
    assert not m.REC['model_valid'] and not m.REC['known'] and not m.REC['bad'];n+=1
    assert not m.REC['force_ok'] and not m.REC['force'].any() and m.score(ob)==0.;n+=1
    for k in range(30):sample(ob,hands(ob),0)
    assert not m.REC['model_valid'] and m.score(ob)==0.;n+=1
    # Independent-review startup reversal: native passive damping funds bounded activation.
    m.REC=m.fresh()
    for q in [*np.linspace(0,.15,16),.16,.15]:
        ob=scene(q);sample(ob,hands(ob),0)
    r=m.REC
    assert r['model_valid'] and not r['bad'] and r['energy_error']==0.;n+=1
    assert 0<r['native_damping_work'] and r['returned']<=r['work']+1e-9;n+=1
    assert r['juice']==0. and r['spill']==0.;n+=1
    # One-step publication bound holds before a reversal, not only after observing one.
    assert max(r['force'])*.180*m.MAX_MODEL_STEP<=r['energy']+1e-12;n+=1
    # Small fast follower oscillation: actual angular velocity must veto a still hold.
    m.REC,base,far=copy.deepcopy(completed)
    for k in range(42):
        theta=.002*math.sin(100*.05*k);omega=.2*math.cos(100*.05*k)
        ob=copy.deepcopy(base);st=m.PRESS.state(joints={'strainer_pivot':theta})
        centre=m.world(st,m.C0);w=np.array([0.,0.,omega])
        for body in ('strainer_pivot','strainer'):
            pp,_=m.link_pose(st,body);st.link_velocities[body]=(np.cross(w,pp-centre),w.copy())
        ob['citrus_press']=st;sample(ob,far,0)
    assert m.REC['model_valid'] and not m.REC['bad'] and m.REC['hold']<1. and m.score(ob)==0.;n+=1
    for k in range(30):sample(base,far,0)
    assert m.score(base)==1.;n+=1
    # A nonfinite articulated-body velocity invalidates sensor history and cannot recover credit.
    m.REC,ob,far=copy.deepcopy(completed)
    ob['citrus_press'].link_velocities['strainer_pivot']=(np.zeros(3),np.array([0.,0.,np.nan]))
    sample(ob,far,0);assert m.REC['bad'] and not m.REC['known'] and m.score(ob)==0.;n+=1
    print(f'{n} four-bar/material/history/bypass controls PASS')
if __name__=='__main__':checks()
