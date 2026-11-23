"""Offline endpoint, material-model and fail-closed state checks."""
import numpy as np
import task
from residue import Residue,SITES
CASES=[('untouched starting layout',{},0.,False),
 ('unknown hands cannot certify a sieve set down flat',{'sieve':((.12,.15,task.TOP+.022225),None,None)},0.,False),
 ('held frame is not release',{'sieve':((.12,.15,task.TOP+.12),None,None)},0.,False),
 ('rotated unsupported frame',{'sieve':((.12,.15,task.TOP+.12),None,None,(0,.5,0,.8660254))},0.,False),
 ('sieve left resting on the pan',{'sieve':((.25,-.03,task.TOP+.04445+.022225),None,None)},0.,False),
 ('pan moved under the sieve is not cleaning',{'pan':((.20,.15,task.TOP),None,None)},0.,False)]

def checks():
    n=0
    def test(label,got,want):
        nonlocal n;n+=1
        assert got==want,(label,got,want)
    def ep(c=20,r=0,b=0,e=0,bad=False,known=True,released=True):return task.endpoint(c,r,b,e,bad,known,released)
    for c,want in [(0,0),(9,0),(10,.25),(14,.25),(15,.5),(17,.5),(18,.75),(19,.75),(20,1.)]:test('collected tier boundary'+str(c),ep(c=c,r=20-c),want)
    for label,kwargs in [('damage',dict(bad=True)),('lost visible grain',dict(e=1)),('unknown hands',dict(known=False)),('still held',dict(released=False))]:test(label,ep(**kwargs),0.)
    # Hard parts of the brush: only the soft bristle head may touch the wires.
    inv=np.diag([-1.,1.,-1.]);mesh0=np.array((.25,-.03,task.TOP+.0817))
    pressed=task.BRUSH.state(pos=mesh0+np.array((0,0,.000315+.0961-.003)))
    test('bristles pressed on mesh are not hard contact',task.hard_contact(pressed,mesh0,inv),False)
    lying=task.BRUSH.state(pos=mesh0+np.array((.06,0,.011+.0003)),quat_xyzw=np.array((0.,np.sin(np.pi/4),0.,np.cos(np.pi/4))))
    test('ferrule or handle laid on mesh is damage',task.hard_contact(lying,mesh0,inv),True)
    poke=task.BRUSH.state(pos=mesh0+np.array((0,0,-.03)),quat_xyzw=np.array((0.,1.,0.,0.)))
    test('handle pushed through the wire plane is damage',task.hard_contact(poke,mesh0,inv),True)
    test('reblocked new aperture cannot earn full',ep(c=19,r=0,b=1),0.)
    m=Residue();origin=np.array((.23,0,task.TOP+.12));R=np.eye(3);m.initialize(origin,R);pan=np.array((.23,0,task.TOP));pR=np.eye(3)
    for i in range(20):m.advance(.05,origin,R,origin+np.array((i*.002,0,-.0005)),np.array((0,0,1.)),0.,pan,pR)
    test('hover and travel without load releases nothing',int(m.retained.sum()),20)
    test('hover remains clean damage history',m.damaged,False)
    # Place explicit landed grains in the actual pan, not a cleared-site flag.
    m.retained[:10]=False;m.pos[:10]=pan+np.array((0,0,.0017));m.vel[:]=0
    test('ten genuinely landed grains',m.counts(origin,R,pan,pR),(10,0,0))
    m.pos[0]=np.array((.6,0,task.TOP+.0007));test('scattered grain is unaccounted',m.counts(origin,R,pan,pR),(9,0,1))
    m.pos[0]=pan+np.array((0,0,.045));test('grain above pan rim unaccounted',m.counts(origin,R,pan,pR),(9,0,1))
    m.pos[0]=origin;test('freed grain back at mesh is reblocked',m.counts(origin,R,pan,pR),(9,1,0))
    m=Residue();m.initialize(origin,R)
    for x in np.linspace(-.03,.03,60):m.advance(.05,origin,R,origin+np.array((x,0,.000315)),np.array((0,0,-1.)),1.5,pan,pR)
    test('wrong-side loading cannot unseat native wedge',int(m.retained.sum()),20)
    m.retained[:]=False;m.pos[:]=pan+np.array((0,0,.0017));m.vel[:]=0
    m.advance(.05,origin,R,origin+np.array((0,0,.000315)),np.array((0,0,-1.)),4.,pan,pR)
    test('overload damages empty native mesh too',m.damaged,True)
    m.advance(.05,origin,R,origin+np.array((.2,0,.2)),np.array((0,0,-1.)),0.,pan,pR)
    test('withdrawing after damage does not erase it',m.damaged,True)
    m.bad=True;test('model invalid data is sticky',m.bad,True)
    from residue import RADII
    points=np.array([pan+np.array((x,y,.001+RADII[i])) for i,(x,y) in enumerate([(x,y) for x in (-.018,-.009,0,.009,.018) for y in (-.016,-.005,.005,.016)])])
    objects={o.name:o.state() for o in task.OBJECTS};pan=np.asarray(task.PAN.pos)
    points[:,:2]+=pan[:2]-np.array((.23,0))
    test('uncovered grain macro rays clear',bool(task.visible_grains(objects,points,RADII).all()),True)
    objects['brush']=task.BRUSH.state(pos=pan+np.array((0,0,.0971)))
    test('brush-hidden grains cannot be witnessed',bool(task.visible_grains(objects,points,RADII).all()),False)
    objects['brush']=task.BRUSH.state(pos=points[0]+np.array((.015875+RADII[0],0,.09447)))
    test('brush-side contact is not collected even if centre ray clears',bool(task.visible_grains(objects,points,RADII)[0]),False)
    tap=Residue();tap.initialize(origin,R)
    tap.advance(.05,origin,R,origin+np.array((.5,0,.2)),np.array((0,0,-1.)),0.,pan,pR)
    tap.advance(.05,origin+np.array((.5,0,0)),R,origin+np.array((.5,.2,.2)),np.array((0,0,-1.)),0.,pan,pR)
    test('sufficient native-frame inertia can release without brush',tap.release_count>0,True)
    test('remote brush motion cannot choose inertial slip direction',bool(np.max(np.abs(tap.offset[:,1]))<1e-12),True)
    # Regression checks for co-moving receiver friction and native cylindrical bounds.
    moving=Residue();far_mesh=np.array((.6,.4,task.TOP+.2));moving.initialize(far_mesh,R)
    moving.retained[0]=False;pp=np.array((.23,0,task.TOP));moving.pos[0]=pp+(0,0,.001+RADII[0]);far_tip=far_mesh+(0,0,.4)
    for _ in range(10):moving.advance(.05,far_mesh,R,far_tip,np.array((0,0,-1.)),0.,pp,R)
    for _ in range(20):
        pp=pp+(.001,0,0);moving.advance(.05,far_mesh,R,far_tip,np.array((0,0,-1.)),0.,pp,R)
    test('pan friction carries landed grain rather than leaving world coordinates',abs(moving.pos[0,0]-pp[0])<.0001,True)
    for _ in range(20):moving.advance(.05,far_mesh,R,far_tip,np.array((0,0,-1.)),0.,pp,R)
    test('carried grain settles and remains collected',moving.counts(far_mesh,R,pp,R),(1,0,0))
    spin=np.array((0.,0.,np.sin(np.pi/8),np.cos(np.pi/8)));a=task.BRUSH.state();b=task.BRUSH.state(quat_xyzw=spin)
    test('cylinder bounds invariant under axial spin',bool(np.allclose(a.low,b.low,atol=1e-12) and np.allclose(a.high,b.high,atol=1e-12)),True)
    # Public-score integration fixtures certify observation/history explicitly.
    # These are scorer tests, not simulated arm or feasibility demonstrations.
    names=('_model','_known','_bad','_hold','_finish_hold','_cert')
    saved={k:getattr(task,k) for k in names}
    try:
        from residue import RADII
        state={o.name:o.state() for o in task.OBJECTS}
        state['sieve']=task.SIEVE.state(quat_xyzw=np.array((0.,0.,0.,1.)))
        mesh,rot=task.frame(state['sieve']);panpos=np.asarray(state['pan'].pos)
        for count,want in ((10,.25),(15,.5),(20,1.)):
            model=Residue();model.initialize(mesh,rot);model.retained[:count]=False
            for i in range(count):model.pos[i]=panpos+np.array((SITES[i,0],SITES[i,1],.001+RADII[i]))
            task._model=model;task._known=True;task._bad=False;task._hold=1.;task._finish_hold=1.;task._cert=task.signature(state)
            test('public scorer released progress'+str(count),task.score(state),want)
        model.damaged=True;test('public scorer damage overrides full collection',task.score(state),0.);model.damaged=False
        for count,want in ((17,.5),(18,.75),(19,.75)):
            model=Residue();model.initialize(mesh,rot);model.retained[:count]=False
            for i in range(count):model.pos[i]=panpos+np.array((SITES[i,0],SITES[i,1],.001+RADII[i]))
            task._model=model;task._cert=task.signature(state)
            test('public scorer partial collection'+str(count),task.score(state),want)
        task._hold=.999;test('public scorer hold below1s',task.score(state),0.);task._hold=1.
        task._known=False;test('public scorer unknown hand observation',task.score(state),0.);task._known=True
        old=state['brush'];state['brush']=task.BRUSH.state(pos=panpos+np.array((0,0,.0971)));task._cert=task.signature(state)
        test('public scorer hidden collected grains',task.score(state),0.)
        state['brush']=old;task._cert=task.signature(state)
        model.pos[0]=np.array((.8,0,task.TOP+.001));test('public scorer lost grain',task.score(state),0.)
    finally:
        for k,v in saved.items():setattr(task,k,v)
    print(f'{n} material/endpoint checks passed')
    return n
checks()
if __name__=='__main__':
    assert task.score({o.name:o.state() for o in task.OBJECTS})==0.
    print('fail-closed public scorer passed')
