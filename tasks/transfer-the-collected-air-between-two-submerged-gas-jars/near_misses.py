"""Offline scorer and declared-physics controls. No native or physical success is inferred."""
import math
import numpy as np
import task
CASES = [
    ('unobserved start, no-op',{},0.,False),
    ('swapped jars does not transfer air',{task.DONOR.name:(task.RECEIVER.pos,None,None),task.RECEIVER.name:(task.DONOR.pos,None,None)},0.,False),
    ('empty jar dipped from room air',{task.RECEIVER.name:((.335,-.080,task.TOP+.180),None,None)},0.,False),
    ('held motionless without observed hands',{},0.,False),
]

def run_controls():
    for a,s in ((21.999,.0),(22.,.25),(41.999,.25),(42.,.5),(61.999,.5),(62.,.75),(73.999,.75),(74.,1.)):
        assert task.tier(a)==s,(a,task.tier(a),s)
    d,r=task.DONOR.state(),task.RECEIVER.state()
    t=task.AirTrial();t.advance([d,r],5.,True)
    assert np.allclose(t.ml,[80,0]) and not t.bubbles,'no-op transfer'
    # Mouth centre submerged, but one tilted rim point above the surface.
    q=task.parts._matrix_to_quat(task.parts._rot((0,1,0),math.radians(45)))
    crossed=task.DONOR.state(pos=(.25,.05,task.WATER+.075/math.sqrt(2)-.010),quat_xyzw=q)
    t=task.AirTrial();t.advance([crossed,r],.05,True)
    assert t.zero and t.ml[0]==0 and t.lost==80,'partial rim did not vent immediately'
    t.advance([d,r],.05,True);assert t.zero and t.ml[0]==0,'return underwater erased violation'
    # No imaginary water exists outside the physical trough footprint.
    outside=task.DONOR.state(pos=(.110,0.,task.TOP+.079))
    t=task.AirTrial();t.advance([outside,r],.05,True)
    assert t.zero and t.ml[0]==0.,'infinite-water-plane bypass'
    # Surface contact at equality is a violation.
    d0=task.DONOR.state(pos=(.25,.05,task.WATER+.075))
    t=task.AirTrial();t.advance([d0,r],.05,True);assert t.zero
    # Correct projected overlap at the wrong height cannot catch rising bubbles.
    tipped=task.DONOR.state(pos=(.265,-.04,task.TOP+.035),quat_xyzw=task.parts._matrix_to_quat(task.parts._rot((0,-1,0),math.radians(88))))
    mouth=task.geometry(tipped)[2]
    wrong=task.RECEIVER.state(pos=(mouth[0],mouth[1],task.TOP+.099))
    t=task.AirTrial()
    for _ in range(260):t.advance([tipped,wrong],.05,True)
    assert t.ml[1]==0 and t.lost>65,'overlap credited without rising-mouth crossing'
    # Actual bubble flight into the physically upper receiver; also compute loads.
    rec=task.RECEIVER.state(pos=(mouth[0],mouth[1],task.TOP+.142))
    t=task.AirTrial()
    for _ in range(300):t.advance([tipped,rec],.05,True)
    assert t.ml[1]>=74 and not t.zero,(t.ml,t.zero)
    assert abs(t.ml.sum()+t.lost+sum(v for p,v,i in t.bubbles)-80)<1e-6
    trough=task.TROUGH.state()
    t.loads([tipped,rec],trough,.05)
    assert np.isfinite(t.force).all() and np.linalg.norm(t.force)>0
    assert np.allclose(t.force.sum(0)+t.reaction,0)
    assert np.allclose(t.torque.sum(0)+sum((np.cross(s.pos-trough.pos,f) for s,f in zip((tipped,rec),t.force)),np.zeros(3))+t.reaction_torque,0)
    t0=task.AirTrial()
    for _ in range(100):t0.advance([tipped,rec],.05,False)
    assert np.allclose(t0.ml,[80,0]),'credited transfer without applied load'
    assert not task.hand_clear(None,[d,r]),'unknown hands passed'
    # A broad hand face just0.5mm from body, distant vertices: face samples fail clear.
    face=np.array([d.pos+np.array((.0255,0.,0.))])
    assert not task.hand_clear(face,[d,r])
    # A moving mouth cannot retrospectively catch a parcel at its final pose.
    old_receiver=task.RECEIVER.state(pos=(.30,0.,task.TOP+.150))
    new_receiver=task.RECEIVER.state(pos=(.27,0.,task.TOP+.150))
    parcel=np.array((.27,0.,task.TOP+.075-.00018))
    moving=task.AirTrial();moving.ml[:]=0.;moving.bubbles=[[parcel,80.,0]];moving.last_states=[d,old_receiver]
    moving.advance([d,new_receiver],.05,True)
    assert moving.ml[1]==0.,'end-pose retroactive capture'
    # Opaque face covering the opening loses bubbles; it does not channel them through.
    hand=np.array([(mouth[0]+x,mouth[1]+y,task.TOP+.067) for x in np.arange(-.03,.031,.002) for y in np.arange(-.03,.031,.002)])
    covered=task.AirTrial()
    for _ in range(300):covered.advance([tipped,rec],.05,True,hand=hand)
    assert covered.ml[1]==0. and covered.blocked_ml>65,'opaque hand transmitted air'
    # A cover3.5mm inside the mouth is beyond the old entry-plane margin,
    # but blocks the actual modeled4mm throat. It must not create retained gas.
    throat_face=hand+np.array((0.,0.,.0035))
    throat_blocked=task.AirTrial()
    for _ in range(300):throat_blocked.advance([tipped,rec],.05,True,hand=throat_face)
    assert throat_blocked.ml[1]==0. and throat_blocked.blocked_ml>65,'receiver throat obstruction bypass'
    rmat,axis,opening,*_=task.geometry(tipped)
    donor_face=np.array([opening+rmat@np.array((x,y,0.)) for x in np.arange(-.03,.031,.002) for y in np.arange(-.03,.031,.002)])
    sealed=task.AirTrial()
    for _ in range(300):sealed.advance([tipped,rec],.05,True,hand=donor_face)
    assert sealed.ml[1]==0. and sealed.emitted==0. and sealed.source_blocked_attempt_ml>1.,'donor cover bypassed by spawning outside rim'
    # Qualifying support/release/stillness gates use the current scene, not an
    # internally injected success flag. This fixture is an offline scorer unit test.
    saved=task.T
    try:
        objects={o.name:o.state() for o in task.OBJECTS}
        task.T=t;task.T.force_ready=True;task.T.last_states=[objects[task.DONOR.name],objects[task.RECEIVER.name]]
        hand=np.array(((.8,.4,task.TOP+.4),))
        for k in range(25):task.observe(objects,hand,(k+1)*.05)
        assert task.score(objects)==1. and task.success(objects),'valid released hold rejected'
        task.T.ready=False;assert task.score(objects)==0.;task.T.ready=True
        # Pose drift resets the one-second hold even with zero reported velocity.
        moving=dict(objects);moving[task.RECEIVER.name]=task.RECEIVER.state(pos=np.asarray(task.RECEIVER.pos)+(.0011,0,0))
        task.observe(moving,hand,1.30);assert task.score(moving)==0.,'moving jar passed stillness'
        for k in range(24):task.observe(objects,hand,1.35+k*.05)
        assert task.score(objects)==1.,'renewed one-second hold failed'
        unknown=task.T.state_stamp;task.T.state_stamp=None;assert task.score(objects)==0.;task.T.state_stamp=unknown
        assert not task.park_ok([objects[task.DONOR.name],objects[task.RECEIVER.name]],{n:o for n,o in objects.items() if n!='receiver_shelf'})
        shifted=dict(objects);shifted['receiver_shelf']=task.SHELVES[0].state(pos=np.asarray(task.SHELVES[0].pos)+(.03,0,0))
        task.observe(shifted,hand,2.55)
        assert task.score(shifted)==0.,'absent support still credited'
        # Physics observations must catch motion/touches hidden between20Hz endpoints.
        far=np.r_[np.array((.8,.4,task.TOP+.4)),[0.,0.,0.,1.]]
        vertices=np.vstack([h.vertices for h in task.robot_hulls()['gripper']])
        face=vertices[np.abs(vertices[:,0]-vertices[:,0].min())<1e-6]
        near=objects[task.RECEIVER.name].pos+np.array((.0255,0.,0.))-face.mean(0)
        for bad in ('motion','touch','missing_hands','none'):
            task.T=task.AirTrial();task.T.ml=np.array((0.,80.));task.T.force_ready=True;task.T.active=True
            task.T.last_states=[objects[task.DONOR.name],objects[task.RECEIVER.name]]
            for k in range(1,26):
                path=[]
                for j in range(1,13):
                    poses={n:np.r_[o.pos,o.quat_xyzw].copy() for n,o in objects.items()}
                    robots=[(name,far.copy()) for _ in range(2) for name in task.hand_mesh()]
                    if bad=='motion':poses[task.RECEIVER.name][0]+=.002*math.sin(math.pi*j/12)
                    if bad=='touch' and j==6:robots[0]=('gripper',np.r_[near,[0.,0.,0.,1.]])
                    if bad=='missing_hands':robots=[]
                    path.append((1/240.,poses,robots))
                task.T.physics_path=path;task.observe(objects,hand,k*.05)
            assert task.score(objects)==(1. if bad=='none' else 0.),('physics hold bypass',bad,task.score(objects))
    finally:task.T=saved
    print('gasjar controls PASS: tiers, provenance, partial rim, vent latch, upward flight, reciprocal load, missing loads/hands, face contact')

if __name__=='__main__':run_controls()
