"""Synthetic contact-series controls, not native/physical evidence."""
from types import SimpleNamespace
import numpy as np
from force import Certificate


def run():
    n=2;face=.265;length=.009525;z=.83
    pose=np.tile([0.,0.,z,0.,0.,0.,1.],(n,1))
    contact=np.zeros((n,1,3))
    c=Certificate.__new__(Certificate)
    c.task=SimpleNamespace(N=n,FACE=face,DEPTH=length,SEAL_Z=z,KERF=.003175)
    c.required=12.;c.origin=np.zeros(3);c.dt=1/240
    c.bodies=SimpleNamespace(get_transforms=lambda:pose.copy())
    c.contacts=SimpleNamespace(get_contact_force_matrix=lambda dt:contact.copy())
    def feed(depth,force):
        pose[:,0]=face+depth-(.0079375+length)
        pose[:,1]=np.arange(n)*.024
        contact[:,:,0]=-force;c.collect()
    checks=0
    def check(ok,label):
        nonlocal checks
        assert ok,label
        checks+=1
    c.reset()
    for depth in np.linspace(0.,.0083,201):feed(depth,12.)
    check(c.certified.all(),'loaded forward insertion qualifies')
    check(np.all(c.work>.015),'actual force-distance work recorded')
    c.reset()
    for depth in np.linspace(0.,.0083,201):feed(depth,11.999)
    check(not c.certified.any(),'subthreshold force cannot qualify')
    c.reset();feed(.0083,12.)
    for _ in range(100):feed(.0083,12.)
    check(not c.certified.any(),'static press cannot qualify')
    c.reset();feed(-.006,0.);feed(.0095,40.)
    check(not c.certified.any(),'rear-stop impact cannot qualify')
    c.reset()
    for depth in np.linspace(0.,.0083,101):feed(depth,2.)
    for _ in range(30):
        feed(.007,12.);feed(.0083,12.)
    check(not c.certified.any(),'repeated pumping cannot replace first insertion work')
    feed(-.001,0.)
    for depth in np.linspace(-.001,.0083,201):feed(depth,12.)
    check(c.certified.all(),'complete withdrawal permits a valid new insertion')
    c.reset();feed(-.006,0.);feed(.0083,12.)
    check(not c.certified.any(),'one loaded interval after free jump cannot qualify')
    c.reset();feed(-.006,0.);feed(-.0059,12.);feed(.0083,12.)
    check(not c.certified.any(),'large two-interval impact cannot qualify')
    return checks
