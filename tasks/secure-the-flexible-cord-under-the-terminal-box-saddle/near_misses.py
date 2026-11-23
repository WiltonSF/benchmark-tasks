"""Offline adversarial scorer checks. Contact histories here are injected unit-test inputs, not physics evidence."""
import pathlib,sys,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import task
CASES=[('do nothing',{},0.,False),('unknown hands fail closed',{'cable':task.CABLE.state(joints={'feed':.019})},0.,False),
       ('bar off shanks',{'clamp_bar':task.BAR.state(pos=(task.X+.03,task.Y,task.BAR_Z))},0.,False),
       ('driver lost',{'driver':task.DRIVER.state(pos=(.155,.185,task.TOP-.03))},0.,False)]
def verify():
    t=task;start={p.name:p.state() for p in t.OBJECTS}
    def state(feed=.019):
        o=dict(start)
        for screw in t.SCREWS:o[screw.name]=screw.state(joints={'advance':.0004,'spin':-2*math.pi*.0004/t.PITCH})
        o['cable']=t.CABLE.state(joints={'feed':feed});o['driver']=t.DRIVER.state(pos=(t.TRAY_X,t.TRAY_Y,t.TOP+.003));return o
    base=dict(known=True,bad=False,clear=True,held=1.,end_forces=[2.,2.],feed_speed=0.,pull='passed')
    def check(label,o,expected,**changes):
        t.H.clear();t.H.update(base);t.H['pull_ref']=t.proof_state(o);t.H.update(changes)
        got=t.score(o);assert got==expected,(label,got,expected)
        print('ok',label,got)
    o=state()
    check('full observable outcome',o,1.)
    check('no hand report',o,0.,known=False)
    check('hand remains on cable',o,0.,clear=False)
    check('only 0.99 seconds clear',o,0.,held=.99)
    check('whole trial bar violation persists',o,0.,bad=True)
    check('one seated end',o,.25,end_forces=[2.,0.])
    check('two ends no pull',o,.5,pull='none')
    check('pull failed',o,.5,pull='failed')
    check('sheath below target',state(.012998),0.)
    check('sheath at target',state(.013),1.)
    translated=state(.013);translated['clamp_bar']=t.BAR.state(pos=(t.X,t.Y+.001,t.BAR_Z))
    check('nominal coverage cannot hide translated bar edge',translated,0.)
    tilted=state();tilted['clamp_bar']=t.BAR.state(pos=(t.X,t.Y,t.BAR_Z-.001),quat_xyzw=(0,math.sin(.04/2),0,math.cos(.04/2)))
    r=t.parts._quat_to_matrix(tilted['clamp_bar'].quat_xyzw)
    for screw,dx in zip(t.SCREWS,(-.018,.018)):
        endz=(tilted['clamp_bar'].pos+r@np.array([dx,0,0]))[2]+.0015
        advance=t.HEAD_BOTTOM-endz
        tilted[screw.name]=screw.state(joints={'advance':advance,'spin':-2*math.pi*advance/t.PITCH})
    check('level difference exceeds 1 mm',tilted,.75)
    badtool=state();badtool['driver']=t.DRIVER.state(pos=(.1,.1,t.TOP+.002))
    check('tool left outside tray',badtool,.75)
    check('cable still moving',o,0.,feed_speed=.01)
    clouds=[np.array([[t.X,t.Y,t.BAR_Z+.002]])]*6
    assert not t.release_from(clouds,o),'face point beside scored bar must fail release'
    print('ok sampled face within 0.5 mm is not released')
    touched=state();t.H.clear();t.H.update(pull='passed',pull_q=[touched[s.name].joints['spin'] for s in t.SCREWS],pull_start=touched['cable'].joints['feed'])
    t.invalidate_proof(False,touched);assert t.H['pull']=='none'
    print('ok intervention invalidates prior proof')
    baseline=state();t.H.clear();t.H.update(pull='passed',pull_q=[baseline[s.name].joints['spin'] for s in t.SCREWS],pull_start=.019,pull_result_feed=.019,pull_ref=t.proof_state(baseline))
    shifted=dict(baseline);shifted['clamp_bar']=t.BAR.state(pos=(t.X+.0002,t.Y,t.BAR_Z))
    t.invalidate_proof(True,shifted);assert t.H['pull']=='none'
    print('ok 0.2 mm bar shift invalidates prior proof')
    parked_on_clamp=state();parked_on_clamp['driver']=t.DRIVER.state(pos=(t.X,t.Y,t.TOP+.040))
    assert not t.tool_clear(parked_on_clamp)
    print('ok released tool on clamp is not clear')
    t.H.clear()
if __name__=='__main__':verify()
