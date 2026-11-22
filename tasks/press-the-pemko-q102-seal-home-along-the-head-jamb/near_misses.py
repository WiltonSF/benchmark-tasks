"""Offline scorer contracts; synthetic states are not robot demonstrations."""
from types import SimpleNamespace
import numpy as np
import task
from measurement import calibration_errors,readiness_errors

CASES = [('untouched loose strip', {}, 0., False)]


def states(home=34, offset=0.):
    out = {o.name: o.state() for o in task.OBJECTS}
    end = np.array([task.HOME if home else task.START_X, -task.LENGTH/2 + offset, task.SEAL_Z])
    for i, s in enumerate(task.SEALS):
        dx = 0. if i < home else max(task.START_X - end[0], -.0025)
        dy = np.sqrt(task.PITCH**2 - dx**2)
        delta = np.array([dx,dy,0.])
        theta = np.arctan2(-dx,dy)
        out[s.name] = s.state(pos=end+delta/2, quat_xyzw=(0.,0.,np.sin(theta/2),np.cos(theta/2)))
        end += delta
    return out


def run():
    checks = 0
    original_sensor=task._sensor
    task._sensor=SimpleNamespace(observed=True,error=None,certified=np.ones(task.N,bool),damaged=False)
    task._state["force_flags"]=np.ones(task.N,bool)
    def check(ok, label):
        nonlocal checks
        assert ok, label
        checks += 1
    for count, want in [(0,0.), (8,0.), (9,.25), (30,.25), (31,.5), (33,.5), (34,1.)]:
        st = states(count)
        n, flush, intact, still = task.measure(st)
        check(n == count, f'actual geometry {count}')
        check(task.tier(n, flush, True, still, 1., intact) == want, f'visible tier {count}')
    for count in range(35):
        for kwargs, label in [({'released':False},'held'), ({'still':False},'moving'),
                              ({'known':False},'unknown'), ({'intact':False},'damage')]:
            args = dict(count=count, flush=True, released=True, still=True, held_s=1., intact=True, known=True)
            args.update(kwargs)
            check(task.tier(**args) == 0., f'{label} {count}')
    check(task.tier(34, True, True, True, .999) == .5, 'one-second boundary below')
    check(task.tier(34, True, True, True, 1.) == 1., 'one-second boundary at')
    st = states(offset=.0021)
    check(not task.measure(st)[1], '2.1mm end error')
    check(task.tier(34, False, True, True, 1.) == .5, 'unflush caps score')
    st = states()
    far = [np.array([[2.,2.,2.]]), np.array([[-2.,-2.,2.]])]
    check(task.faces_clear(st, far), 'distant hands clear')
    check(not task.faces_clear(st, []), 'missing hands fail closed')
    check(not task.faces_clear(st, [far[0], np.array([[np.nan,0.,0.]])]), 'nonfinite hands fail closed')
    # Simulate the middle of a pad face 0.5 mm from the bulb; vertices need not be nearby.
    p = st[task.SEALS[17].name].pos + np.array([-.0079375-.0005,0.,0.])
    check(not task.faces_clear(st, [far[0], np.array([p])]), 'pad-face proximity')
    check(task.tool_clear(st), 'unused tool clear of completed seal')
    tool_holding = dict(st)
    tool_holding[task.BLOCK.name] = task.BLOCK.state(pos=st[task.SEALS[17].name].pos)
    check(not task.tool_clear(tool_holding), 'loose tool wedged against seal fails release')
    unknown_tool=dict(st)
    unknown_tool[task.BLOCK.name]=task.BLOCK.state(pos=(np.inf,0.,task.TOP))
    check(not task.tool_clear(unknown_tool), 'infinite tool position fails release')
    for gap,want in [(.00149,34),(.00151,0)]:
        shifted=dict(st)
        for spec in task.SEALS:
            shifted[spec.name]=spec.state(pos=st[spec.name].pos+(-gap,0.,0.))
        check(task.measure(shifted)[0]==want,f'bulb-gap boundary {gap}')
    theta=np.deg2rad(5.)
    d=.0079375+task.DEPTH
    x=task.FACE+task.DEPTH-.0015-d*np.cos(theta)+.00075*np.sin(theta)+1e-8
    z=task.SEAL_Z+d*np.sin(theta)
    pitched=dict(st)
    for spec in task.SEALS:
        pitched[spec.name]=spec.state(pos=(x,st[spec.name].pos[1],z),quat_xyzw=(0.,np.sin(theta/2),0.,np.cos(theta/2)))
    check(task.measure(pitched)[0]==0,'pitched fin tip does not hide bulb-front gap')
    edge_close=dict(st)
    edge_close[task.BLOCK.name]=task.BLOCK.state(pos=(.22858356997467483,.4152373180950474,.8448422102396095),
        quat_xyzw=(.4426152537512567,.5938422705127144,.0004248162134878919,-.6718950174173042))
    check(not task.tool_clear(edge_close),'9.999mm rotated tool edge remains too close')
    task._state.update(known=False, certificate=None, held=0., damaged=False, clear=False)
    check(task.score(st) == 0., 'full geometry without runtime evidence fails closed')
    check(not task.success(st), 'success matches missing evidence')
    task._state.update(known=True, certificate=task._stamp(st), held=1., damaged=False, clear=True)
    check(task.score(st) == 1. and task.success(st), 'same certified state success matches score')
    task._state['force_flags']=np.zeros(task.N,bool)
    check(task.score(st)==0., 'geometric seating without force/path certificate scores zero')
    task._state['force_flags']=np.ones(task.N,bool)
    task._state['invalid'] = True
    check(task.score(st) == 0., 'historical unknown interval persists')
    task._state['invalid'] = False
    task._state['damaged'] = True
    check(task.score(st) == 0., 'historical damage persists after geometry restored')
    task._state['damaged'] = False
    moved = dict(st)
    moved[task.SEALS[0].name] = task.SEALS[0].state(pos=st[task.SEALS[0].name].pos + (.0001,0.,0.))
    check(task.score(moved) == 0., 'stale runtime certificate')
    for count,want in [(9,.25),(30,.25),(31,.5),(33,.5),(34,1.)]:
        candidate=states(count)
        task._state.update(known=True,invalid=False,damaged=False,held=1.,clear=True,certificate=task._stamp(candidate))
        check(task.score(candidate)==want, f'certified full scorer tier {count}')
    import release
    old_observer = release.hand_clouds
    release.hand_clouds = lambda sim: far
    try:
        task._state.update(known=False,invalid=False,damaged=False,held=0.,time=None,certificate=None,clear=False,eligible=False)
        for i in range(20):
            task.step(SimpleNamespace(t=i*.05, states=lambda:st))
        check(task.score(st)==.5, 'runtime 0.95 second hold remains partial')
        task.step(SimpleNamespace(t=1., states=lambda:st))
        check(task.score(st)==1., 'runtime full second hold earns success')
        release.hand_clouds = lambda sim: []
        task.step(SimpleNamespace(t=1.05, states=lambda:st))
        check(task.score(st)==0., 'runtime unknown hands block score')
        release.hand_clouds = lambda sim: far
        task.step(SimpleNamespace(t=1.1, states=lambda:st))
        check(task.score(st)==0., 'runtime unknown interval remains invalid')
    finally:
        release.hand_clouds = old_observer
    task._state.update(known=False,invalid=False,certificate=None,held=0.,damaged=False,clear=False,eligible=False,time=None)
    check(bool(calibration_errors()), 'missing physical measurement is reported')
    import json
    record=json.loads((task.Path(__file__).resolve().parent/'calibration.json').read_text())
    for key in ['kerf_width_mm','kerf_depth_mm','pressed_length_mm']:
        bad=dict(record);bad[key]=float('nan')
        check(any(key in error for error in calibration_errors(bad)),f'nonfinite calibration {key}')
    check(any('Native' in error for error in readiness_errors(record)), 'readiness needs native resistance comparison')
    task._sensor=original_sensor
    task._state['force_flags']=np.zeros(task.N,bool)
    import force_controls
    checks+=force_controls.run()
    print(f'PASS {checks} offline scoring, release and evidence checks. No physical or GPU validation.')


if __name__ == '__main__':
    run()
