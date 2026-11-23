"""Controls for shaft contact, hidden intact clumps, release and conservative mass."""
import math
import numpy as np
import task
CASES=[('untouched reset',{},0.,False),('missing hand provenance',{},0.,False),
       ('rotated basket alone is not progress',{'portafilter':task.BASKET.state(quat_xyzw=(0,0,math.sin(.2),math.cos(.2)))},0.,False),
       ('parked tool alone is not progress',{'wdt':task.TOOL.state()},0.,False)]

def run():
    checks=0
    def check(label,condition):
        nonlocal checks
        assert condition,label
        checks+=1
    check('untouched has six clumps',task.Coffee().count()==6)
    discarded=task.Coffee();discarded.broken[:len(task.PAIRS)]=True
    discarded.grains[len(task.LOCAL):,0]+=.10
    check('discarding intact clumps is detected despite adequate mass',discarded.mass()>=.019 and discarded.escaped_clump())
    check('true span excludes empty box corners',task.span([[.001,0,0],[-.001,0,0],[0,.001,0],[0,-.001,0]])<.003)
    for count,relief,mass,park,want in [(6,.001,.020,True,0),(5,.001,.020,True,.25),(2,.004,.019,False,.5),(2,.004001,.019,True,.25),(0,.002,.019,False,.75),(0,.002,.019,True,1),(0,.002001,.020,True,.5),(0,.001,.018999,True,0)]:
        check('tier boundary',task.tier(count,relief,mass,park)==want)
    b=task.BASKET.state();coffee=task.Coffee()
    for t in np.arange(0,2,.05):
        p=np.array(b.pos)+np.array([.01*math.cos(t*5),.01*math.sin(t*5),.16])
        coffee.advance(task.TOOL.state(pos=p),b,.05,applied=True);coffee.prev_force=coffee.next_force.copy()
    check('above-bed loop cannot cut',not coffee.broken.any())
    check('above-bed loop cannot level',np.max(np.abs(coffee.volume-task.Coffee().volume))==0)
    check('above-bed mass conserved',abs(coffee.mass()-.020)<1e-12)
    # Bury every intact clump without severing its graph: counting ignores occlusion.
    coffee.grains[:,2]-=.008
    check('buried intact clumps count',coffee.count()==6)
    check('unknown release fails closed',not task.released([],{}))
    # Hand-face points lie beside the middle of the body, not at hull vertices.
    objects={p.name:p.state() for p in task.OBJECTS}
    point=np.asarray(task.TOOL.pos)+[.0159,0,0]
    check('face0.4mm from tool fails release',not task.released([point.reshape(1,3)],objects))
    saved=task.REC
    try:
        task.REC=task.fresh()
        for t in np.arange(21)/20:task.update_hold(float(t),objects,True)
        check('pose stillness needs full1s',abs(task.REC['hold']-1.)<1e-9)
        task.update_hold(1.05,objects,False)
        check('hand reentry clears hold',task.REC['hold']==0.)
        task.REC=task.fresh()
        for t in np.arange(21)/20:
            moved=dict(objects);moved['wdt']=task.TOOL.state(pos=np.asarray(task.TOOL.pos)+[.001001*t,0,0])
            task.update_hold(float(t),moved,True)
        check('more than1mm motion resets hold',task.REC['hold']==0.)
    finally:task.REC=saved
    tray=dict(objects);tray['portafilter']=task.BASKET.state(pos=np.asarray(task.MACHINE.pos)+[-.105,0,.040])
    check('original appliance tray remains a legal support',task.basket_supported(tray))
    airborne=dict(objects);airborne['portafilter']=task.BASKET.state(pos=np.asarray(task.BASKET.pos)+[0,0,.030])
    check('airborne basket has no invented surface support',not task.basket_supported(airborne))
    check('shaft-side scraping rejected when distal tip is inside',bool(task.shaft_damage([.024,0,.020],[.028,0,.060])))
    check('clear upright shaft inside basket is safe',not task.shaft_damage([0,0,.024],[0,0,.074]))
    check('shaft-floor penetration rejected',bool(task.shaft_damage([0,0,.016],[0,0,.066])))
    saved=task.REC
    try:
        far=[np.array([[10.,10.,10.]])]
        task.REC=task.fresh();task.safety_history(objects,far)
        check('native counter support is allowed',not task.REC['bad'])
        raised=dict(objects);raised['portafilter']=task.BASKET.state(pos=np.asarray(task.BASKET.pos)+[0,0,.030])
        task.safety_history(raised,far)
        check('upright unsupported release/drop latches zero',task.REC['bad'])
        task.safety_history(objects,far)
        check('drop violation persists after upright landing',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3);hand=np.asarray(raised['portafilter'].pos)+[0,-.097,.028]
        task.safety_history(raised,[hand.reshape(1,3)],com_velocity=np.zeros(3))
        check('legal hand-supported basket lift allowed',not task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        falling=dict(raised);falling['portafilter']=task.BASKET.state(pos=raised['portafilter'].pos,lin_vel=(0,0,-.4905))
        task.safety_history(falling,[hand.reshape(1,3)],com_velocity=falling['portafilter'].lin_vel)
        check('close hand following unsupported freefall latches zero',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        task.safety_history(falling,[hand.reshape(1,3)],com_velocity=np.array([0,0,-.4]))
        check('light upward touch cannot excuse falling basket',task.REC['bad'])
        task.safety_history(objects,far)
        check('close-hand freefall violation persists after landing',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        near=hand+np.array([.016,0,0])
        task.safety_history(raised,[near.reshape(1,3)],com_velocity=np.zeros(3))
        check('hand2mm off native handle does not prove support',task.REC['bad'])
        task.REC=task.fresh();task.safety_history(raised,[hand.reshape(1,3)])
        check('unsupported basket with unknown reaction fails closed',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3);task.safety_history(raised,[],com_velocity=np.zeros(3))
        check('unknown hand cannot support airborne basket',task.REC['bad'])
        task.REC=task.fresh()
        for t in np.arange(21)/20:
            moved=dict(objects);a=.001*math.sin(t*100)
            moved['wdt']=task.TOOL.state(pos=np.asarray(task.TOOL.pos)+[.00005*math.sin(t*100),0,0],quat_xyzw=(0,0,math.sin(a/2),math.cos(a/2)))
            task.update_hold(float(t),moved,True)
        check('subthreshold native pose jitter permits full1s',task.REC['hold']>=1.-1e-9)
    finally:task.REC=saved
    saved_c,saved_r=task.COFFEE,task.REC
    try:
        task.COFFEE=task.Coffee();task.REC=task.fresh()
        inverted=dict(objects);inverted['wdt']=task.TOOL.state(pos=np.asarray(task.BASKET.pos)+[0,0,.0615],quat_xyzw=(1,0,0,0))
        task.body_safety(inverted)
        check('inverted cap inside dose latches forbidden-body zero',task.REC['bad'])
        task.body_safety(objects)
        check('restoring parked tool cannot clear body violation',task.REC['bad'])
        task.REC=task.fresh();horizontal=dict(objects);horizontal['wdt']=task.TOOL.state(pos=np.asarray(task.BASKET.pos)+[0,0,.045],quat_xyzw=(0,math.sin(math.pi/4),0,math.cos(math.pi/4)))
        task.body_safety(horizontal)
        check('horizontal body rim touches dose volume',task.REC['bad'])
        task.REC=task.fresh();a=dict(objects);sweep_end=dict(objects)
        a['wdt']=task.TOOL.state(pos=np.asarray(task.BASKET.pos)+[-.06,0,.055]);sweep_end['wdt']=task.TOOL.state(pos=np.asarray(task.BASKET.pos)+[.06,0,.055])
        task.body_safety(a);check('body entirely beside dose is safe',not task.REC['bad'])
        task.body_safety(sweep_end);check('body swept through dose between safe endpoints latches zero',task.REC['bad'])
        task.REC=task.fresh();safe=dict(objects);safe['wdt']=task.TOOL.state(pos=np.asarray(task.BASKET.pos)+[0,0,.102])
        task.body_safety(safe);check('normal loaded-needle body clearance remains legal',not task.REC['bad'])
        turn_start=dict(objects);turn_end=dict(objects);turn_pos=np.asarray(task.BASKET.pos)+[.045,0,.045]
        turn_start['wdt']=task.TOOL.state(pos=turn_pos);turn_end['wdt']=task.TOOL.state(pos=turn_pos,quat_xyzw=(0,1,0,0))
        task.REC=task.fresh();task.body_safety(turn_start);check('upright rotational endpoint outside dose',not task.REC['bad'])
        task.REC=task.fresh();task.body_safety(turn_end);check('inverted rotational endpoint outside dose',not task.REC['bad'])
        task.REC=task.fresh();task.body_safety(turn_start);task.body_safety(turn_end)
        check('rotation-only swept entry between clear endpoints latches zero',task.REC['bad'])
        from dataclasses import replace
        task.REC=task.fresh();fast=dict(objects);fast['wdt']=replace(task.TOOL.state(),ang_vel=np.array([0,100.,0]))
        task.body_safety(fast);check('unresolved fast body spin fails closed',task.REC['bad'])
        task.COFFEE=task.Coffee();task.COFFEE.volume[:]=0;task.COFFEE.active[:]=False;task.COFFEE.mass_grains=0
        check('empty basket has no invented coffee-volume collision',not task.body_in_dose(np.array([0,0,.04]),np.eye(3)))
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        misleading=dict(raised);misleading['portafilter']=task.BASKET.state(pos=raised['portafilter'].pos,lin_vel=(0,0,5))
        task.safety_history(misleading,[hand.reshape(1,3)],com_velocity=np.array([0,0,-.4905]))
        check('rotating origin cannot mask zero COM support reaction',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        task.safety_history(raised,[hand.reshape(1,3)],com_velocity=np.array([0,0,float('nan')]))
        check('nonfinite COM support data fails closed',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        task.safety_history(raised,[np.array([[float('nan'),0,0]])],com_velocity=np.zeros(3))
        check('invalid hand with known upward reaction fails closed',task.REC['bad'])
    finally:task.COFFEE=saved_c;task.REC=saved_r
    from dataclasses import replace
    static=task.Coffee();before=static.volume.copy()
    for _ in range(20):static.transport(task.BASKET.state(),.05)
    check('counter no-op has no autonomous bulk flow',np.array_equal(static.volume,before))
    gentle=task.Coffee();before=gentle.volume.copy()
    for i in range(20):gentle.transport(replace(task.BASKET.state(),lin_vel=np.array([.01,0,0])),.05)
    check('gentle carry below friction yield retains field',np.array_equal(gentle.volume,before))
    shaken=task.Coffee()
    for i in range(80):shaken.transport(replace(task.BASKET.state(),lin_vel=np.array([.5 if i%4<2 else -.5,0,0])),.05)
    check('vigorous basket jerks cause finite spill',shaken.spilled>.001)
    check('basket transport conserves all finite mass',abs(shaken.mass()+shaken.spilled-.020)<1e-12)
    check('basket shaking cannot replace needle bond rupture',shaken.count()==6 and not shaken.broken.any())
    saved_c,saved_r=task.COFFEE,task.REC
    try:
        task.COFFEE=task.Coffee();task.REC=task.fresh();task.wire_safety(objects)
        check('original cradle reset has no needle/solid collision',not task.COFFEE.damage)
        hit=dict(objects);hit['wdt']=task.TOOL.state(pos=np.asarray(task.STAND.pos)+[0,.002,.1477])
        task.REC=task.fresh();task.wire_safety(hit)
        check('needle entering cradle prong latches damage',task.COFFEE.damage)
        task.wire_safety(objects)
        check('needle damage persists after return',task.COFFEE.damage)
        planes=task.box_planes(np.array([.02,.02,.02]))
        check('full shaft through solid hand geometry rejected',task.segment_in_solid(np.array([0,0,-.02]),np.array([0,0,.02]),planes))
        check('parallel clear shaft is allowed',not task.segment_in_solid(np.array([.02,0,-.02]),np.array([.02,0,.02]),planes))
    finally:task.COFFEE,task.REC=saved_c,saved_r
    # Real contacted sweep changes bonds and surface, while mass is retained.
    coffee=task.Coffee();p=np.array(b.pos)+[0,0,.102];coffee.advance(task.TOOL.state(pos=p),b,.05,applied=True);coffee.prev_force=coffee.next_force.copy()
    old=np.array([-.009,-.009,.102])
    for swap in (False,True):
        for i,y in enumerate(np.arange(-.009,.00901,.0006)):
            xs=np.arange(-.009,.00901,.0004)
            if i%2:xs=xs[::-1]
            for x in xs:
                end=np.array([y,x,.102] if swap else [x,y,.102]);n=max(1,int(np.ceil(np.linalg.norm(end-old)/.0004)))
                for alpha in np.linspace(0,1,n+1)[1:]:
                    p=np.array(b.pos)+old+alpha*(end-old)
                    coffee.advance(task.TOOL.state(pos=p),b,.05,applied=True);coffee.prev_force=coffee.next_force.copy()
                old=end
    check('loaded contact breaks bonds',coffee.broken.any() and coffee.total_work>0)
    check('loaded reaction opposes actual motion',coffee.peak_load>0)
    check('sweep conserves dose',abs(coffee.mass()+coffee.spilled-.020)<1e-12)
    check('safe sweep avoids declared damage',not coffee.damage)
    check('dense loaded sweep fully disperses',coffee.count()==0)
    check('whole-bed relief under2mm',coffee.relief()<=.002)
    no_load=task.Coffee()
    for x in np.linspace(.004,.010,100):
        no_load.advance(task.TOOL.state(pos=np.asarray(b.pos)+[x,0,.102]),b,.05,applied=False)
    check('prescribed path without applied reaction cannot cut',not no_load.broken.any())
    saved_coffee,saved_record=task.COFFEE,task.REC
    try:
        task.COFFEE=coffee;task.REC=task.fresh()
        task.REC.update(known=True,bad=False,hold=1.,certificate=task.stamp(objects))
        check('full release certificate matches success and score',task.score(objects)==1. and task.success(objects))
        task.body_safety(inverted);task.body_safety(objects)
        check('inverted cap after legitimate progress and restored park scores zero',task.score(objects)==0.)
        task.REC=task.fresh();task.REC.update(known=True,hold=1.,certificate=task.stamp(objects),support_velocity=np.zeros(3))
        task.safety_history(falling,[hand.reshape(1,3)],com_velocity=falling['portafilter'].lin_vel);task.safety_history(objects,far)
        check('close-hand drop after legitimate progress and landing scores zero',task.score(objects)==0.)
        task.REC=task.fresh();task.REC.update(known=True,hold=1.,certificate=task.stamp(objects))
        task.COFFEE=discarded
        check('discarded intact aggregates get no partial credit',task.score(objects)==0.)
        task.COFFEE=coffee
        task.REC['hold']=.999
        check('hold below1s gets zero',task.score(objects)==0.)
        task.REC['hold']=1.;task.REC['known']=False
        check('unknown hand data gets zero after progress',task.score(objects)==0.)
        task.REC['known']=True;task.REC['bad']=True
        check('whole-trial violation survives completed bed',task.score(objects)==0.)
        task.REC['bad']=False
        moved=dict(objects);moved['wdt']=task.TOOL.state(pos=np.asarray(task.TOOL.pos)+[.005,0,0])
        check('stale observation certificate fails closed',task.score(moved)==0.)
    finally:task.COFFEE,task.REC=saved_coffee,saved_record
    class NativeLikeArray:
        def numpy(self):return np.array([[1.,2.,3.]])
    check('native Warp-style array conversion is supported',np.array_equal(task.native_array(NativeLikeArray()),[[1.,2.,3.]]))
    # Real-surface regression: a round base's AABB corner is not contact.
    saved_c,saved_r=task.COFFEE,task.REC
    try:
        gap=dict(objects);gap['portafilter']=task.BASKET.state(pos=(.251,.190,task.TOP+.005))
        actual=dict(objects);actual['portafilter']=task.BASKET.state(pos=(.21,.16,task.TOP+.005))
        check('10.725mm round-base corner gap is unsupported',not task.basket_supported(gap))
        check('actual spout-to-round-base contact is supported',task.basket_supported(actual))
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        task.safety_history(gap,far,com_velocity=np.zeros(3),require_load=True)
        check('corner gap cannot exempt even a known weight-sized reaction',task.REC['bad'])
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        task.safety_history(actual,far,com_velocity=np.zeros(3),require_load=True)
        check('loaded actual stand contact remains legal',not task.REC['bad'])
        task.REC=task.fresh();task.safety_history(objects,far,require_load=True)
        check('native surface query cannot exempt unknown load',task.REC['bad'])
        # A 24.5ms ballistic hop is absent at both 20Hz endpoints. Use the real
        # physics-history handler at each 1/240s sample; this is a state control,
        # not a native robot trajectory or physical specimen measurement.
        from types import SimpleNamespace
        old_body,old_hand,old_hand_solids=task.native_body,task.native_hand_contact,task.native_hand_solids
        phase={'t':0.};dt=1/240
        def body(base,spec):
            if spec is not task.BASKET:return spec.state(),np.zeros(3)
            tau=max(0.,phase['t']-dt);height=.12*tau-.5*9.81*tau*tau
            airborne=height>0
            state=task.BASKET.state(pos=np.asarray(task.BASKET.pos)+[0,0,max(0.,height)])
            return state,np.array([0.,0.,.12-9.81*tau if airborne else 0.])
        task.native_body=body;task.native_hand_contact=lambda base,objects,solids=None:False
        task.native_hand_solids=lambda base:[task.solid(task.parts.box((.01,.01,.01)),task.TOOL.state(pos=(10.,10.,10.)))]
        task.COFFEE=coffee;task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        sim=SimpleNamespace(env=SimpleNamespace(unwrapped=object()))
        check('brief hop starts in valid counter contact',task.basket_supported(objects))
        for k in range(1,13):
            phase['t']=k*dt;task.physics_history(sim,dt)
        check('physics witness ran all12 substeps',task.REC['physics_steps']==12)
        check('brief hop ends in valid counter contact',task.basket_supported(objects))
        check('sub-control-interval unsupported hop latches zero',task.REC['bad'])
        task.REC.update(known=True,hold=1.,certificate=task.stamp(objects))
        check('brief drop after progress and restored park cannot score',task.score(objects)==0.)
        check('brief-drop handler also checked hands on every tick',task.REC['physics_hand_checks']==12)
        # Supported basket throughout: only a between-control hand entry can fail.
        task.REC=task.fresh();task.REC['support_velocity']=np.zeros(3)
        task.native_body=lambda base,spec:(spec.state(),np.zeros(3))
        far_solid=task.solid(task.parts.box((.01,.01,.01)),task.TOOL.state(pos=(10.,10.,10.)))
        inside_solid=task.solid(task.parts.box((.008,.008,.008)),task.TOOL.state(pos=np.asarray(task.BASKET.pos)+[0,0,.041]))
        frame={'k':0}
        task.native_hand_solids=lambda base:[inside_solid if frame['k']==4 else far_solid]
        for k in range(1,13):
            frame['k']=k;task.physics_history(sim,dt)
        check('supported basket never skips240Hz forbidden-hand check',task.REC['physics_hand_checks']==12)
        check('between-control hand entry latches on supported basket',task.REC['bad'] and task.REC['why']=='physics-step hand inside basket')
        task.REC.update(known=True,hold=1.,certificate=task.stamp(objects))
        check('hand entry after progress survives clear final endpoints',task.score(objects)==0.)

        task.native_body,task.native_hand_contact,task.native_hand_solids=old_body,old_hand,old_hand_solids
        task.REC=task.fresh();task.REC.update(known=True,hold=1.,certificate=task.stamp(objects),support_velocity=np.zeros(3))
        task.safety_history(gap,far,com_velocity=np.zeros(3),require_load=True);task.safety_history(objects,far)
        check('corner-gap violation after progress survives landing',task.score(objects)==0.)
    finally:
        if 'old_body' in locals():task.native_body,task.native_hand_contact,task.native_hand_solids=old_body,old_hand,old_hand_solids
        task.COFFEE,task.REC=saved_c,saved_r
    # Exercise the actual task-local physics dispatcher, including reset gating.
    old_callbacks=(task.parts._ON_STEP,task.parts._ON_RESET,task.parts._ON_PHYSICS_STEP)
    try:
        calls=[]
        task.parts._ON_STEP=[];task.parts._ON_RESET=[];task.parts._ON_PHYSICS_STEP=[lambda sim,dt:calls.append(dt)]
        class Engine:
            def step(self):pass
        class FakeEmbodiment:
            _env=None;_t=0.
            def _task_step(self):return None
            def reset(self,scene,seed=None):
                self._base=SimpleNamespace(sim=Engine(),physics_dt=1/240)
                self._env=SimpleNamespace(unwrapped=self._base)
                for _ in range(3):self._base.sim.step()
            def _observation(self):return None
            def step(self,action):
                for _ in range(12):self._base.sim.step()
                return self._task_step()
        task.parts._patch_embodiment(SimpleNamespace(YamArenaEmbodiment=FakeEmbodiment))
        fake=FakeEmbodiment();fake.reset(None)
        check('reset settling is excluded from episode history',len(calls)==0)
        fake.step(None)
        check('action dispatch invokes witness at240Hz, not20Hz',len(calls)==12 and all(abs(dt-1/240)<1e-12 for dt in calls))
        fake._base.sim.step()
        check('out-of-action settling does not pollute episode latch',len(calls)==12)
    finally:task.parts._ON_STEP,task.parts._ON_RESET,task.parts._ON_PHYSICS_STEP=old_callbacks
    print(dict(checks=checks,clumps=coffee.count(),broken=int(coffee.broken.sum()),relief=coffee.relief(),mass=coffee.mass(),work=coffee.total_work,peak_load=coffee.peak_load))
    return checks
if __name__=='__main__':run()
