"""Scorer and release-rule counterexamples (synthetic states, not robot trials)."""
import math
import numpy as np
import parts


class Cases:
    def __iter__(self):
        m = parts._tablet_namespace
        TOP, PT, PH, N, PX, PY = m['TOP'], m['PT'], m['PH'], m['N'], m['PX'], m['PY']
        HOLES, NAMES, PLATE, SPREADER = m['HOLES'], m['NAMES'], m['PLATE'], m['SPREADER']
        S = m['S']
        LAND = m['BAR'] + m['RISE']                     # joint travel at which a tablet rests on the parchment
        laid = SPREADER.state(pos=(.30, -.30, TOP + m['CT'] / 2), quat_xyzw=(math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))
        def plate(q=None, pos=None, quat=None):
            return PLATE.state(pos=pos, quat_xyzw=quat)
        def batch(k_out, extra=None):
            q = {k: LAND for k in k_out} | (extra or {})
            return {'cavity_plate': plate(), 'spreader': laid} | {
                NAMES[k]: m['TABLETS'][k].state(pos=(m['TABLETS'][k].pos[0], m['TABLETS'][k].pos[1], m['TAB_Z'] - v))
                for k, v in q.items()}
        def rec(held=None, **kw):
            S.clear(); S.update(m['fresh']())
            S.update(known=True, phys=50, steps=50, t=3.0, clear=0., still=0.)
            S['held'] = np.ones(N, bool) if held is None else held
            S['freed'] = ~S['held']
            S.update(kw)
        def emit(label, change, want, win=False, n_out=None, **kw):
            held = np.ones(N, bool)
            if n_out is not None: held[:n_out] = False
            rec(held=held, **kw); return (label, change, want, win)
        every = list(range(N))
        yield emit('batch untouched in the plate', {}, 0.)
        yield emit('all 60 out on the parchment, spreader laid down, hands clear and still 1 s', batch(every), 1., True, n_out=N)
        yield emit('all out but still only 0.999 s', batch(every), .75, n_out=N, t=.999 + 0., still=0., clear=-1.)
        yield emit('all out, hands clear only 0.999 s', batch(every), 0., n_out=N, t=.999)
        yield emit('all out but a hand within 30 mm', batch(every), 0., n_out=N, clear=None)
        yield emit('arm state unknown', batch(every), 0., n_out=N, known=False)
        yield emit('no physics contact history', batch(every), 0., n_out=N, phys=0)
        for n, want in ((59, .75), (45, .75), (44, .5), (30, .5), (29, .25), (15, .25), (14, 0.)):
            yield emit(f'{n} out on the parchment, the rest still in the plate', batch(range(n)), want, n_out=n)
        for why in ('6.2 N pushed down on the plate', '4.1 N on one tablet', 'a gripper touched a tablet', 'plate knocked'):
            yield emit(why + ': stays 0', batch(every), 0., n_out=N, zero=True, why=why)
        # Tablets that do not count.
        yield emit('one tablet hanging 1.1 mm above the parchment', batch(every, {0: LAND - .0011}), .75, n_out=N)
        yield emit('one tablet half out of its hole', batch(every, {0: .002}), .75, n_out=N)
        rec(held=np.zeros(N, bool)); S['tab_v'] = np.zeros(N); S['tab_v'][0] = .02
        yield ('one tablet still moving at 20 mm/s', batch(every), .75, False)
        yield emit('tablets on the parchment but marked held (not let go by the rule)', batch(every), 0., n_out=0)
        moved = plate({k: LAND for k in every}, pos=(PX, PY - .0105, PLATE.pos[2]))
        yield emit('plate slid 10.5 mm on its bars', batch(every) | {'cavity_plate': moved}, 0., n_out=N)
        a = math.radians(6) / 2
        yield emit('plate tipped 6 degrees', {'cavity_plate': plate(quat=(math.sin(a), 0, 0, math.cos(a))), 'spreader': laid}, 0., n_out=N)
        yield emit('plate slid 9.5 mm (still on its bars)', batch(every) | {'cavity_plate': plate(pos=(PX, PY - .0095, PLATE.pos[2]))}, 1., True, n_out=N)
        # Verify the contact adapter sums compressive contact points before netting opposing forces.
        from types import SimpleNamespace
        class ContactView:
            def get_contact_data(self, dt):
                forces = np.array([[.4], [.2], [.3]])
                normals = np.array([[0., 0., -1.], [0., 0., 1.], [0., 0., -1.]])
                counts = np.zeros((N+1, 2), dtype=int); starts = np.zeros_like(counts)
                counts[1, 0] = 2; counts[2, 0] = 1; starts[2, 0] = 2
                return forces, np.tile([0.,0.,-m['CH']/2-m['PIN_L']],(3,1)), normals, np.zeros(3), counts, starts
        rec(); S['views'] = (ContactView(), 2); S['native_poses']=np.tile([0.,0.,0.,0.,0.,0.,1.],(N+5,1))
        sim = SimpleNamespace(env=SimpleNamespace(unwrapped=SimpleNamespace(physics_dt=1./240,scene=SimpleNamespace(env_origins=np.zeros((1,3))))))
        measured, total, hand, plate_f, all_push, nonpin = m['contact_loads'](sim, np.array([0.,0.,1.]))
        assert abs(measured[0] - .4) < 1e-12 and abs(measured[1] - .3) < 1e-12
        assert abs(total - .7) < 1e-12 and hand == 0. and plate_f == 0. and nonpin == 0.
        class FrameView(ContactView):
            def get_contact_data(self,dt):
                f,p,n,sep,c,st=super().get_contact_data(dt); p[0]=[0.,0.,.0225]; return f,p,n,sep,c,st
        S['views']=(FrameView(),2)
        pin,total,hand,plate_signed,all_push,nonpin=m['contact_loads'](sim,np.array([0.,0.,1.]))
        assert pin[0]==0. and abs(all_push[0]-.4)<1e-12 and abs(nonpin-.4)<1e-12
        m['rule_step'](pin,hand,total,0.,np.full(N,m['STROKE']),.001,all_push=all_push,nonpin_force=nonpin)
        assert not S['freed'].any() and S['zero']
        yield ('fixed sensor frame force and full stroke cannot substitute for pin contact',{},0.,False)
        yield ('opposing 0.2 N does not cancel 0.4 N compressive contact, neighbor isolated', {}, 0., False)
        class TwoHandsView:
            def get_contact_data(self,dt):
                counts=np.zeros((N+1,3),int); starts=np.zeros_like(counts)
                counts[1,1:]=1; starts[1,2]=1
                return np.array([.015,.015]),np.zeros((2,3)),np.array([[0.,0.,1.],[0.,0.,1.]]),np.zeros(2),counts,starts
        rec(); S['views']=(TwoHandsView(),3); S['native_poses']=np.tile([0.,0.,0.,0.,0.,0.,1.],(N+5,1))
        pin,total,hand,plate_signed,all_push,nonpin=m['contact_loads'](sim,np.array([0.,0.,1.]))
        assert abs(hand-.03)<1e-12
        m['rule_step'](pin,hand,total,0.,np.full(N,m['STROKE']),.001,all_push=all_push,nonpin_force=nonpin)
        assert S['zero'] and not S['freed'].any()
        yield ('two gripper links at0.015N each cannot bypass0.02N tablet touch limit',{},0.,False)
        # Release rule, driven through the real rule code: per-tablet push of the spreader on that tablet's face.
        F_REL, F_TAB, F_HI = m['F_REL'], m['F_TAB'], m['F_HI']
        def rule(p, h, total, t=0., travel=None, support_gap=.001):
            return m['rule_step'](p, h, total, t, np.full(N, m['STROKE']) if travel is None else travel, support_gap)
        def push(ks=(), f=0.):
            p = np.zeros(N); p[list(ks)] = f; return p
        rec(); out = rule(push(), 0., 3.0)
        assert out.sum() == 0 and not S['freed'].any() and not S['zero']
        yield ('spreader pressing the plate at 3 N with no contact on any tablet: nothing lets go', {}, 0., False)
        rec(); out = rule(push([0], F_REL), 0., 1., travel=np.full(N, m['STROKE'] - .00001))
        assert not out.any() and not S['freed'].any()
        yield ('0.40 N without the full 2.2 mm resisted stroke does not free a tablet', {}, 0., False)
        rec(); travel=np.full(N, m['STROKE']); travel[0]=0.
        out=rule(push([0], F_REL), 0., 1., travel=travel)
        assert not out.any()
        yield ('neighbor stroke does not substitute for target stroke', {}, 0., False)
        rec(); out = rule(push([0], F_REL), 0., 1., support_gap=0.)
        assert not out.any() and S['zero']
        yield ('loaded push while plate parked: zero, no credited release', {}, 0., False)
        rec(); out = rule(push([0], F_REL - .01), 0., 1.0)
        assert out.sum() == 0
        rec(); out = rule(push([0], F_REL), 0., 1.0)
        assert out.sum() == 1 and S['freed'][0] and not S['held'][0] and S['held'][1:].all()
        yield ('0.39 N on tablet 0 frees nothing; 0.40 N frees tablet 0 only', {}, 0., False)
        rec(); out = rule(push([1, 6], F_REL - .01), 0., 2.0)
        assert out.sum() == 0 and not S['freed'][0]
        yield ('pushing its neighbours (tablets 1 and 6) at 0.39 N does not free tablet 0', {}, 0., False)
        rec(); rule(push(range(6), F_REL), 0., 6 * F_REL); assert S['freed'][:6].all() and S['freed'].sum() == 6
        yield ('a row pushed at 0.40 N each lets go; the others stay', batch(range(6)), 0., False)
        rec(); rule(push([0], F_TAB), 0., F_TAB); assert not S['zero']
        rec(); rule(push([0], F_TAB + .01), 0., F_TAB + .01); assert S['zero']
        yield ('4.01 N on one tablet crushes it (4.00 N does not)', batch(every), 0., False)
        rec(); rule(push(), 0., F_HI); assert not S['zero']
        rec(); rule(push(), 0., F_HI + .01); assert S['zero']
        yield ('6.01 N exceeds the protocol plate-load limit (6.0 N does not)', batch(every), 0., False)
        rec(); rule(push(), 0., 5.9); assert not S['zero']
        yield ('a gripper steadying the plate at 5.9 N is allowed', {}, 0., False)
        rec(); rule(push(), .021, 0.); assert S['zero']
        yield ('a fingertip pushing on a tablet (0.021 N)', batch(every), 0., False)
        rec(); rule(push([0], F_REL), .021, 1.); assert S['zero'] and not S['freed'].any()
        yield ('spreader and fingertip on a tablet together: zero, nothing freed', batch(every), 0., False)
        for bad in (push()[:-1], np.full(N, np.nan)):
            rec()
            try: rule(bad, 0., 1.); raise AssertionError('unknown contact state accepted')
            except ValueError: pass
        yield ('unknown contact state is refused (the hook then scores 0)', {}, 0., False)
        # Sweep row by row above F_REL per tablet: everything lets go; the observer then scores it.
        rec()
        for r in range(m['NROW']): rule(push(range(6 * r, 6 * r + 6), F_REL + .05), 0., 6 * (F_REL + .05))
        assert not S['held'].any() and S['freed'].all() and not S['zero']
        start = {o.name: o.state() for o in m['OBJECTS']}
        observed = start | batch(every); far = np.array([[-.4, -.5, TOP + .5]])
        S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        for j in range(21): m['observe'](observed, far, j * .05)
        yield ('a sweep pushing each tablet at 0.45 N, all fall, observed 1 s still and clear', batch(every), 1., True)
        # A rotated local bounding BOX extends below the true convex hull; use actual collider vertices.
        q=(-.753,-.025,.48,.45); r=parts._quat_to_matrix(q)
        vv=np.asarray(m['_tp'])@r.T; z=PT-float(vv[:,2].min())
        edge=m['TABLETS'][0].state(pos=(PX,PY,z),quat_xyzw=q)
        assert edge.low[2] < TOP-.001
        edge_change=batch(every)|{NAMES[0]:edge}; edge_observed=start|edge_change
        rec(held=np.zeros(N,bool)); S.update(steps=0,known=False,prev=None,clear=None,still=None,t=None)
        for j in range(21):m['observe'](edge_observed,far,j*.05)
        assert not S['zero'] and not m['on_parchment'](edge_observed,0)
        assert abs(m['tablet_box'](edge_observed,0)[0][2]-PT)<1e-9
        yield ('tilted real hull on parchment is not off-counter, but remains nonflat',edge_change,.75,False)
        for label, local in (('hand on connector', m['FOOT_C']), ('hand on grip tab', m['HANDLE_C']),('hand on grip stop cap',m['CAP_C'])):
            rec(held=np.zeros(N, bool)); S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
            h=np.array([np.asarray(PLATE.pos)+local])
            for j in range(21): m['observe'](observed, h, j*.05)
            yield (label+' cannot be called hands clear', batch(every), 0., False)
        for dz,want in ((.0299,False),(.0301,True)):
            h=np.array([np.asarray(PLATE.pos)+m['CAP_C']+np.array([0.,0.,m['CAP_S'][2]/2+dz])])
            assert m['hands_clear'](h,observed)==want
        yield emit('hand 29.9 mm above new stop cap is not clear',batch(every),0.,n_out=N,clear=None)
        upright=m['SPREADER'].state(pos=(.3,-.3,TOP+.1))
        h=np.array([upright.pos+np.array([0.,.006+.0299,.0225])])
        assert not m['hands_clear'](h, observed|{'spreader':upright})
        yield emit('hand 29.9 mm from wide tool boss is not clear', batch(every), 0., n_out=N, clear=None)
        for spec in [*m['BARS'],m['PARK'],m['BLOCK']]:
            rec(held=np.zeros(N,bool)); S.update(steps=0,known=False,prev=None,clear=None,still=None,t=None)
            moved=spec.state(pos=np.asarray(m['lots'].current().pos(spec.name))+np.array([.0051,0.,0.]))
            m['observe'](observed|{spec.name:moved},far,0.)
            for j in range(1,21):m['observe'](observed,far,j*.05)
            assert S['zero']
            yield ('parking fixture moved5.1 mm then returned: '+spec.name,batch(every),0.,False)
        rec(held=np.zeros(N, bool)); S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        near = np.array([[PX, PY, m['BACK'] + .02]])
        for j in range(41): m['observe'](observed, near, j * .05)
        yield ('a hand 20 mm above the plate', batch(every), 0., False)
        rec(held=np.zeros(N, bool)); S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        knocked = observed | {'cavity_plate': plate({k: LAND for k in every}, pos=(PX + .02, PY, PLATE.pos[2]))}
        m['observe'](knocked, far, 0.)
        for j in range(1, 41): m['observe'](observed, far, j * .05)
        yield ('plate knocked 20 mm and put back', batch(every), 0., False)
        m['observe'](observed, None, 2.1)
        yield ('lost arm state stays 0', batch(every), 0., False)
        rec(); S.update(m['fresh']())


CASES = Cases()
