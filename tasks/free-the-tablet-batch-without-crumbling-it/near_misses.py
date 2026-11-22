"""Scorer and release-rule counterexamples (synthetic states, not robot trials)."""
import math
import numpy as np
import parts


class Cases:
    def __iter__(self):
        m = parts._tablet_namespace
        TOP, PT, PH, N, PX, PY = m['TOP'], m['PT'], m['PH'], m['N'], m['PX'], m['PY']
        HOLES, NAMES, TABLETS, PLATE, SPREADER = m['HOLES'], m['NAMES'], m['TABLETS'], m['PLATE'], m['SPREADER']
        S = m['S']
        def down(k, dz=0., dx=0., quat=None, vel=None):
            return TABLETS[k].state(pos=(PX + HOLES[k][0] + dx, PY + HOLES[k][1], PT + m['TAB_H'] / 2 + dz), quat_xyzw=quat, lin_vel=vel)
        laid = SPREADER.state(pos=(.30, -.30, TOP + m['CT'] / 2), quat_xyzw=(math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))
        def batch(k_out):
            out = {NAMES[k]: down(k) for k in k_out}; out['spreader'] = laid; return out
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
        for why in ('6.2 N pushed down on the plate', 'a gripper touched a tablet', 'a tablet fell off the counter', 'plate knocked'):
            yield emit(why + ': stays 0', batch(every), 0., n_out=N, zero=True, why=why)
        # Tablets that do not count.
        side = down(0, dz=(m['A_BOT'] * 2 - m['TAB_H']) / 2 + .0002, quat=(math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)))
        yield emit('one tablet on its edge', batch(every) | {NAMES[0]: side}, .75, n_out=N)
        yield emit('one tablet still sliding', batch(every) | {NAMES[0]: down(0, vel=(.02, 0, 0))}, .75, n_out=N)
        yield emit('one tablet off the parchment on the counter', batch(every) | {NAMES[0]: TABLETS[0].state(pos=(PX, -.20, TOP + m['TAB_H'] / 2))}, .75, n_out=N)
        yield emit('one tablet 1.1 mm up on a fragment', batch(every) | {NAMES[0]: down(0, dz=.0011)}, .75, n_out=N)
        a35 = math.radians(35) / 2
        tipped = TABLETS[0].state(pos=(PX + HOLES[0][0], PY + HOLES[0][1], PT + .0032), quat_xyzw=(math.sin(a35), 0, 0, math.cos(a35)))
        assert tipped.high[2] > PT + .005 and PT - .001 <= tipped.low[2] <= PT + .001
        yield emit('one tablet propped at 35 degrees (top over 5 mm up)', batch(every) | {NAMES[0]: tipped}, .75, n_out=N)
        yield emit('tablets on the parchment but marked held (not let go by the rule)', batch(every), 0., n_out=0)
        moved = PLATE.state(pos=(PX, PY - .0105, PLATE.pos[2]))
        yield emit('plate slid 10.5 mm on its bars', batch(every) | {'cavity_plate': moved}, 0., n_out=N)
        a = math.radians(6) / 2
        yield emit('plate tipped 6 degrees', batch(every) | {'cavity_plate': PLATE.state(quat_xyzw=(math.sin(a), 0, 0, math.cos(a)))}, 0., n_out=N)
        yield emit('plate slid 9.5 mm (still on its bars)', batch(every) | {'cavity_plate': PLATE.state(pos=(PX, PY - .0095, PLATE.pos[2]))}, 1., True, n_out=N)
        # Release rule, driven through the real rule code (edge in the plate frame: centre line of the rounded edge).
        CW, ROD, PHh = m['CW'], m['ROD'], PH / 2
        def edge(y, x0=0., gap=0.):
            z = PHh + ROD + gap
            return np.array([(x0 - CW / 2, y, z), (x0 + CW / 2, y, z)])
        row0 = HOLES[0][1]
        rec(); out = m['rule_step'](1.0, 0., 1.0, edge(row0))
        assert out.sum() == 6 and not S['zero'] and S['freed'].sum() == 6
        assert m['drop_step'](edge(row0)).sum() == 0 and S['held'].all()
        assert m['drop_step'](edge(row0 - .0043)).sum() == 0
        assert m['drop_step'](edge(row0 - .0053)).sum() == 6 and S['held'].sum() == N - 6
        yield ('edge on the first row at 1.0 N frees its six tablets; they drop once the edge is 5.3 mm past', batch(range(6)), 0., False)
        rec(); out = m['rule_step'](1.0, 0., 1.0, edge(row0)); m['drop_step'](edge(row0, gap=.0021))
        assert S['held'].sum() == N - 6
        yield ('freed tablets drop when the edge lifts 2.1 mm off', {}, 0., False)
        rec(); out = m['rule_step'](.99, 0., .99, edge(row0))
        assert out.sum() == 0 and not S['freed'].any()
        yield ('edge over the first row at 0.99 N: nothing lets go', {}, 0., False)
        rec(); out = m['rule_step'](3.0, 0., 3.0, edge(row0, gap=.00031))
        assert out.sum() == 0
        rec(); out = m['rule_step'](3.0, 0., 3.0, edge(row0, gap=.00029))
        assert out.sum() == 6
        yield ('edge 0.31 mm above the plate frees nothing; 0.29 mm frees the row', {}, 0., False)
        # A card rolled so one end of the edge presses and the other end is 1 mm up frees only the holes under the
        # part of the edge within 0.3 mm of the plate (two of six columns).
        z0 = PHh + ROD
        rolled = np.array([(-CW / 2, row0, z0), (CW / 2, row0, z0 + .001)])
        rec(); out = m['rule_step'](3.0, 0., 3.0, rolled)
        assert out.sum() == 2 and S['freed'][0] and S['freed'][1] and not S['freed'][2]
        yield ('rolled card touching at one end: only the two columns under the low end are freed', {}, 0., False)
        rec(); out = m['rule_step'](3.0, 0., 3.0, edge(row0 + m['A_TOP'] + .0001))
        assert out.sum() == 0
        rec(); out = m['rule_step'](3.0, 0., 3.0, edge(row0 + m['A_TOP'] - .0001))
        assert out.sum() == 6
        yield ('edge 2.74 mm from the hole centres releases, 2.76 mm does not', {}, 0., False)
        rec(); out = m['rule_step'](3.0, 0., 3.0, edge(row0, x0=.062))
        assert out.sum() == 2 and S['freed'][4] and S['freed'][5] and not S['freed'][0]
        yield ('edge only over two columns releases only those two', {}, 0., False)
        rec(); m['rule_step'](6.0, 0., 6.0, edge(row0)); assert not S['zero']
        rec(); m['rule_step'](6.01, 0., 6.01, edge(row0)); assert S['zero']
        yield ('6.01 N spreader press crushes (6.0 N does not)', batch(every), 0., False)
        rec(); m['rule_step'](3.0, 0., 6.01, edge(row0)); assert S['zero']
        yield ('3 N spreader plus 3.01 N from a gripper on the plate: over 6 N in total', batch(every), 0., False)
        rec(); m['rule_step'](0., 0., 6.01, edge(.5)); assert S['zero']
        yield ('a gripper striking the plate at 6.01 N', batch(every), 0., False)
        rec(); m['rule_step'](0., 0., 5.9, edge(.5)); assert not S['zero']
        yield ('a gripper steadying the plate at 5.9 N is allowed', {}, 0., False)
        rec(); m['rule_step'](0., .021, 0., edge(.5)); assert S['zero']
        yield ('a fingertip pushing on a tablet (0.021 N)', batch(every), 0., False)
        # Sweep over all rows at 1.5 N: everything lets go; the observer then scores it.
        rec()
        for y in np.linspace(.071, -.071, 600): m['rule_step'](1.5, 0., 1.5, edge(y)); m['drop_step'](edge(y))
        m['drop_step'](edge(-.071, gap=.01))
        assert not S['held'].any() and S['freed'].all() and not S['zero']
        start = {o.name: o.state() for o in m['OBJECTS']}
        observed = start | batch(every); far = np.array([[-.4, -.5, TOP + .5]])
        S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        for j in range(21): m['observe'](observed, far, j * .05)
        yield ('one 1.5 N sweep, all fall, observed 1 s still and clear', batch(every), 1., True)
        rec(held=np.zeros(N, bool)); S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        near = np.array([[PX, PY, m['BACK'] + .02]])
        for j in range(41): m['observe'](observed, near, j * .05)
        yield ('a hand 20 mm above the plate', batch(every), 0., False)
        rec(held=np.zeros(N, bool)); S.update(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        knocked = observed | {'cavity_plate': PLATE.state(pos=(PX + .02, PY, PLATE.pos[2]))}
        m['observe'](knocked, far, 0.)
        for j in range(1, 41): m['observe'](observed, far, j * .05)
        yield ('plate knocked 20 mm and put back', batch(every), 0., False)
        m['observe'](observed, None, 2.1)
        yield ('lost arm state stays 0', batch(every), 0., False)
        rec(); S.update(m['fresh']())


CASES = Cases()
