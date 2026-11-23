"""Near misses and unintended strategies for the wave-generator scorer.

CASES are end states with no trial history: the scorer has observed nothing, so each must score 0 (fail closed).
``python near_misses.py`` replays short trajectories through ``observe()`` and checks every tier boundary, the mesh
rule (turn under a light press), the eccentric-mesh (dedoidal) rule and its recovery, the overload, tilt, drop,
teeth and housing rules, release and stillness, and lot B.
"""
import math

CASES = [
    ('nothing observed: start layout', {}, 0.0, False),
    ('generator set on the flexspline mouth, no history', {'wg': ((0.290, -0.090, 0.8075), None, None)}, 0.0, False),
    ('generator dropped home, no history', {'wg': ((0.290, -0.090, 0.7991), None, None)}, 0.0, False),
    ('housing pushed 50 mm', {'joint': ((0.340, -0.090, 0.7475), None, None)}, 0.0, False),
]


def run():
    import dataclasses
    import numpy as np
    import task as t
    import lots

    checks = []

    def check(label, cond):
        checks.append((label, bool(cond)))
        print(('ok   ' if cond else 'FAIL ') + label)

    far = {'left': np.array([[0.10, 0.30, 1.10]]), 'right': np.array([[0.10, -0.30, 1.10]])}

    class Trial:
        """Drive observe() with the generator over the gear axis of the current lot."""

        def __init__(self, lot='A'):
            lots.use(lot)
            self.lot = lots.current()
            t.REC = t.fresh()
            self.tm = 0.0
            self.phi = 0.0
            self.jpos = np.asarray(self.lot.pos('joint'))
            self.hands = far
            self.offset = np.zeros(2)  # plug centre off the axis (m)
            self.tilt = 0.0  # degrees about x
            self.vz = 0.0

        def states(self, depth_mm, load, spin):
            Rw = t.parts._rot((1, 0, 0), math.radians(self.tilt)) @ t.parts._rot((0, 0, 1), self.phi)
            pos = self.jpos + np.array([self.offset[0], self.offset[1], (t.Z_TOP - depth_mm) * t.MM])
            q = (t.REC['front'] - t.MOUTH) * t.MM + (load + t.M_STOP * 9.81) / t.K_STOP
            objs = {p.name: p.state() for p in t.OBJECTS}
            objs['joint'] = t.JOINT.state(joints={'stop': q}, pos=tuple(self.jpos))
            w = t.WG.state(pos=tuple(pos), quat_xyzw=tuple(t.parts._matrix_to_quat(Rw)), lin_vel=(0.0, 0.0, self.vz))
            objs['wg'] = dataclasses.replace(w, ang_vel=np.array([0.0, 0.0, spin]))
            return objs, q

        def step(self, depth_mm=None, load=0.0, turn_deg=0.0, n=1, follow=True):
            for _ in range(n):
                self.phi += math.radians(turn_deg)
                self.tm += 0.05
                d = depth_mm
                if d is None or follow:
                    d = (t.REC['front'] + max(0.0, load) / t.K_STOP / t.MM) if depth_mm is None else depth_mm
                objs, q = self.states(d, load, math.radians(turn_deg) / 0.05)
                t.observe(objs, self.hands, self.tm, q)
            return objs

        def insert(self, until=t.HOME, stroke=12.0, load=6.0):
            """Turn back and forth in 12-degree steps under a light press, the plug riding on the mesh front."""
            self.step(-3.0, 0.0, n=4, follow=False)
            self.step(t.MOUTH, 0.0, n=2, follow=False)
            sign = 1.0
            for k in range(400):
                if t.REC['front'] >= until - 1e-9 or t.REC['fail'] or t.REC['ded']:
                    break
                if k % 10 == 0:
                    sign = -sign
                self.step(None, load, sign * stroke)
            return self

        def settle(self, s=1.2, hands=None):
            self.hands = far if hands is None else hands
            return self.step(None, 0.0, 0.0, n=int(round(s / 0.05)))

    def score_of(objs):
        return t.score(objs)

    # 1. The full job: turned in under a light press, released, still for 1 s.
    tr = Trial('A').insert()
    check('turned in under a light press reaches home', t.REC['front'] >= t.HOME - 1e-9 and not t.REC['ded'])
    objs = tr.settle(0.5)
    check('home, released, still under 1 s: 0.75', score_of(objs) == 0.75)
    objs = tr.settle(1.2)
    check('home, released, still 1 s: 1.0', score_of(objs) == 1.0 and t.success(objs))
    # 2. Home but still held (gripper on the hub).
    tr = Trial('A').insert()
    hub = tr.jpos + np.array([0.0, 0.0, (t.Z_TOP - t.HOME + 14.0) * t.MM])
    held = {'left': far['left'], 'right': np.array([hub + [0.0, 0.0085, 0.0], hub + [0.0, -0.0085, 0.0]])}
    objs = tr.settle(1.5, hands=held)
    check('home but still gripped: 0.75', score_of(objs) == 0.75)
    # 3. Straight push, no turning: the teeth are forced a tooth over (dedoidal) and the generator drops home.
    tr = Trial('A')
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 25.0, 0.0, n=8)
    check('straight 25 N push without turning: dedoidal', t.REC['ded'])
    objs = tr.settle(1.5)
    check('dedoidal generator home and released: 0', score_of(objs) == 0.0)
    # 4. ...withdrawn and turned in properly: the flexspline springs back round; full score again.
    tr.step(-2.0, 0.0, n=3, follow=False)
    check('withdrawn: dedoidal cleared, front back at the mouth', not t.REC['ded'] and t.REC['front'] == t.MOUTH)
    tr.insert()
    objs = tr.settle(1.2)
    check('redone properly after a dedoidal: 1.0', score_of(objs) == 1.0)
    # 5. A firm 15 N straight push only stalls on the teeth (no turn, no progress, no damage).
    tr = Trial('A')
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 15.0, 0.0, n=40)
    objs = tr.settle(1.2)
    check('15 N push without turning: stalls at the mouth, 0', score_of(objs) == 0.0 and not t.REC['ded'])
    # 6. Turning with no press: the generator spins on the rim and does not go in.
    tr = Trial('A')
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 1.0, 12.0, n=60)
    check('turning at 1 N (under 1.5 N): no progress', t.REC['front'] == t.MOUTH)
    # 7. Turning hard (25 N) while turning is still allowed: not dedoidal; but over 45 N overloads the bearing.
    tr = Trial('A')
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 25.0, 12.0, n=6)
    check('25 N while turning: no dedoidal', not t.REC['ded'] and not t.REC['fail'])
    tr.step(None, 28.0, 12.0, n=6)
    check('28 N while turning: under the 30 N limit', not t.REC['fail'])
    tr.step(None, 35.0, 12.0, n=6)
    check('35 N press: bearing overloaded, 0 for the trial', t.REC['fail'].startswith('wave generator bearing'))
    tr.insert()
    check('overload stays 0 after a proper finish', score_of(tr.settle(1.2)) == 0.0)
    # 8. Tilted 4 degrees: no progress; pressed at 8 N while tilted: dedoidal.
    tr = Trial('A')
    tr.tilt = 4.0
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 5.0, 12.0, n=30)
    check('tilted 4 degrees at 5 N: jams, no progress', t.REC['front'] == t.MOUTH and not t.REC['ded'])
    tr.step(None, 9.0, 12.0, n=6)
    check('tilted 4 degrees pressed 9 N: dedoidal', t.REC['ded'])
    # 9. Tier boundaries: released part way.
    for until, want in ((t.MOUTH + 0.9, 0.0), (t.MOUTH + 1.1, 0.25), (t.MOUTH + 3.3, 0.25), (t.MOUTH + 3.5, 0.5),
                        (t.HOME - 0.1, 0.5)):
        tr = Trial('A').insert(until=until)
        got = score_of(tr.settle(1.2))
        check(f'released with the plug {until - t.MOUTH:.2f} mm into the teeth: {want}', got == want)
    # 10. Pulled part way back out above the mesh front: scored where the plug is.
    tr = Trial('A').insert(until=t.MOUTH + 4.0)
    objs = tr.step(1.2, 0.0, n=25, follow=False)
    check('lifted back to 1.2 mm deep after 4 mm in: 0', score_of(objs) == 0.0)
    # 11. Gripper inside the flexspline on the teeth: 0 for the trial.
    tr = Trial('A').insert(until=t.MOUTH + 2.0)
    inside = tr.jpos + np.array([0.0, 0.016, (t.Z_TOP - 3.0) * t.MM])
    tr.hands = {'left': far['left'], 'right': inside[None]}
    tr.step(None, 3.0, 12.0)
    tr.insert()
    check('fingertip on the teeth inside the flexspline: 0', score_of(tr.settle(1.2)) == 0.0)
    # 12. Dropped: falling faster than 0.4 m/s.
    tr = Trial('A')
    tr.vz = -0.6
    tr.step(-5.0, 0.0, n=2, follow=False)
    tr.vz = 0.0
    tr.insert()
    check('generator dropped on the way: 0', score_of(tr.settle(1.2)) == 0.0)
    # 12b. Lowered fast (0.6 m/s) while gripped: not a drop.
    tr = Trial('A')
    hub = tr.jpos + np.array([0.0, 0.0, (t.Z_TOP + 5.0 + 14.0) * t.MM])
    tr.hands = {'left': far['left'], 'right': np.array([hub + [0.0, 0.0085, 0.0], hub + [0.0, -0.0085, 0.0]])}
    tr.vz = -0.6
    tr.step(-5.0, 0.0, n=2, follow=False)
    tr.vz = 0.0
    tr.hands = far
    tr.insert()
    check('lowered fast while gripped: still 1.0', score_of(tr.settle(1.2)) == 1.0)
    # 12c. Eccentric mesh: the pressed plug drops home inside the rim at 0.6 m/s; lifted out and redone: 1.0.
    tr = Trial('A')
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 25.0, 0.0, n=6)
    tr.vz = -0.6
    tr.step(t.HOME, 25.0, 0.0, n=2, follow=False)
    tr.vz = 0.0
    check('eccentric mesh drop inside the rim is not a fall', t.REC['ded'] and not t.REC['fail'])
    tr.step(-2.0, 0.0, n=3, follow=False)
    tr.insert()
    check('eccentric mesh, lifted out, redone: 1.0', score_of(tr.settle(1.2)) == 1.0)
    # 12d. Dropped 30 mm back onto the tray (far from the gear), then fitted properly: still 0.
    tr = Trial('A')
    tr.offset = np.array([-0.095, -0.135])
    tr.vz = -0.8
    tr.step(-30.0, 0.0, n=2, follow=False)
    tr.vz = 0.0
    tr.offset = np.zeros(2)
    tr.insert()
    check('dropped onto the tray, then fitted: 0', score_of(tr.settle(1.2)) == 0.0)
    # 13b. Housing turned 8 degrees about its axis (tape marks): 0.
    tr = Trial('A').insert()
    objs = tr.settle(0.2)
    q = (t.REC['front'] - t.MOUTH) * t.MM + t.M_STOP * 9.81 / t.K_STOP * 0 + (0.0 + t.M_STOP * 9.81) / t.K_STOP
    objs['joint'] = t.JOINT.state(joints={'stop': q}, pos=tuple(tr.jpos),
                                  quat_xyzw=tuple(t.parts._matrix_to_quat(t.parts._rot((0, 0, 1), math.radians(8)))))
    tr.tm += 0.05
    t.observe(objs, far, tr.tm, q)
    check('housing turned 8 degrees: 0', score_of(objs) == 0.0)
    # 13. Housing knocked 8 mm.
    tr = Trial('A').insert()
    tr.jpos = tr.jpos + np.array([0.008, 0.0, 0.0])
    check('housing knocked 8 mm: 0', score_of(tr.settle(1.2)) == 0.0)
    # 14. Off the axis by 1.5 mm (sitting on the rim): no progress.
    tr = Trial('A')
    tr.offset = np.array([0.0015, 0.0])
    tr.step(t.MOUTH, 0.0, n=2, follow=False)
    tr.step(None, 5.0, 12.0, n=30)
    check('plug 1.5 mm off the axis: no progress', t.REC['front'] == t.MOUTH)
    # 15. Upside down (hub first): never engages.
    tr = Trial('A')
    tr.tilt = 180.0
    tr.step(t.MOUTH, 5.0, 12.0, n=30, follow=False)
    check('generator upside down: no progress, 0', t.REC['front'] == t.MOUTH and score_of(tr.settle(1.2)) == 0.0)
    # 16. Lot B: same gear, the full job scores 1.0.
    tr = Trial('B').insert()
    check('lot B full job: 1.0', score_of(tr.settle(1.2)) == 1.0)
    lots.use(None)
    bad = [c for c, ok in checks if not ok]
    print(f'\n{len(checks) - len(bad)}/{len(checks)} checks passed')
    return not bad


if __name__ == '__main__':
    import sys
    sys.exit(0 if run() else 1)
