"""Near misses and unintended strategies for the pivot-reaming scorer.

CASES are end states with no trial history: the scorer has observed nothing, so each must score 0 (fail closed).
``python near_misses.py`` replays short trajectories through ``observe()`` and checks every tier boundary, the
ratchet rule, the bell-mouth, gouge, seat, urethane, overload, release and stillness rules, and lot B.
"""
import math

CASES = [
    ('nothing observed: start layout', {}, 0.0, False),
    ('wrench lifted out of the stand, never used', {'tool': ((0.20, -0.20, 0.95), None, None)}, 0.0, False),
    ('wrench stood on the knuckle', {'tool': ((0.29, -0.085, 0.86), None, None)}, 0.0, False),
    ('vise pushed 50 mm', {'vise': ((0.34, -0.085, 0.7475), None, None)}, 0.0, False),
    ('wrench dropped on the bench, lying', {'tool': ((0.25, -0.15, 0.76), None, None, (0.7071068, 0, 0, 0.7071068))},
     0.0, False),
]


def run():
    import numpy as np
    import task as t
    import lots

    checks = []

    def check(label, cond):
        checks.append((label, bool(cond)))
        print(('ok   ' if cond else 'FAIL ') + label)

    far = {'left': np.array([[0.10, 0.30, 1.10]]), 'right': np.array([[0.10, -0.30, 1.10]])}

    class Trial:
        """Drive observe() with a tool held over the bore of the current lot."""

        def __init__(self, lot='A'):
            lots.use(lot)
            self.lot = lots.current()
            t.REC = t.fresh()
            self.tm = 0.0
            self.phi = t.TOOL_YAW
            self.vpos = np.asarray(self.lot.pos('vise'))
            self.vyaw = self.lot.yaw_of('vise')
            self.Rv = t.parts._rot((0, 0, 1), self.vyaw)
            self.hands = far
            self.offset = np.zeros(3)  # tool axis offset from the bore (vise frame, m)
            self.tilt = 0.0  # degrees about the vise x axis

        def states(self, depth_mm, load, tool_pos=None, R=None):
            Rt = self.Rv @ t.parts._rot((1, 0, 0), math.radians(self.tilt)) @ t.parts._rot((0, 0, 1), self.phi)
            tip_v = np.array([0.0, 0.0, t.BED + (t.EAR_TOP - depth_mm) * t.MM]) + self.offset
            tip_w = self.Rv @ tip_v + self.vpos
            pos = tip_w - Rt @ np.array([0.0, 0.0, t.TIP_Z]) if tool_pos is None else np.asarray(tool_pos)
            Rt = Rt if R is None else R
            q = t.front_depth(t.REC) * t.MM + (load + t.M_FRONT * 9.81) / t.K_FRONT
            vq = t.parts._matrix_to_quat(self.Rv)
            objs = {p.name: p.state() for p in t.OBJECTS}
            objs['vise'] = t.VISE.state(joints={'front': q}, pos=tuple(self.vpos), quat_xyzw=tuple(vq))
            objs['tool'] = t.TOOL.state(pos=tuple(pos), quat_xyzw=tuple(t.parts._matrix_to_quat(Rt)))
            return objs, q

        def step(self, depth_mm, load=5.0, turn_deg=0.0, n=1, **kw):
            for _ in range(n):
                self.phi -= math.radians(turn_deg)  # clockwise from above is negative yaw
                self.tm += 0.05
                objs, q = self.states(depth_mm, load, **kw)
                t.observe(objs, self.hands, self.tm, q)
            return objs

        def ream(self, ear_cut=(t.EAR_CUT, t.EAR_CUT), stroke=30.0, load=5.0):
            """Turn clockwise in 30-degree steps under load, the tip following the cut front."""
            for _ in range(400):
                if t.REC['c2'] >= ear_cut[0] - 1e-9 and (ear_cut[0] < t.EAR_CUT or t.REC['c1'] >= ear_cut[1] - 1e-9):
                    break
                if t.REC['fail']:
                    break
                d = t.front_depth(t.REC)
                if d >= t.PARK:
                    break
                lim = ear_cut[0] if t.REC['c2'] < t.EAR_CUT else (t.EAR_TOP - t.EARS[1][1]) + ear_cut[1]
                if d >= lim - 1e-9:
                    break
                self.step(d + 0.05, load, stroke)
                self.step(min(t.front_depth(t.REC), d + 0.4) + 0.05, load, -stroke)  # ratchet back-swing
            return self

        def withdraw_and_park(self, hold_s=1.2, hands=None):
            self.step(-30.0, 0.0, 0.0, n=4)
            park = np.asarray(self.lot.pos('tool'))
            self.hands = far if hands is None else hands
            R = t.parts._rot((0, 0, 1), t.TOOL_YAW)
            return self.step(-30.0, 0.0, 0.0, n=int(round(hold_s / 0.05)), tool_pos=park, R=R)

    # -- tiers -------------------------------------------------------------------------------------------------
    tr = Trial()
    objs = tr.step(-30.0, 0.0, 0.0, n=30)
    check('no-op: 0', t.score(objs) == 0.0)
    tr = Trial().ream(ear_cut=(1.885, 0), stroke=5.0)
    objs = tr.withdraw_and_park()
    check('upper ear cut %.3f mm past the chamfer (go pin enters under 1.5 mm): 0' % (t.REC['c2'] - t.LEAD),
          t.REC['c2'] < 1.9 and t.score(objs) == 0.0)
    tr = Trial().ream(ear_cut=(1.9, 0), stroke=5.0)
    objs = tr.withdraw_and_park()
    check('upper ear cut %.3f mm past the chamfer: 0.25' % (t.REC['c2'] - t.LEAD), t.score(objs) == 0.25)
    tr = Trial().ream(ear_cut=(t.EAR_CUT, 0.0))
    objs = tr.withdraw_and_park()
    check('upper ear finished, lower untouched: 0.5', t.score(objs) == 0.5)
    tr = Trial().ream(ear_cut=(t.EAR_CUT, 2.0))
    objs = tr.withdraw_and_park()
    check('upper ear finished, lower ear part cut: 0.5', t.score(objs) == 0.5)
    tr = Trial().ream()
    check('both ears finished in the bore: reamer still in the knuckle, 0', t.score(tr.step(t.PARK - 2, 0.0)) == 0.0)
    objs = tr.withdraw_and_park(hold_s=1.2)
    check('both ears, wrench parked, hands clear, still 1.2 s: 1.0', t.score(objs) == 1.0 and t.success(objs))
    tr = Trial().ream()
    objs = tr.withdraw_and_park(hold_s=0.9)
    check('both ears, still only 0.9 s: 0.75', t.score(objs) == 0.75)
    tr = Trial().ream()
    tr.step(-30.0, 0.0, 0.0, n=4)
    objs, _ = tr.states(-30.0, 0.0)
    body = np.asarray(objs['tool'].pos) + t.rot(objs['tool']) @ np.array([0.0, 0.0, t.GRIP_Z])
    tr.hands = {'right': body + np.array([[0.0, 0.009, 0.0], [0.0, -0.009, 0.0]]), 'left': far['left']}
    objs = tr.step(-30.0, 0.0, 0.0, n=30)
    check('both ears, reamer withdrawn but still held over the knuckle: 0.75', t.score(objs) == 0.75)
    tr = Trial().ream()
    near = {'right': np.asarray(t.lots.current().pos('tool')) + np.array([[0.0, 0.015, 0.05]]), 'left': far['left']}
    objs = tr.withdraw_and_park(hold_s=1.5, hands=near)
    check('both ears, gripper 15 mm from the parked wrench: 0.75', t.score(objs) == 0.75)
    near = {'right': np.asarray(t.lots.current().pos('tool')) + np.array([[0.0, 0.0305, 0.05]]), 'left': far['left']}
    tr = Trial().ream()
    objs = tr.withdraw_and_park(hold_s=1.5, hands=near)
    check('both ears, gripper 21.5 mm from the wrench body: 1.0', t.score(objs) == 1.0)

    # -- cutting rule -----------------------------------------------------------------------------------------
    tr = Trial()
    objs = tr.step(0.3, 20.0, 0.0, n=60)
    check('pushing 20 N without turning cuts nothing', t.REC['c2'] == 0.0 and not t.REC['fail'])
    tr = Trial()
    tr.step(0.05, 0.5, 30.0, n=60)
    check('turning at 0.5 N (no pressure) cuts nothing', t.REC['c2'] == 0.0)
    tr = Trial()
    tr.step(-2.0, 0.0, 30.0, n=60)
    check('turning 2 mm above the ear cuts nothing', t.REC['c2'] == 0.0)
    tr = Trial()
    tr.step(0.05, 5.0, -30.0, n=60)
    check('counter-clockwise turning (ratchet slip) cuts nothing', t.REC['c2'] == 0.0)
    tr = Trial()
    tr.step(0.05, 5.0, 30.0, n=2)
    check('one 30-degree clockwise turn cuts at most 0.083 mm', 0.0 < t.REC['c2'] <= 1.0 / 12 + 1e-9)
    tr = Trial()
    tr.step(0.05, 5.0, 120.0, n=1)
    check('cut never runs ahead of the reamer tip (0.25 mm cap)', t.REC['c2'] <= 0.25 + 1e-9)
    tr = Trial()
    tr.step(0.2, 26.0, 0.0, n=6)
    check('axial force over 25 N splits the ear: 0 for the trial', t.REC['fail'] != '')
    tr.step(0.2, 5.0, 30.0, n=200)
    check('after an ear split, further reaming still scores 0', t.score(tr.withdraw_and_park()) == 0.0)

    # -- bore axis -----------------------------------------------------------------------------------------------
    for off, want in ((0.00049, False), (0.00051, True)):
        tr = Trial()
        tr.offset = np.array([off, 0.0, 0.0])
        tr.ream()
        check('starting an ear %.2f mm off the bore axis %s' % (off * 1000, 'cuts its face: 0' if want else 'is captured'),
              bool(t.REC['fail']) == want)
    for rad, want in ((0.12, False), (0.20, True)):
        tr = Trial()
        tr.ream(ear_cut=(0.3, 0))
        k = 0
        while t.REC['c2'] < t.EAR_CUT and not t.REC['fail'] and k < 400:
            a = k * math.pi / 6
            tr.offset = np.array([rad * math.cos(a), rad * math.sin(a), 0.0]) * 1e-3
            tr.step(t.front_depth(t.REC) + 0.05, 3.0, 30.0)
            k += 1
        check('reaming an ear while the axis circles %.2f mm %s' % (rad, 'bell-mouths it (6.70 mm no-go enters): 0'
              if want else 'stays under the 6.70 mm no-go'), ('no-go' in t.REC['fail']) == want)
    tr = Trial()
    tr.ream(ear_cut=(0.3, 0))
    tr.offset = np.array([0.0, 0.0003, 0.0])
    tr.ream(ear_cut=(t.EAR_CUT, 0))
    check('a steady 0.3 mm offset in a started ear is not a bell-mouth', t.REC['fail'] == '' and t.REC['c2'] >= t.EAR_CUT)
    tr = Trial()
    tr.ream(ear_cut=(t.EAR_CUT, 0.5))
    k = 0
    while t.REC['c1'] < t.EAR_CUT and not t.REC['fail'] and k < 400:
        a = k * math.pi / 6
        tr.offset = np.array([0.25 * math.cos(a), 0.25 * math.sin(a), 0.0]) * 1e-3
        tr.step(t.front_depth(t.REC) + 0.05, 3.0, 30.0)
        k += 1
    objs = tr.withdraw_and_park()
    check('bell-mouth is sticky even after both ears finish: 0', 'no-go' in t.REC['fail'] and t.score(objs) == 0.0)
    tr = Trial()
    tr.ream(ear_cut=(t.EAR_CUT, 0))
    tr.offset = np.array([0.0007, 0.0, 0.0])
    tr.ream()
    check('the lower ear must also be started within 0.5 mm', 'beside the bore' in t.REC['fail'])
    for tilt, want in ((4.9, False), (5.1, True)):
        tr = Trial()
        tr.tilt = tilt
        tr.ream(ear_cut=(0.6, 0))
        check('reaming tilted %.1f deg %s' % (tilt, 'is levering: 0' if want else 'passes'), bool(t.REC['fail']) == want)
    tr = Trial()
    tr.step(-30.0, 0.0, 0.0, n=2, tool_pos=tr.Rv @ t.f2p((20.0, 10.0, 15.0)) + tr.vpos - np.array([0, 0, 0.05]))
    check('wrench body pushed through the knuckle: 0', 'wrench pushed into the knuckle' in t.REC['fail'])
    tr = Trial()
    tr.offset = np.array([0.0012, 0.0, 0.0])
    tr.step(1.5, 0.0, 0.0, n=2)
    check('reamer pushed 1.2 mm off axis into the ear (no turning) is still inside the float', t.REC['fail'] == '')
    tr = Trial()
    tr.offset = np.array([0.0014, 0.0, 0.0])
    tr.step(1.5, 0.0, 0.0, n=2)
    check('reamer pushed 1.4 mm off axis into the ear gouges the wall', 'knuckle wall' in t.REC['fail'])
    tr = Trial()
    tr.offset = np.array([0.0, -0.006, 0.0])
    tr.step(8.0, 0.0, 0.0, n=2)
    check('reamer driven into the knuckle 6 mm off the bore gouges', 'knuckle wall' in t.REC['fail'])

    # -- the adjacent routing-pin seat and the scaffold face ----------------------------------------------------
    tr = Trial()
    seat = t.f2p((t.SEAT_XY[0], t.SEAT_XY[1], 0.0))
    tr.offset = np.array([seat[0], seat[1], 0.0])
    tr.step(t.EAR_TOP - 31.75 - 0.2, 3.0, 30.0, n=2)
    check('reamer turned on the routing-pin seat mouth: 0', 'routing-pin seat' in t.REC['fail'])
    tr = Trial()
    tr.offset = np.array([seat[0], seat[1], 0.0])
    tr.step(t.EAR_TOP - 31.75 - 0.2, 3.0, 0.0, n=4)
    check('reamer only resting on the seat (no turn) is not damage', t.REC['fail'] == '')
    tr = Trial()
    face = t.f2p((15.0, 40.0, 0.0))
    tr.offset = np.array([face[0], face[1], 0.0])
    tr.step(t.EAR_TOP - 31.75 - 0.2, 3.0, 30.0, n=2)
    check('reamer turned on the scaffold top face: 0', 'scaffold face' in t.REC['fail'])
    tr = Trial()
    pad = t.f2p((26.0, 40.0, 0.0))
    tr.offset = np.array([pad[0], pad[1], 0.0])
    tr.step(t.EAR_TOP - 20.0, 0.0, 0.0, n=2)
    check('reamer pushed into the cast pad: 0', 'pad or flexure' in t.REC['fail'])

    # -- hands --------------------------------------------------------------------------------------------------
    tr = Trial()
    knuckle = tr.Rv @ t.f2p((15.0, 5.0, 15.0)) + tr.vpos
    tr.hands = {'right': knuckle[None], 'left': far['left']}
    tr.step(-30.0, 0.0, 0.0)
    check('gripper inside the knuckle wall: 0', 'knuckle' in t.REC['fail'])
    tr = Trial()
    flex = tr.Rv @ t.f2p((20.0, 70.0, 15.0)) + tr.vpos
    tr.hands = {'right': flex[None], 'left': far['left']}
    tr.step(-30.0, 0.0, 0.0)
    check('gripper pressing the cast flexure: 0', 'pad or flexure' in t.REC['fail'])
    tr = Trial()
    objs, q = tr.states(-30.0, 0.0)
    t.observe(objs, None, 0.05, q)
    check('hand state unavailable: fails closed', t.score(objs) == 0.0 and t.REC['fail'])
    tr = Trial()
    tr.ream()
    objs, q = tr.states(-30.0, 0.0)
    moved = t.VISE.state(joints={'front': q}, pos=tuple(tr.vpos + np.array([0.04, 0, 0])),
                         quat_xyzw=tuple(t.parts._matrix_to_quat(tr.Rv)))
    objs['vise'] = moved
    tr.tm += 0.05
    t.observe(objs, far, tr.tm, q)
    check('vise knocked 40 mm: 0', 'vise' in t.REC['fail'])

    # -- cutting load: a resisting torque only while the lead bears on uncut ABS and turns clockwise ---------------
    import dataclasses

    def torque_at(depth, load, spin):
        tr = Trial()
        tr.step(depth, load, 0.0)
        objs, q = tr.states(depth, load)
        objs['tool'] = dataclasses.replace(objs['tool'], ang_vel=np.array([0.0, 0.0, spin]))
        tr.tm += 0.05
        t.observe(objs, far, tr.tm, q)
        return t.REC['torque']
    check('cutting torque 0.15 N m when turning clockwise at 3 N on uncut ABS', abs(torque_at(0.3, 3.0, -1.0) - 0.15) < 1e-9)
    check('cutting torque scales with the bearing load (0.75 N: 0.075 N m)', abs(torque_at(0.3, 0.75, -1.0) - 0.075) < 1e-9)
    check('no cutting torque on a ratchet back-swing', torque_at(0.3, 3.0, 1.0) == 0.0)
    check('no cutting torque above the ear', torque_at(-3.0, 0.0, -1.0) == 0.0)
    check('cutting torque does not drive a reamer at rest', torque_at(0.3, 3.0, 0.0) == 0.0)

    # -- turning the reamer backwards by its chuck nut (the ratchet's output) --------------------------------------
    tr = Trial()
    tr.ream(ear_cut=(1.0, 0))
    objs, _ = tr.states(t.front_depth(t.REC), 3.0)
    nut = np.asarray(objs['tool'].pos) + t.rot(objs['tool']) @ np.array([0.0, 0.0, 0.011])
    tr.hands = {'right': nut + np.array([[0.0, 0.0105, 0.0], [0.0, -0.0105, 0.0]]), 'left': far['left']}
    tr.step(t.front_depth(t.REC), 3.0, -10.0, n=1)
    check('10 degrees back by the chuck nut is tolerated', t.REC['fail'] == '')
    tr.step(t.front_depth(t.REC), 3.0, -15.0, n=1)
    check('25 degrees back by the chuck nut in a started ear: 0', 'backwards' in t.REC['fail'])
    tr = Trial()
    tr.ream(ear_cut=(1.0, 0))
    objs, _ = tr.states(t.front_depth(t.REC), 3.0)
    housing = np.asarray(objs['tool'].pos) + t.rot(objs['tool']) @ np.array([0.0, 0.0, t.GRIP_Z])
    tr.hands = {'right': housing + np.array([[0.0, 0.009, 0.0], [0.0, -0.009, 0.0]]), 'left': far['left']}
    tr.step(t.front_depth(t.REC), 3.0, -90.0, n=2)
    check('back-swings by the ratchet housing are pawl slips, not damage', t.REC['fail'] == '')

    # -- lot B: the stand moves; the lot-A habit cuts nothing -------------------------------------------------
    tr = Trial('B').ream()
    objs = tr.withdraw_and_park()
    check('lot B: full sequence, wrench back on the moved stand, scores 1.0', t.score(objs) == 1.0)
    tr = Trial('B')
    objs = tr.step(-40.0, 0.0, 0.0, n=40, tool_pos=np.asarray(t.lots.current().pos('tool')),
                   R=t.parts._rot((0, 0, 1), t.TOOL_YAW))
    check('lot B: closing on lot A\'s stand position (empty) leaves the bore uncut: 0', t.score(objs) == 0.0)
    lots.use('B')
    import task as tt
    plan = tt.demo()
    grab = [w for w in plan['right'] if w.label == 'grip the ratchet housing'][0].pos
    check('lot B demo grips the wrench at its lot B stand, not lot A\'s',
          abs(grab[0] - t.RACK_B[0]) < 0.01 and abs(grab[1] - t.RACK_B[1]) < 0.01)
    lots.use(None)
    bad = [c for c, ok in checks if not ok]
    print('%d checks, %d failed' % (len(checks), len(bad)))
    return not bad


if __name__ == '__main__':
    import sys
    sys.exit(0 if run() else 1)
