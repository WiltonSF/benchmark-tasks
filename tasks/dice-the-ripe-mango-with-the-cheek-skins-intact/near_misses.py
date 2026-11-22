"""Endpoint cases read by the checker (CASES), and offline controls that drive the cutting, skin and push rules with
moving knife and hand states (run: python near_misses.py in the Arena Python environment)."""
from pathlib import Path
import importlib.util
import math
import numpy as np

CASES = [
    ('unknown hand state fails closed', {}, 0., False),
    ('cheek slid to another place earns nothing', {'cheek': ((0.30, 0.05, 0.789), None, None)}, 0., False),
    ('knife laid on the cheek earns nothing', {'knife': ((0.310, -0.09, 0.80), None, None)}, 0., False),
]


def load():
    spec = importlib.util.spec_from_file_location('mango_cases', Path(__file__).with_name('task.py'))
    t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)
    return t


def run(verbose=True, lot_name='A', cal=False):
    t = load()
    t.lots.use(lot_name)
    lot = t.lots.current()
    c0 = np.array(lot.pos('cheek')); cyaw = lot.yaw_of('cheek'); R = t.rz(cyaw)
    q = lambda r: t.parts._matrix_to_quat(r)
    far = {'left': np.array([[0., .5, t.TOP + .3]]), 'right': np.array([[0., -.5, t.TOP + .3]])}
    results = []

    def base_states():
        st = {o.name: o.state() for o in t.OBJECTS}
        for o in t.MOVE:   # the lot's own start poses
            st[o.name] = o.state(pos=lot.pos(o.name), quat_xyzw=q(t.rz(lot.yaw_of(o.name))))
        return st

    def knife_at(tip, phi, pitch=t.PITCH, roll=0.):
        rk = t.rz(phi) @ t.ry(pitch) @ t.parts._rot((1, 0, 0), roll)
        return t.KNIFE.state(pos=np.asarray(tip) - rk @ t.TIP, quat_xyzw=q(rk))

    def hands(st, hold=True, tool=True, hold_x=t.HOLD_X):
        cc = t.HandClouds(); cc.update({a: v.copy() for a, v in far.items()})
        for arm in t.ARMS:
            cc.fingers[arm] = {'tip_left': far[arm].copy(), 'tip_right': far[arm].copy()}; cc.opening[arm] = .094
        if hold:
            ch = st['cheek']; r = t.rot(ch.quat_xyzw)
            w = t.half_width(hold_x, -.004, 0.)
            xs = np.linspace(hold_x - .010, hold_x + .014, 7)
            a = np.array([ch.pos + r @ np.array([x, w + .001, -.002]) for x in xs]); b = np.array([ch.pos + r @ np.array([x, -w - .001, -.002]) for x in xs])
            cc['left'] = np.vstack([a, b]); cc.fingers['left'] = {'tip_left': a, 'tip_right': b}; cc.opening['left'] = 2 * w
        if tool:
            k = st['knife']; r = t.rot(k.quat_xyzw)
            a = k.pos + r @ np.array([0., .011, 0]); b = k.pos + r @ np.array([0., -.011, 0])
            cc['right'] = np.array([a, b]); cc.fingers['right'] = {'tip_left': np.array([a]), 'tip_right': np.array([b])}; cc.opening['right'] = .022
        return cc

    class Trial:
        def __init__(self):
            t.fresh(); t._REC['armed'] = True; self.time = 0.; self.st = base_states()
            self.step(hold=False, tool=False)

        def step(self, **kw):
            self.time += .05
            t.update(self.st, hands(self.st, **kw), self.time)

        def tip_path(self, pts, phi, hold=True, tool=True, pitch=t.PITCH, roll=0., per_step=.00125):
            for a, b in zip(pts, pts[1:]):
                n = max(1, int(math.ceil(np.linalg.norm(b - a) / per_step)))
                for w in np.linspace(0, 1, n + 1)[1:]:
                    self.st['knife'] = knife_at(a + (b - a) * w, phi, pitch, roll)
                    self.step(hold=hold, tool=tool)

        def cut(self, family, value, forward, aim_shift=0., hold=True, phi_extra=0., roll=0., pitch=None):
            phi = (cyaw - math.pi / 2 if family == 'cross' else cyaw + math.pi) + phi_extra
            path = t.stroke(t.line_points(family, value, forward), family, lot, cal=cal)
            path = [p + np.array([0, 0, aim_shift]) for p in path]
            up = np.array([0, 0, 1.])
            above = lambda p: p + up * (.006 - (p - c0) @ up)
            self.tip_path([above(path[0]) + up * .02, above(path[0]), *path, above(path[-1]), above(path[-1]) + up * .02], phi, hold=hold, roll=roll, pitch=t.PITCH if pitch is None else pitch)

        def grid(self, **kw):
            for v in t.CROSS_X:
                self.cut('cross', v, True, **kw)
            for v in t.LONG_Y:
                self.cut('long', v, True, **kw)

        def knife_home(self):
            k = self.st['knife']
            self.st['knife'] = t.KNIFE.state(pos=np.asarray(k.pos) + [0, 0, .10], quat_xyzw=k.quat_xyzw); self.step(hold=False, tool=False)
            self.st['knife'] = t.KNIFE.state(); self.step(hold=False, tool=False)

        def push(self, depth):
            ram = self.st['ramekin']; top = np.asarray(ram.pos)[2] + t.RAM_TOP
            for d in list(np.linspace(-.01, depth, 12)) + [depth] * 5 + list(np.linspace(depth, -.02, 8)):
                face = top - d + t.H
                self.st['cheek'] = t.CHEEK.state(pos=(ram.pos[0], ram.pos[1], face), quat_xyzw=q(R))
                self.step(hold=True, tool=False, hold_x=0.)
            self.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); self.step(hold=True, tool=False, hold_x=0.)

        def finish(self):
            for _ in range(25):
                self.step(hold=False, tool=False)
            p = t.pieces()
            return t.score(self.st), p['cubes'], t._REC['zero'], t._REC['inverted']

    def case(label, want, fn):
        tr = Trial(); fn(tr); got = tr.finish()
        ok = abs(got[0] - want) < 1e-9
        results.append(ok)
        if verbose:
            print(('ok  ' if ok else 'FAIL'), f'{label}: score {got[0]} (want {want}); cubes {got[1]}; zero {got[2]}; inverted {got[3]}')

    case('full grid, knife put back, skin pushed 31 mm onto the ramekin (turned out)', 1., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.031)))
    case('full grid, skin pushed only 16 mm: cuts not yet fanned', .75, lambda tr: (tr.grid(), tr.knife_home(), tr.push(.016)))
    case('full grid, skin pushed 22 mm: still cupped, not turned out', .75, lambda tr: (tr.grid(), tr.knife_home(), tr.push(.022)))
    case('full grid, knife put back, never turned out', .75, lambda tr: (tr.grid(), tr.knife_home()))
    case('grid cut 4.5 mm too deep: through the skin', 0., lambda tr: (tr.grid(aim_shift=-.0045), tr.knife_home(), tr.push(.016)))
    case('grid cut 8.5 mm short of the skin: cubes stay joined', 0., lambda tr: (tr.grid(aim_shift=.0055), tr.knife_home(), tr.push(.016)))
    case('full grid with no steadying hand: no cut counts', 0., lambda tr: (tr.grid(hold=False), tr.knife_home(), tr.push(.016)))
    case('full grid, knife left standing in the fruit', 0., lambda tr: (tr.grid(), tr.push(.016), tr.st.__setitem__('knife', knife_at(c0 + R @ np.array([.02, 0, -.010]), cyaw + math.pi))))
    case('pushed 26 mm: the skin only just flat, not turned out', .75, lambda tr: (tr.grid(), tr.knife_home(), tr.push(.026)))
    case('pushed 28.5 mm: just turned out', 1., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.0285)))
    case('pushed 34 mm: turned out, short of a split', 1., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.034)))
    case('pushed 38 mm: skin split', 0., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.038)))
    def carry_over(tr):
        # The ramekin lifted 10 cm and carried over the cheek, then put back: it pushes nothing.
        ram = tr.st['ramekin']
        for z in (.05, .10, .10, .05):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=(c0[0], c0[1], float(ram.pos[2]) + z), quat_xyzw=ram.quat_xyzw); tr.step(hold=False, tool=False)
        tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)
    case('ramekin carried over the cheek, never pushed', .75, lambda tr: (tr.grid(), tr.knife_home(), carry_over(tr)))

    def carry_over_held(tr):
        # The same, with a gripper pinching the cheek: a ramekin above the cheek is never a split.
        ram = tr.st['ramekin']
        for z in (.05, .10, .06, .04, .10, .05):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=(c0[0], c0[1], float(ram.pos[2]) + z), quat_xyzw=ram.quat_xyzw)
            tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)
    case('ramekin carried over a held cheek, never pushed', .75, lambda tr: (tr.grid(), tr.knife_home(), carry_over_held(tr)))

    def face_down(tr):
        # The held cheek turned cut face down over the ramekin (face 5 mm above its base), then set back: no split.
        ram = tr.st['ramekin']; top = np.asarray(ram.pos)[2] + t.RAM_TOP
        flip = R @ t.parts._rot((1, 0, 0), math.pi)
        for dz in (.03, .005, .005, .03):
            tr.st['cheek'] = t.CHEEK.state(pos=(ram.pos[0], ram.pos[1], top + dz), quat_xyzw=q(flip)); tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.step(hold=True, tool=False, hold_x=0.)
    case('held cheek turned face down over the ramekin, never pushed', .75, lambda tr: (tr.grid(), tr.knife_home(), face_down(tr)))
    case('pushed only 8 mm: not turned out', .75, lambda tr: (tr.grid(), tr.knife_home(), tr.push(.008)))
    case('turned out before scoring, grid cut afterwards', 0., lambda tr: (tr.knife_home(), tr.push(.031), tr.grid(), tr.knife_home()))

    def under_rim(tr):
        # Upright blade brought down beside the cheek, slid in under the rim and back out: its edge stays 3.8 mm or
        # more below the skin (outside the fruit) while its spine is in the flesh, so only the blade face crosses the
        # skin.
        y = .020; zs = float(t.outer_z(0., y)); low = zs - .0038; phi = cyaw
        out_hi, out_lo = c0 + R @ np.array([.150, y, .060]), c0 + R @ np.array([.150, y, low])
        tips = [out_hi, out_lo, c0 + R @ np.array([.090, y, low]), c0 + R @ np.array([.060, y, low]), out_lo, out_hi]
        tr.tip_path(tips, phi, pitch=0.)
    case('blade edge slid under the rim, face through the skin', 0., lambda tr: (tr.grid(), under_rim(tr), tr.knife_home(), tr.push(.031)))
    case('blade lying flat (rolled 90 degrees) dragged along the lines', 0., lambda tr: (tr.grid(roll=math.pi / 2, pitch=0.), tr.knife_home(), tr.push(.016)))
    case('blade turned across the lines, pushed flat side first', 0., lambda tr: (tr.grid(phi_extra=math.pi / 2), tr.knife_home(), tr.push(.016)))
    case('cross cuts only', .25, lambda tr: ([tr.cut('cross', v, True) for v in t.CROSS_X], tr.knife_home(), tr.push(.016)))
    def rim_scratch(tr):
        # A shallow scratch 1 mm below the face, 2 mm inside the edge, held and loaded like a real cut.
        phi = cyaw - math.pi / 2; pts = []
        for x in np.linspace(.048, .008, 15):
            yb = t.B * math.sqrt(max(0., 1 - (x / t.A) ** 2)) - .002
            pts.append(c0 + R @ np.array([x, yb, -.001]))
        tr.tip_path([pts[0] + np.array([0, 0, .02]), *pts, pts[-1] + np.array([0, 0, .02])], phi + math.pi / 2)
    case('shallow scratch along the thin rim, then pushed onto the ramekin', 0., lambda tr: (rim_scratch(tr), tr.knife_home(), tr.push(.016)))
    case('extra cut next to the steadying hand (blade within 10 mm)', 0., lambda tr: (tr.grid(), tr.cut('cross', t.HOLD_X + .016, True), tr.knife_home(), tr.push(.016)))
    def from_above(tr):
        # A base-up ramekin brought down from above onto the pinched cheek, its rim 1 mm into the cut face.
        ram = tr.st['ramekin']
        for dz in (.03, .01, -.001, -.001, -.001, .03):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=c0 + R @ np.array([0., 0., dz]), quat_xyzw=ram.quat_xyzw)
            tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)

    def into_rim_up(tr):
        # The pinched cheek turned face down and lowered into a ramekin standing rim up, face 15 mm above the board.
        ram = tr.st['ramekin']; flip = t.parts._rot((1, 0, 0), math.pi)
        tr.st['ramekin'] = t.RAMEKIN.state(pos=(ram.pos[0], ram.pos[1], float(ram.pos[2]) + t.RAM_TOP), quat_xyzw=q(flip))
        for dz in (.06, .03, .015, .015, .015, .06):
            tr.st['cheek'] = t.CHEEK.state(pos=(ram.pos[0], ram.pos[1], float(ram.pos[2]) + dz), quat_xyzw=q(R @ flip))
            tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)

    def side_on(tr):
        # The ramekin standing on the board pushed 3 mm into the side of the resting cheek (their collision is off).
        ram = tr.st['ramekin']
        for k in range(6):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=c0 + R @ np.array([t.A + .024 - .003, 0., -(c0[2] - float(ram.pos[2]))]),
                                               quat_xyzw=ram.quat_xyzw)
            tr.step(hold=False, tool=False)
        tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)

    def jab(tr):
        # The held cheek jabbed onto the ramekin, 0 to 31 mm in two steps and straight back: the resistance has not
        # built up (it rises 0.5 N a step), so the push does not count.
        ram = tr.st['ramekin']; top = np.asarray(ram.pos)[2] + t.RAM_TOP
        for d in (-.005, .015, .031, .031, -.01, -.02):
            tr.st['cheek'] = t.CHEEK.state(pos=(ram.pos[0], ram.pos[1], top - d + t.H), quat_xyzw=q(R)); tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.step(hold=True, tool=False, hold_x=0.)

    def through_lifted(tr):
        # The pinched cheek lifted (face 80 mm above the board) and a base-up ramekin plunged down through it from
        # above until the base's centre is 1 mm below the cut face, held there, then taken away.
        ram = tr.st['ramekin']; lift = np.array([0., 0., .080 - (c0[2] - float(ram.pos[2]))])
        tr.st['cheek'] = t.CHEEK.state(pos=c0 + lift, quat_xyzw=q(R)); tr.step(hold=True, tool=False, hold_x=0.)
        for dz in [.06, .03, .01] + [-.001 - t.RAM_TOP] * 12 + [.06]:
            tr.st['ramekin'] = t.RAMEKIN.state(pos=c0 + lift + R @ np.array([0., 0., dz]), quat_xyzw=ram.quat_xyzw)
            tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['ramekin'] = ram; tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.step(hold=False, tool=False)

    def side_then_lift(tr):
        # The ramekin overlapping the side of the resting cheek, then the cheek lifted 15 mm while still overlapping.
        ram = tr.st['ramekin']
        side = c0 + R @ np.array([t.A + .024 - .003, 0., -(c0[2] - float(ram.pos[2]))])
        tr.st['ramekin'] = t.RAMEKIN.state(pos=side, quat_xyzw=ram.quat_xyzw)
        for dz in (0., 0., .005, .010, .015, .015, .015):
            tr.st['cheek'] = t.CHEEK.state(pos=c0 + np.array([0, 0, dz]), quat_xyzw=q(R)); tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['ramekin'] = ram; tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.step(hold=False, tool=False)

    def slid_in(tr):
        # The held cheek, its cut face 10 mm above the ramekin's base, slid sideways along its long axis until the base
        # is under its middle, then lowered 13 mm onto it: the base came in through the side, so it never presses.
        ram = tr.st['ramekin']; rp = np.asarray(ram.pos); face = rp[2] + t.RAM_TOP + .010
        for x in list(np.arange(.090, -.0001, -.003)) + [0.] * 2:
            tr.st['cheek'] = t.CHEEK.state(pos=np.r_[rp[:2] - (R @ np.array([x, 0, 0]))[:2], face], quat_xyzw=q(R))
            tr.step(hold=True, tool=False, hold_x=0.)
        for d in list(np.linspace(0, .013, 6)) + [.013] * 4:
            tr.st['cheek'] = t.CHEEK.state(pos=np.r_[rp[:2], face - d], quat_xyzw=q(R)); tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.step(hold=False, tool=False)

    def tip_then_raised(tr):
        # The held cheek slid sideways until the base's centre is 16 mm beyond its tip, then the ramekin raised 40 mm
        # above the cut face: never pressing, so no split.
        ram = tr.st['ramekin']; rp = np.asarray(ram.pos); face = rp[2] + t.RAM_TOP + .010
        for x in np.arange(.090, .0715, -.002):
            tr.st['cheek'] = t.CHEEK.state(pos=np.r_[rp[:2] - (R @ np.array([x, 0, 0]))[:2], face], quat_xyzw=q(R))
            tr.step(hold=True, tool=False, hold_x=0.)
        for dz in np.arange(0., .040, .004):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=rp + np.array([0, 0, dz]), quat_xyzw=ram.quat_xyzw); tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['ramekin'] = ram; tr.st['cheek'] = t.CHEEK.state(pos=c0, quat_xyzw=q(R)); tr.step(hold=False, tool=False)

    def past_tip(tr):
        # A base-up ramekin lowered from 30 mm above the cut face to 5 mm below it, its base's edge overlapping the
        # resting cheek's tip by 1.5 mm, then nudged 3 mm and 10 mm toward the middle: it came from above, so nothing.
        ram = tr.st['ramekin']; cx = t.A + .024 - .0015
        for dz in list(np.linspace(.030, -.005, 8)) + [-.005]:
            tr.st['ramekin'] = t.RAMEKIN.state(pos=c0 + R @ np.array([cx, 0., dz - t.RAM_TOP]), quat_xyzw=ram.quat_xyzw)
            tr.step(hold=True, tool=False, hold_x=0.)
        for dx in (.003, .010, .010):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=c0 + R @ np.array([cx - dx, 0., -.005 - t.RAM_TOP]), quat_xyzw=ram.quat_xyzw)
            tr.step(hold=True, tool=False, hold_x=0.)
        tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)

    for label, fn in (('ramekin brought down from above onto a held cheek', from_above),
                      ('held cheek slid sideways onto the ramekin, then lowered 13 mm', slid_in),
                      ('cheek slid past the ramekin, ramekin then raised 40 mm above its face', tip_then_raised),
                      ('ramekin lowered past the cheek tip from above, then nudged in', past_tip),
                      ('ramekin plunged down through a lifted, held cheek', through_lifted),
                      ('ramekin against the cheek\'s side, cheek then lifted 15 mm', side_then_lift),
                      ('held cheek turned face down into a rim-up ramekin', into_rim_up),
                      ('ramekin pushed into the side of the resting cheek', side_on),
                      ('held cheek jabbed 31 mm onto the ramekin and straight back', jab)):
        tr = Trial(); tr.grid(); tr.knife_home(); fn(tr); f = t._REC['peak_push_force']; got = tr.finish()
        want_f = (lambda v: v > 0.) if fn is jab else (lambda v: v == 0.)
        ok = abs(got[0] - .75) < 1e-9 and want_f(f)
        results.append(ok)
        if verbose:
            print(('ok  ' if ok else 'FAIL'), f'{label}: score {got[0]} (want 0.75); peak resistance {f:.2f} N; zero {got[2]}; inverted {got[3]}')
    for label, fn in (('ramekin carried over a held cheek', carry_over_held), ('held cheek face down over the ramekin', face_down)):
        tr = Trial(); tr.grid(); tr.knife_home(); fn(tr)
        ok = t._REC['peak_push_force'] == 0.
        results.append(ok)
        if verbose:
            print(('ok  ' if ok else 'FAIL'), f'{label}: no skin-bending resistance ({t._REC["peak_push_force"]:.2f} N)')
    # The declared skin-bending resistance: 0.2 N per mm of press, capped at 5 N.
    for depth, want in ((.010, 2.0), (.022, 4.4), (.030, 5.0)):
        tr = Trial(); tr.grid(); tr.knife_home(); tr.push(depth)
        ok = abs(t._REC['peak_push_force'] - want) < .05
        results.append(ok)
        if verbose:
            print(('ok  ' if ok else 'FAIL'), f'resistance at a {depth * 1000:.0f} mm push: {t._REC["peak_push_force"]:.2f} N (want {want} N)')
    t.lots.use(None)
    return all(results)


def exact_grid(t, name):
    """The demo's calibrated cut waypoints followed exactly (every blade point, edge, spine and face, sampled every
    0.5 mm of motion): no skin cut. Prints the cubes this idealised path would leave (the real arm runs lower and
    further at the cut ends, so the traced runs leave more)."""
    lot = t.lots.current(); c0 = np.array(lot.pos('cheek')); R = t.rz(lot.yaw_of('cheek')); cyaw = lot.yaw_of('cheek')
    t.fresh(); cut = False
    for fam, vals in (('cross', t.CROSS_X), ('long', t.LONG_Y)):
        rk = t.rz(cyaw - math.pi / 2 if fam == 'cross' else cyaw + math.pi) @ t.ry(t.PITCH)
        for v in vals:
            path = t.stroke(t.line_points(fam, v, True), fam, lot, cal=True)
            up = np.array([0, 0, 1.]); above = lambda p: p + up * (.006 - (p - c0) @ up)
            path = [above(path[0]) + up * .02, above(path[0]), *path, above(path[-1]), above(path[-1]) + up * .02]
            for a, b in zip(path, path[1:]):
                for w in np.linspace(0, 1, max(1, int(math.ceil(np.linalg.norm(b - a) / .0005))) + 1)[1:]:
                    tip = a + (b - a) * w
                    loc = ((tip - rk @ t.TIP) + t.BLADE_PTS @ rk.T - c0) @ R
                    cut |= t.skin_cut(loc); t.record_cut(loc[:t.N_EDGE])
    print(('ok  ' if not cut else 'FAIL'), f'lot {name}: calibrated cut waypoints followed exactly: skin cut {cut}; '
          f'cubes left {t.pieces(t._REC["minz"])["cubes"]}')
    return not cut


if __name__ == '__main__':
    import sys
    ok = run()
    if '--exact' in sys.argv:   # the calibrated waypoints followed exactly, in both lots: must never cut the skin
        for name in ('A', 'B'):
            t = load(); t.lots.use(name)
            ok &= exact_grid(t, name)
    sys.exit(0 if ok else 1)
