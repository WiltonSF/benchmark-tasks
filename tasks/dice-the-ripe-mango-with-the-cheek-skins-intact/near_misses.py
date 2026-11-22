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

    case('full grid, knife put back, skin pushed 16 mm onto the ramekin', 1., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.016)))
    case('full grid, knife put back, never turned out', .75, lambda tr: (tr.grid(), tr.knife_home()))
    case('grid cut 4.5 mm too deep: through the skin', 0., lambda tr: (tr.grid(aim_shift=-.0045), tr.knife_home(), tr.push(.016)))
    case('grid cut 8.5 mm short of the skin: cubes stay joined', 0., lambda tr: (tr.grid(aim_shift=.0055), tr.knife_home(), tr.push(.016)))
    case('full grid with no steadying hand: no cut counts', 0., lambda tr: (tr.grid(hold=False), tr.knife_home(), tr.push(.016)))
    case('full grid, knife left standing in the fruit', 0., lambda tr: (tr.grid(), tr.push(.016), tr.st.__setitem__('knife', knife_at(c0 + R @ np.array([.02, 0, -.010]), cyaw + math.pi))))
    case('pushed 26 mm: a full inside-out turn', 1., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.026)))
    case('pushed 38 mm: skin split', 0., lambda tr: (tr.grid(), tr.knife_home(), tr.push(.038)))
    def carry_over(tr):
        # The ramekin lifted 10 cm and carried over the cheek, then put back: it pushes nothing.
        ram = tr.st['ramekin']
        for z in (.05, .10, .10, .05):
            tr.st['ramekin'] = t.RAMEKIN.state(pos=(c0[0], c0[1], float(ram.pos[2]) + z), quat_xyzw=ram.quat_xyzw); tr.step(hold=False, tool=False)
        tr.st['ramekin'] = ram; tr.step(hold=False, tool=False)
    case('ramekin carried over the cheek, never pushed', .75, lambda tr: (tr.grid(), tr.knife_home(), carry_over(tr)))
    case('pushed only 8 mm: not turned out', .75, lambda tr: (tr.grid(), tr.knife_home(), tr.push(.008)))
    case('turned out before scoring, grid cut afterwards', 0., lambda tr: (tr.knife_home(), tr.push(.016), tr.grid(), tr.knife_home()))
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
    t.lots.use(None)
    return all(results)


if __name__ == '__main__':
    import sys
    ok = run()
    if '--exact' in sys.argv:   # the calibrated waypoints followed exactly, in both lots: must never cut the skin
        for name in ('A', 'B'):
            ok &= run(lot_name=name, cal=True, verbose=True) or True
    sys.exit(0 if ok else 1)
