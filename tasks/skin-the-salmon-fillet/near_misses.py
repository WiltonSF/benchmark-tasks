"""Near misses and unintended strategies for the salmon-skinning scorer. The scorer is fail-closed: with no observed
trial history (unknown hands, load or objects) every state scores 0, so the CASES below pin the end-state shortcuts;
run() exercises the declared cutting rule and the score ladder directly (python near_misses.py)."""
CASES = [
    ('start, nothing done', {}, 0., False),
    ('flesh carried to the plate with the skin still bonded', {'flesh': ((.370, .310, .7535), None, None)}, 0., False),
    ('knife moved only', {'knife': ((.20, .0, .90), None, None)}, 0., False),
    ('knife dropped off the counter', {'knife': ((.20, -.40, .60), None, None)}, 0., False),
]


def run():
    import math
    import numpy as np
    import task as t
    out = []
    def check(label, cond):
        out.append((label, bool(cond))); print(('ok   ' if cond else 'FAIL ') + label)
    flat = np.eye(3)  # blade frame = flesh frame: edge toward -y, plane horizontal
    E = lambda y, z, x0=-.05, x1=.05: np.array([[x0, y, z], [x1, y, z]])
    # Edge-leading motion.
    check('edge-first push cuts', t.edge_leads(np.array([0, -.005, 0]), flat))
    check('draw along the edge cuts', t.edge_leads(np.array([.005, 0, 0]), flat))
    check('spine-first pull does not cut', not t.edge_leads(np.array([0, .005, 0]), flat))
    check('flat-side press does not cut', not t.edge_leads(np.array([0, 0, -.005]), flat))
    check('under 1 mm in 0.5 s does not cut', not t.edge_leads(np.array([0, -.0009, 0]), flat))
    # Full pass along the interface separates every cell with no flesh left.
    t.REC = t.fresh(); t.sweep(E(t.YT + .004, -.0003), E(t.YH - .02, -.0003), True)
    check('full pass at the interface separates 100%', t.cut_fraction() == 1. and t.loss_fraction() == 0.)
    # Band edges.
    for z, want in ((t.BAND_LO + 1e-4, True), (t.BAND_LO - 1e-4, False), (t.BAND_HI - 1e-4, True)):
        t.REC = t.fresh(); t.sweep(E(t.YT + .004, z), E(t.YT - .03, z), True)
        check('edge %.1f mm from the interface %s' % (z * 1000, 'separates' if want else 'does not separate'), (t.cut_fraction() > 0) == want)
    # Angled up: 4 mm into the flesh separates but leaves about 3 mm of flesh on the skin (over 10%: the trial fails).
    t.REC = t.fresh(); t.sweep(E(t.YT + .004, .004), E(t.YH - .02, .004), True)
    check('4 mm high pass leaves more than 10% of the flesh on the skin', t.loss_fraction() > .10)
    # Gouge: the edge 7 mm up inside the flesh fails the trial, cutting or not.
    t.REC = t.fresh(); t.sweep(E(0., .007), E(-.01, .007), False)
    check('edge 7 mm up inside the flesh fails', t.REC['bad'])
    t.REC = t.fresh(); t.sweep(E(t.YT + .004, .005), E(t.YT + .004, .005 + .02), False)
    check('lifting the edge above the tab (outside the flesh) is allowed', not t.REC['bad'])
    # A pass with the rule's conditions unmet separates nothing.
    t.REC = t.fresh(); t.sweep(E(t.YT + .004, 0.), E(t.YH - .02, 0.), False)
    check('pass without hold, load or leading edge separates nothing', t.cut_fraction() == 0. and not t.REC['bad'])
    # Ladder from the separated fraction (end gate satisfied artificially).
    objs = {p.name: p.state() for p in t.OBJECTS}
    def ladder(frac, plate=False, lost=0.):
        t.REC = t.fresh(); n = int(math.floor(frac * t.NX * t.NY + 1e-9))
        order = np.argsort(t.ROW, kind='stable'); t.REC['cut'][order[:n]] = True
        t.REC['left'][:] = lost / (t.CELL_AREA * 1050 / t.FLESH_MASS) / (t.NX * t.NY)
        t.REC.update(known=True, clear=True, hold=1.)
        o = dict(objs)
        if plate: o['flesh'] = t.FLESH.state(pos=(t.PLATE_C[0], t.PLATE_C[1], t.PLATE_TOP))
        return t.score(o)
    for frac, want in ((.2499, 0.), (.25, .25), (.4999, .25), (.5, .5), (.999, .5), (1., .75)):
        check('separated %.2f%% scores %.2f' % (frac * 100, want), ladder(frac) == want)
    check('fully separated, flesh on the plate scores 1.0', ladder(1., True) == 1.)
    check('fully separated, flesh on the plate, 6% left on skin scores 0.75', ladder(1., True, .06) == .75)
    check('11% left on skin scores 0', ladder(1., True, .11) == 0.)
    t.REC.update(hold=.95); check('hold under 1 s scores 0', t.score(dict(objs, flesh=t.FLESH.state(pos=(t.PLATE_C[0], t.PLATE_C[1], t.PLATE_TOP)))) == 0.)
    t.REC = t.fresh(); check('fresh (unknown) history scores 0', t.score(objs) == 0.)
    # Holding the knife: pads on opposite narrow sides (picked up lying flat) or opposite flat sides (side pinch).
    k = t.KNIFE.state()
    g = np.array([[x, y, 0.] for x in np.linspace(-.008, .008, 5) for y in np.linspace(-.006, .006, 4)])
    def pads(axis, a, b):
        def face(sign):
            p = g.copy()
            if axis == 1: p = np.c_[p[:, 0], np.full(len(p), sign * t.HANDLE[1] / 2), p[:, 1] * 1.5]
            else: p = np.c_[p[:, 0], p[:, 1], np.full(len(p), sign * t.HANDLE[2] / 2)]
            return t.world(k, p)
        h = t.HandClouds(right=np.concatenate([face(a), face(b)])); h.pads['right'] = (face(a), face(b)); h.fingers['right'] = np.array([-.01, -.01])
        return t.native_grasps(k, h)['right']
    check('pads on the two narrow sides hold the knife', pads(1, 1, -1))
    check('pads on the two flat sides (side pinch) hold the knife', pads(2, 1, -1))
    check('both pads on one side do not hold the knife', not pads(2, 1, 1))
    return out


if __name__ == '__main__':
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    res = run(); print('%d/%d passed' % (sum(ok for _, ok in res), len(res)))
