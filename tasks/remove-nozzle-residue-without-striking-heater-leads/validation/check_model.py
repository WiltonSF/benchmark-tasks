"""Offline tests of the declared bristle and removal rule (no simulator): python validation/check_model.py
A rigid brush (bristles straight up) is moved along set paths under the installed block. Nothing here is a robot result."""
import math, pathlib, sys
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import model
TOP = .7475
HOT = np.array((.27, 0., TOP + .182)); UNDER = HOT[2] - .006; BED = TOP + .100
BLOCK_BOX = (HOT, model.BLOCK_R, np.array((.013, .020, .012)), 1)
SHROUD = [(HOT + np.array((0, .025, .005)), np.eye(3), np.array((.047, .008, .023)), 2),
          (HOT + np.array((0, -.025, .005)), np.eye(3), np.array((.047, .008, .023)), 2),
          (HOT + np.array((.027, 0, .005)), np.eye(3), np.array((.008, .050, .023)), 2)]
BOXES = [BLOCK_BOX] + SHROUD
WIRES = []  # leads square to the block's back left face at mid-height, as in task.py
for yl, r in ((.0040, .0011), (.0062, .0011), (.0078, .00065), (.0090, .00065)):
    a = HOT + model.BLOCK_R @ np.array((.0065, yl, 0.)); b = HOT + model.BLOCK_R @ np.array((.0185, yl, 0.))
    WIRES += [(a, b, r), (b, b + np.array((0., 0., .029)), r)]
PAN = np.array((.25, -.01, BED)); I = np.eye(3); DT = 1 / 240


def head_origin(x, y, bend):
    """Brush origin that puts the bristle-root centre at (x, y) from the nozzle axis with `bend` m of bend at the underside."""
    root = UNDER - (model.BRISTLE_LENGTH - bend)
    return np.array((HOT[0] + x - .075, HOT[1] + y, root - .006))


def run(path, res=None, rot=I):
    """path: (x, y, bend, seconds) knots, linearly interpolated at 240 Hz."""
    res = res or model.Residue(HOT)
    for (x0, y0, b0, _), (x1, y1, b1, t) in zip(path, path[1:]):
        n = max(1, int(round(t / DT)))
        for k in range(1, n + 1):
            s = k / n
            res.advance(DT, head_origin(x0 + s * (x1 - x0), y0 + s * (y1 - y0), b0 + s * (b1 - b0)), rot, BOXES, PAN, I, BED, WIRES)
    return res


def lanes(bend, lanes_xy, strokes=3):
    path = [(-.05, lanes_xy[0][0], -.004, 0.)]
    for y, back, front in lanes_xy:
        path += [(back, y, -.004, 1.), (back, y, bend, .8)]
        for _ in range(strokes):
            path += [(front, y, bend, 1.5), (back, y, bend, 1.5)]
        path += [(front, y, bend, 1.5), (front, y, -.004, .5)]
    return path


DEMO_LANES = [(-.006, .005, -.003), (-.002, .005, -.003), (.002, .005, -.003), (.006, -.013, -.021), (0., .005, -.003)]
fails = []


def check(label, ok, detail=''):
    print(('ok    ' if ok else 'FAIL  ') + label + (f' ({detail})' if detail else ''))
    if not ok:
        fails.append(label)


GOOD = dict(known=True, clear=True, still=True, released=True, hold=2.)
r = model.Residue(HOT)
check('no contact: nothing removed, score 0', r.removed() == 0 and r.endpoint(**GOOD) == 0.)
r = run([(.0, -.002, .003, 0.), (.0, -.002, .003, 10.)])
check('pressed but not moving for 10 s: nothing removed', r.removed() == 0, f'removed {r.removed()}, load {r.force:.2f} N')
r = run([(-.003, .0, .003, 0.)] + [(.005, .0, .003, 1.5), (.005, .0, -.004, .2), (-.003, .0, -.004, .5), (-.003, .0, .003, .2)] * 6)
check('loaded strokes only toward the back (lifted clear coming forward): no underside cell removed', int((~r.attached[:40]).sum()) == 0, f'underside {int((~r.attached[:40]).sum())}, taper {int((~r.attached[40:]).sum())} (pressing onto the cone slides tufts along it)')
r = run([(-.004, -.005, .003, 0.)] + [(.004, .0006, .003, 1.5), (.004, .0006, -.004, .2), (-.004, -.005, -.004, .5), (-.004, -.005, .003, .2)] * 6)
check('loaded strokes only toward the lead exit (back left, lifted clear coming back): nothing removed', r.removed() == 0, f'removed {r.removed()}')
r = run([(.0, .006, .003, 0.)] + [(.0, -.006, .003, 1.5), (.0, -.006, -.004, .2), (.0, .006, -.004, .5), (.0, .006, .003, .2)] * 4)
check('loaded sideways strokes away from the leads (to the right) do remove residue', r.removed() > 0 and not r.wire_hit, f'removed {r.removed()}')
r = run(lanes(.001, DEMO_LANES))
check('light touch, 1 mm bend (threshold 1.5 mm): at most 4 underside cells (beside the nozzle shoulder) removed, score 0.25 or less', int((~r.attached[:40]).sum()) <= 4 and r.endpoint(**GOOD) <= .25, f'underside {int((~r.attached[:40]).sum())}, taper {int((~r.attached[40:]).sum())}, score {r.endpoint(**GOOD)}')
r = run(lanes(.003, DEMO_LANES))
check('demo lanes with 3 mm bend: at least 57/60 removed, no strike, no wood touch, score 1.0', r.removed() >= 57 and not r.wire_hit and r.damage == 0. and r.endpoint(**GOOD) == 1., f'removed {r.removed()}, peak {r.peak:.2f} N, max bundle {r.max_bundle:.2f} N')
for k in GOOD:
    bad = dict(GOOD); bad[k] = 0. if k == 'hold' else False
    check(f'full removal but {k} gate failing: score 0', r.endpoint(**bad) == 0.)
bad = dict(GOOD); bad['hold'] = 1.9
check('full removal with only 1.9 s still: score 0', r.endpoint(**bad) == 0.)
r2 = run(lanes(.003, DEMO_LANES)); r2.invalid = True
check('full removal after a finger came within 3 mm of the hot block: score 0', r2.endpoint(**GOOD) == 0.)
naive = [(-.006, .005, -.003), (0., .005, -.003), (.006, .005, -.003)]
r = run(lanes(.006, naive))
check('naive hard scrub (6 mm bend) over the whole block up to the lead corner: strike, score 0', r.wire_hit and r.endpoint(**GOOD) == 0., f'removed {r.removed()}, strike {r.wire_hit}')
r = run(lanes(.003, naive))
check('3 mm bend on the lead-side lane up to the back: no strike (bristle tips stay 3 mm above the underside)', not r.wire_hit)
r = run(lanes(.0055, [(.006, .005, -.003)]))
check('5.5 mm bend on the lead-side lane at the back: strike', r.wire_hit)
r = run([(-.05, 0., -.004, 0.), (0., 0., -.004, 1.), (0., 0., .0095, 2.)])
check('wood head pushed up onto the nozzle: wood-touch latch, score 0', r.damage > 0 and r.endpoint(**GOOD) == 0., f'overlap {r.damage * 1000:.2f} mm')
r = run([(-.05, 0., -.004, 0.), (0., 0., -.004, 1.), (0., 0., .0075, 2.)])
check('wood head stopped 0.5 mm below the nozzle tip: no touch', r.damage == 0., f'overlap {r.damage * 1000:.2f} mm')
r = run(lanes(-.002, DEMO_LANES))
check('forward strokes with bristle tips 2 mm short of the underside: no underside cell removed', r.removed() <= 20 and int((~r.attached[:40]).sum()) == 0, f'removed {r.removed()} (taper cells hang 6 mm lower)')
for n, want in ((14, 0.), (15, .25), (29, .25), (30, .5), (47, .5), (48, .75), (56, .75), (57, 1.), (60, 1.)):
    t = model.Residue(HOT); t.attached[:n] = False
    check(f'{n} cells removed scores {want}', t.endpoint(**GOOD) == want)
print('PASS' if not fails else f'{len(fails)} FAILURE(S)')
if fails:
    raise SystemExit(1)
