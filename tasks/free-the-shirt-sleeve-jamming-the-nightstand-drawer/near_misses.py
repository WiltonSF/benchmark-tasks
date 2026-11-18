"""Near misses and unintended strategies for the jammed nightstand drawer.

The score rests on a history (the sleeve's peak strain, the let-go, the 1 s still hold, unknown states), so each case
drives the scorer's own recorder (`task.record`, which the simulator calls every control step) through a sequence of
nightstand states and hand points. CASES (run by factory/check.py) then check the end state of the last history."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402

DT = 0.05
FAR = np.array([[0.10, 0.45, task.TOP + 0.30], [0.10, -0.45, task.TOP + 0.30]])  # both hands parked away
FAILS: list[str] = []
NS = task.NIGHTSTAND


def flat_joints():
    """Sleeve hinge angles that lay every segment flat on the shirt, pointing into the drawer."""
    pitches = []
    for (xa, za), (xb, zb) in zip(task.SLEEVE_PTS[:-1], task.SLEEVE_PTS[1:]):
        pitches.append(-math.atan2(zb - za, xb - xa))
    out, prev = {}, 0.0
    for name, p in zip(task.SEGMENTS, pitches):
        out[name] = -p - prev
        prev = -p
    return out


FLAT = flat_joints()


def st(opening=task.D0, sleeve=None):
    j = {"drawer": opening - task.D0}
    j.update(sleeve or {})
    return NS.state(joints=j)


def run(n, state, hands=FAR, reset=True):
    if reset:
        task.reset_record()
    for k in range(n):
        s = state(k) if callable(state) else state
        h = hands(k) if callable(hands) else hands
        task.record(s, h, DT)


def case(label, setup, want):
    setup()
    got = task.score(None)
    if abs(got - want) > 1e-9:
        FAILS.append(f"{label}: got {got}, want {want}")
    print(f"{'ok  ' if abs(got - want) <= 1e-9 else 'FAIL'} {label}: {got}")


OPEN = st(0.24, FLAT)

# --- the start and doing nothing --------------------------------------------------------------------------------
case("start, nothing done for 3 s", lambda: run(60, st()), 0.0)
case("the start state is caught", lambda: None if task.snag_gap(task.sleeve_points(st())) is not None
     else FAILS.append("start not caught"), 0.0)

# --- the intended outcome and its tiers --------------------------------------------------------------------------
case("sleeve pressed flat, drawer open 24 cm, let go and still", lambda: run(60, OPEN), 1.0)
case("sleeve pressed flat, drawer open exactly 22 cm", lambda: run(60, st(task.FULL, FLAT)), 1.0)
case("sleeve pressed flat, drawer open 21.9 cm", lambda: run(60, st(task.FULL - 0.001, FLAT)), 0.5)
case("sleeve pressed flat, drawer left where it jammed (10 cm)", lambda: run(60, st(task.D0, FLAT)), 0.5)
case("sleeve pressed flat, drawer pushed shut", lambda: run(60, st(0.0, FLAT)), 0.5)

# --- pulling harder ------------------------------------------------------------------------------------------------
G0 = task.snag_gap(task.sleeve_points(st()))
EDGE = task.D0 + (G0 - task.ENGAGE) + task.TEAR_N / task.K_SNAG  # opening at which the strain reaches 20 N


def pulled(o):
    def go():
        run(10, st(o))
        run(60, OPEN, reset=False)
    return go


case("pulled against the snag to 19.6 N, then pressed flat and opened", pulled(EDGE - 0.0001), 1.0)
case("pulled against the snag to 20.4 N (torn), then pressed flat and opened", pulled(EDGE + 0.0001), 0.0)
case("pulled straight out, drawer stays jammed at the snag", lambda: run(60, st(EDGE - 0.001)), 0.0)
case("drawer pushed shut with the sleeve still caught", lambda: run(60, st(0.0)), 0.0)
case("drawer pulled to 24 cm with the sleeve still standing (torn)", lambda: run(60, st(0.24)), 0.0)

# --- partial presses and other ways of getting it out -------------------------------------------------------------
part = {"sleeve1": 0.12}  # about 0.2 rad frees it
case("front of the sleeve pressed only part way, the tail still behind the rail",
     lambda: (None if task.snag_gap(task.sleeve_points(st(task.D0, part))) is not None
              else FAILS.append("partial press not caught"), run(60, st(task.D0, part))), 0.0)
out = {"sleeve1": -0.72, "sleeve2": -1.6, "sleeve3": -0.6}
case("sleeve pulled forward out of the drawer and left hanging over its front",
     lambda: run(60, st(0.24, out)), 0.0)
tall = dict(FLAT, sleeve4=FLAT["sleeve4"] - 1.4)
case("sleeve flat but its last two segments standing up out of the drawer (over 3 cm above its sides)",
     lambda: (None if not task.inside_drawer(task.sleeve_points(st(0.24, tall)), 0.24)
              else FAILS.append("tall tail counted inside"), run(60, st(0.24, tall))), 0.0)
curl = dict(FLAT, sleeve5=FLAT["sleeve5"] - 1.2)
case("sleeve flat with its end curled up, still under 3 cm above the drawer sides", lambda: run(60, st(0.24, curl)), 1.0)

# --- unknown state fails closed -----------------------------------------------------------------------------------
case("hand state unknown for one step", lambda: (run(1, OPEN, None), run(60, OPEN, reset=False)), 0.0)
case("nightstand state unknown for one step", lambda: (run(1, None), run(60, OPEN, reset=False)), 0.0)
case("non-finite hand point", lambda: run(60, OPEN, np.array([[np.nan, 0.0, 0.0]])), 0.0)

# --- let-go and stillness -----------------------------------------------------------------------------------------
bx = task.FX - task.STEM - task.HANDLE[0] / 2 - (0.24 - task.D0)  # the bar's centre x when open 24 cm
bar_top = task.TOP + task.HANDLE_Z + task.HANDLE[2] / 2


def at(x, y, z):
    return np.vstack([FAR, [[x, y, z]]])


case("a fingertip resting on the bar's top", lambda: run(60, OPEN, at(bx, 0.0, bar_top + 0.0005)), 0.0)
case("a fingertip 4 mm above the bar", lambda: run(60, OPEN, at(bx, 0.0, bar_top + 0.004)), 0.0)
case("a fingertip 6 mm above the bar", lambda: run(60, OPEN, at(bx, 0.0, bar_top + 0.006)), 1.0)
case("a fingertip 4 mm above the flattened sleeve",
     lambda: run(60, OPEN, at(0.30, 0.10, task.sleeve_points(OPEN)[:, 2].max() + 0.004)), 0.0)


def _rz(a):
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _down(n):
    """A rotation taking unit vector n to -z."""
    v = np.cross(n, [0.0, 0.0, -1.0])
    c = float(np.dot(n, [0.0, 0.0, -1.0]))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def face_case():
    """The gripper's largest flat face lies 0.5 mm above the bar, across it, with every hull vertex more than 5 mm
    from the drawer: only the face sampling sees it."""
    from yam_arena import analysis

    pts = task.hand_points("gripper")
    boxes = [(c, r, s_) for n, c, r, s_ in task.drawer_boxes(OPEN)]
    best = None
    for h in analysis.link_hulls()["gripper"]:
        for plane in h.planes:
            on = h.vertices[np.abs(h.vertices @ plane[:3] - plane[3]) < 1e-6]
            if len(on) < 3:
                continue
            r0 = _down(plane[:3])
            for a in np.linspace(0, math.pi, 37):
                r = _rz(a) @ r0
                cen = on.mean(axis=0)
                t = np.array([bx, 0.0, bar_top + 0.0005]) - r @ cen
                vmin = min(analysis.box_signed_distance(h.vertices @ r.T + t, c, rr, s_).min() for c, rr, s_ in boxes)
                if vmin > task.CLEAR + 0.001 and (best is None or vmin > best[0]):
                    best = (vmin, r, t)
    if best is None:
        FAILS.append("face case: no face found whose vertices all stay over 5 mm away")
        return
    _, r, t = best
    run(60, OPEN, np.vstack([FAR, pts @ r.T + t]))


case("a finger face 0.5 mm off the bar, no vertex within 5 mm (face sampling)", face_case, 0.0)


def creeping():
    run(60, lambda k: st(0.24 + 0.0002 * k, FLAT))


case("drawer still sliding (4 mm in the last second)", creeping, 0.0)
case("opened and let go only 0.6 s ago", lambda: (run(30, st(task.D0, FLAT)), run(12, OPEN, reset=False)), 0.0)

case("history for CASES: pressed flat, opened, let go", lambda: run(60, OPEN), 1.0)

if FAILS:
    raise SystemExit(f"{len(FAILS)} near-miss case(s) failed: {FAILS}")

CASES = [
    ("end state of a valid history: open 24 cm, sleeve inside and free, hands clear, still", {}, 1.0, True),
]
