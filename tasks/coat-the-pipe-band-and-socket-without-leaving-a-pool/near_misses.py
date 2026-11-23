"""Near misses and shortcuts.

CASES are end states with no coating history; check.py scores them in lot A (all must be 0: nothing was coated).
Running this file (python near_misses.py) replays scripted strategies through the scorer's declared coating rule
offline: the dauber follows the gripper exactly while held (no arm sag), then the end-state rules are applied. Each
strategy's expected score is asserted. The simulator run adds physics; this checks the rule itself.
"""

import math
import pathlib
import sys

import numpy as np

CASES = [
    ("do nothing", {}, 0.0, False),
    ("dauber lifted out and hung back without coating", {}, 0.0, False),
]


SAG = (0.0, 0.0, -0.001)  # the held dauber ran about 1 mm below its commanded height in the GPU traces


def _replay(T, W, lot="A", offset=SAG):
    from yam_arena.task_file import ObjectState

    T.lots.use(lot)
    L = T.lots.current()
    can = np.asarray(L.pos("cement_can"))
    m = T.fresh()
    held, dauber = False, np.asarray(L.pos("dauber"), float)
    cur = np.asarray(W[0].pos) + (0, 0, 0.1)
    for w in W:
        n = max(1, round(w.duration * 20))
        start, goal = cur.copy(), np.asarray(w.pos) + offset
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)
            cur = start + (goal - start) * s
            if held:
                dauber = cur - (0, 0, T.GRIP_ABOVE_LID)
            T.sweep(m, dauber, np.eye(3), T.PIPE_XY, T.COUP_XY, can)
        if w.finger > T.FINGER_OPEN + 0.01 and abs(cur[0] - dauber[0]) < 0.01 and abs(cur[1] - dauber[1]) < 0.01 \
                and abs(cur[2] - T.GRIP_ABOVE_LID - dauber[2]) < 0.004:
            held = True
        if w.finger <= T.FINGER_OPEN + 1e-6 and held:
            held = False
            neck = can[2] + T.CAN_H / 2 + T.NECK_H
            if np.hypot(*(dauber[:2] - can[:2])) < 0.006 and dauber[2] - T.LID_H / 2 - neck < 0.004:
                dauber = np.array([can[0], can[1], neck + T.LID_H / 2])
    T.M = m
    T.H = dict(known=True, bad=None, t=0.0, released=0.0 if held else 2.0, start=can)
    st = lambda p: ObjectState(pos=np.asarray(p, float), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                               ang_vel=np.zeros(3), low=np.asarray(p, float), high=np.asarray(p, float))
    objs = {"dauber": st(dauber), "cement_can": st(can)}
    out = T.score(objs)
    T.lots.use(None)
    return out, m


def _cut(W, first, last):
    """Drop waypoints from the first whose label contains `first` through the last whose label contains `last`."""
    i = next(k for k, w in enumerate(W) if first in w.label)
    j = max(k for k, w in enumerate(W) if last in w.label)
    return W[:i] + W[j + 1:]


def _move(W, label_part, d_radius, centre):
    """Push the waypoints with `label_part` radially away from `centre` by d_radius (m)."""
    from yam_arena.demo import Waypoint

    out = []
    for w in W:
        if label_part in w.label:
            v = np.asarray(w.pos[:2]) - centre
            r = np.linalg.norm(v)
            p = np.asarray(centre) + v / r * (r + d_radius)
            w = Waypoint(w.duration, (float(p[0]), float(p[1]), w.pos[2]), w.finger, w.label, w.yaw)
        out.append(w)
    return out


def _run():
    folder = pathlib.Path(__file__).resolve().parent
    sys.path.insert(0, str(folder))
    import importlib.util

    spec = importlib.util.spec_from_file_location("task_nm", folder / "task.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)
    from yam_arena.demo import Waypoint

    T.lots.use("A")
    D = T.demo()["right"]
    T.lots.use(None)
    grab = D[:4]  # take the lid and lift the dauber out
    stow = D[-5:]  # over the can, lid on the neck, let go, clear, still
    f = T.FINGER_LID
    cx, cy = T.COUP_XY
    px, py = T.PIPE_XY
    rim = T.H_COUP + 0.004 + T.TCP_TO_TIP
    results = []

    def case(name, W, want, lot="A", offset=SAG):
        got, m = _replay(T, W, lot, offset)
        ok = abs(got - want) < 1e-9
        results.append(ok)
        print(f"{'ok ' if ok else 'BAD'} {name}: {got} (expected {want}); coverage {T.coverage(m)}, fail {m['fail']}")

    case("demo, lot A", D, 1.0)
    T.lots.use("B")
    DB = T.demo()["right"]
    T.lots.use(None)
    case("demo, lot B", DB, 1.0, "B")
    case("lot A demo replayed in lot B (fixed habit)", D, 0.0, "B")
    for off in [(0, 0, -0.0004), (0, 0, -0.0016), (0.0014, 0, -0.001), (0, -0.0014, -0.001)]:  # around the 1 mm sag
        case(f"demo with a {off} m tracking error", D, 1.0, "A", off)
    case("demo 3.5 mm too low (swab end past the stop)", D, 0.0, "A", (0, 0, -0.0035))
    case("no second pipe coat", _cut(D, "over the can (second coat)", "up (second coat)"), 0.75)
    case("dauber kept in the hand", [w for w in D if w.label not in ("let go", "hands clear", "still")], 0.0)
    case("pipe only, then back in the can", _cut(D, "over the can (for the socket)", "up (second coat)"), 0.25)
    case("socket only, then back in the can",
         _cut(_cut(D, "beside the pipe (first coat)", "up (first coat)"), "over the can (second coat)", "up (second coat)"), 0.25)
    case("no re-dip before the socket (dauber runs dry)", _cut(D, "over the can (for the socket)", "lift out (for the socket)"), 0.25)
    names = ("up to rim height", "round the pipe vise", "over the socket", "down into the socket", "round the socket wall",
             "back to the centre", "up out of the socket", "back round the pipe vise")
    sock = [w for w in D if w.label in names]
    i = max(k for k, w in enumerate(D) if w.label == "back round the pipe vise")
    dip2 = [w for w in D if w.label in ("over the can (second coat)", "dip (second coat)", "soak (second coat)", "lift out (second coat)")]
    case("socket coated again from a second dip (puddle)", D[:i + 1] + dip2 + sock + D[i + 1:], 0.0)
    # Order: the can asks for pipe, socket, then pipe again. Both coats on the pipe first, then the socket: 0.75.
    a0 = next(k for k, w in enumerate(D) if w.label == "over the can (for the socket)")
    b0 = next(k for k, w in enumerate(D) if w.label == "over the can (second coat)")
    b1 = max(k for k, w in enumerate(D) if w.label == "up (second coat)")
    first = [w for w in D[4:a0]]  # first pipe coat (after taking the dauber)
    socket_part = D[a0:b0]  # dip, socket
    second = D[b0:b1 + 1]  # dip, pipe again
    case("pipe, pipe, then socket (order wrong)", D[:4] + first + second + socket_part + stow, 0.75)
    redip_first = [w for w in D if w.label in ("over the can (for the socket)", "dip (for the socket)", "soak (for the socket)",
                                               "lift out (for the socket)")]
    case("socket first, then pipe twice (order wrong)", D[:4] + socket_part[4:] + redip_first + first + second + stow, 0.75)
    case("pressed 3 mm harder on the pipe (squeezed out)", _move(D, "round the band", -0.003, np.array((px, py))), 0.0)
    case("swab held 2 mm off the pipe (no contact; socket still done)", _move(D, "round the band", 0.0049, np.array((px, py))), 0.25)
    case("socket swab held 2 mm off the wall (pipe still done)", _move(D, "round the socket wall", -0.0049, np.array((cx, cy))), 0.25)
    jab = grab + [Waypoint(1.5, (D[3].pos[0], D[3].pos[1], rim), f, "up"), Waypoint(2.0, (T.HUB[0], T.HUB[1], rim), f, "hub"),
                  Waypoint(2.0, (cx, cy, rim), f, "over socket"),
                  Waypoint(1.5, (cx, cy, T.Z_STOP - 0.004 + T.TCP_TO_TIP), f, "jab to the bottom")] + stow
    case("loaded dauber pushed straight down into the socket (puddle in the bore)", jab, 0.0)
    pipe_jab = grab + [Waypoint(2.0, (px, py, T.H_PIPE + 0.006 + T.TCP_TO_TIP), f, "over the pipe end"),
                       Waypoint(1.5, (px, py, T.H_PIPE - 0.01 + T.TCP_TO_TIP), f, "into the pipe end")] + stow
    case("swab pushed into the pipe end", pipe_jab, 0.0)
    low = _move(D, "round the band", 0.0, np.array((px, py)))
    low = [Waypoint(w.duration, (w.pos[0], w.pos[1], w.pos[2] + (0.008 if "band" in w.label else 0.0)), w.finger, w.label, w.yaw)
           for w in low]
    case("band coated 8 mm short of the line (bottom rows bare)", low, 0.5)
    print("PASS" if all(results) else "FAIL", f"{sum(results)}/{len(results)} strategies scored as declared")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(_run())
