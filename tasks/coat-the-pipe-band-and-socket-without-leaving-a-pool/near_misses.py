"""Pure scorer controls using synthetic contact records, not physical simulation results.

These fixtures explicitly provide wool poses and contact forces to test the scoring contract. GPU runs separately
validate that real simulator contacts occur. No synthetic control is a measured demo or a robot result.
"""
import importlib.util
import math
import pathlib
import sys
import numpy as np

CASES = [("no-op", {}, 0.0, False), ("objects at start without coating history", {}, 0.0, False)]


def _run():
    folder = pathlib.Path(__file__).resolve().parent
    sys.path.insert(0, str(folder))
    spec = importlib.util.spec_from_file_location("coat_controls", folder / "task.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)
    T.lots.use("A")
    can = np.asarray(T.lots.current().pos("cement_can"))
    rot = np.eye(3)
    passed = []

    def check(label, truth):
        passed.append(bool(truth))
        print(f"{'ok ' if truth else 'BAD'} {label}")

    def pose(surf, a=0):
        if surf == "pipe":
            xy, r, tip = T.PIPE_XY, T.R_PIPE + T.SWAB_R - .0025, T.H_PIPE - T.DEPTH - .0034
        else:
            xy, r, tip = T.COUP_XY, T.R_SOCK - T.SWAB_R + .0025, T.Z_STOP + .0024
        return np.array([xy[0] + r * math.cos(a), xy[1] + r * math.sin(a), tip - T.SWAB_TIP])

    def record(surf, a=0, force=.4):
        xy, r, low, part = (T.PIPE_XY, T.R_PIPE, T.H_PIPE - T.DEPTH, "pipe_vise") if surf == "pipe" else (
            T.COUP_XY, T.R_SOCK, T.Z_STOP, "coupling_vise")
        return (part, np.array([xy[0] + r * math.cos(a), xy[1] + r * math.sin(a), low + T.DEPTH / 2]), force)

    def dip(m):
        T.apply_contacts(m, np.array([can[0], can[1], T.TOP + .20]), rot, [], can)
        T.apply_contacts(m, np.array([can[0], can[1], T.TOP + .02 - T.SWAB_TIP]), rot, [], can)

    def scan(m, surf, force=.4, reported=True):
        for a in np.linspace(0, 2 * math.pi, 97):
            T.apply_contacts(m, pose(surf, a), rot, [record(surf, a, force)] if reported else [], can)

    def sequence(names, force=.4, reported=True):
        m = T.fresh()
        for k, name in enumerate(names):
            if k:
                dip(m)
            scan(m, name, force, reported)
        T.apply_contacts(m, np.array([can[0], can[1], T.TOP + .20]), rot, [], can)
        return m

    def score(m, released=2.0):
        T.M = m
        T.H = dict(known=True, bad=None, released=released)
        return T.score({"dauber": T.DAUBER.state(), "cement_can": T.CAN.state()})

    good = sequence(["pipe", "socket", "pipe"])
    check("complete contact sequence reaches 1.0", score(good) == 1)
    check("no-op is zero", score(T.fresh()) == 0)
    check("no contacts cannot paint despite overlap", score(sequence(["pipe", "socket", "pipe"], reported=False)) == 0)
    check("0.19 N cannot paint", score(sequence(["pipe", "socket", "pipe"], force=.19)) == 0)
    check("0.20 N can paint", score(sequence(["pipe", "socket", "pipe"], force=.20)) == 1)
    check("force above 1 N is not a squeeze penalty", score(sequence(["pipe", "socket", "pipe"], force=1.2)) == 1)
    check("held dauber earns zero", score(good, released=0) == 0)
    check("less than one second released earns zero", score(good, released=.95) == 0)
    check("no second pipe pass is 0.75", score(sequence(["pipe", "socket"])) == .75)
    check("pipe, pipe, socket has wrong order", score(sequence(["pipe", "pipe", "socket"])) == .75)
    check("socket, pipe, pipe has wrong order", score(sequence(["socket", "pipe", "pipe"])) == .75)
    check("extra socket coat is not penalized", score(sequence(["pipe", "socket", "socket", "pipe"])) == 1)
    check("returning to socket after final pipe invalidates order", score(sequence(["pipe", "socket", "pipe", "socket"])) == .75)
    check("pipe alone is 0.25", score(sequence(["pipe"])) == .25)
    check("socket alone is 0.25", score(sequence(["socket"])) == .25)

    # A contact at the bottom of the wall must not be confused with the stop face.
    low_wall = np.array([T.COUP_XY[0] + T.R_SOCK, T.COUP_XY[1], T.Z_STOP + .0005])
    check("low wall contact with radial normal is socket", T.classify("coupling_vise", low_wall, [1, 0, 0]) == "socket")
    check("stop face with vertical normal is stop", T.classify("coupling_vise", low_wall, [0, 0, 1]) == "stop")
    edge = low_wall.copy(); edge[2] = T.Z_STOP + .0003
    check("radial contact at conservative wall exclusion boundary is not stop", T.classify("coupling_vise", edge, [1, 0, 0]) == "other")

    m = T.fresh()
    p = pose("socket")
    T.apply_contacts(m, p, rot, [record("pipe", force=1)], can)
    check("force on a different surface cannot credit socket", not np.any(m["s_coats"]))
    m = T.fresh()
    # A wire contact near the socket mouth, with the actual wool entirely below the contact.
    wire_pose = record("socket")[1] + np.array([0, 0, .03])
    T.apply_contacts(m, wire_pose, rot, [record("socket", force=1)], can)
    check("wire contact is not wool force", not np.any(m["s_coats"]))

    def fault(label, part, point, expected):
        m = sequence(["pipe", "socket", "pipe"])
        p = np.asarray(point) + np.array([0, 0, -(T.SWAB_TOP + T.SWAB_TIP) / 2])
        T.apply_contacts(m, p, rot, [(part, np.asarray(point), .01)], can)
        check(label, m["fail"] == expected and score(m) == 0)

    fault("any wet wool contact on pipe bore fails", "pipe_vise",
          (T.PIPE_XY[0] + T.R_BORE, T.PIPE_XY[1], T.H_PIPE - .005), "cement inside the pipe bore")
    fault("any wet wool contact below socket stop fails", "coupling_vise",
          (T.COUP_XY[0] + T.R_BORE, T.COUP_XY[1], T.Z_STOP - .003), "cement past the stop into the bore")
    fault("visible stop-face contamination fails", "coupling_vise",
          (T.COUP_XY[0] + T.R_BORE + .001, T.COUP_XY[1], T.Z_STOP), "cement on the coupling stop")
    fault("actual wet wool bench contact fails", "bench", (.40, .20, T.TOP), "cement on the bench")
    m = T.fresh()
    T.apply_contacts(m, np.array([T.PIPE_XY[0], T.PIPE_XY[1], T.H_PIPE + .060]), rot, [], can)
    check("swab in empty bore air without contact is not contamination", m["fail"] is None)
    m = T.fresh()
    T.apply_contacts(m, pose("socket"), rot, [("coupling_vise", record("socket")[1], float('nan'))], can)
    check("nonfinite force fails closed", m["fail"] == "unreadable contact record" and not np.any(m["s_coats"]))
    # A short angular stroke cannot fill a whole surface or the before/after phase masks.
    m = T.fresh()
    for a in np.linspace(0, math.pi, 49):
        T.apply_contacts(m, pose("pipe", a), rot, [record("pipe", a)], can)
    check("half an orbit is not a complete surface", T.coverage(m)["pipe1"] < 1 and score(m) == 0)
    T.lots.use(None)
    print(f"{'PASS' if all(passed) else 'FAIL'} {sum(passed)}/{len(passed)} synthetic scorer controls")
    return 0 if all(passed) else 1


if __name__ == "__main__":
    raise SystemExit(_run())
