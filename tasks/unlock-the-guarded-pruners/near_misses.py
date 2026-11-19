"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-step record what the step callback would have measured for a still end
state with both grippers 50 mm away, held for 2 s. _run() then drives the step-side functions directly (the forced-
through and lost-from-guard rules, the release distance from gripper faces, the stillness test, the 2 s hold and the
missing-measurement rule) in both lots, and raises if any scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import PRUNER, STAND, Q_OPEN, Q_SQUEEZE, STOP_FREE, TOP, REST_H

DEG = math.radians


def quat_z(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def scene(q=Q_OPEN, stop=STOP_FREE, dxy=(0.0, 0.0), dyaw=0.0, dz=0.0, sdxy=(0.0, 0.0), sdyaw=0.0, lot=None):
    lot = lots.current() if lot is None else lot
    x, y, z = lot.pos("pruner")
    a = lot.yaw_of("pruner")
    sx, sy, sz = lot.pos("stand")
    return {
        "stand": STAND.state(pos=(sx + sdxy[0], sy + sdxy[1], sz), quat_xyzw=quat_z(a + sdyaw)),
        "pruner": PRUNER.state(joints={"handle": q, "stopper": stop}, pos=(x + dxy[0], y + dxy[1], z + dz),
                               quat_xyzw=quat_z(a + dyaw)),
    }


def away():
    task._fresh()
    task._rec.update(gap=0.05, t=10.0, since=8.0, held=True, still=True)


away()
lots.use("A")
_half_q = next(math.radians(d / 10) for d in range(0, 200) if task.tip_gap(math.radians(d / 10)) >= task.LOCKED_GAP + 0.010)
CASES = [
    ("stopper off, handles at their opening stop, in the rest", scene(), 1.0, True),
    ("handles exactly 5 degrees short of the stop", scene(q=Q_OPEN - DEG(5.0)), 1.0, True),
    ("handles 5.5 degrees short of the stop (half credit)", scene(q=Q_OPEN - DEG(5.5)), 0.5, False),
    ("handle ends exactly 10 mm wider than locked", scene(q=_half_q), 0.5, False),
    ("handle ends 9 mm wider than locked", scene(q=_half_q - DEG(0.5)), 0.0, False),
    ("still locked (do nothing)", scene(q=0.0, stop=0.0), 0.0, False),
    ("squeezed shut, stopper off, held there", scene(q=Q_SQUEEZE), 0.0, False),
    ("stopper off but handles shut", scene(q=0.0), 0.0, False),
    ("open with the stopper turned only 14 degrees", scene(stop=DEG(14.0)), 0.0, False),
    ("open with the stopper turned exactly 15 degrees", scene(stop=DEG(15.0)), 1.0, True),
    ("open but the tool slid 9 mm in its rest (half credit)", scene(dxy=(0.009, 0.0)), 0.5, False),
    ("open, the tool slid exactly 8 mm", scene(dxy=(0.008, 0.0)), 1.0, True),
    ("open, the tool turned 6 degrees in its rest", scene(dyaw=DEG(6.0)), 0.5, False),
    ("open, the tool lifted 3 mm off the rest", scene(dz=0.003), 0.5, False),
    ("open, the stand pushed 9 mm", scene(sdxy=(0.0, 0.009)), 0.5, False),
    ("open, the stand turned 4 degrees", scene(sdyaw=DEG(4.0)), 0.5, False),
]


def box_hull(center, half, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    half = np.asarray(half, float)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    verts = corners @ r.T + np.asarray(center, float)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = sign * r[:, i]
            planes.append([*n, float(n @ (np.asarray(center, float) + sign * half[i] * r[:, i]))])

    class _H:
        pass

    h = _H()
    h.vertices, h.planes = verts, np.array(planes)
    return h


def _run():
    bad = []
    for lot in ("A", "B"):
        lots.use(lot)
        done = scene()

        def fresh():
            task._fresh()

        def run_hold(times, gap=0.05, objs=done, still=True):
            fresh()
            for t in times:
                task._rec.update(gap=gap, t=t, still=still)
                task._hold(objs, t)
            return task.score(objs)

        # The hold: 1.95 s is not enough, 2.0 s is.
        if run_hold((0.0, 1.95)) != 0.0 or run_hold((0.0, 2.0)) != 1.0:
            bad.append(f"lot {lot}: 2 s hold wrong")
        # Release distance: 6 mm counts, 4 mm and exactly 5 mm do not.
        for gap, want in ((0.006, 1.0), (0.004, 0.0), (task.CLEAR, 0.0)):
            if run_hold((0.0, 2.0), gap=gap) != want:
                bad.append(f"lot {lot}: gap {gap * 1000:.1f} mm scored wrongly")
        # Not still: no credit.
        if run_hold((0.0, 2.0), still=False) != 0.0:
            bad.append(f"lot {lot}: moving end state scored")
        # No gripper measurement: no credit.
        fresh()
        if task.score(done) != 0.0:
            bad.append(f"lot {lot}: no gripper measurement scored")
        # Held 2 s, then a gripper comes back to 4 mm.
        fresh()
        for t, gap in ((0.0, 0.05), (2.0, 0.05), (2.05, 0.004)):
            task._rec.update(gap=gap, t=t, still=True)
            task._hold(done, t)
        if task.score(done) != 0.0:
            bad.append(f"lot {lot}: gripper back within 5 mm scored")
        # Forced open past the stopper earlier, then a good finish: 0.
        fresh()
        if not task.forced_through(scene(q=DEG(3.5), stop=DEG(2.0))["pruner"]):
            bad.append(f"lot {lot}: handle 3.5 deg open with stopper on not flagged")
        if task.forced_through(scene(q=DEG(2.5), stop=DEG(0.0))["pruner"]):
            bad.append(f"lot {lot}: 2.5 deg play flagged as forced")
        if task.forced_through(scene(q=DEG(10.0), stop=DEG(25.0))["pruner"]):
            bad.append(f"lot {lot}: proper release flagged as forced")
        task._rec["forced"] = True
        for t in (0.0, 2.0):
            task._rec.update(gap=0.05, t=t, still=True)
            task._hold(done, t)
        if task.score(done) != 0.0:
            bad.append(f"lot {lot}: forced-through then good finish scored")
        # Taken out of the guard and put back: 0.
        for label, st, want in (("pivot 21 mm away", scene(dxy=(0.021, 0.0)), True),
                                ("pivot 19 mm away", scene(dxy=(0.019, 0.0)), False),
                                ("lifted 16 mm", scene(dz=0.016), True),
                                ("lifted 14 mm", scene(dz=0.014), False),
                                ("stand pushed 21 mm", scene(sdxy=(0.021, 0.0)), True)):
            if task.lost(st) != want:
                bad.append(f"lot {lot}: lost rule, {label}: {task.lost(st)}")
        fresh()
        task._rec["lost"] = True
        for t in (0.0, 2.0):
            task._rec.update(gap=0.05, t=t, still=True)
            task._hold(done, t)
        if task.score(done) != 0.0:
            bad.append(f"lot {lot}: lifted out then put back scored")
        # Stillness: 4 mm/s still, 6 mm/s not; a stopper flicking at 0.06 rad/s not.
        a = task._snapshot(done, 0.0)
        for dx, want in ((0.0002, True), (0.0003, False)):
            b = task._snapshot(scene(dxy=(dx, 0.0)), 0.05)
            if task.still_between(a, b) != want:
                bad.append(f"lot {lot}: stillness at {dx / 0.05 * 1000:.0f} mm/s wrong")
        b = task._snapshot(scene(stop=STOP_FREE - 0.003), 0.05)
        if task.still_between(a, b):
            bad.append(f"lot {lot}: stopper turning 0.06 rad/s counted as still")
        # Release measured from gripper faces: a 60 mm plate 4 mm over the grips, no corner near, is not let go.
        x, y, z = lots.current().pos("pruner")
        yaw = lots.current().yaw_of("pruner")
        cx, cy = task._world(lots.current(), (-0.11, 0.0))
        for dz, want in ((0.004, False), (0.006, True)):
            plate = box_hull((cx, cy, z + task.HOOK_Z1 + 0.003 + dz + 0.002), (0.030, 0.030, 0.002), yaw)
            wh = task._WorldHull(plate, task._face_points(plate), np.eye(4))
            g_faces, g_corners = task._gap(done, [wh]), task._gap(done, [plate])
            if (g_faces > task.CLEAR) != want or g_corners <= task.CLEAR:
                bad.append(f"lot {lot}: plate {dz * 1000:.0f} mm over: faces {g_faces * 1000:.1f} mm, "
                           f"corners {g_corners * 1000:.1f} mm")
    # A fixed habit: the lot-A grip replayed in lot B lands off the grips.
    lots.use("A")
    ga = np.array(task.demo()["right"][1].pos[:2])
    lots.use("B")
    gb = np.array(task.demo()["right"][1].pos[:2])
    if np.linalg.norm(ga - gb) < 0.03:
        bad.append("lots A and B grip points under 30 mm apart")
    lots.use("A")
    away()
    if bad:
        raise AssertionError("; ".join(bad))


_run()

if __name__ == "__main__":
    for label, states, want, _ in CASES:
        away()
        print(f"{task.score(states):.2f} (want {want:.2f})  {label}")
    print("rule checks passed")
