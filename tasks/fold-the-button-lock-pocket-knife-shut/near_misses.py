"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-step record what the step callback would have measured for a state with
both grippers 50 mm away that has held for 1 s. _run() below then drives the step-side functions directly with
synthetic gripper hulls (boxes) to check the edge touch, the sweep while the blade turns, the release distance, the
1 s hold and the missing-measurement rule, in every lot, and raises if any scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import CLOSED, KNIFE, TOP
from yam_arena.analysis import Hull

DEG = math.radians


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), math.cos(a))


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def knife(q=CLOSED, button=0.0, pos=None, qx=None, vel=None):
    return {"knife": KNIFE.state(joints={"blade": q, "button": button}, pos=pos, quat_xyzw=qx, lin_vel=vel)}


def tipped(deg, lift=0.0):
    """Rolled deg about the handle's long axis, its lowest corner lift above the table."""
    q = qmul(quat((0, 0, 1), math.degrees(task.KNIFE_YAW)), quat((0, 1, 0), deg))
    st = KNIFE.state(joints={"blade": CLOSED}, quat_xyzw=q)
    z = TOP - (st.low[2] - TOP) + lift
    return knife(pos=(task.KNIFE_XY[0], task.KNIFE_XY[1], z), qx=q)


def away():
    """What the step callback records for a still, released end state held 1 s: grippers 50 mm away."""
    task._rec.update(cut=False, q_prev=None, gap=0.05, t=10.0, since=9.0, held=True)


away()
X, Y = task.KNIFE_XY
CASES = [
    ("shut, flat, let go and still", knife(), 1.0, True),
    ("shut to exactly 5 degrees out", knife(CLOSED - DEG(5.0)), 1.0, True),
    ("6 degrees short of shut", knife(CLOSED - DEG(6.0)), 0.0, False),
    ("folded halfway and left (edge still out)", knife(DEG(90.0)), 0.0, False),
    ("still open on its lock (do nothing)", knife(0.0), 0.0, False),
    ("open with the button held in", knife(0.0, 0.0018), 0.0, False),
    ("shut but still moving (dropped or sliding)", knife(vel=(0.0, 0.05, 0.0)), 0.0, False),
    ("shut, lying 9 degrees off flat on the table", tipped(9.0), 1.0, True),
    ("shut, on its side (11 degrees off flat)", tipped(11.0), 0.0, False),
    ("shut, held 2 mm off the table", knife(pos=(X, Y, TOP + 0.002)), 0.0, False),
    ("shut, lowest point exactly 1 mm up", knife(pos=(X, Y, TOP + 0.001)), 1.0, True),
    ("shut, moved across the table", knife(pos=(X + 0.08, Y - 0.05, TOP)), 1.0, True),
]


def box_hull(center, half, yaw=0.0):
    """A gripper part as a box hull (world frame): centre, half sizes, turned yaw about z."""
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    half = np.asarray(half, float)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    verts = corners @ r.T + np.asarray(center, float)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = sign * r[:, i]
            planes.append([*n, float(n @ np.asarray(center, float) + half[i])])
    return Hull(vertices=verts, planes=np.array(planes))


def _run():
    bad = []

    def fresh():
        task._rec.update(cut=False, q_prev=None, gap=None, t=None, since=None, held=False)

    def at(lot, q):
        lots.use(lot)
        yaw = lots.current().yaw_of("knife")
        px, py, pz = lots.current().pos("knife")
        return KNIFE.state(joints={"blade": q}, pos=(px, py, pz), quat_xyzw=(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))

    def world(lot, q, u, v, z):
        """A blade-frame point (u along, v across toward the edge, z above the table) in the world, in a lot."""
        lots.use(lot)
        x, y = task._blade_to_knife(np.array([[u, v, z]]), q)[0, :2]
        wx, wy = task._world(lots.current(), (x, y))
        return np.array([wx, wy, TOP + z])

    def heading(lot, q):
        lots.use(lot)
        return task._heading(lots.current(), q)

    zc = task.BLADE_ZC
    for lot in ("A", "B", "C"):
        q0 = DEG(40.0)
        h = heading(lot, q0)
        # A finger as a 4 x 16 x 50 mm slab whose 16 x 50 mm face lies along the blade, beside the edge or the spine.
        # Its centre is offset across the blade by v; tall slabs reach from below the blade to far above it.
        across = h - math.pi / 2  # the direction of +v (toward the edge) at this angle

        def slab(v_face, z0, z1, side=+1):
            c = world(lot, q0, 0.05, v_face + side * 0.002, (z0 + z1) / 2)
            return box_hull(c, (0.008, 0.002, (z1 - z0) / 2), across + math.pi / 2)

        edge = task.EDGE_V
        for label, hull, moving, want in [
            ("finger face 0.5 mm off the edge, tips 1 mm over the table (no corner near the edge)",
             slab(edge + 0.0005, 0.001, 0.051), 0.0, True),
            ("finger face 0.9 mm off the edge, from 20 mm up down to the table",
             slab(edge + 0.0009, 0.000, 0.020), 0.0, True),
            ("finger face 2 mm off the edge", slab(edge + 0.002, 0.001, 0.051), 0.0, False),
            ("finger face on the spine", slab(task.SPINE_V - 0.0002, 0.001, 0.051, side=-1), 0.0, False),
            ("finger face on the spine, blade turning", slab(task.SPINE_V - 0.0002, 0.001, 0.051, side=-1), 1.0,
             False),
            ("finger tips 5 mm over the blade, above the edge",
             box_hull(world(lot, q0, 0.05, edge, task.BLADE_Z0 + task.BLADE_T + 0.005 + 0.025),
                      (0.008, 0.004, 0.025), h), 0.0, False),
            ("finger 20 mm ahead of the edge, blade turning (in the sweep)",
             slab(edge + 0.020, 0.001, 0.051), 1.0, True),
            ("finger 20 mm ahead of the edge, blade still", slab(edge + 0.020, 0.001, 0.051), 0.0, False),
            ("finger 20 mm ahead, blade turning at 1 degree/s (0.05 per step)",
             slab(edge + 0.020, 0.001, 0.051), 0.05, False),
            ("finger 20 mm ahead, blade turning at 2 degrees/s (0.1 per step)",
             slab(edge + 0.020, 0.001, 0.051), 0.1, True),
            ("finger 20 mm ahead with its tips 3 mm over the blade, turning",
             slab(edge + 0.020, task.BLADE_Z0 + task.BLADE_T + 0.003, 0.051), 1.0, False),
            ("finger 20 mm ahead with its tips 1 mm over the blade, turning",
             slab(edge + 0.020, task.BLADE_Z0 + task.BLADE_T + 0.001, 0.051), 1.0, True),
        ]:
            fresh()
            k0, k1 = at(lot, q0 - DEG(moving)), at(lot, q0)
            task.judge(k0, [hull])
            task.judge(k1, [hull])
            if task._rec["cut"] != want:
                bad.append(f"lot {lot}: {label}: cut={task._rec['cut']}, expected {want}")
        # Holding the handle across its open side, where the blade comes in, while it turns.
        fresh()
        lots.use(lot)
        hx, hy = task._world(lots.current(), (task.HANDLE_W / 2 + 0.004, -0.05))
        side = box_hull((hx, hy, TOP + 0.026), (0.002, 0.008, 0.025), lots.current().yaw_of("knife"))
        task.judge(at(lot, q0 - DEG(1.0)), [side])
        task.judge(at(lot, q0), [side])
        if not task._rec["cut"]:
            bad.append(f"lot {lot}: pinching the handle's open side while the blade turns was not a cut")
        # Pressing the button on the upper scale while the blade turns.
        fresh()
        bx, by = task._world(lots.current(), task.BUTTON_XY)
        press = box_hull((bx, by, TOP + task.HANDLE_T + 0.0002 + 0.025), (0.008, 0.004, 0.025))
        task.judge(at(lot, q0 - DEG(1.0)), [press])
        task.judge(at(lot, q0), [press])
        if task._rec["cut"]:
            bad.append(f"lot {lot}: pressing the button counted as a cut")
    lots.use(None)

    # Release, hold and the missing measurement, through the step-side hold counter.
    shut = knife()["knife"]
    for label, gap, times, want in [
        ("shut, grippers 6 mm away, held 1 s", 0.006, (0.0, 0.5, 1.0), 1.0),
        ("shut, grippers 6 mm away, held 0.95 s", 0.006, (0.0, 0.5, 0.95), 0.0),
        ("shut, a gripper 4 mm away (still holding)", 0.004, (0.0, 0.5, 1.0), 0.0),
        ("shut, grippers exactly 5 mm away", 0.005, (0.0, 0.5, 1.0), 0.0),
        ("shut, no gripper measurement", None, (0.0, 0.5, 1.0), 0.0),
    ]:
        fresh()
        for t in times:
            task._rec.update(gap=gap, t=t)
            if gap is None:
                task._rec.update(since=None, held=False)
            else:
                task._hold(shut, t)
        s = task.score({"knife": shut})
        if abs(s - want) > 1e-9:
            bad.append(f"{label}: {s}, expected {want}")
    # Held 1 s, then a gripper comes back to 4 mm: no credit at that step.
    fresh()
    for t, gap in ((0.0, 0.05), (1.0, 0.05), (1.05, 0.004)):
        task._rec.update(gap=gap, t=t)
        task._hold(shut, t)
    if task.score({"knife": shut}) != 0.0:
        bad.append("held, then a gripper back within 5 mm: scored")
    # A cut earlier in the trial is not erased by finishing well.
    fresh()
    task._rec.update(cut=True)
    for t in (0.0, 1.0):
        task._rec.update(gap=0.05, t=t)
        task._hold(shut, t)
    if task.score({"knife": shut}) != 0.0:
        bad.append("edge touched earlier, then shut and let go: scored")
    away()
    if bad:
        raise AssertionError("; ".join(bad))


_run()

if __name__ == "__main__":
    for label, states, want, _ in CASES:
        away()
        print(f"{task.score(states):.2f} (want {want:.2f})  {label}")
    print("rule checks passed")
