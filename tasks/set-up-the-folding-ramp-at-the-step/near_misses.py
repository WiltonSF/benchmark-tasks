"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: spec}, expected score, expected success). The ramp's spec is a ready state from its own
kinematics (RAMP.state(...)).
"""

import math

import numpy as np

from task import FLAT, L, RAMP, STEP_SIZE, STEP_TOP, STEP_X0, STEP_Y, TOP, X_H, Y_R

H = STEP_SIZE[2]


def _quat(m):
    """xyzw from a rotation matrix (Shepperd's method, safe at 180 deg turns)."""
    t = np.trace(m)
    if t > max(m[0, 0], m[1, 1], m[2, 2]):
        w = math.sqrt(1 + t) / 2
        q = [(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w]
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        r = math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / (2 * r)]
        q[i] = r / 2
        q[j] = (m[j, i] + m[i, j]) / (2 * r)
        q[k] = (m[k, i] + m[i, k]) / (2 * r)
    return np.asarray(q) / np.linalg.norm(q)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _slope(onto):
    """The incline at which a flat 2L ramp, low end on the floor and underside on the step's edge, reaches onto
    metres past the edge."""
    lo, hi = 0.01, 1.2
    for _ in range(80):
        a = (lo + hi) / 2
        if 2 * L * math.cos(a) - H / math.tan(a) > onto:
            hi = a
        else:
            lo = a
    return (lo + hi) / 2


def at_step(onto=0.030, joint=math.pi, reverse=False, yaw=0.0, dy=0.0, dz=0.0, vel=None):
    """The unfolded ramp set from the floor onto the step. reverse: the second panel's end on the step."""
    a = _slope(onto)
    low_x = STEP_X0 - H / math.tan(a)
    rot = _rz(yaw) @ _ry(-a)  # base panel rising toward +x
    u = np.array([math.cos(a), 0.0, math.sin(a)])
    if reverse:
        rot = _rz(yaw) @ _rz(math.pi) @ _ry(a)
    hinge = np.array([low_x, STEP_Y + dy, TOP + dz]) + L * u
    if yaw:
        low = np.array([low_x, STEP_Y + dy, TOP + dz])
        hinge = low + _rz(yaw) @ (hinge - low)
    return RAMP.state(joints={"hinge": joint}, pos=tuple(hinge), quat_xyzw=_quat(rot), lin_vel=vel)


def flat(x=X_H, y=Y_R, z=TOP, joint=math.pi, upside_down=False):
    rot = _ry(math.pi) if upside_down else np.eye(3)
    zz = z + (0.025 if upside_down else 0.0)  # upside down: the rails' tops on the table
    return RAMP.state(joints={"hinge": joint}, pos=(x, y, zz), quat_xyzw=_quat(rot))


def raised(h=0.020):
    """Unfolded, the second panel's end on the floor and the base panel's end held h up."""
    a = math.asin(h / (2 * L))
    low = np.array([X_H - L, Y_R, TOP])
    hinge = low + L * np.array([math.cos(a), 0.0, math.sin(a)])
    return RAMP.state(joints={"hinge": math.pi}, pos=tuple(hinge), quat_xyzw=_quat(_ry(-a)))


def folded_against_step():
    """Laid folded from the floor onto the step's edge (a steep, half-length ramp)."""
    a = math.asin(H / (L - 0.01))
    rot = _ry(-a)
    return RAMP.state(joints={"hinge": 0.0}, pos=(STEP_X0 - (L - 0.01) * math.cos(a) - 0.002, STEP_Y, TOP),
                      quat_xyzw=_quat(rot))


def kinked_onto_step():
    """Base panel flat on the floor, its hinge 20 mm short of the step; the second panel only partly unfolded,
    resting at an angle on the step's edge."""
    return RAMP.state(joints={"hinge": math.pi - math.radians(25)}, pos=(STEP_X0 - 0.03, STEP_Y, TOP),
                      quat_xyzw=_quat(_rz(math.pi)))


MOVING = (0.0, 0.0, -0.05)

CASES = [
    ("set at the step, 30 mm onto it", {"ramp": at_step()}, 1.0, True),
    ("set at the step the other way round", {"ramp": at_step(reverse=True)}, 1.0, True),
    ("set at the step, 16 mm onto it", {"ramp": at_step(onto=0.016)}, 1.0, True),
    ("set at the step, only 14 mm onto it", {"ramp": at_step(onto=0.014)}, 0.5, False),
    ("set at the step but still moving", {"ramp": at_step(vel=MOVING)}, 0.0, False),
    ("set at the step, 1.4 mm high (settling)", {"ramp": at_step(dz=0.0014)}, 1.0, True),
    ("held 3 mm above where it would rest", {"ramp": at_step(dz=0.003)}, 0.0, False),
    ("set at the step, turned 9 deg", {"ramp": at_step(yaw=math.radians(9))}, 1.0, True),
    ("set at the step, turned 11 deg", {"ramp": at_step(yaw=math.radians(11))}, 0.5, False),
    ("set at the step, 60 mm to one side (inside the step's width)", {"ramp": at_step(dy=0.06)}, 1.0, True),
    ("set at the step, 70 mm to one side (a rail past the step's end)", {"ramp": at_step(dy=0.07)}, 0.5, False),
    ("set at the step with the hinge 7.9 deg short of flat",
     {"ramp": at_step(joint=math.pi - FLAT + math.radians(0.1))}, 0.0, False),
    ("unfolded flat on the floor", {"ramp": flat()}, 0.5, False),
    ("base panel flat, hinge 7.9 deg short of flat: the other end 14 mm up, unsupported",
     {"ramp": flat(joint=math.pi - FLAT + math.radians(0.1))}, 0.0, False),
    ("on the floor, hinge 8.1 deg short of flat", {"ramp": flat(joint=math.pi - FLAT - math.radians(0.1))}, 0.0, False),
    ("half unfolded (panel standing up)", {"ramp": flat(joint=math.pi / 2)}, 0.0, False),
    ("unfolded but held 3 mm above the floor", {"ramp": flat(z=TOP + 0.003)}, 0.0, False),
    ("unfolded flat on top of the step", {"ramp": flat(x=STEP_X0 + 0.02, y=STEP_Y, z=STEP_TOP)}, 0.5, False),
    ("unfolded, one end held 20 mm up off the floor", {"ramp": raised()}, 0.0, False),
    ("unfolded, the far end held up on the step's edge but the near end 20 mm off the floor",
     {"ramp": at_step(dz=0.02)}, 0.0, False),
    ("unfolded, upside down on the floor", {"ramp": flat(upside_down=True)}, 0.0, False),
    ("laid folded from the floor onto the step's edge", {"ramp": folded_against_step()}, 0.0, False),
    ("habit: folded ramp pushed square against the step's front face",
     {"ramp": RAMP.state(pos=(STEP_X0 - L, STEP_Y, TOP))}, 0.0, False),
    ("habit: folded ramp left where it lies (start)", {"ramp": RAMP.state()}, 0.0, False),
    ("folded, moved onto the step", {"ramp": RAMP.state(pos=(STEP_X0 + 0.0, STEP_Y, STEP_TOP))}, 0.0, False),
    ("base flat, second panel kinked up onto the step", {"ramp": kinked_onto_step()}, 0.0, False),
    ("unfolded, knocked off the table", {"ramp": flat(x=0.3, y=0.9, z=TOP - 0.2)}, 0.0, False),
]


def _run() -> int:
    import task

    bad = 0
    start = {o.name: (o.state() if hasattr(o, "state") else None) for o in task.OBJECTS}
    from yam_arena.task_file import ObjectState

    def plain(o):
        p = np.asarray(o.pos, float)
        h = np.asarray(o.size, float) / 2
        return ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)

    base = {n: (s if s is not None else plain(next(o for o in task.OBJECTS if o.name == n))) for n, s in start.items()}
    for label, changes, want, want_success in CASES:
        st = dict(base) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
