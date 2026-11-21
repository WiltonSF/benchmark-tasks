"""On the station table: four loose pine 2x3 offcuts lie flat, two on each side, the way they collect beside a saw.
One of them has a long common nail standing up out of it. In the middle, two 2x4 bearers are fixed to the table as
dunnage for a stack. Nearer the arms stands a dark grey plastic tote printed DENAIL in black on yellow, on its front and on its floor.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import hands  # noqa: E402
import label  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s hold is counted in score(); this only ends the run right after it
EPS = 1e-6

PINE = (0.86, 0.73, 0.52)
DUNNAGE = (0.66, 0.53, 0.36)
STEEL = (0.30, 0.30, 0.32)
TOTE_GREY = (0.22, 0.24, 0.27)

# -- the offcuts: pine 2x3 (38 x 64 mm) lying flat, length along the board's own x ------------------------------
WIDTH, THICK = 0.064, 0.038
DENSITY = 500.0  # kg/m3, dry spruce-pine-fir
LENGTH = {"offcut_a": 0.220, "offcut_b": 0.235, "offcut_c": 0.250, "nailed_offcut": 0.240}
NAILED = "nailed_offcut"
CLEAN = ("offcut_a", "offcut_b", "offcut_c")
BOARDS = (*CLEAN, NAILED)

# The nail: a 6 in (60d) common nail, 150 mm long, 6.7 mm shank, 14 mm head, driven 35 mm into the board's top face
# 50 mm from one end, so 115 mm stands proud, head up (the point is buried in the wood).
NAIL_LEN, NAIL_DRIVEN, NAIL_D, HEAD_D, HEAD_T = 0.150, 0.035, 0.0067, 0.014, 0.002
NAIL_PROUD = NAIL_LEN - NAIL_DRIVEN
NAIL_FROM_END = 0.050
NAIL_X = LENGTH[NAILED] / 2 - NAIL_FROM_END  # in the board frame, toward the board's +x end
NAIL_MASS = 0.041  # kg, a 60d common nail (about 11 to the pound)

# The nail region: the nail and the last 75 mm of board at its end (from the end to 25 mm past the nail).
ZONE_X0 = NAIL_X - 0.025
ZONE_CLEAR = 0.005  # no gripper point may ever come this close to it

# -- start spots (centre x, y; yaw): two on the left, two on the right --------------------------------------------
SPOT = {"P1": (0.240, 0.225, 0.05), "P2": (0.390, 0.335, -0.04), "P3": (0.240, -0.225, -0.05), "P4": (0.390, -0.335, 0.04)}
START = {"offcut_a": "P1", NAILED: "P2", "offcut_b": "P3", "offcut_c": "P4"}  # lot A, the screenshot

# -- bearers: two 2x4s (38 x 89 mm) laid flat, 300 mm long, fixed; their tops are where the stack starts -----------
BEARER = (0.089, 0.300, 0.038)
STACK_X = 0.310
BEARER_X = (STACK_X - 0.065, STACK_X + 0.065)
BEARER_TOP = TOP + BEARER[2]
STACK_LO = (BEARER_X[0] - BEARER[0] / 2, -BEARER[1] / 2)  # the bearers' outline seen from above
STACK_HI = (BEARER_X[1] + BEARER[0] / 2, BEARER[1] / 2)

# -- the tote: inside 140 (x) x 320 (y) mm, 90 mm deep, 6 mm walls and floor, fixed --------------------------------
TOTE_X = 0.095
TOTE_IN = (0.140, 0.320)
TOTE_WALL = 0.006
TOTE_H = 0.096  # floor plus 90 mm of wall
TOTE_FLOOR = TOP + TOTE_WALL
TOTE_RIM = TOP + TOTE_H
TOTE_OUT = (TOTE_IN[0] + 2 * TOTE_WALL, TOTE_IN[1] + 2 * TOTE_WALL)
LABEL_SIZE = (0.220, 0.070)  # printed label on the front face, centred 48 mm up
FLOOR_LABEL = (0.240, 0.0764)  # the same print, larger, on the tote's floor


def _board(name):
    x, y, yaw = SPOT[START[name]]
    length = LENGTH[name]
    shapes = [parts.box((length, WIDTH, THICK), (0, 0, 0), PINE)]
    mass = DENSITY * length * WIDTH * THICK
    if name == NAILED:
        shapes += [parts.cylinder(NAIL_D / 2, NAIL_PROUD, (NAIL_X, 0, THICK / 2 + NAIL_PROUD / 2), STEEL),
                   parts.cylinder(HEAD_D / 2, HEAD_T, (NAIL_X, 0, THICK / 2 + NAIL_PROUD + HEAD_T / 2), STEEL)]
        mass += NAIL_MASS
    return parts.Part(name, pos=(float(x), float(y), float(TOP + THICK / 2 + 0.0005)), yaw=float(yaw),
                      mass=float(mass), shapes=shapes, friction=(0.5, 0.4))


def _tote():
    ix, iy = TOTE_IN
    ox, oy = TOTE_OUT
    wall_h = TOTE_H - TOTE_WALL
    zc = TOTE_WALL + wall_h / 2
    return parts.Part("denail_tote", pos=(TOTE_X, 0.0, TOP), friction=(0.5, 0.4), shapes=[
        parts.box((ox, oy, TOTE_WALL), (0, 0, TOTE_WALL / 2), TOTE_GREY),
        parts.box((TOTE_WALL, oy, wall_h), (-(ix + TOTE_WALL) / 2, 0, zc), TOTE_GREY),
        parts.box((TOTE_WALL, oy, wall_h), ((ix + TOTE_WALL) / 2, 0, zc), TOTE_GREY),
        parts.box((ix, TOTE_WALL, wall_h), (0, (iy + TOTE_WALL) / 2, zc), TOTE_GREY),
        parts.box((ix, TOTE_WALL, wall_h), (0, -(iy + TOTE_WALL) / 2, zc), TOTE_GREY)])


BEARERS = parts.Part("bearers", pos=(STACK_X, 0.0, TOP), friction=(0.5, 0.4), shapes=[
    parts.box(BEARER, (bx - STACK_X, 0, BEARER[2] / 2), DUNNAGE) for bx in BEARER_X])
TOTE = _tote()
BOARD_PARTS = {n: _board(n) for n in BOARDS}
OBJECTS = [BEARERS, TOTE, *BOARD_PARTS.values()]


def _at(spot, name):
    x, y, _ = SPOT[spot]
    return (x, y, TOP + THICK / 2 + 0.0005)


# Lots move which offcut carries the nail: lot A has it at the left far spot (nail end away from the arms), lot B at
# the right near spot, lot C at the left near spot turned end for end (nail end toward the tote).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={NAILED: _at("P2", NAILED)}),
    lots.Lot("B", place={NAILED: _at("P3", NAILED), "offcut_b": _at("P2", "offcut_b")},
             yaw={NAILED: SPOT["P3"][2], "offcut_b": SPOT["P2"][2]}),
    lots.Lot("C", place={NAILED: _at("P1", NAILED), "offcut_a": _at("P2", "offcut_a")},
             yaw={NAILED: SPOT["P1"][2] + math.pi, "offcut_a": SPOT["P2"][2]}),
)


# -- the printed label: patch the tote's spawn so the textured quad is part of the scene from its first frame -------
_author_plain = parts._author


def _author_with_label(part, root_path, translation, orientation):
    root = _author_plain(part, root_path, translation, orientation)
    if part.name == TOTE.name:
        from isaaclab.sim.utils import get_current_stage

        stage = get_current_stage()
        x, (w, h) = -TOTE_OUT[0] / 2 - 0.0008, (LABEL_SIZE[0] / 2, LABEL_SIZE[1] / 2)  # front face, read from the arms
        label.attach(stage, root_path, "front_label", [(x, w, 0.048 - h), (x, -w, 0.048 - h), (x, -w, 0.048 + h),
                                                      (x, w, 0.048 + h)], (-1, 0, 0))
        z, (w, h) = TOTE_WALL + 0.0005, (FLOOR_LABEL[0] / 2, FLOOR_LABEL[1] / 2)  # floor, read from the arms
        label.attach(stage, root_path, "floor_label", [(-h, w, z), (-h, -w, z), (h, -w, z), (h, w, z)], (0, 0, 1))
    return root


parts._author = _author_with_label


# -- geometry ------------------------------------------------------------------------------------------------------
def _box_distance(points, centre, rot, size):
    """Distance (m) from each point to an oriented box; 0 inside."""
    local = (np.asarray(points) - centre) @ rot
    return np.linalg.norm(np.maximum(np.abs(local) - np.asarray(size) / 2, 0.0), axis=1)


def _frame(state):
    p, q = np.asarray(state.pos, float), np.asarray(state.quat_xyzw, float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError("unknown pose")
    return p, parts._quat_to_matrix(q)


def _boxes(name, state):
    """(centre, rotation, size) of the board and, for the nailed one, its nail shank and head."""
    p, r = _frame(state)
    return [(c, rr, s) for _, c, rr, s in BOARD_PARTS[name].world_boxes({"base": (p, r)})]


def _corners(name, state):
    p, r = _frame(state)
    half = np.array([LENGTH[name], WIDTH, THICK]) / 2
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    return p + (signs * half) @ r.T


def _surface(name, state, step=0.008):
    """Points on the board's six faces (about 8 mm apart) and along the nail, world frame."""
    p, r = _frame(state)
    half = np.array([LENGTH[name], WIDTH, THICK]) / 2
    pts = []
    for axis in range(3):
        a, b = [k for k in range(3) if k != axis]
        ua = np.linspace(-half[a], half[a], max(2, int(2 * half[a] / step) + 1))
        ub = np.linspace(-half[b], half[b], max(2, int(2 * half[b] / step) + 1))
        ga, gb = np.meshgrid(ua, ub, indexing="ij")
        for sign in (-1, 1):
            q = np.zeros((ga.size, 3))
            q[:, a], q[:, b], q[:, axis] = ga.ravel(), gb.ravel(), sign * half[axis]
            pts.append(q)
    if name == NAILED:
        z = np.linspace(THICK / 2, THICK / 2 + NAIL_PROUD + HEAD_T, 18)
        pts.append(np.c_[np.full_like(z, NAIL_X), np.zeros_like(z), z])
    return p + np.concatenate(pts) @ r.T


def _flat(state):
    _, r = _frame(state)
    return abs(r[2, 2]) >= math.cos(math.radians(10.0)) - EPS


def _over_stack(xy):
    return bool(STACK_LO[0] - EPS <= xy[0] <= STACK_HI[0] + EPS and STACK_LO[1] - EPS <= xy[1] <= STACK_HI[1] + EPS)


def _hand_gap(clouds, name, state):
    """Smallest distance (m) from either gripper to the board (and its nail)."""
    pts = np.concatenate(clouds)
    return min(float(_box_distance(pts, c, r, s).min()) for c, r, s in _boxes(name, state))


def _in_nail_region(clouds, state):
    """True if any gripper point is within 5 mm of the nail or of the board's last 75 mm at the nailed end."""
    p, r = _frame(state)
    pts = np.concatenate(clouds)
    zone = [(p + r @ np.array([(ZONE_X0 + LENGTH[NAILED] / 2) / 2, 0, 0]), r,
             np.array([LENGTH[NAILED] / 2 - ZONE_X0, WIDTH, THICK]))]
    zone += [(c, rr, s) for c, rr, s in _boxes(NAILED, state)[1:]]  # shank and head
    return any(float(_box_distance(pts, c, rr, s).min()) <= ZONE_CLEAR + EPS for c, rr, s in zone)


SUPPORT_BELOW, SUPPORT_ABOVE = 0.003, 0.010  # a lying offcut's lowest corner: 3 mm into to 10 mm above its support


def _lying_on_stack(objects):
    """The clean offcuts lying on the stack, whether or not let go: flat, centre over the bearers' outline, and the
    lowest corner 3 mm below to 10 mm above a bearer's top, or above the top of an offcut already on the stack whose
    outline (seen from above) contains one of this offcut's bottom corners or its centre. Any number of layers."""
    info = {}
    for n in CLEAN:
        st = objects[n]
        c = _corners(n, st)
        info[n] = (_flat(st) and _over_stack(np.asarray(st.pos, float)[:2]), float(c[:, 2].min()), float(c[:, 2].max()),
                   np.vstack([c[np.argsort(c[:, 2])[:4]], np.asarray(st.pos, float)[None]]))
    on = set()
    changed = True
    while changed:
        changed = False
        for n in CLEAN:
            ok, low, _, feet = info[n]
            if n in on or not ok:
                continue
            if BEARER_TOP - SUPPORT_BELOW - EPS <= low <= BEARER_TOP + SUPPORT_ABOVE + EPS:
                on.add(n)
                changed = True
                continue
            for o in sorted(on):
                top = info[o][2]
                if not top - SUPPORT_BELOW - EPS <= low <= top + SUPPORT_ABOVE + EPS:
                    continue
                p, r = _frame(objects[o])
                local = (feet - p) @ r
                if ((np.abs(local[:, 0]) <= LENGTH[o] / 2 + EPS) & (np.abs(local[:, 1]) <= WIDTH / 2 + EPS)).any():
                    on.add(n)
                    changed = True
                    break
    return [n for n in CLEAN if n in on]


def _in_tote(name, state):
    """Every corner of the board inside the tote's inner walls, on or above its floor and no higher than its rim."""
    c = _corners(name, state)
    x0, x1 = TOTE_X - TOTE_IN[0] / 2, TOTE_X + TOTE_IN[0] / 2
    y0, y1 = -TOTE_IN[1] / 2, TOTE_IN[1] / 2
    return bool((c[:, 0] >= x0 - 0.001).all() and (c[:, 0] <= x1 + 0.001).all() and (c[:, 1] >= y0 - 0.001).all()
                and (c[:, 1] <= y1 + 0.001).all() and (c[:, 2] >= TOTE_FLOOR - 0.002).all()
                and (c[:, 2] <= TOTE_RIM + EPS).all())


def _clean_in_tote(state):
    p = np.asarray(state.pos, float)
    return bool(TOTE_X - TOTE_IN[0] / 2 <= p[0] <= TOTE_X + TOTE_IN[0] / 2 and abs(p[1]) <= TOTE_IN[1] / 2
                and p[2] <= TOTE_RIM)


def _touches_stack(objects):
    """True if the nailed board (or its nail) is within 5 mm of a bearer, or of a clean offcut lying on the stack."""
    pts = _surface(NAILED, objects[NAILED])
    for _, c, r, s in BEARERS.world_boxes({"base": (np.asarray(BEARERS.pos, float), BEARERS.rotation)}):
        if float(_box_distance(pts, c, r, s).min()) <= 0.005 + EPS:
            return True
    for name in _lying_on_stack(objects):
        p, r = _frame(objects[name])
        if float(_box_distance(pts, p, r, np.array([LENGTH[name], WIDTH, THICK])).min()) <= 0.005 + EPS:
            return True
    return False


STILL_SPEED, STILL_SPIN = 0.01, 0.2  # m/s, rad/s


def _still(state):
    return state.resting(STILL_SPEED, STILL_SPIN)


def evaluate(objects, clouds):
    """Everything the score needs from one moment: which clean boards are stacked and let go, whether the nailed one
    is let go in the tote, and whether the whole scene is let go and still."""
    gaps = {n: _hand_gap(clouds, n, objects[n]) for n in BOARDS}
    released = {n: gaps[n] > 0.005 + EPS for n in BOARDS}
    still = {n: _still(objects[n]) for n in BOARDS}
    stacked = [n for n in _lying_on_stack(objects) if released[n] and still[n]]
    binned = _in_tote(NAILED, objects[NAILED]) and released[NAILED] and still[NAILED]
    clean_binned = [n for n in CLEAN if _clean_in_tote(objects[n])]
    full = (len(stacked) == len(CLEAN) and binned and not clean_binned and all(released.values())
            and all(still.values()))
    return dict(stacked=stacked, binned=bool(binned), clean_binned=clean_binned, full=bool(full),
                gaps={n: round(v, 4) for n, v in gaps.items()})


# -- the trial's record: kept from the scene reset to the end of the trial ----------------------------------------
HOLD_S = 1.0
PER_STACKED = 0.10  # each clean offcut stacked, let go and still
BINNED = 0.40  # the nailed offcut let go and still in the tote
_known = False  # a hand state has been read this trial
_invalid = False  # some step's hand state or pose could not be read: no credit for the trial
_nail_hit = False  # a gripper came within 5 mm of the nail region
_on_stack = False  # the nailed board touched the bearers or a board on them
_clouds = None
_time = 0.0
_held = 0.0
_prev_full = False
_stamp = ()


def _stamp_of(objects):
    return tuple(float(v) for n in BOARDS for v in np.r_[objects[n].pos, objects[n].quat_xyzw, objects[n].lin_vel])


def reset_record():
    global _known, _invalid, _nail_hit, _on_stack, _clouds, _time, _held, _prev_full, _stamp
    _known = _invalid = _nail_hit = _on_stack = _prev_full = False
    _clouds, _time, _held, _stamp = None, 0.0, 0.0, ()


def update(objects, clouds, t):
    """One control step of the whole-trial record. ``clouds`` is None when the hand state could not be read."""
    global _known, _invalid, _nail_hit, _on_stack, _clouds, _time, _held, _prev_full, _stamp
    dt = float(t) - _time
    _time = float(t)
    try:
        if clouds is None or len(clouds) != 2 or any(c.ndim != 2 or c.shape[1] != 3 or not len(c) or
                                                     not np.isfinite(c).all() for c in clouds):
            raise ValueError("unknown hand state")
        _clouds = [np.asarray(c, float) for c in clouds]
        _known = True
        _nail_hit |= _in_nail_region(_clouds, objects[NAILED])
        _on_stack |= _touches_stack(objects)
        full = evaluate(objects, _clouds)["full"]
    except (ValueError, KeyError, AttributeError, IndexError, TypeError):
        _invalid, full = True, False
    _held = _held + max(dt, 0.0) if full and _prev_full else 0.0
    _prev_full = full
    _stamp = _stamp_of(objects) if all(n in objects for n in BOARDS) else ()


@parts.on_reset
def _on_reset(sim):
    global _time
    reset_record()
    _time = float(sim.t)


@parts.on_step
def _on_step(sim):
    states = sim.states()
    try:
        clouds = hands.clouds(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        clouds = None
    update(states, clouds, sim.t)
    if os.environ.get("TRACE_OUT") and round(float(sim.t) * 20) % 5 == 0:
        row = dict(t=float(sim.t), lot=lots.current().name, known=_known, invalid=_invalid, nail_hit=_nail_hit,
                   on_stack=_on_stack, held=_held, score=score(states))
        if clouds is not None:
            row.update(evaluate(states, clouds))
        row["boards"] = {n: dict(pos=np.round(states[n].pos, 4).tolist(), quat=np.round(states[n].quat_xyzw, 4).tolist(),
                                 v=round(float(np.linalg.norm(states[n].lin_vel)), 4)) for n in BOARDS}
        try:
            from yam_arena.constants import TCP_OFFSET

            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in hands.ARMS:
                robot = base.scene[arm]
                pose = parts._np(robot.data.body_link_pose_w)[0, robot.body_names.index("gripper")]
                tcp = pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)
                q = parts._np(robot.data.joint_pos)[0]
                row[arm] = dict(tcp=np.round(tcp, 4).tolist(), fingers=[round(float(q[robot.joint_names.index(j)]), 4)
                                                                        for j in ("joint7", "joint8")])
        except Exception as exc:  # trace only
            row["tcp_error"] = repr(exc)
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("offcuts.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


def score(objects):
    """0 if the hand state was ever unknown, a gripper came within 5 mm of the nail region, or the nailed board ever
    touched the stack. Otherwise 0.10 per clean offcut lying flat on the stack, let go and still,
    plus 0.40 for the nailed offcut let go and still inside the tote; 1.0 once all three clean offcuts are stacked, the
    nailed one is in the tote, no clean one is in the tote, and every offcut is let go and still for 1 s."""
    try:
        if not _known or _invalid or _nail_hit or _on_stack or _clouds is None or _stamp != _stamp_of(objects):
            return 0.0
        e = evaluate(objects, _clouds)
        if e["full"] and _held >= HOLD_S - 1e-9:
            return 1.0
        return round(PER_STACKED * len(e["stacked"]) + (BINNED if e["binned"] else 0.0), 6)
    except (ValueError, KeyError, AttributeError, IndexError, TypeError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


# -- scripted demonstration: open loop from where each offcut starts (visible to the top camera) ------------------
CLOSE = -0.029  # 3 mm past each face of a 64 mm board
EASE = -0.034  # opened 5 mm each side to let go between neighbours or inside the tote
CARRY = TOP + 0.130
CARRY_NAILED = TOP + 0.155  # the board's underside 40 mm above the tote's rim
GRASP = TOP + THICK / 2 + 0.003
ON_STACK = BEARER_TOP + THICK / 2 + 0.007  # let go 7 mm above the bearers
IN_TOTE = TOTE_FLOOR + THICK / 2 + 0.008  # let go 8 mm above the tote's floor
SLOT_Y = {"left": (0.100, 0.0), "right": (-0.100, 0.0)}  # lateral slot first, the middle slot last
PARK = {"left": (0.200, 0.250), "right": (0.200, -0.250)}


def _wrap(a):
    """A board's yaw as a grip yaw in (-pi/2, pi/2]: a parallel grip is the same turned half a turn."""
    while a > math.pi / 2:
        a -= math.pi
    while a <= -math.pi / 2:
        a += math.pi
    return a


def _pick(xy, yaw, carry):
    x, y = xy
    return [Waypoint(2.0, (x, y, TOP + 0.10), FINGER_OPEN, "over offcut", yaw),
            Waypoint(2.0, (x, y, GRASP), FINGER_OPEN, "down to offcut", yaw),
            Waypoint(1.2, (x, y, GRASP), FINGER_OPEN, "settle", yaw),
            Waypoint(1.0, (x, y, GRASP), CLOSE, "grip offcut", yaw),
            Waypoint(1.5, (x, y, carry), CLOSE, "lift", yaw)]


def _put(xy, yaw, z, carry, what, via=None):
    x, y = xy
    out = [] if via is None else [Waypoint(1.5, (*via, carry), CLOSE, "out over the bench, clear of the tote", yaw)]
    return out + [Waypoint(2.5, (x, y, carry), CLOSE, f"carry to {what}", yaw),
            Waypoint(1.5, (x, y, z), CLOSE, f"lower onto {what}", yaw),
            Waypoint(1.0, (x, y, z), EASE, "let go", yaw),
            Waypoint(1.5, (x, y, carry), EASE, "rise", yaw)]


def demo():
    lot = lots.current()
    start = {n: (lot.pos(n), lot.yaw.get(n, BOARD_PARTS[n].yaw)) for n in BOARDS}
    side = {n: "left" if start[n][0][1] > 0 else "right" for n in BOARDS}
    x_arm = side[NAILED]  # the arm on the nailed offcut's side
    y_arm = "right" if x_arm == "left" else "left"
    plan = {}

    # The arm with the nailed offcut: first the nailed one to the tote, gripped 15 mm from the middle on the end away
    # from the nail and turned a quarter turn to lie along the tote; then its clean one to the stack. (Carrying a clean
    # offcut first could sweep it into the standing nail.)
    (nx, ny, _), nyaw = start[NAILED]
    g = _wrap(nyaw)
    axis = np.array([math.cos(nyaw), math.sin(nyaw)])  # toward the nailed end
    grip = np.array([nx, ny]) - 0.015 * axis
    turn = -math.pi / 2 if x_arm == "left" else math.pi / 2
    board_after = nyaw + turn
    drop = np.array([TOTE_X, 0.0]) - 0.015 * np.array([math.cos(board_after), math.sin(board_after)])
    seq = [Waypoint(2.0, (*grip, TOP + 0.17), FINGER_OPEN, "high over the grip, above the nail's head", g)]
    seq += _pick(grip, g, CARRY_NAILED)
    seq += [Waypoint(3.0, (*drop, CARRY_NAILED), CLOSE, "carry over tote", g + turn),
            Waypoint(1.5, (*drop, IN_TOTE), CLOSE, "lower into tote", g + turn),
            Waypoint(1.0, (*drop, IN_TOTE), EASE, "let go", g + turn),
            Waypoint(1.5, (*drop, CARRY_NAILED), EASE, "rise", g + turn),
            Waypoint(1.5, (drop[0] + 0.11, drop[1], CARRY_NAILED), EASE, "back off past the nail", g + turn)]
    x_binned = sum(w.duration for w in seq)  # the other arm stays parked on its own side until then
    clean_x = next(n for n in CLEAN if side[n] == x_arm)
    (cx, cy, _), cyaw = start[clean_x]
    seq += _pick((cx, cy), _wrap(cyaw), CARRY) + _put((STACK_X, SLOT_Y[x_arm][0]), _wrap(cyaw), ON_STACK, CARRY,
                                                      "bearers", via=(STACK_X, cy))
    seq += [Waypoint(2.0, (*PARK[x_arm], TOP + 0.15), FINGER_OPEN, "park", 0.0)]
    x_time = sum(w.duration for w in seq)
    plan[x_arm] = seq + [Waypoint(16.0, (*PARK[x_arm], TOP + 0.15), FINGER_OPEN, "wait", 0.0)]

    # The other arm: both its clean offcuts, the second to the middle slot once the tote run is over.
    mine = [n for n in CLEAN if side[n] == y_arm]
    mine.sort(key=lambda n: start[n][0][0])
    seq = [Waypoint(2.0, (0.25, PARK[y_arm][1], TOP + 0.15), FINGER_OPEN, "aside", 0.0),
           Waypoint(x_binned - 1.5, (0.25, PARK[y_arm][1], TOP + 0.15), FINGER_OPEN, "wait for the tote run", 0.0)]
    for k, name in enumerate(mine):
        (bx, by, _), byaw = start[name]
        seq += _pick((bx, by), _wrap(byaw), CARRY)
        if k == 0:
            seq += _put((STACK_X, SLOT_Y[y_arm][0]), _wrap(byaw), ON_STACK, CARRY, "bearers", via=(STACK_X, by))
        else:
            hold = (0.30, 0.20 if y_arm == "left" else -0.20)
            seq += [Waypoint(2.0, (*hold, CARRY), CLOSE, "hold clear of the tote run", _wrap(byaw))]
            wait = max(0.5, x_time + 0.5 - sum(w.duration for w in seq))
            seq += [Waypoint(wait, (*hold, CARRY), CLOSE, "wait", _wrap(byaw))]
            seq += _put((STACK_X, 0.0), _wrap(byaw), ON_STACK, CARRY, "bearers")
    seq += [Waypoint(2.0, (*PARK[y_arm], TOP + 0.15), FINGER_OPEN, "park", 0.0),
            Waypoint(4.0, (*PARK[y_arm], TOP + 0.15), FINGER_OPEN, "still", 0.0)]
    plan[y_arm] = seq
    return plan
