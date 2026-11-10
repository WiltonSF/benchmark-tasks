"""On the station table, to the robot's right of the midline: a model ambulance stretcher standing on two pairs of
legs, its head end pointing away from the robot, and beyond it a raised grey loading bay with a yellow sill edge,
low side rails and a back stop. The stretcher has an orange deck, a black push handle at its foot end, small grey
skids under both ends of the deck and a red tab on the deck above each leg pair. How far away the bay stands changes
between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (parts before lots: its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

ORANGE, RED, BLACK = (0.92, 0.50, 0.10), (0.85, 0.10, 0.08), (0.08, 0.08, 0.09)
STEEL, SKID, BAY_GREY, RAIL, YELLOW = (0.72, 0.74, 0.77), (0.45, 0.46, 0.48), (0.60, 0.62, 0.64), (0.30, 0.31, 0.33), \
    (0.95, 0.80, 0.10)

# Stretcher, part frame: origin on the table under the middle of the deck, x toward the head end, z up.
L, W, DECK_T = 0.180, 0.056, 0.010  # deck length, width, thickness
D = 0.068  # deck underside above the table when standing on its legs
LEG = 0.008  # leg cross-section
LEG_LEN = D  # pivot (at the deck's underside) to foot
FRONT_PIVOT = L / 2 - 0.040  # 40 mm behind the head end
REAR_PIVOT = -L / 2 + 0.075  # 75 mm in front of the foot end
FRONT_LEG_Y, REAR_LEG_Y = 0.012, 0.024  # each pair's legs at +-y (the front pair inside the rear pair)
SKID_H, SKID_LEN, SKID_W, SKID_Y = 0.014, 0.015, 0.010, 0.010
HANDLE = (0.014, 0.050, 0.018)  # a crossbar across the foot end
HANDLE_GAP = 0.025  # between the deck's foot end and the crossbar, bridged by two side arms
HANDLE_X = -L / 2 - HANDLE_GAP - HANDLE[0] / 2
ARM_Y = HANDLE[1] / 2 - 0.004
TAB = (0.025, 0.016, 0.016)
TAB_Y = 0.020  # on the robot's-left half of the deck
TAB_TRAVEL, TAB_SPRING = 0.008, 150.0
FOLD_UP = math.pi / 2  # folded flat under the deck (the stop)
FOLD_TARGET = 2.2  # the fold spring's rest point, past the stop, so a folded pair is pressed onto it
FOLD_SPRING = 0.15  # N m/rad
FOLD_DAMPING = 0.45  # N m s/rad: a freed pair takes about 3 s to swing up, like a damped gas strut
BODY_MASS, LEG_MASS, TAB_MASS = 0.35, 0.03, 0.02
FRICTION = (0.4, 0.4)  # low, standing in for the castors under the feet and the skids' rollers


def _legs(name, x, y):
    shapes = [parts.box((LEG, LEG, LEG_LEN), at=(x, s * y, LEG_LEN / 2), color=STEEL) for s in (-1, 1)]
    shapes.append(parts.box((LEG, 2 * y, LEG), at=(x, 0.0, LEG / 2 + 0.004), color=STEEL))  # cross bar near the feet
    return parts.Hinge(name, shapes=shapes, pivot=(x, 0.0, D), axis=(0.0, 1.0, 0.0), lower=0.0, upper=FOLD_UP,
                       mass=LEG_MASS, spring=FOLD_SPRING, damping=FOLD_DAMPING)


def _tab(name, x):
    return parts.Slider(name, shapes=[parts.box(TAB, at=(x, TAB_Y, D + DECK_T + TAB[2] / 2), color=RED)],
                        axis=(0.0, 0.0, -1.0), travel=TAB_TRAVEL, mass=TAB_MASS, spring=TAB_SPRING, damping=1.0)


def _stretcher_shapes():
    out = [parts.box((L, W, DECK_T), at=(0.0, 0.0, D + DECK_T / 2), color=ORANGE),
           parts.box(HANDLE, at=(HANDLE_X, 0.0, D + DECK_T / 2), color=BLACK)]
    out += [parts.box((HANDLE_GAP + 0.002, 0.008, DECK_T), at=(-L / 2 - HANDLE_GAP / 2, s * ARM_Y, D + DECK_T / 2),
                      color=BLACK) for s in (-1, 1)]
    for x in (L / 2 - SKID_LEN / 2, -L / 2 + SKID_LEN / 2):
        for s in (-1, 1):
            out.append(parts.box((SKID_LEN, SKID_W, SKID_H), at=(x, s * SKID_Y, D - SKID_H / 2), color=SKID))
    return out


START = (0.235, -0.015)  # the deck's centre
STRETCHER = parts.Part(
    "stretcher", pos=(*START, TOP + 0.0005), mass=BODY_MASS, shapes=_stretcher_shapes(), friction=FRICTION,
    joints=[_legs("front_legs", FRONT_PIVOT, FRONT_LEG_Y), _legs("rear_legs", REAR_PIVOT, REAR_LEG_Y),
            _tab("front_tab", FRONT_PIVOT), _tab("rear_tab", REAR_PIVOT)])

# Bay, part frame: origin on the table at the middle of the sill (its near top edge), x into the bay.
BAY_H, BAY_LEN, BAY_W = 0.050, 0.255, 0.150
RAIL_H, RAIL_W = 0.015, 0.010
STOP = 0.015  # back stop, 15 mm thick, standing 20 mm above the floor
INNER_LEN = BAY_LEN - STOP  # 240 mm of floor from the sill to the back stop
INNER_HALF_W = BAY_W / 2 - RAIL_W  # 65 mm
BAY_MASS = 3.0  # loose but heavy only so that the lots can move it; nothing in the task moves it
BAY = parts.Part("bay", pos=(0.340, START[1], TOP), mass=BAY_MASS, shapes=[
    parts.box((BAY_LEN, BAY_W, BAY_H), at=(BAY_LEN / 2, 0.0, BAY_H / 2), color=BAY_GREY),
    parts.box((0.012, BAY_W - 2 * RAIL_W, 0.002), at=(0.006, 0.0, BAY_H + 0.001), color=YELLOW),  # sill strip
    parts.box((BAY_LEN, RAIL_W, RAIL_H), at=(BAY_LEN / 2, INNER_HALF_W + RAIL_W / 2, BAY_H + RAIL_H / 2), color=RAIL),
    parts.box((BAY_LEN, RAIL_W, RAIL_H), at=(BAY_LEN / 2, -INNER_HALF_W - RAIL_W / 2, BAY_H + RAIL_H / 2), color=RAIL),
    parts.box((STOP, BAY_W - 2 * RAIL_W, 0.020), at=(BAY_LEN - STOP / 2, 0.0, BAY_H + 0.010), color=RAIL),
])

OBJECTS = [STRETCHER, BAY]

SILL = {"A": 0.340, "B": 0.370}  # the sill's distance in front of the arm bases, per lot
LOTS = lots.declare(OBJECTS, *[lots.Lot(n, place={"bay": (x, START[1], TOP)}) for n, x in SILL.items()])

EPS = 1e-6
FOLDED = math.radians(75.0)  # a pair counts as folded within 15 degrees of flat under the deck
CATCH = FOLDED  # once a pair reaches this, the catch holds it up for the rest of the trial
PRESSED = 0.004  # a tab pressed 4 mm or more frees its leg pair
PAST_SILL = 0.020
ON_FLOOR = 0.003
LEVEL, PART_LEVEL, TIPPED = math.radians(5.0), math.radians(10.0), math.radians(12.0)
BAY_MOVE, BAY_TILT = 0.010, math.radians(5.0)
CLEAR = 0.005
STILL_Q = 0.005  # per control step, rad or m
SUCCESS_HOLD_S = 1.0
LOCK = 0.005  # a locked pair may play 0.005 rad


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _in_bay(bay, pts) -> np.ndarray:
    """World points in the bay's frame (x past the sill, y across, z above the table under the bay)."""
    return (np.asarray(pts) - np.asarray(bay.pos)) @ _rot(bay.quat_xyzw)


def _points(s, local) -> np.ndarray:
    return np.asarray(s.pos) + np.asarray(local) @ _rot(s.quat_xyzw).T


def _skid_bottoms(s, end):
    x = (L / 2 if end == "front" else -L / 2)
    xs = (x, x - SKID_LEN) if end == "front" else (x, x + SKID_LEN)
    return _points(s, [(xx, sy * (SKID_Y + sw * SKID_W / 2), D - SKID_H) for xx in xs for sy in (-1, 1)
                       for sw in (-1, 1)])


def _deck_end(s, end):
    x = L / 2 if end == "front" else -L / 2
    return _points(s, [(x, sy * W / 2, z) for sy in (-1, 1) for z in (D, D + DECK_T)])


def _end_past(s, bay, end) -> float:
    """How far the deck's end edge (its nearest corner) is past the sill line."""
    return float(_in_bay(bay, _deck_end(s, end))[:, 0].min())


def _end_on_floor(s, bay, end) -> bool:
    """That end's skids rest on the bay floor: their bottoms within 3 mm above it, inside the rails."""
    p = _in_bay(bay, _skid_bottoms(s, end))
    return bool(np.all(p[:, 2] >= BAY_H - ON_FLOOR - EPS) and np.all(p[:, 2] <= BAY_H + ON_FLOOR + EPS)
                and np.all(p[:, 0] >= -EPS) and np.all(p[:, 0] <= INNER_LEN + EPS)
                and np.all(np.abs(p[:, 1]) <= INNER_HALF_W + EPS))


def folded(s, pair) -> bool:
    return bool(s.joints[pair] >= FOLDED - EPS)


# Whole-trial record: did the deck ever tilt over 12 degrees (an end dropped) or the stretcher leave the table, or was the bay shoved over 10 mm or
# tilted over 5 degrees? Cleared only by a scene reset: the stretcher back within 0.5 mm of its start and at rest,
# after a jump of over 20 mm in one control step (a robot cannot move it back that way).
_record = {"fault": False, "prev": None}
# Filled every control step in the simulator by the hook below: grippers clear of the stretcher, its joints still.
# Offline (near_misses.py) there is no robot, so both stay True.
_sim = {"clear": True, "still": True, "q": None, "limits": None, "held": {}}


def _update(objects) -> None:
    s, bay = objects["stretcher"], objects["bay"]
    pos = np.asarray(s.pos, float)
    prev = _record["prev"]
    start = np.array([*START, TOP + 0.0005])
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and s.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    bay_start = np.asarray(lots.current().pos("bay"), float)
    if (_tilt(s) > TIPPED + EPS or s.low[2] < TOP - 0.010 - EPS
            or np.linalg.norm(np.asarray(bay.pos) - bay_start) > BAY_MOVE + EPS or _tilt(bay) > BAY_TILT + EPS):
        _record["fault"] = True


# --- the leg catches (simulator only) ------------------------------------------------------------------------------
def _leg_limits(q) -> dict[str, tuple[float, float]]:
    """Each pair is locked down until its tab is pressed 4 mm or more. That trips the lock: the pair then swings
    freely until it reaches 75 degrees, where its catch holds it up for the rest of the trial."""
    out = {}
    for pair, tab in (("front_legs", "front_tab"), ("rear_legs", "rear_tab")):
        state = _sim["held"].get(pair, "locked")
        if state != "caught" and q[pair] >= CATCH - EPS:
            state = "caught"
        elif state == "locked" and q[tab] >= PRESSED - EPS:
            state = "free"
        _sim["held"][pair] = state
        out[pair] = {"locked": (0.0, LOCK), "free": (0.0, FOLD_UP), "caught": (CATCH, FOLD_UP)}[state]
    return out


def _write_limits(sim, limits) -> None:
    import torch

    key = tuple(limits[p] for p in ("front_legs", "rear_legs"))
    if key == _sim["limits"]:
        return
    _sim["limits"] = key
    base = sim.env.unwrapped
    asset = base.scene["stretcher"]
    ids = asset.find_joints(["front_legs", "rear_legs"], preserve_order=True)[0]
    lim = torch.tensor([[list(limits["front_legs"]), list(limits["rear_legs"])]], dtype=torch.float32,
                       device=base.device)
    asset.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_reset
def _lock_legs(sim) -> None:
    """Put the stretcher back on its legs where it starts (limits from the last trial could hold a folded pair), then
    lock both pairs and set the fold springs."""
    import torch

    _sim.update(clear=True, still=True, q=None, limits=None, held={})
    _write_limits(sim, {"front_legs": (0.0, FOLD_UP), "rear_legs": (0.0, FOLD_UP)})
    base = sim.env.unwrapped
    dev = base.device
    asset = base.scene["stretcher"]
    names = ["front_legs", "rear_legs", "front_tab", "rear_tab"]
    ids = asset.find_joints(names, preserve_order=True)[0]
    asset.write_joint_position_to_sim_index(position=torch.zeros((1, len(ids)), device=dev), joint_ids=ids)
    asset.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, len(ids)), device=dev), joint_ids=ids)
    pose = torch.tensor([[*STRETCHER.pos, 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=dev)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))
    _write_limits(sim, {"front_legs": (0.0, LOCK), "rear_legs": (0.0, LOCK)})
    for pair in ("front_legs", "rear_legs"):
        sim.set_target("stretcher", pair, FOLD_TARGET)


def _hulls(base):
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    return hulls


def _boxes(s):
    poses = {"base": (np.asarray(s.pos), _rot(s.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in s.links.items()})
    return [(c, r, sz) for _, c, r, sz in STRETCHER.world_boxes(poses)]


@parts.on_step
def _catches(sim) -> None:
    from yam_arena import analysis

    s = sim.states()["stretcher"]
    _write_limits(sim, _leg_limits(s.joints))
    q = np.array([s.joints[k] for k in ("front_legs", "rear_legs", "front_tab", "rear_tab")])
    _sim["still"] = bool(_sim["q"] is None or np.abs(q - _sim["q"]).max() <= STILL_Q + EPS)
    _sim["q"] = q
    hulls = _hulls(sim.env.unwrapped)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, sz)[0] > CLEAR for c, r, sz in _boxes(s))


# --- scoring --------------------------------------------------------------------------------------------------------
def _settled(s) -> bool:
    return bool(s.resting() and _sim["still"] and _sim["clear"])


def _loaded(s, bay) -> bool:
    return bool(folded(s, "front_legs") and folded(s, "rear_legs") and _end_past(s, bay, "rear") >= PAST_SILL - EPS
                and _end_on_floor(s, bay, "front") and _end_on_floor(s, bay, "rear") and _tilt(s) <= LEVEL + EPS)


def _head_in(s, bay) -> bool:
    return bool(folded(s, "front_legs") and _end_past(s, bay, "front") >= PAST_SILL - EPS
                and _end_on_floor(s, bay, "front") and _tilt(s) <= PART_LEVEL + EPS)


def success(objects):
    """Both leg pairs folded, the stretcher on the bay floor past the sill, level, let go and still."""
    _update(objects)
    s, bay = objects["stretcher"], objects["bay"]
    return bool(not _record["fault"] and _settled(s) and _loaded(s, bay))


def distance(objects):
    return max(0.0, PAST_SILL - _end_past(objects["stretcher"], objects["bay"], "rear"))


def score(objects):
    """1.0 on success. 0.5: the head end's legs folded and its skids on the bay floor past the sill, let go and
    still, the rest not yet in."""
    _update(objects)
    s, bay = objects["stretcher"], objects["bay"]
    if _record["fault"] or not _settled(s):
        return 0.0
    if _loaded(s, bay):
        return 1.0
    return 0.5 if _head_in(s, bay) else 0.0


# --- demo -----------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
TRAVEL = 0.14
GRIP_Z = D + DECK_T / 2 - HANDLE[2] / 2 + 0.003 + TIP  # tips 3 mm above the handle's underside
WIDE = -0.020
GRIP_YAW = math.pi / 2  # fingers close along x, across the crossbar: the pads then hold its pitch
LIFT = 0.012  # the foot end raised this much while the rear legs fold and cross the sill
PRESS = 0.030  # the pressing tips commanded this far below a tab's top (8 mm of travel, then the soft arm gives)
HOLD = 6.0  # s the right arm holds still while a leg pair is freed and swings up
LAG = -0.009  # the handle's lag behind the pushing arm's target (set from a trace)


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def demo():
    """Right arm: grip the push handle and push the head end in over the bay floor; left arm: press the front tab.
    Right arm: push on until the rear legs near the sill and hold the foot end up; left arm: press the rear tab.
    Right arm: lift the foot end a little, push the stretcher home, lower it and let go. The sill's place is what the cameras show for the trial's
    lot."""
    sill = lots.current().pos("bay")[0]
    x0, y0 = START
    hx = x0 + HANDLE_X
    d1 = sill - (x0 + FRONT_PIVOT + LEG / 2) - 0.005  # front legs 5 mm short of the sill
    d2 = sill - (x0 + REAR_PIVOT + LEG / 2) - 0.008  # rear legs 8 mm short of the sill
    d3 = sill + 0.045 - (x0 - L / 2)  # foot end commanded 45 mm past the sill (it lags about 10 mm)
    tab_z = D + DECK_T + TAB[2]
    ty = y0 + TAB_Y
    gz = GRIP_Z
    right = [
        Waypoint(1.5, _w(hx, y0, TRAVEL), WIDE, "over the handle", GRIP_YAW),
        Waypoint(1.5, _w(hx, y0, gz), WIDE, "down round the handle", GRIP_YAW),
        Waypoint(0.8, _w(hx, y0, gz), FINGER_CLOSED, "grip", GRIP_YAW),
        Waypoint(3.0, _w(hx + d1, y0, gz), FINGER_CLOSED, "push the head end in over the bay floor", GRIP_YAW),
        Waypoint(HOLD, _w(hx + d1, y0, gz), FINGER_CLOSED, "hold while the front legs are released", GRIP_YAW),
        Waypoint(3.0, _w(hx + d2, y0, gz), FINGER_CLOSED, "push on until the rear legs near the sill", GRIP_YAW),
        Waypoint(HOLD, _w(hx + d2, y0, gz), FINGER_CLOSED, "hold the foot end while the rear legs are released",
                 GRIP_YAW),
        Waypoint(1.0, _w(hx + d2, y0, gz + LIFT), FINGER_CLOSED, "lift the foot end", GRIP_YAW),
        Waypoint(4.0, _w(hx + d3, y0, gz + LIFT), FINGER_CLOSED, "push it home over the sill", GRIP_YAW),
        Waypoint(1.2, _w(hx + d3, y0, gz - 0.004), FINGER_CLOSED, "lower the foot end onto the bay floor", GRIP_YAW),
        Waypoint(0.8, _w(hx + d3, y0, gz - 0.004), WIDE, "let go", GRIP_YAW),
        Waypoint(1.0, _w(hx + d3, y0, gz + 0.02), WIDE, "up", GRIP_YAW),
        Waypoint(1.0, _w(hx + d3 - 0.10, y0, TRAVEL), WIDE, "back toward the robot", GRIP_YAW),
        Waypoint(1.2, _w(0.18, -0.25, TRAVEL), WIDE, "park"),
        Waypoint(2.0, _w(0.18, -0.25, TRAVEL), WIDE, "wait while it settles"),
    ]
    t_push1 = 1.5 + 1.5 + 0.8 + 3.0
    fx = x0 + FRONT_PIVOT + d1 + LAG
    rx = x0 + REAR_PIVOT + d2 + LAG
    t_hold = t_push1 + HOLD + 3.0  # the right arm holds the foot end from here
    left = [
        Waypoint(t_push1 - 1.5, _w(0.15, 0.22, 0.12), FINGER_CLOSED, "wait"),
        Waypoint(1.0, _w(fx, ty + 0.06, tab_z + TIP + 0.015), FINGER_CLOSED, "beside the front tab"),
        Waypoint(0.5, _w(fx, ty, tab_z + TIP + 0.015), FINGER_CLOSED, "over the front tab"),
        Waypoint(1.0, _w(fx, ty, tab_z + TIP - PRESS), FINGER_CLOSED, "press the front tab"),
        Waypoint(0.3, _w(fx, ty, tab_z + TIP - PRESS), FINGER_CLOSED, "hold it: the front legs are freed"),
        Waypoint(0.5, _w(fx, ty, tab_z + TIP + 0.015), FINGER_CLOSED, "up"),
        Waypoint(0.8, _w(fx, ty + 0.06, tab_z + TIP + 0.015), FINGER_CLOSED, "aside, off the deck"),
        Waypoint(1.0, _w(0.15, 0.22, 0.12), FINGER_CLOSED, "clear of the deck while the legs swing up"),
        Waypoint(t_hold + 0.3 - t_push1 - 2.6 - 1.0 - 1.5, _w(0.15, 0.22, 0.12), FINGER_CLOSED,
                 "wait while it is pushed on"),
        Waypoint(1.0, _w(rx, ty + 0.06, tab_z + TIP + 0.015), FINGER_CLOSED, "beside the rear tab"),
        Waypoint(0.5, _w(rx, ty, tab_z + TIP + 0.015), FINGER_CLOSED, "over the rear tab"),
        Waypoint(0.8, _w(rx, ty, tab_z + TIP - PRESS), FINGER_CLOSED, "press the rear tab"),
        Waypoint(0.3, _w(rx, ty, tab_z + TIP - PRESS), FINGER_CLOSED, "hold it: the rear legs are freed"),
        Waypoint(0.5, _w(rx, ty, tab_z + TIP + 0.015), FINGER_CLOSED, "up"),
        Waypoint(0.8, _w(rx, ty + 0.06, tab_z + TIP + 0.015), FINGER_CLOSED, "aside, off the deck"),
        Waypoint(1.0, _w(0.15, 0.22, 0.12), FINGER_CLOSED, "park, clear of the deck while the legs swing up"),
    ]
    total = sum(w.duration for w in right)
    left.append(Waypoint(total - sum(w.duration for w in left), _w(0.15, 0.22, 0.12), FINGER_CLOSED, "wait"))
    return {"left": left, "right": right}
