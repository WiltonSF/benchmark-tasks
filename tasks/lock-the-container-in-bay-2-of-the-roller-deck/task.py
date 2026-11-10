"""On the station table, between the arms: a roller deck running from the robot's left to its right, with a lane of
rollers between two low guide rails. Its right end stands on a block, so the deck slopes gently down toward the
robot's left. The lane is divided into three bays, numbered by one, two and three white squares on the deck's near
edge, bay 1 on the robot's left. Low fixed stops close the lane at both ends. Between bays 1 and 2 and between bays
2 and 3 is a yellow pop-up lock: a plate across the lane, joined under the deck to a pedal hanging off the deck's
near edge. Pressing a pedal all the way down drops a raised lock flush with the deck, where it stays; pressing it all
the way down again raises the lock. A raised lock cannot be pushed down from the lane; only its pedal moves it. A
loose silver cargo container, wider than the gripper opens, sits on the rollers in one of the bays; a lock may
already be down.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots: its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

DECK_GREY, GUIDE_GREY, ROLLER, WHITE = (0.30, 0.31, 0.33), (0.45, 0.46, 0.48), (0.70, 0.71, 0.73), (0.95, 0.95, 0.95)
YELLOW, STOP_DARK, SILVER, BAND = (0.95, 0.75, 0.08), (0.18, 0.18, 0.20), (0.74, 0.76, 0.80), (0.20, 0.32, 0.62)

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
DECK_X = (0.215, 0.395)  # the deck plate, near to far
DECK_Y = 0.200  # it runs from y -0.200 to +0.200
DECK_H = 0.045  # deck top
LANE_X = (0.245, 0.365)  # between the guide rails (120 mm)
LANE_C = (LANE_X[0] + LANE_X[1]) / 2
GUIDE_T, GUIDE_H = 0.010, 0.012  # guide rails: 10 mm wide, 12 mm above the deck
ROLL_R = 0.006
ROLL_TOP = DECK_H + 0.002  # roller tops: the container's underside rests here
BAY_Y = (0.124, 0.0, -0.124)  # bay 1 (robot's left), bay 2, bay 3 centres
LOCK_Y = (0.062, -0.062)  # lock 1|2 and lock 2|3 centres
LOCK_T = 0.012  # plate thickness along the lane
BAY_LEN = 2 * (LOCK_Y[0] - LOCK_T / 2 - BAY_Y[1])  # 112 mm between lock faces
END_IN = BAY_Y[0] + BAY_LEN / 2  # 0.180: inner faces of the end stops at y = +-0.180
END_T, END_H = 0.012, ROLL_TOP + 0.010  # end stops 10 mm above the rollers
LOCK_UP = ROLL_TOP + 0.018  # a raised lock's top: 18 mm above the rollers
PEDAL_X = (0.140, 0.164)  # pedal, 5 cm out from the deck's near edge (joined to its plate under the deck)
PEDAL = 0.024  # pedal square
PEDAL_T = 0.012  # pedal thickness

# The lock slides straight down. 0 is up (its upper limit); 0.030 is the bottom of the pedal's travel.
TRAVEL = 0.030
DOWN_TARGET = 0.020  # a dropped lock is held here (it sags about 3 mm more under its own weight: plate 5 mm below the deck)
UP_TARGET = -0.010  # a raised lock is pressed against its upper limit
PRESS = 0.027  # pressed this far (pedal 27 mm down) the catch toggles
REARM = 0.025  # and it must come back above this before it can toggle again
LOCK_SPRING, LOCK_MASS = 40.0, 0.012
HELD_BELOW, HELD_TRAVEL = 0.002, 0.001  # a raised lock within 2 mm of the top is held there (1 mm of play)
PEDAL_TOUCH = 0.002  # pressed: a finger or gripper surface within 2 mm of the pedal...
PEDAL_PRESSED = 0.0007  # ...and the pedal pushed down at least 0.7 mm, against the 1 mm hold

# The deck slopes 3 degrees down toward the robot's left (bay 1), its right end propped on a block: a container
# left on it rolls toward bay 1 unless a raised lock holds it. Deck-frame coordinates: x, y as the world's at the
# deck's middle, z up from the deck's underside.
SLOPE = math.radians(3.0)
ROLL = -SLOPE  # about +x: the +y end goes down
Z0 = DECK_Y * math.sin(SLOPE) + 0.0005  # the deck's frame origin above the table: its low corner just clears it
FRICTION = 0.02  # deck, rollers and container: standing in for free-turning rollers


def _rx(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])


R_DECK = _rx(ROLL)
DECK_SHIFT = -LOCK_Y[0]  # the deck sits 62 mm to the robot's right, so its 1|2 lock is on the midline (both arms reach it)
DECK_ORIGIN = np.array([0.0, DECK_SHIFT, TOP + Z0])


def to_world(x, y, z):
    """A deck-frame point (z above the deck's underside) in world coordinates."""
    return tuple(float(v) for v in DECK_ORIGIN + R_DECK @ np.array([x, y, z], float))


def to_deck(p):
    return R_DECK.T @ (np.asarray(p, float) - DECK_ORIGIN)

_sx = (DECK_X[0] + DECK_X[1]) / 2


def _rollers():
    out = []
    for by in BAY_Y:
        for dy in (-0.042, -0.014, 0.014, 0.042):
            out.append(parts.cylinder(ROLL_R, LANE_X[1] - LANE_X[0] - 0.002,
                                      at=(LANE_C, by + dy, ROLL_TOP - ROLL_R), color=ROLLER, axis="x"))
    return out


def _dots():
    out = []
    for n, by in enumerate(BAY_Y, start=1):
        for k in range(n):
            dy = (k - (n - 1) / 2) * 0.022
            out.append(parts.box((0.016, 0.016, 0.001), at=(0.225, by + dy, DECK_H + 0.0005), color=WHITE))
    return out


def _lock(name, y):
    plate_h = LOCK_UP - TRAVEL - 0.006  # its underside stays 6 mm above the table at the bottom of its travel
    return parts.Slider(
        name, axis=(0.0, 0.0, -1.0), travel=TRAVEL, mass=LOCK_MASS, spring=LOCK_SPRING, damping=1.0,
        shapes=[
            parts.box((LANE_X[1] - LANE_X[0] - 0.004, LOCK_T, plate_h), at=(LANE_C, y, LOCK_UP - plate_h / 2),
                      color=YELLOW),
            parts.box((PEDAL_X[1] - PEDAL_X[0], PEDAL, PEDAL_T), at=((PEDAL_X[0] + PEDAL_X[1]) / 2, y,
                                                                      LOCK_UP - PEDAL_T / 2), color=YELLOW),
            # the arm joining the pedal to the plate, running in under the deck
            parts.box((LANE_X[0] - PEDAL_X[1], 0.010, 0.008), at=((PEDAL_X[1] + LANE_X[0]) / 2, y, LOCK_UP - 0.008),
                      color=STOP_DARK),
        ],
    )


DECK = parts.Part(
    "roller_deck",
    pos=to_world(0.0, 0.0, 0.0),
    roll=ROLL,
    friction=(FRICTION, FRICTION),
    shapes=[
        parts.box((DECK_X[1] - DECK_X[0], 2 * DECK_Y, DECK_H), at=(_sx, 0.0, DECK_H / 2), color=DECK_GREY),
        parts.box((GUIDE_T, 2 * DECK_Y, GUIDE_H), at=(LANE_X[0] - GUIDE_T / 2, 0.0, DECK_H + GUIDE_H / 2),
                  color=GUIDE_GREY),
        parts.box((GUIDE_T, 2 * DECK_Y, GUIDE_H), at=(LANE_X[1] + GUIDE_T / 2, 0.0, DECK_H + GUIDE_H / 2),
                  color=GUIDE_GREY),
        parts.box((LANE_X[1] - LANE_X[0], END_T, END_H - DECK_H), at=(LANE_C, END_IN + END_T / 2, (END_H + DECK_H) / 2),
                  color=STOP_DARK),
        parts.box((LANE_X[1] - LANE_X[0], END_T, END_H - DECK_H),
                  at=(LANE_C, -END_IN - END_T / 2, (END_H + DECK_H) / 2), color=STOP_DARK),
        *_rollers(),
        *_dots(),
    ],
    joints=[_lock("lock_12", LOCK_Y[0]), _lock("lock_23", LOCK_Y[1])],
)

BOX = (0.100, 0.100, 0.080)
CONTAINER = parts.Part(
    "container",
    pos=to_world(LANE_C, END_IN - 0.050 - 0.0005, ROLL_TOP),  # origin at the centre of its underside
    roll=ROLL,
    mass=1.2,
    friction=(FRICTION, FRICTION),
    shapes=[
        parts.box(BOX, at=(0.0, 0.0, BOX[2] / 2), color=SILVER),
        parts.box((0.002, 0.060, 0.010), at=(-BOX[0] / 2 - 0.001, 0.0, 0.060), color=BAND),  # label band, near face
    ],
)

# The block under the deck's raised right end (fixed, plain grey).
_riser_y = -DECK_Y + 0.015
RISER = Box("riser_block", pos=(_sx, _riser_y + DECK_SHIFT, TOP + 0.010), size=(DECK_X[1] - DECK_X[0], 0.030, 0.020),
            color=GUIDE_GREY)

OBJECTS = [DECK, CONTAINER, RISER]

# Where the container rests: in bay 1 against the left end stop, in bay 3 against the 2|3 lock (deck-frame y).
REST_BAY1 = END_IN - BOX[1] / 2 - 0.0005
REST_BAY3 = LOCK_Y[1] - LOCK_T / 2 - BOX[1] / 2 - 0.0005


def _rest(y):
    return to_world(LANE_C, y, ROLL_TOP)


# Each lot starts a different set of locks down, so a fixed routine of pedal presses ends with both locks up in at
# most one lot. Lot A (the screenshot): container in bay 1, both locks up. Lot B: container in bay 3 resting on the
# 2|3 lock, the 1|2 lock down. Lot C: container in bay 1, the 2|3 lock down.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"container": _rest(REST_BAY1)}, down=()),
    lots.Lot("B", place={"container": _rest(REST_BAY3)}, down=("lock_12",)),
    lots.Lot("C", place={"container": _rest(REST_BAY1)}, down=("lock_23",)),
)

EPS = 1e-6
TARGET_Y = BAY_Y[1]
CENTRE_TOL = 0.015  # container centre within 15 mm of bay 2's centre along the lane
LANE_PLAY = (LANE_X[1] - LANE_X[0] - BOX[0]) / 2  # 10 mm: its centre within the guide rails' play
ON_ROLLERS = 0.002  # underside within 2 mm of the roller tops
MAX_TILT = math.radians(5.0)
LOCKED = 0.003  # a lock is up when it stands within 3 mm of its upper limit
LIFT_FAULT = 0.010  # underside more than 10 mm above the roller tops at any time
CLEAR = 0.005  # released: no finger or gripper surface within 5 mm of the container or either lock
SUCCESS_HOLD_S = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    """The container's tilt from square on the deck (rad)."""
    up = (R_DECK.T @ _rot(state.quat_xyzw))[:, 2]
    return math.acos(float(np.clip(up[2], -1.0, 1.0)))


def _lift(c) -> float:
    """How far the container's underside centre is above the roller tops, square to the deck (m)."""
    return float(to_deck(c.pos)[2] - ROLL_TOP)


# Whole-trial record: was the container ever lifted more than 10 mm off the rollers? Cleared only by a scene reset:
# the container back within 0.5 mm of its start and at rest, after a jump of over 20 mm in one control step.
_record = {"fault": False, "prev": None}
# Released: measured in the simulator after every control step (see _measure_grip). Offline it stays True.
_grip = {"clear": True}
# The push-push catch of each lock: which way it is set, and whether it may toggle again.
_catch = {"mode": {"lock_12": "up", "lock_23": "up"}, "armed": {"lock_12": True, "lock_23": True}}


def _update(objects) -> None:
    c = objects["container"]
    pos = np.asarray(c.pos, float)
    prev = _record["prev"]
    start = np.asarray(lots.current().pos("container"), float)
    if prev is not None and np.linalg.norm(pos - start) < 0.0005 and c.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if _lift(c) > LIFT_FAULT + EPS:
        _record["fault"] = True


def _set(sim, lock, mode) -> None:
    _catch["mode"][lock] = mode
    sim.set_target("roller_deck", lock, DOWN_TARGET if mode == "down" else UP_TARGET)


@parts.on_reset
def _reset_locks(sim) -> None:
    import torch

    down = lots.current().down
    base = sim.env.unwrapped
    deck = base.scene["roller_deck"]
    names = ["lock_12", "lock_23"]
    for lock in names:
        _catch["armed"][lock] = True
        _set(sim, lock, "down" if lock in down else "up")
    q = [DOWN_TARGET + 0.003 if n in down else 0.0 for n in names]
    ids = deck.find_joints(names, preserve_order=True)[0]
    lim = torch.tensor([[[0.0, TRAVEL]] * 2], dtype=torch.float32, device=base.device)
    deck.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)
    deck.write_joint_position_to_sim_index(position=torch.tensor([q], device=base.device), joint_ids=ids)
    deck.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, 2), device=base.device), joint_ids=ids)
    _record["fault"], _record["prev"] = False, None
    _catch["held"] = None


def _step_locks(sim, joints, pedal_touched) -> None:
    """The push-push catch, and the hold: a raised lock is held up rigidly (its slide limited to 1 mm) unless its
    pedal is being pressed (a finger on it and the pedal pushed down against the hold), so the container cannot
    shove it down; only the pedal moves it."""
    import torch

    for lock in ("lock_12", "lock_23"):
        q = joints[lock]
        if q > PRESS and _catch["armed"][lock]:
            _catch["armed"][lock] = False
            _set(sim, lock, "up" if _catch["mode"][lock] == "down" else "down")
        elif q < REARM:
            _catch["armed"][lock] = True
    held = tuple(_catch["mode"][k] == "up" and joints[k] < HELD_BELOW and not pedal_touched[k]
                 for k in ("lock_12", "lock_23"))
    if held == _catch.get("held"):
        return
    _catch["held"] = held
    base = sim.env.unwrapped
    deck = base.scene["roller_deck"]
    ids = deck.find_joints(["lock_12", "lock_23"], preserve_order=True)[0]
    lim = torch.tensor([[[0.0, HELD_TRAVEL if h else TRAVEL] for h in held]], dtype=torch.float32, device=base.device)
    deck.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


def _lock_boxes(deck, lock):
    p, q = deck.links[lock]
    return [(c, r, s) for _, c, r, s in DECK.world_boxes({lock: (np.asarray(p), _rot(q))})]


def _pedal_box(deck, lock):
    p, q = deck.links[lock]
    boxes = DECK.world_boxes({lock: (np.asarray(p), _rot(q))})
    return [(c, r, s) for _, c, r, s in boxes[1:2]]  # the second shape of a lock link is its pedal


def _container_boxes(c):
    return [(cc, r, s) for _, cc, r, s in CONTAINER.world_boxes({"base": (np.asarray(c.pos), _rot(c.quat_xyzw))})]


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    boxes = _container_boxes(st["container"]) + _lock_boxes(st["roller_deck"], "lock_12") + \
        _lock_boxes(st["roller_deck"], "lock_23")
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)
    deck = st["roller_deck"]
    touched = {k: deck.joints[k] >= PEDAL_PRESSED - EPS and any(
        analysis.hulls_to_box(hulls, c, r, sz)[0] < PEDAL_TOUCH for c, r, sz in _pedal_box(deck, k))
        for k in ("lock_12", "lock_23")}
    _step_locks(sim, deck.joints, touched)


def _in_bay2(c) -> bool:
    local = to_deck(c.pos)
    return bool(
        abs(local[1] - TARGET_Y) <= CENTRE_TOL + EPS
        and abs(local[0] - LANE_C) <= LANE_PLAY + EPS
        and abs(_lift(c)) <= ON_ROLLERS + EPS
        and _tilt(c) <= MAX_TILT + EPS
    )


def _locked(deck) -> bool:
    return all(deck.joints[k] <= LOCKED + EPS for k in ("lock_12", "lock_23"))


def _stowed(objects) -> bool:
    c = objects["container"]
    return bool(_in_bay2(c) and _locked(objects["roller_deck"]) and c.resting() and _grip["clear"])


def success(objects):
    """The container sits in bay 2 on the rollers, both locks either side of it fully up, let go and still; it was
    never lifted off the rollers."""
    _update(objects)
    return bool(not _record["fault"] and _stowed(objects))


def distance(objects):
    return float(abs(objects["container"].pos[1] - TARGET_Y))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _update(objects)
    return 1.0 if not _record["fault"] and _stowed(objects) else 0.0


# --- demo ------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
HOVER = 0.17  # TCP height above the deck's frame while moving: tips well over the container's top
PEDAL_XC = (PEDAL_X[0] + PEDAL_X[1]) / 2
PRESS_TCP = LOCK_UP - TRAVEL - 0.014 + TIP  # tips commanded 14 mm past the pedal's full travel (the arm gives way)
PUSH_TIP = ROLL_TOP + 0.025  # finger tips 25 mm above the rollers while pushing: over the end stops and locks
FINGER_HALF = 0.010  # closed finger tips reach about 10 mm either side of the TCP along y (wider higher up)
PUSH_OVER = 0.008  # commanded past where the container stops against the far lock
PARK = {"left": (0.18, 0.30), "right": (0.18, -0.18)}  # deck frame


class _Plan:
    """Both arms' waypoints, built one step at a time; the arm not moving holds where it is."""

    def __init__(self):
        self.w = {"left": [], "right": []}
        self.at = {a: to_world(*PARK[a], 0.14) for a in self.w}

    def move(self, arm, steps):
        t = 0.0
        for d, (x, y, z), label in steps:
            p = to_world(x, y, z)
            self.w[arm].append(Waypoint(d, p, FINGER_CLOSED, label))
            self.at[arm] = p
            t += d
        other = "right" if arm == "left" else "left"
        self.w[other].append(Waypoint(t, self.at[other], FINGER_CLOSED, "hold" if self.w[other] else "wait"))

    def done(self):
        for a in self.w:
            self.w[a].append(Waypoint(1.5, self.at[a], FINGER_CLOSED, "wait"))
        return self.w


def _press(y, label, low=False):
    """Press a pedal fully and let it up. low: come in and leave low from the robot's side, under the other arm."""
    if low:
        near = (PEDAL_XC - 0.07, y, LOCK_UP + TIP + 0.010)
        come = [(1.5, near, f"low, in front of the {label} pedal")]
        leave = [(0.8, (PEDAL_XC, y, LOCK_UP + TIP + 0.010), f"up off the {label} pedal"), (1.0, near, "back out")]
    else:
        come = [(1.5, (PEDAL_XC, y, HOVER), f"over the {label} pedal")]
        leave = [(1.0, (PEDAL_XC, y, HOVER), f"off the {label} pedal")]
    return come + [
        (1.0, (PEDAL_XC, y, LOCK_UP + TIP + 0.008), f"onto the {label} pedal"),
        (1.2, (PEDAL_XC, y, PRESS_TCP), f"press the {label} pedal down"),
        (0.5, (PEDAL_XC, y, PRESS_TCP), f"hold the {label} pedal"),
    ] + leave


def _park(arm):
    return [(1.0, (*PARK[arm], 0.14), "park")]


def _push_uphill():
    """From bay 1, push the container's rear face up the slope until it rests on the 2|3 lock; stay there."""
    rear = REST_BAY1 + BOX[1] / 2
    behind = rear + FINGER_HALF + 0.016
    end = (LOCK_Y[1] + LOCK_T / 2) + BOX[1] + FINGER_HALF - PUSH_OVER
    z = PUSH_TIP + TIP
    return [
        (1.5, (LANE_C, behind, HOVER), "over the container's rear"),
        (1.2, (LANE_C, behind, z), "down behind it"),
        (4.5, (LANE_C, end, z), "push it up the rollers onto the 2|3 lock"),
        (0.5, (LANE_C, end, z), "hold it there"),
    ]


def _let_back():
    end = (LOCK_Y[1] + LOCK_T / 2) + BOX[1] + FINGER_HALF - PUSH_OVER
    z = PUSH_TIP + TIP
    return [
        (1.5, (LANE_C, end + 0.030, z), "ease off: it rolls back onto the 1|2 lock"),
        (1.0, (LANE_C, end + 0.030, 0.150), "up"),
    ]


def demo():
    """Read from the scene: which bay the container is in and which locks are down. Raise any lock that is down on
    the far side; drop the lock in the container's way; get the container into bay 2 (pushed up the slope and held,
    or let roll down onto the far lock); raise the lock behind it."""
    lot = lots.current()
    plan = _Plan()
    if to_deck(lot.pos("container"))[1] > 0:  # bay 1, the low end: push it up the slope
        if "lock_23" in lot.down:
            plan.move("right", _press(LOCK_Y[1], "2|3 lock") + _park("right"))
        plan.move("left", _press(LOCK_Y[0], "1|2 lock") + _push_uphill())
        plan.move("right", _press(LOCK_Y[0], "1|2 lock", low=True))
        plan.move("left", _let_back() + _park("left"))
        plan.move("right", _park("right"))
    else:  # bay 3, the high end, resting on the 2|3 lock: let it roll down into bay 2
        if "lock_12" in lot.down:
            plan.move("left", _press(LOCK_Y[0], "1|2 lock") + _park("left"))
        plan.move("right", _press(LOCK_Y[1], "2|3 lock") + _park("right"))
        plan.move("left", [(1.5, (*PARK["left"], 0.14), "wait while it rolls into bay 2")])
        plan.move("right", _press(LOCK_Y[1], "2|3 lock") + _park("right"))
    return plan.done()
