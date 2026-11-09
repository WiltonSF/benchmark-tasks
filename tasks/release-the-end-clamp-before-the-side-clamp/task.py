"""On the station table, in front of the left arm: a heavy steel machining fixture standing loose on the table. Its
dark plate carries an end locator block across one end and a side locator rail running lengthwise 25 mm in from one
long edge. A light grey plastic blank lies loose on the plate, a little short of both locators. Two identical yellow spring pushers are held back by
latches: one behind the blank's short end, one beside its long side. Each has a red release button on top of its
own housing; pressed, it releases that pusher, which springs forward and stays pressed against whatever stops it.
Which way round the fixture stands changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so the on_reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK, YELLOW, RED = (0.62, 0.64, 0.67), (0.22, 0.23, 0.25), (0.95, 0.78, 0.10), (0.85, 0.12, 0.10)
BLANK_COLOR = (0.80, 0.81, 0.82)

# World frame: x away from the arm bases, y to the robot's left, z up. The fixture is one loose 5 kg part; its frame
# has its origin at the plate's centre on the table top, x and y as the world's in lot A. Everything below is given
# in that frame (lot A: add FIX_XY).
FIX_XY = (0.295, 0.165)
FIX_MASS = 5.0
PLATE_T = 0.010
PLATE = (0.170, 0.130)
END_FACE_X = 0.045  # the end locator's face (the blank's far end must reach it); world x 0.340 in lot A
SIDE_FACE_Y = 0.025  # the side locator's face (the blank's left side must reach it); world y 0.190 in lot A
# The side rail spans y 0.025-0.040, so its outer face is 25 mm in from the plate's left edge (y 0.065).
LOC_H = 0.020
BLANK = (0.080, 0.040, 0.025)
BLANK_MASS = 0.10  # acetal plastic; a solid acetal blank this size is about 0.11 kg
SHORT_END, SHORT_SIDE = 0.015, 0.008  # how far the blank starts from each locator
BLANK_LOCAL = (END_FACE_X - SHORT_END - BLANK[0] / 2, SIDE_FACE_Y - SHORT_SIDE - BLANK[1] / 2)  # (-0.010, -0.003)
GAP = 0.005  # each pusher's face starts 5 mm from the blank
TRAVEL = 0.040
END_K, SIDE_K = 80.0, 280.0  # N/m: about 1.6 N and 7.6 N where each stops on the clamped blank
BUTTON_TRAVEL, BUTTON_K, PRESSED = 0.008, 50.0, 0.005

HEAD_LIFT, HEAD_H = 0.005, 0.025  # heads ride 5 mm over the plate, 25 mm tall
HEAD_Z = PLATE_T + HEAD_LIFT + HEAD_H / 2
END_HEAD = (0.020, 0.040, HEAD_H)
SIDE_HEAD = (0.040, 0.020, HEAD_H)
SIDE_HEAD_X = 0.0  # the side head's centre along the blank
END_FACE0 = BLANK_LOCAL[0] - BLANK[0] / 2 - GAP  # x of the end pusher's face, retracted (-0.055)
SIDE_FACE0 = BLANK_LOCAL[1] - BLANK[1] / 2 - GAP  # y of the side pusher's face, retracted (-0.028)
HOUSING_H = 0.045  # housings stand on the table, 45 mm tall; the buttons stand on top
END_HOUSING = (0.030, 0.050, HOUSING_H)
SIDE_HOUSING = (0.050, 0.030, HOUSING_H)
END_HOUSING_XY = (END_FACE0 - END_HEAD[0] - 0.005 - END_HOUSING[0] / 2, BLANK_LOCAL[1])  # (-0.095, -0.003)
SIDE_HOUSING_XY = (SIDE_HEAD_X, SIDE_FACE0 - SIDE_HEAD[1] - 0.005 - SIDE_HOUSING[1] / 2)  # (0.0, -0.068)
BUTTON_R, BUTTON_H = 0.010, 0.010
END_BUTTON_LOCAL, SIDE_BUTTON_LOCAL = END_HOUSING_XY, SIDE_HOUSING_XY

END_PUSHER = parts.Slider(
    "end_pusher",
    shapes=[parts.box(END_HEAD, at=(END_FACE0 - END_HEAD[0] / 2, BLANK_LOCAL[1], HEAD_Z), color=YELLOW),
            parts.box((0.035, 0.010, 0.010), at=(END_FACE0 - END_HEAD[0] - 0.0175, BLANK_LOCAL[1], HEAD_Z),
                      color=STEEL)],
    axis=(1.0, 0.0, 0.0), travel=TRAVEL, mass=0.05, spring=END_K, damping=2.0,
)
SIDE_PUSHER = parts.Slider(
    "side_pusher",
    shapes=[parts.box(SIDE_HEAD, at=(SIDE_HEAD_X, SIDE_FACE0 - SIDE_HEAD[1] / 2, HEAD_Z), color=YELLOW),
            parts.box((0.010, 0.035, 0.010), at=(SIDE_HEAD_X, SIDE_FACE0 - SIDE_HEAD[1] - 0.0175, HEAD_Z),
                      color=STEEL)],
    axis=(0.0, 1.0, 0.0), travel=TRAVEL, mass=0.05, spring=SIDE_K, damping=4.0,
)


def _button(name, xy):
    return parts.Slider(
        name,
        shapes=[parts.cylinder(BUTTON_R, BUTTON_H, at=(*xy, HOUSING_H + BUTTON_H / 2), color=RED, axis="z")],
        axis=(0.0, 0.0, -1.0), travel=BUTTON_TRAVEL, mass=0.01, spring=BUTTON_K, damping=0.5,
    )


END_BUTTON = _button("end_button", END_BUTTON_LOCAL)
SIDE_BUTTON = _button("side_button", SIDE_BUTTON_LOCAL)

FIXTURE = parts.Part(
    "fixture",
    pos=(float(FIX_XY[0]), float(FIX_XY[1]), float(TOP)),
    mass=FIX_MASS,
    shapes=[
        parts.box((*PLATE, PLATE_T), at=(0.0, 0.0, PLATE_T / 2), color=DARK),
        parts.box((0.015, 0.090, LOC_H), at=(END_FACE_X + 0.0075, -0.005, PLATE_T + LOC_H / 2), color=STEEL),
        parts.box((0.090, 0.015, LOC_H), at=(-0.005, SIDE_FACE_Y + 0.0075, PLATE_T + LOC_H / 2), color=STEEL),
        parts.box(END_HOUSING, at=(*END_HOUSING_XY, HOUSING_H / 2), color=YELLOW),
        parts.box(SIDE_HOUSING, at=(*SIDE_HOUSING_XY, HOUSING_H / 2), color=YELLOW),
    ],
    joints=[END_PUSHER, SIDE_PUSHER, END_BUTTON, SIDE_BUTTON],
)
BLANK_Z = TOP + PLATE_T + BLANK[2] / 2
BLANK_BOX = Box("blank", pos=(float(FIX_XY[0] + BLANK_LOCAL[0]), float(FIX_XY[1] + BLANK_LOCAL[1]), float(BLANK_Z)),
                size=BLANK, mass=BLANK_MASS, color=BLANK_COLOR)

OBJECTS = [FIXTURE, BLANK_BOX]


def _lot_point(local_xy, yaw, centre=FIX_XY):
    c, s = math.cos(yaw), math.sin(yaw)
    return (centre[0] + c * local_xy[0] - s * local_xy[1], centre[1] + s * local_xy[0] + c * local_xy[1])


# Lots: A as drawn (the screenshot). B turns the fixture end for end about its centre, with the blank where the
# fixture puts it: the end locator then faces the robot and the side locator is on the right. Lot A gets a
# zero-distance move so it is settled and re-rendered like B.
# B's fixture also stands 6 mm nearer the robot, so that the blank's lot B start does not overlap lot A's end locator
# while the lot moves the parts.
B_SHIFT = (-0.006, 0.0)
_B_FIX = (FIX_XY[0] + B_SHIFT[0], FIX_XY[1] + B_SHIFT[1])
_B_BLANK = _lot_point(BLANK_LOCAL, math.pi, _B_FIX)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"fixture": FIXTURE.pos}),
    lots.Lot("B", place={"fixture": (float(_B_FIX[0]), float(_B_FIX[1]), float(TOP)),
                         "blank": (float(_B_BLANK[0]), float(_B_BLANK[1]), float(BLANK_Z))},
             yaw={"fixture": math.pi}),
)

EPS = 1e-6
SEATED = 0.001  # the blank within 1 mm of a locator face, along its whole end or side
BEARING = 0.001  # a released pusher's face within 1 mm of the blank
FLAT = 0.001  # the blank's underside within 1 mm of the plate
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the blank or a pusher head
FIX_MOVE, FIX_TURN = 0.005, math.radians(3.0)  # the fixture may not move more than this during the trial

# Latches, emulated: a pusher's spring is aimed at its retracted end (0) until its button is pressed 5 mm, then at
# its far end (40 mm), and stays released until the scene resets. Offline (near_misses.py) "released" is read from
# the pusher's position instead (see _released).
_latch = {"end_pusher": False, "side_pusher": False, "sim": False}
_grip = {"clear": True}
_fault = {"moved": False, "ref": None}
_DEBUG = bool(__import__("os").environ.get("CLAMP_DEBUG"))  # debugging only: print the checks every 20 steps
_dbg = {"k": 0}
# Still: pressed hard against a locator, the simulated blank reports a steady contact velocity of about 1.3 cm/s
# while it does not move at all, so in the simulator "still" is judged from its position: it moved less than
# 0.5 mm over the last 1 s (20 control steps). Offline it is the velocity test.
STILL_STEPS, STILL_MOVE = 20, 0.0005
_hist: list = []


@parts.on_reset
def _reset_latches(sim) -> None:
    _latch.update(end_pusher=False, side_pusher=False, sim=True)
    _fault.update(moved=False, ref=None)
    _hist.clear()


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _fix_frame(fx):
    return np.asarray(fx.pos, float), _rot(fx.quat_xyzw)


def _update_fixture(fx) -> None:
    """Whole-trial record: the fixture moved more than 5 mm, turned or tipped more than 3 degrees from where it stood
    at the trial's first step (in the simulator; offline, from its lot's declared pose). The lot move leaves the
    simulated fixture about 2 mm from its declared lot B centre, so the start is the pose actually measured."""
    p, r = _fix_frame(fx)
    yaw = math.atan2(r[1, 0], r[0, 0])
    if _fault["ref"] is None:
        lot = lots.current()
        _fault["ref"] = (p.copy(), yaw) if _latch["sim"] else (np.asarray(lot.pos("fixture"), float),
                                                               lot.yaw_of("fixture"))
    p0, yaw0 = _fault["ref"]
    turn = abs((yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    tipped = math.acos(min(1.0, abs(float(r[2, 2]))))
    if np.linalg.norm(p - p0) > FIX_MOVE + EPS or turn > FIX_TURN + EPS or tipped > FIX_TURN + EPS:
        _fault["moved"] = True


@parts.on_step
def _step(sim) -> None:
    from yam_arena import analysis

    _latch["sim"] = True
    st = sim.states()
    fx = st["fixture"]
    for pusher, button in (("end_pusher", "end_button"), ("side_pusher", "side_button")):
        if not _latch[pusher] and fx.joints[button] > PRESSED:
            _latch[pusher] = True
            sim.set_target("fixture", pusher, TRAVEL)
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    b = st["blank"]
    _hist.append(np.asarray(b.pos, float))
    del _hist[:-STILL_STEPS]
    boxes = [(np.asarray(b.pos), _rot(b.quat_xyzw), np.array(BLANK))]
    links = {n: (fx.links[n][0], _rot(fx.links[n][1])) for n in ("end_pusher", "side_pusher")}
    boxes += [(c, r, s) for name, c, r, s in FIXTURE.world_boxes(links)
              if any(np.allclose(s, h) for h in (END_HEAD, SIDE_HEAD))]  # the two heads, not their rods
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
    if _DEBUG:
        _dbg["k"] += 1
        if _dbg["k"] % 20 == 0:
            d = [round(float(analysis.hulls_to_box(hulls, c, r, s_)[0]), 4) for c, r, s_ in boxes]
            print(f"[clamp] k={_dbg['k']} lot={lots.current().name} latch={_latch} clear={_grip['clear']} dists={d} "
                  f"fix={np.round(fx.pos, 4).tolist()} moved={_fault['moved']} clamped={_clamped(st)}",
                  file=sys.stderr, flush=True)


def _corners_local(b, fx) -> np.ndarray:
    """The blank's eight corners in the fixture's frame (origin at the plate's centre on the table top)."""
    r = _rot(b.quat_xyzw)
    half = np.array(BLANK) / 2
    world = np.array([b.pos + r @ (half * [sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    p, rf = _fix_frame(fx)
    return (world - p) @ rf


def _still(b) -> bool:
    if not _latch["sim"]:
        return b.resting()
    return len(_hist) == STILL_STEPS and max(np.linalg.norm(p - _hist[-1]) for p in _hist) < STILL_MOVE


def _released(fx, pusher) -> bool:
    """In the simulator: its latch was released by its button. Offline: it has moved over 2 mm off its stop."""
    return _latch[pusher] if _latch["sim"] else fx.joints[pusher] > 0.002


def _clamped(objects) -> bool:
    fx, b = objects["fixture"], objects["blank"]
    _update_fixture(fx)
    c = _corners_local(b, fx)
    far_end = np.sort(c[:, 0])[-4:]  # the four corners of the blank's far end
    left_side = np.sort(c[:, 1])[-4:]  # the four corners of its locator-side long face
    end_face = END_FACE0 + fx.joints["end_pusher"]
    side_face = SIDE_FACE0 + fx.joints["side_pusher"]
    return bool(
        not _fault["moved"]
        and np.all(far_end >= END_FACE_X - SEATED - EPS) and np.all(far_end <= END_FACE_X + 0.001 + EPS)
        and np.all(left_side >= SIDE_FACE_Y - SEATED - EPS) and np.all(left_side <= SIDE_FACE_Y + 0.001 + EPS)
        and abs(c[:, 2].min() - PLATE_T) <= FLAT + EPS
        and _released(fx, "end_pusher") and _released(fx, "side_pusher")
        and c[:, 0].min() - end_face <= BEARING + EPS and c[:, 1].min() - side_face <= BEARING + EPS
        and _still(b) and _grip["clear"]
    )


def success(objects):
    """The blank flat on the plate against both locators, both pushers released and bearing on it, let go and still;
    the fixture never moved."""
    return _clamped(objects)


def distance(objects):
    fx, b = objects["fixture"], objects["blank"]
    p, rf = _fix_frame(fx)
    lb = (np.asarray(b.pos, float) - p) @ rf
    return float(np.hypot(END_FACE_X - BLANK[0] / 2 - lb[0], SIDE_FACE_Y - BLANK[1] / 2 - lb[1]))


def score(objects):
    """Pass/fail: 1.0 when clamped against both locators, otherwise 0."""
    return 1.0 if _clamped(objects) else 0.0


# --- demo: left arm only ----------------------------------------------------------------------------------------
TIP = 0.013
PRESS_Z = TOP + HOUSING_H + BUTTON_H - BUTTON_TRAVEL - 0.004 + TIP  # closed tips commanded 4 mm past full travel
DEMO_TOUCHES = ["fixture"]  # the closed finger tips press the buttons, down onto their housings
ABOVE = TOP + 0.12
DEMO_SIDE_FIRST = bool(__import__("os").environ.get("CLAMP_SIDE_FIRST"))  # counter-test only: side button first


def _press(xy, label):
    x, y = xy
    return [
        Waypoint(1.5, (x, y, ABOVE), FINGER_CLOSED, f"over the {label} button"),
        Waypoint(1.2, (x, y, PRESS_Z), FINGER_CLOSED, f"press the {label} button"),
        Waypoint(0.6, (x, y, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (x, y, ABOVE), FINGER_CLOSED, "up"),
    ]


def demo():
    """Left arm: press the button on the pusher behind the blank's end, wait for the blank to reach the end locator,
    then the button on the pusher beside it. Where the buttons are follows the fixture's pose, as the cameras show."""
    lot = lots.current()
    yaw, centre = lot.yaw_of("fixture"), lot.pos("fixture")[:2]
    end_xy, side_xy = _lot_point(END_BUTTON_LOCAL, yaw, centre), _lot_point(SIDE_BUTTON_LOCAL, yaw, centre)
    first, second = ((side_xy, "side"), (end_xy, "end")) if DEMO_SIDE_FIRST else ((end_xy, "end"), (side_xy, "side"))
    plan = _press(*first) + [Waypoint(1.0, (first[0][0], first[0][1], ABOVE), FINGER_CLOSED, "wait")] + _press(*second)
    return {"left": plan + [Waypoint(1.5, (0.18, 0.30, ABOVE), FINGER_CLOSED, "clear of the fixture")]}
