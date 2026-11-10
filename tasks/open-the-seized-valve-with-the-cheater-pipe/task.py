"""On the station table: a raised steel deck plate over a buried valve. A red quarter-turn valve handle, a short
square bar on a round hub, sits just above the deck, pointing toward the robot; the valve is shut. A yellow line on
the deck runs beside the handle's fully open position, and a white line beside its shut position. A loose length of square aluminium
tube lies on the deck. Where the tube lies changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

RED, BRASS, STEEL = (0.80, 0.10, 0.08), (0.70, 0.56, 0.25), (0.62, 0.64, 0.67)
DECK_GREY, YELLOW, WHITE = (0.32, 0.33, 0.35), (0.95, 0.80, 0.10), (0.92, 0.92, 0.92)

# World frame: x away from the arm bases, y to the robot's left, z up (m).
# Deck plate: a fixed steel plate 40 mm thick covering the valve pit.
DECK_H = 0.040
DECK_X, DECK_Y = (0.06, 0.50), (-0.46, 0.24)
DECK = parts.Part(
    "deck", pos=((DECK_X[0] + DECK_X[1]) / 2, (DECK_Y[0] + DECK_Y[1]) / 2, TOP),
    shapes=[parts.box((DECK_X[1] - DECK_X[0], DECK_Y[1] - DECK_Y[0], DECK_H), at=(0.0, 0.0, DECK_H / 2),
                      color=DECK_GREY)],
    friction=(0.5, 0.4),
)

# Valve: stem axis at (VX, VY). The handle is a 20 x 20 mm square bar reaching 80 mm from the axis on a 40 mm hub,
# 7 mm above the deck. Angle 0 = pointing toward the robot (-x), shut; it opens counterclockwise seen from above
# to the open stop at 90 degrees, pointing to the robot's right (-y).
VX, VY = 0.40, -0.05
H_LEN, H_W = 0.080, 0.020
HUB_R = 0.020
HANDLE_Z = DECK_H + 0.007 + H_W / 2  # handle centre above the table: 57 mm
OPEN = math.pi / 2
FRICTION_TORQUE = 12.0  # N m the stem's packing resists with, moving or still
STIFF = 200.0  # N m/rad: elastic give before the stem slips (FRICTION_TORQUE / STIFF = 3.4 degrees)

HANDLE = parts.Hinge(
    "handle", pivot=(0.0, 0.0, HANDLE_Z), axis=(0.0, 0.0, 1.0), lower=0.0, upper=OPEN, mass=0.2,
    spring=STIFF, damping=0.05, max_effort=1.1 * FRICTION_TORQUE,
    shapes=[parts.box((H_LEN - 0.005, H_W, H_W), at=(-(H_LEN + 0.005) / 2, 0.0, HANDLE_Z), color=RED),
            parts.cylinder(HUB_R, H_W, at=(0.0, 0.0, HANDLE_Z), color=RED)])
MARK_OFF, MARK_R0, MARK_R1 = 0.040, 0.050, 0.130  # 10 mm lines on the deck, 40 mm beside each handle position
VALVE = parts.Part("valve", pos=(VX, VY, TOP), shapes=[
    parts.cylinder(0.030, 0.006, at=(0.0, 0.0, DECK_H + 0.003), color=BRASS),  # gland ring on the deck
    # yellow line parallel to the open position, on its far side (+x); white line beside the shut position (+y)
    parts.box((0.010, MARK_R1 - MARK_R0, 0.001), at=(MARK_OFF, -(MARK_R0 + MARK_R1) / 2, DECK_H + 0.0005),
              color=YELLOW),
    parts.box((MARK_R1 - MARK_R0, 0.010, 0.001), at=(-(MARK_R0 + MARK_R1) / 2, MARK_OFF, DECK_H + 0.0005),
              color=WHITE),
], joints=[HANDLE])

# Cheater tube: square aluminium tube 350 mm long, 36 mm outside, 3 mm wall (bore 30 mm: 10 mm total play round the
# 20 mm handle, 5 mm each side).
P_LEN, P_OUT, P_WALL = 0.350, 0.036, 0.003
P_MASS = 0.4
_b = P_OUT - P_WALL
PIPE_SHAPES = [
    parts.box((P_LEN, P_OUT, P_WALL), at=(0.0, 0.0, P_WALL / 2), color=STEEL),
    parts.box((P_LEN, P_OUT, P_WALL), at=(0.0, 0.0, P_OUT - P_WALL / 2), color=STEEL),
    parts.box((P_LEN, P_WALL, P_OUT - 2 * P_WALL), at=(0.0, (P_OUT - P_WALL) / 2, P_OUT / 2), color=STEEL),
    parts.box((P_LEN, P_WALL, P_OUT - 2 * P_WALL), at=(0.0, -(P_OUT - P_WALL) / 2, P_OUT / 2), color=STEEL),
]
PIPE_A = (0.27, 0.10, TOP + DECK_H)  # part origin: middle of the tube's underside
PIPE = parts.Part("tube", pos=PIPE_A, mass=P_MASS, shapes=PIPE_SHAPES, friction=(1.0, 0.9))

OBJECTS = [DECK, VALVE, PIPE]

# Lot A (the screenshot): tube along x to the left of the valve. Lot B: on the valve's right, along x.
# Lot C: nearer the robot, turned to run left to right.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", arm="left", place={"tube": PIPE_A}),
    lots.Lot("B", arm="right", place={"tube": (0.24, -0.25, TOP + DECK_H)}),
    lots.Lot("C", arm="left", place={"tube": (0.17, 0.05, TOP + DECK_H)}, yaw={"tube": math.pi / 2}),
)

EPS = 1e-6
OPEN_TOL = math.radians(5.0)
CATCH = math.radians(4.0)  # within 4 degrees of the stop the handle is drawn onto it and stays
HALF = math.radians(45.0)
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the handle or the tube


# -- the stem's packing: holds the handle where it is left, gives way only above FRICTION_TORQUE -----------------------
_fr = {"target": 0.0}
# Released: an on_step callback measures, every step, how close both arms' fingers and grippers are to the handle and
# the tube. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


@parts.on_reset
def _reset_friction(sim) -> None:
    _fr["target"] = 0.0
    sim.set_target("valve", "handle", 0.0)


@parts.on_step
def _packing(sim) -> None:
    """Static and sliding friction at the stem: the drive holds the handle at a rest point that is dragged along only
    when the handle is pushed more than FRICTION_TORQUE / STIFF past it. At the open stop it stays."""
    st = sim.states()
    q = float(st["valve"].joints["handle"])
    band = FRICTION_TORQUE / STIFF
    t = _fr["target"]
    if q > t + band:
        t = q - band
    elif q < t - band:
        t = q + band
    if q >= OPEN - CATCH:
        t = OPEN
    t = min(max(t, 0.0), OPEN)
    if t != _fr["target"]:
        _fr["target"] = t
        sim.set_target("valve", "handle", t)
    _measure_grip(sim, st)


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _measure_grip(sim, st) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    v, tube = st["valve"], st["tube"]
    hp, hq = v.links["handle"]
    boxes = VALVE.world_boxes({"handle": (np.asarray(hp), _rot(hq))})
    boxes += PIPE.world_boxes({"base": (np.asarray(tube.pos), _rot(tube.quat_xyzw))})
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)


def opened(objects) -> float:
    """How far the handle has turned from shut toward open (rad, counterclockwise seen from above)."""
    return float(objects["valve"].joints["handle"])


def _released(objects) -> bool:
    return bool(_grip["clear"] and objects["tube"].resting())


def success(objects):
    """Handle within 5 degrees of the open stop, both grippers clear of handle and tube, tube still."""
    return bool(opened(objects) >= OPEN - OPEN_TOL - EPS and _released(objects))


def distance(objects):
    return max(0.0, OPEN - opened(objects))


def score(objects):
    """1.0 open (within 5 degrees of the stop); 0.5 turned at least 45 degrees; both released and still. Else 0."""
    if not _released(objects):
        return 0.0
    a = opened(objects)
    if a >= OPEN - OPEN_TOL - EPS:
        return 1.0
    return 0.5 if a >= HALF - EPS else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# One arm picks the tube where the cameras show it, sets it down in line with the handle in front of it and slides
# it on until the hub stops it. The right arm then grips the tube 260 mm from the axis and swings it round to the
# open stop, twisting its wrist with the tube, and lets go.
TIP = 0.013
DECK_TOP = TOP + DECK_H
GRIP_Z = DECK_TOP + 0.004 + TIP  # finger tips 4 mm above the deck, beside the tube
HI = TOP + 0.13
ON_TUBE = -0.014  # pads 28 mm apart: 4 mm into the 36 mm tube
MOUTH_STOP = HUB_R - 0.005  # commanded slide end: the mouth aimed 15 mm from the axis; the gland ring stops it at 30 mm
SWING_R = 0.30
STROKE2_FROM, STROKE2_TO, STROKE2_R = math.radians(68.0), math.radians(110.0), 0.25
PLAY = math.radians(10.0)  # the tube turns about this much on the handle before it bears
RESTAGE = (0.24, 0.08)  # lot C: where the turned tube is set down to be gripped square
SWING_END = math.radians(102.0)  # commanded past the stop: the soft arm lags under load
TWIST = 0.70  # extra wrist twist (rad) while swinging
LAG = math.radians(12.0)
MAX_YAW = 1.9  # the right wrist's reach here  # the loaded arm trails its commanded arc by about this much


def _w(x, y, z):
    return (float(x), float(y), float(z))


def _pick_and_fit(lot):
    px, py, _ = lot.pos("tube")
    along_y = abs(math.sin(lot.yaw_of("tube"))) > 0.5
    # pads close across the tube: yaw 0 closes them along y, right for a tube along x
    gyaw = 0.0 if not along_y else (-math.pi / 2 if lot.arm == "left" else math.pi / 2)
    start_cx = VX - H_LEN - 0.025 - P_LEN / 2  # tube mouth 25 mm short of the handle's end
    end_cx = VX - MOUTH_STOP + 0.005 - P_LEN / 2
    w = [
        Waypoint(1.5, _w(px, py, HI), FINGER_OPEN, "over the tube", gyaw),
        Waypoint(1.5, _w(px, py, GRIP_Z), FINGER_OPEN, "down", gyaw),
        Waypoint(0.8, _w(px, py, GRIP_Z), ON_TUBE, "grip", gyaw),
        Waypoint(1.0, _w(px, py, GRIP_Z + 0.02), ON_TUBE, "lift", gyaw),
    ]
    if along_y:
        # turn it to run front to back, set it down and take a fresh square grip
        sx, sy = RESTAGE
        w += [
            Waypoint(2.0, _w(sx, sy, GRIP_Z + 0.02), ON_TUBE, "turn it front to back"),
            Waypoint(0.8, _w(sx, sy, GRIP_Z + 0.006), ON_TUBE, "set it down"),
            Waypoint(0.6, _w(sx, sy, GRIP_Z + 0.006), FINGER_OPEN, "let go"),
            Waypoint(0.8, _w(sx, sy, GRIP_Z), FINGER_OPEN, "down beside it"),
            Waypoint(0.8, _w(sx, sy, GRIP_Z), ON_TUBE, "grip it again"),
            Waypoint(0.8, _w(sx, sy, GRIP_Z + 0.02), ON_TUBE, "lift"),
        ]
    return w + [
        Waypoint(2.5, _w(start_cx, VY, GRIP_Z + 0.02), ON_TUBE, "carry in line with the handle"),
        Waypoint(1.0, _w(start_cx, VY, GRIP_Z + 0.003), ON_TUBE, "lower to the deck"),
        Waypoint(2.0, _w(end_cx, VY, GRIP_Z + 0.003), ON_TUBE, "slide it over the handle"),
        Waypoint(0.6, _w(end_cx, VY, GRIP_Z + 0.003), FINGER_OPEN, "let go"),
        Waypoint(1.0, _w(end_cx, VY, HI), FINGER_OPEN, "up"),
    ]


def _swing():
    def at(a, r=SWING_R):
        # handle angle a: pointing from the axis at world angle 180 + a degrees
        return _w(VX - r * math.cos(a), VY - r * math.sin(a), GRIP_Z + 0.003)

    w = [
        Waypoint(1.5, (*at(0.0)[:2], HI), FINGER_OPEN, "over the tube's far part"),
        Waypoint(1.2, at(0.0), FINGER_OPEN, "down"),
        Waypoint(0.8, at(0.0), FINGER_CLOSED, "grip"),
    ]
    n = 16
    for i in range(1, n + 1):
        a = SWING_END * i / n
        k = min(1.0, 3 * i / n)
        w.append(Waypoint(0.55, at(a), FINGER_CLOSED, "swing toward open", min(a + k * (TWIST - LAG), MAX_YAW)))
    a_end = min(SWING_END + TWIST - LAG, MAX_YAW)
    # Second stroke: the stem holds the handle where the first stroke left it (about 70 degrees), so the arm lets
    # go, takes a fresh grip nearer the axis and swings again, commanding well past the stop.
    g2 = STROKE2_FROM
    w += [
        Waypoint(0.6, at(SWING_END), FINGER_OPEN, "let go", a_end),
        Waypoint(0.8, (*at(SWING_END)[:2], GRIP_Z + 0.03), FINGER_OPEN, "up off the tube", a_end),
        Waypoint(1.5, (*at(g2, STROKE2_R)[:2], GRIP_Z + 0.03), FINGER_OPEN, "over the tube again", g2 + PLAY),
        Waypoint(0.8, at(g2, STROKE2_R), FINGER_OPEN, "down", g2 + PLAY),
        Waypoint(0.8, at(g2, STROKE2_R), FINGER_CLOSED, "grip", g2 + PLAY),
    ]
    n = 8
    for i in range(1, n + 1):
        a = g2 + (STROKE2_TO - g2) * i / n
        w.append(Waypoint(0.6, at(a, STROKE2_R), FINGER_CLOSED, "second stroke to the stop",
                          min(a + PLAY + TWIST * i / n, MAX_YAW)))
    a2 = min(STROKE2_TO + PLAY + TWIST, MAX_YAW)
    w += [
        Waypoint(1.5, at(STROKE2_TO, STROKE2_R), FINGER_CLOSED, "hold at the stop", a2),
        Waypoint(0.6, at(STROKE2_TO, STROKE2_R), FINGER_OPEN, "let go", a2),
        Waypoint(1.0, (*at(STROKE2_TO, STROKE2_R)[:2], GRIP_Z + 0.03), FINGER_OPEN, "up off the tube", a2),
        Waypoint(1.0, (*at(OPEN, 0.20)[:2], GRIP_Z + 0.06), FINGER_OPEN, "back and up", OPEN),
        Waypoint(1.5, _w(0.25, -0.30, HI), FINGER_OPEN, "away"),
        Waypoint(3.0, _w(0.25, -0.30, HI), FINGER_OPEN, "wait"),
    ]
    return w


def demo():
    lot = lots.current()
    fit = _pick_and_fit(lot)
    t_fit = sum(w.duration for w in fit)
    if lot.arm == "left":
        left = fit + [Waypoint(1.0, _w(0.22, 0.30, HI), FINGER_OPEN, "park")]
        right = [Waypoint(t_fit, _w(0.25, -0.30, HI), FINGER_OPEN, "wait")] + _swing()
    else:
        left = [Waypoint(1.0, _w(0.22, 0.30, HI), FINGER_OPEN, "park")]
        right = fit + _swing()
    return {"left": left, "right": right}
