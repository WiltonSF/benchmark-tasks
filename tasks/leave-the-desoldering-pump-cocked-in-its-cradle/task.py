"""On the station table: a manual desoldering pump (a solder sucker) lying on a grey mat in front of the left arm,
its nozzle pointing away from the robot, and, to the right, its empty bench cradle: a grey channel with an end stop
at the far end. The pump's plunger knob is at its back end; its red release button is on top near the nozzle.
Whether the pump starts cocked changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so on_reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLUE, WHITE, RED, BLACK, GREY, DARK = (
    (0.15, 0.35, 0.75), (0.93, 0.93, 0.90), (0.85, 0.08, 0.06), (0.06, 0.06, 0.07), (0.60, 0.62, 0.65),
    (0.30, 0.31, 0.33))

# The pump. Part frame: origin on the barrel's axis at its middle, x toward the nozzle, z up.
BARREL_R, BARREL_HALF = 0.0125, 0.080
NOZZLE_R, NOZZLE_LEN = 0.006, 0.035
CAP_R, CAP_LEN = 0.014, 0.008
KNOB_R, KNOB_LEN = 0.009, 0.020
SHAFT_R = 0.004
STROKE = 0.070  # the plunger's travel from out (released) to in (cocked)
KNOB_IN_X = -(BARREL_HALF + CAP_LEN + KNOB_LEN / 2)  # knob centre when cocked: against the rear cap
KNOB_OUT_X = KNOB_IN_X - STROKE
BUTTON = (0.012, 0.010, 0.006)
BUTTON_X = 0.050  # 30 mm behind the barrel's front end
BUTTON_TRAVEL = 0.004
FIRE = 0.0025  # button pressed this far: the catch lets go
LATCH = 0.067  # plunger pushed this far in: the catch holds it

PLUNGER_SPRING, PLUNGER_OUT, PLUNGER_IN = 50.0, -0.050, 0.080  # N/m; drive targets (m): spring out / caught in
BUTTON_SPRING, BUTTON_UP = 300.0, -0.005

PUMP_START = (0.27, 0.15)
MAT_T = 0.002
PUMP = parts.Part(
    "pump",
    pos=(*PUMP_START, TOP + MAT_T + BARREL_R),
    mass=0.060,
    shapes=[
        parts.cylinder(BARREL_R, 2 * BARREL_HALF, at=(0.0, 0.0, 0.0), color=BLUE, axis="x"),
        parts.cylinder(NOZZLE_R, NOZZLE_LEN, at=(BARREL_HALF + NOZZLE_LEN / 2, 0.0, 0.0), color=WHITE, axis="x"),
        parts.cylinder(CAP_R, CAP_LEN, at=(-BARREL_HALF - CAP_LEN / 2, 0.0, 0.0), color=BLACK, axis="x"),
    ],
    joints=[
        # Plunger: written out (released); pushed in (+x) by up to 70 mm. Its spring pushes it out; the catch is
        # modelled in on_step below by moving the drive's rest point.
        parts.Slider(
            "plunger",
            shapes=[
                parts.cylinder(SHAFT_R, STROKE + 0.004, at=(KNOB_OUT_X + KNOB_LEN / 2 + (STROKE + 0.004) / 2, 0.0, 0.0),
                               color=GREY, axis="x"),
                parts.cylinder(KNOB_R, KNOB_LEN, at=(KNOB_OUT_X, 0.0, 0.0), color=BLACK, axis="x"),
            ],
            axis=(1.0, 0.0, 0.0), travel=STROKE, mass=0.020, spring=PLUNGER_SPRING, damping=1.0, max_effort=20.0,
        ),
        parts.Slider(
            "button",
            shapes=[parts.box(BUTTON, at=(BUTTON_X, 0.0, BARREL_R + BUTTON[2] / 2 - 0.001), color=RED)],
            axis=(0.0, 0.0, -1.0), travel=BUTTON_TRAVEL, mass=0.010, spring=BUTTON_SPRING, damping=0.5,
        ),
    ],
)
MAT = parts.Part("mat", pos=(0.27, 0.15, TOP), shapes=[parts.box((0.30, 0.10, MAT_T), at=(0.0, 0.0, MAT_T / 2),
                                                                  color=DARK)])

# The cradle (fixed, grey ABS). Part frame: origin on the table at the end stop's inner face, on the channel's
# centre line; the channel runs back (-x) toward the robot.
CRADLE_XY = (0.40, -0.04)
BASE_T, BASE_LEN, BASE_W = 0.010, 0.175, 0.050
CHANNEL_W, RAIL_W, RAIL_H, RAIL_LEN = 0.030, 0.008, 0.008, 0.160
STOP_T, STOP_H, NOTCH = 0.008, 0.032, 0.016
CRADLE = parts.Part(
    "cradle",
    pos=(*CRADLE_XY, TOP),
    shapes=[
        parts.box((BASE_LEN, BASE_W, BASE_T), at=(-BASE_LEN / 2, 0.0, BASE_T / 2), color=GREY),
        parts.box((RAIL_LEN, RAIL_W, RAIL_H), at=(-RAIL_LEN / 2, (CHANNEL_W + RAIL_W) / 2, BASE_T + RAIL_H / 2),
                  color=GREY),
        parts.box((RAIL_LEN, RAIL_W, RAIL_H), at=(-RAIL_LEN / 2, -(CHANNEL_W + RAIL_W) / 2, BASE_T + RAIL_H / 2),
                  color=GREY),
        # End stop: two cheeks either side of a 16 mm notch for the nozzle; the barrel's front face bears on them.
        parts.box((STOP_T, (BASE_W - NOTCH) / 2, STOP_H), at=(STOP_T / 2, (NOTCH + (BASE_W - NOTCH) / 2) / 2,
                                                              BASE_T + STOP_H / 2), color=DARK),
        parts.box((STOP_T, (BASE_W - NOTCH) / 2, STOP_H), at=(STOP_T / 2, -(NOTCH + (BASE_W - NOTCH) / 2) / 2,
                                                              BASE_T + STOP_H / 2), color=DARK),
        parts.box((STOP_T, BASE_W, BASE_T), at=(STOP_T / 2, 0.0, BASE_T / 2), color=GREY),
    ],
)

OBJECTS = [CRADLE, MAT, PUMP]

# Lot A (the screenshot): the pump starts released, its plunger 70 mm out. Lot B: it starts cocked.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"pump": PUMP.pos}, cocked=False),  # zero moves: each lot settles afresh
    lots.Lot("B", place={"pump": PUMP.pos}, cocked=True),
)

EPS = 1e-6
SEATED_Z = 0.002  # the barrel's underside within 2 mm of the channel floor
SIDE = 0.004  # the barrel's axis within 4 mm of the channel's centre line
AXIS = math.radians(8.0)  # the barrel within 8 degrees of the channel, nozzle toward the end stop
TO_STOP = 0.010  # the barrel's front face within 10 mm of the end stop
CLEAR = 0.005
HOLD = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# In the simulator an on_step callback runs the catch, measures how close both grippers are to the pump and times
# the hold. Until it has measured, nothing is credited (fail closed). Offline (near_misses.py, no simulator) a
# plunger pushed fully in counts as caught, and each case is a state already held 1 s with the grippers away.
_sim = {"measured": False, "clear": False, "since": None, "t": 0.0, "latched": False}


def _offline() -> bool:
    return "isaaclab" not in sys.modules


def cocked(pump) -> bool:
    """The plunger in to within 3 mm of its stop (the knob against the rear cap) and the catch holding it."""
    q = float(pump.joints.get("plunger", 0.0))
    caught = (q >= LATCH - EPS) if _offline() else _sim["latched"]
    return bool(caught and q >= LATCH - EPS)


def in_cradle(pump) -> bool:
    r = _rot(pump.quat_xyzw)
    ax = r[:, 0]  # the barrel's axis, toward the nozzle
    if math.acos(float(np.clip(ax[0], -1.0, 1.0))) > AXIS + EPS:  # along +x (toward the end stop)
        return False
    cx, cy = CRADLE_XY
    p = np.asarray(pump.pos, float)
    under = p[2] - BARREL_R * math.sqrt(max(0.0, 1.0 - ax[2] ** 2))  # lowest point of the barrel's middle
    front = p[0] + BARREL_HALF * ax[0]
    return bool(
        abs(p[1] - cy) <= SIDE + EPS
        and abs(under - (TOP + BASE_T)) <= SEATED_Z + EPS
        and cx - TO_STOP - EPS <= front <= cx + 0.002
    )


def _done(objects) -> bool:
    pump = objects["pump"]
    clear = True if _offline() else (_sim["measured"] and _sim["clear"])
    return bool(cocked(pump) and in_cradle(pump) and pump.resting() and clear)


def success(objects):
    """The pump lies in its cradle, nozzle to the end stop, cocked, let go and still."""
    return _done(objects)


def distance(objects):
    cx, cy = CRADLE_XY
    return float(np.linalg.norm(np.asarray(objects["pump"].pos[:2]) - (cx - BARREL_HALF, cy)))


def score(objects):
    """Pass/fail: 1.0 once the success state has held for 1 s, otherwise 0."""
    if not _done(objects):
        return 0.0
    if _offline():
        return 1.0
    since = _sim["since"]
    return 1.0 if since is not None and _sim["t"] - since >= HOLD - 1e-6 else 0.0


# --- simulator callbacks ---------------------------------------------------------------------------------------


@parts.on_reset
def _new_trial(sim) -> None:
    """Set the trial's start state: lot B starts cocked (plunger caught in), lot A released."""
    start_cocked = bool(lots.current().cocked)
    _sim.update(measured=False, clear=False, since=None, t=0.0, latched=start_cocked)
    sim.set_target("pump", "plunger", PLUNGER_IN if start_cocked else PLUNGER_OUT)
    sim.set_target("pump", "button", BUTTON_UP)


def _gripper_hulls(base, origin):
    from yam_arena import analysis

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


@parts.on_step
def _physics(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    pump = st["pump"]
    q, b = float(pump.joints["plunger"]), float(pump.joints["button"])

    # The catch: it holds the plunger once pushed fully in; the button releases it and the spring shoots it out.
    if _sim["latched"] and b > FIRE:
        _sim["latched"] = False
        sim.set_target("pump", "plunger", PLUNGER_OUT)
    elif not _sim["latched"] and q >= LATCH and b <= FIRE:
        _sim["latched"] = True
        sim.set_target("pump", "plunger", PLUNGER_IN)

    hulls = _gripper_hulls(base, origin)
    poses = {"base": (np.asarray(pump.pos), _rot(pump.quat_xyzw))}
    poses |= {k: (np.asarray(p), _rot(qq)) for k, (p, qq) in pump.links.items()}
    boxes = PUMP.world_boxes(poses)
    _sim["clear"] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)
    _sim["measured"] = True
    _sim["t"] = float(sim.t)
    if cocked(pump) and in_cradle(pump) and pump.resting() and _sim["clear"]:
        if _sim["since"] is None:
            _sim["since"] = float(sim.t)
    else:
        _sim["since"] = None


# --- demo ----------------------------------------------------------------------------------------------------------
TIP = 0.013
WIDE = -0.024  # pads 48 mm apart round the 25 mm barrel
PINCH = -0.010  # pads 20 mm apart: 2.5 mm past each side of the barrel
TRAVEL = 0.10
PUSH_OVER = 0.030  # the soft arm gives about 3 cm against the spring at full stroke


def _w(x, y, z):
    return (x, y, TOP + z)


def _park(arm):
    return _w(0.18, 0.20 if arm == "left" else -0.20, TRAVEL)


def _push():
    """Right arm: closed fingers behind the knob, push it forward to the catch and back off. On a pump already
    cocked the push meets the knob at the rear cap and only presses the pump against the end stop."""
    cx, cy = CRADLE_XY
    bx = cx - BARREL_HALF  # the barrel's middle once in the cradle
    kz = BASE_T + BARREL_R  # the knob's centre height
    tz = kz + TIP - 0.004  # tips 4 mm under the knob's centre
    back_out = bx + KNOB_OUT_X - KNOB_LEN / 2  # the knob's back face, released
    back_in = bx + KNOB_IN_X - KNOB_LEN / 2
    return [
        Waypoint(1.5, _w(back_out - 0.03, cy, TRAVEL), FINGER_CLOSED, "behind the knob", 0.0),
        Waypoint(1.5, _w(back_out - 0.03, cy, tz), FINGER_CLOSED, "down behind the knob", 0.0),
        Waypoint(1.0, _w(back_out - 0.008, cy, tz), FINGER_CLOSED, "up to where a released knob stands", 0.0),
        Waypoint(3.0, _w(back_in - 0.008 + PUSH_OVER, cy, tz), FINGER_CLOSED, "push the plunger in", 0.0),
        Waypoint(0.8, _w(back_in - 0.008 + PUSH_OVER, cy, tz), FINGER_CLOSED, "hold", 0.0),
        Waypoint(1.0, _w(back_in - 0.04, cy, tz), FINGER_CLOSED, "back off", 0.0),
        Waypoint(1.2, _w(back_in - 0.04, cy, TRAVEL), FINGER_CLOSED, "up", 0.0),
        Waypoint(1.0, _park("right"), FINGER_CLOSED, "aside", 0.0),
    ]


def _carry():
    """Left arm: pinch the barrel at its middle (clear of the button), carry the pump into the cradle, nozzle to the
    end stop, let go."""
    px, py = PUMP_START
    cx, cy = CRADLE_XY
    gz = MAT_T + BARREL_R + TIP - 0.006  # tips 6 mm under the barrel's axis
    place_x = cx - BARREL_HALF - 0.003  # front face 3 mm short of the stop
    lz = BASE_T + BARREL_R + TIP - 0.006 + 0.006  # released 6 mm above the channel floor
    return [
        Waypoint(1.5, _w(px, py, TRAVEL), WIDE, "over the barrel", 0.0),
        Waypoint(1.5, _w(px, py, gz), WIDE, "down round the barrel", 0.0),
        Waypoint(0.8, _w(px, py, gz), PINCH, "pinch", 0.0),
        Waypoint(1.2, _w(px, py, TRAVEL), PINCH, "lift", 0.0),
        Waypoint(2.0, _w(place_x, cy, TRAVEL), PINCH, "carry over the cradle", 0.0),
        Waypoint(1.5, _w(place_x, cy, lz), PINCH, "lower into the channel", 0.0),
        Waypoint(0.6, _w(place_x, cy, lz), WIDE, "release", 0.0),
        Waypoint(1.2, _w(place_x, cy, TRAVEL), WIDE, "up", 0.0),
        Waypoint(1.0, _park("left"), WIDE, "aside", 0.0),
    ]


def demo():
    """Open loop from the published layout, the same in both lots: the left arm puts the pump in its cradle, then the
    right arm pushes the plunger knob forward until the catch holds; the arm not working waits aside."""
    carry, push = _carry(), _push()
    return {
        "left": carry + [Waypoint(sum(w.duration for w in push) + 3.0, _park("left"), WIDE, "wait aside", 0.0)],
        "right": [Waypoint(sum(w.duration for w in carry), _park("right"), WIDE, "wait aside", 0.0)] + push
        + [Waypoint(3.0, _park("right"), WIDE, "wait", 0.0)],
    }
