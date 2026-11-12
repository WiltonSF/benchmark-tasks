"""On the station table, in front of the robot: an articulated light arm on a weighted base plate. Its two
silver segments run left to right, meeting at a black elbow housing; a small black LED panel light hangs off the
far end on the right, on the arm's end joint. On top of the elbow sits the red clamp lever of the arm's clamp
screw, on a square drive under a round cap. Which way the lever points at the start changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so on_reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SILVER, BLACK, DARK, RED, STEEL, WHITE = (
    (0.74, 0.75, 0.77), (0.07, 0.07, 0.08), (0.18, 0.18, 0.20), (0.80, 0.08, 0.06), (0.55, 0.56, 0.58),
    (0.92, 0.92, 0.90))

ELBOW_XY = (0.27, 0.0)  # the clamp screw's axis (world, m)

# Heights above the table top (m), in the arm's frame (origin on the table under the screw axis).
TUBE_Z, TUBE_R = 0.060, 0.009
HOUSING_R, HOUSING_LO, HOUSING_HI = 0.018, 0.040, 0.080
FLANGE_R, FLANGE_HI = 0.013, 0.083  # the screw's flange, 80 to 83 mm: the lever's seat
HEAD = 0.010  # square drive 10 x 10 mm, 83 to 90 mm
HEAD_HI = 0.090
NECK_R, CAP_LO, CAP_HI, CAP_R = 0.0055, 0.112, 0.115, 0.010
SEAT_Z = FLANGE_HI  # the lever's underside when seated
PIVOT = (0.0, -0.125, TUBE_Z)  # the end joint; the light turns about an axis along x
# The end joint is held by friction from the clamp screw: up to 0.4 N m with the screw tight, falling linearly to
# nothing once the screw is 12 degrees loose. The light's weight pulls with about 0.07 N m, so it slips once the screw
# is about 10 degrees loose, and falls until the joint's stop.
JOINT_HOLD, SCREW_FREE = 0.4, math.radians(12.0)
LIGHT_MASS, LIGHT_ARM = 0.15, (0.050, 0.045)  # the light's centre: 50 mm out from the end joint's axis, 45 mm up
DROOP = 0.85  # rad: where a slipping joint's rest point is put (about 50 degrees, near its 0.9 rad stop)

ARM = parts.Part(
    "light_arm",
    pos=(*ELBOW_XY, TOP),
    shapes=[
        parts.box((0.090, 0.070, 0.010), at=(0.0, 0.150, 0.005), color=DARK),  # weighted base plate
        parts.cylinder(0.012, 0.045, at=(0.0, 0.150, 0.0325), color=STEEL, axis="z"),  # spigot
        parts.box((0.028, 0.028, 0.030), at=(0.0, 0.150, TUBE_Z), color=BLACK),  # base joint
        parts.cylinder(TUBE_R, 0.118, at=(0.0, 0.077, TUBE_Z), color=SILVER, axis="y"),  # segment to the base
        parts.cylinder(HOUSING_R, HOUSING_HI - HOUSING_LO, at=(0.0, 0.0, (HOUSING_LO + HOUSING_HI) / 2), color=BLACK,
                       axis="z"),  # elbow housing
        parts.cylinder(TUBE_R, 0.096, at=(0.0, -0.065, TUBE_Z), color=SILVER, axis="y"),  # segment to the light
        parts.cylinder(0.011, 0.026, at=PIVOT, color=BLACK, axis="x"),  # end joint housing
    ],
    joints=[
        # The clamp screw: at its tight stop (0); counter-clockwise seen from above loosens it. Its friction is
        # a drive capped at 0.2 N m whose rest point follows the screw once it has been turned (on_step below).
        parts.Hinge(
            "screw",
            shapes=[
                parts.cylinder(FLANGE_R, 0.003, at=(0.0, 0.0, FLANGE_HI - 0.0015), color=STEEL, axis="z"),
                parts.box((HEAD, HEAD, HEAD_HI - FLANGE_HI), at=(0.0, 0.0, (FLANGE_HI + HEAD_HI) / 2), color=STEEL),
                parts.cylinder(NECK_R, CAP_LO - HEAD_HI, at=(0.0, 0.0, (HEAD_HI + CAP_LO) / 2), color=STEEL, axis="z"),
                parts.cylinder(CAP_R, CAP_HI - CAP_LO, at=(0.0, 0.0, (CAP_LO + CAP_HI) / 2), color=BLACK, axis="z"),
            ],
            pivot=(0.0, 0.0, FLANGE_HI), axis=(0.0, 0.0, 1.0), lower=0.0, upper=3.0, mass=0.02,
            spring=40.0, damping=0.02, max_effort=0.2,
        ),
        # The end joint and the light: 150 g, a 95 x 60 x 25 mm panel on a short stem. Locked, a stiff drive
        # holds it; once the screw is loose its rest point drops and the light falls toward the robot's right.
        parts.Hinge(
            "tilt",
            shapes=[
                parts.cylinder(0.006, 0.030, at=(0.0, -0.135, TUBE_Z + 0.020), color=BLACK, axis="z"),  # stem
                parts.box((0.060, 0.095, 0.025), at=(0.0, -0.175, TUBE_Z + 0.045), color=BLACK),  # light body
                parts.box((0.050, 0.085, 0.002), at=(0.0, -0.175, TUBE_Z + 0.0585), color=WHITE),  # top label plate
            ],
            pivot=PIVOT, axis=(1.0, 0.0, 0.0), lower=-0.2, upper=0.9, mass=0.15,
            spring=30.0, damping=0.05, max_effort=0.4,
        ),
    ],
)

# The clamp lever: a zinc die-cast lever, 40 g. Part frame: origin on the screw axis at the lever's underside, x
# along the handle. Its hub is a square socket (12 mm opening, 4 mm walls, 8 mm deep) that fits over the screw's
# 10 mm square drive; lifted more than 7 mm it clears the drive and turns freely on the round neck, and the cap
# stops it about 20 mm up. A spring (on_step below) presses it down onto the drive.
SOCKET_IN, SOCKET_WALL, SOCKET_H = 0.012, 0.004, 0.008
_W = SOCKET_IN / 2 + SOCKET_WALL / 2
HANDLE_LO, HANDLE_HI, HANDLE_W, HANDLE_H = SOCKET_IN / 2 + SOCKET_WALL, 0.056, 0.012, 0.010
LEVER = parts.Part(
    "clamp_lever",
    pos=(*ELBOW_XY, TOP + SEAT_Z),
    mass=0.040,
    shapes=[
        parts.box((SOCKET_WALL, SOCKET_IN + 2 * SOCKET_WALL, SOCKET_H), at=(_W, 0.0, SOCKET_H / 2), color=RED),
        parts.box((SOCKET_WALL, SOCKET_IN + 2 * SOCKET_WALL, SOCKET_H), at=(-_W, 0.0, SOCKET_H / 2), color=RED),
        parts.box((SOCKET_IN, SOCKET_WALL, SOCKET_H), at=(0.0, _W, SOCKET_H / 2), color=RED),
        parts.box((SOCKET_IN, SOCKET_WALL, SOCKET_H), at=(0.0, -_W, SOCKET_H / 2), color=RED),
        parts.box((HANDLE_HI - HANDLE_LO, HANDLE_W, HANDLE_H), at=((HANDLE_LO + HANDLE_HI) / 2, 0.0, HANDLE_H / 2),
                  color=RED),
        parts.cylinder(0.007, HANDLE_H + 0.002, at=(HANDLE_HI, 0.0, (HANDLE_H + 0.002) / 2), color=RED, axis="z"),
    ],
)
SPRING_PRELOAD, SPRING_RATE = 1.0, 20.0  # N, N/m: the lever's return spring

OBJECTS = [ARM, LEVER]

STOW = math.pi / 2  # pointing back along the arm toward its base (world +y)
# Lot A (the screenshot): the lever points away from the robot. Lot B: along the arm toward the light.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"clamp_lever": LEVER.pos}),  # a zero move, so lot A also settles afresh
    lots.Lot("B", yaw={"clamp_lever": -math.pi / 2}),
)

EPS = 1e-6
SEATED = 0.0015  # the lever's underside within 1.5 mm of its seat
LEVEL = math.radians(5.0)
AIM = math.radians(15.0)  # handle within 15 degrees of pointing along the arm toward the base
LIGHT_TURN = math.radians(2.0)
LIGHT_MOVE = 0.003
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the lever or the light
HOLD = 1.0  # s


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def lever_yaw(lever) -> float:
    r = _rot(lever.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def _tilt_of(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _light_centre(arm) -> np.ndarray:
    pos, quat = arm.links["tilt"]
    boxes = ARM.world_boxes({"tilt": (np.asarray(pos), _rot(quat))})
    return boxes[1][1]  # the light body's centre


_LIGHT0 = _light_centre(ARM.state())

# Whole-trial record of the light: its worst turn and move from where it starts. Cleared only by the simulator's
# scene reset (an on_reset callback), never by anything the robot does.
_record = {"turn": 0.0, "move": 0.0}
# In the simulator an on_step callback measures, every step, how close both grippers are to the lever and the light,
# and how long the end state has held. Until it has measured, nothing is credited (fail closed). Offline
# (near_misses.py, no simulator) each case is a state already held for 1 s with the grippers away.
_sim = {"measured": False, "clear": False, "since": None, "t": 0.0, "screw_target": 0.0, "loose": False}


def _offline() -> bool:
    return "isaaclab" not in sys.modules


def _update(objects) -> None:
    arm = objects["light_arm"]
    _record["turn"] = max(_record["turn"], abs(arm.joints.get("tilt", 0.0)))
    _record["move"] = max(_record["move"], float(np.linalg.norm(_light_centre(arm) - _LIGHT0)))


def _stowed(objects) -> bool:
    lever = objects["clamp_lever"]
    return bool(
        abs(float(lever.pos[2]) - (TOP + SEAT_Z)) <= SEATED + EPS
        and _tilt_of(lever) <= LEVEL + EPS
        and abs(_wrap(lever_yaw(lever) - STOW)) <= AIM + EPS
        and lever.resting()
    )


def _light_kept() -> bool:
    return _record["turn"] <= LIGHT_TURN + EPS and _record["move"] <= LIGHT_MOVE + EPS


def _done(objects) -> bool:
    _update(objects)
    if _offline():
        clear = True
    else:
        clear = _sim["measured"] and _sim["clear"]
    return bool(_stowed(objects) and _light_kept() and clear)


def success(objects):
    """The lever is seated on the screw's drive pointing back along the arm, let go and still, and the light has
    not moved over the whole trial."""
    return _done(objects)


def distance(objects):
    return abs(_wrap(lever_yaw(objects["clamp_lever"]) - STOW))


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
    _record.update(turn=0.0, move=0.0)
    _sim.update(measured=False, clear=False, since=None, t=0.0, screw_target=0.0, loose=False)


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
    import torch
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    arm, lever = st["light_arm"], st["clamp_lever"]

    # The lever's return spring: pushes it down along the screw axis, harder the further it is lifted.
    lift = max(0.0, float(lever.pos[2]) - (TOP + SEAT_Z))
    force = SPRING_PRELOAD + SPRING_RATE * lift
    asset = base.scene["clamp_lever"]
    dev = base.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[[0.0, 0.0, -force]]], dtype=torch.float32, device=dev),
        positions=torch.tensor([[[*(np.asarray(lever.pos) + origin)]]], dtype=torch.float32, device=dev),
        body_ids=[0], env_ids=torch.tensor([0], dtype=torch.int32, device=dev), is_global=True,
    )

    # Screw friction: once turned, it stays where it was turned.
    q = float(arm.joints["screw"])
    if abs(q - _sim["screw_target"]) > math.radians(1.0):
        _sim["screw_target"] = q
        sim.set_target("light_arm", "screw", q)
    # The end joint's friction falls as the screw is loosened; once the light's weight beats it, the light slips
    # down. While the friction holds, the joint stays wherever the light is.
    tilt = float(arm.joints["tilt"])
    hold = JOINT_HOLD * max(0.0, 1.0 - q / SCREW_FREE)
    weight = LIGHT_MASS * 9.81 * (LIGHT_ARM[0] * math.cos(tilt) + LIGHT_ARM[1] * math.sin(tilt))
    loose = weight > hold
    if loose != _sim["loose"]:
        sim.set_target("light_arm", "tilt", DROOP if loose else tilt)
        _sim["loose"] = loose

    # Release and hold.
    hulls = _gripper_hulls(base, origin)
    lever_boxes = LEVER.world_boxes({"base": (np.asarray(lever.pos), _rot(lever.quat_xyzw))})
    pos, quat = arm.links["tilt"]
    light_boxes = ARM.world_boxes({"tilt": (np.asarray(pos), _rot(quat))})
    _sim["clear"] = bool(hulls) and all(
        analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in lever_boxes + light_boxes)
    _sim["measured"] = True
    _sim["t"] = float(sim.t)
    _update(st)
    if _stowed(st) and _light_kept() and _sim["clear"]:
        if _sim["since"] is None:
            _sim["since"] = float(sim.t)
    else:
        _sim["since"] = None


# --- demo: one arm per quarter turn ---------------------------------------------------------------------------
GRIP_R = 0.040  # grip the handle 40 mm out from the screw axis
TIP = 0.013  # finger tips below the TCP
PINCH = -0.003  # pads 6 mm apart on the 12 mm handle
WIDE = -0.020
LIFT = {"right": 0.020, "left": 0.030}  # commanded rise; the soft arm gives against the spring (the left more,
# where it reaches across), so the lever rises about 10 mm
TRAVEL = 0.125


def _w(x, y, z):
    return (x, y, TOP + z)


SLOTS = 3  # quarter turns the plan has room for (a lever can need up to three counter-clockwise)
_DEMO = {"plans": None, "filled": True}


def _park(arm):
    cx, cy = ELBOW_XY
    return _w(cx - 0.10, cy + (0.14 if arm == "left" else -0.14), TRAVEL)


def _quarter(a0):
    """One arm's quarter turn of the lever from handle direction a0: (arm, waypoints)."""
    cx, cy = ELBOW_XY
    grip_z = SEAT_Z + HANDLE_H / 2 + TIP - 0.002  # pads across the handle, tips 3 mm under its middle

    def at(phi, z):
        return _w(cx + GRIP_R * math.cos(phi), cy + GRIP_R * math.sin(phi), z)

    # The fingers close across the handle (a parallel grip is the same turned half a turn), so the wrist starts
    # between -90 and +90 degrees and turns a quarter turn with the lever: the right arm when it starts at 0, the left
    # when it starts at -90 (each wrist turns the way it reaches).
    y0 = (a0 + math.pi / 2 + 1e-6) % math.pi - math.pi / 2
    arm = "left" if y0 < -0.1 else "right"
    a1, y1, up = a0 + math.pi / 2, y0 + math.pi / 2, grip_z + LIFT[arm]
    seg = [
        Waypoint(1.5, at(a0, TRAVEL), WIDE, "over the handle", y0),
        Waypoint(1.5, at(a0, grip_z), WIDE, "down round the handle", y0),
        Waypoint(0.8, at(a0, grip_z), PINCH, "pinch", y0),
        Waypoint(1.2, at(a0, up), PINCH, "lift off the drive", y0),
    ]
    for k in range(1, 7):
        seg.append(Waypoint(0.7, at(a0 + k / 6 * math.pi / 2, up), PINCH, "swing", y0 + k / 6 * math.pi / 2))
    seg += [
        Waypoint(1.0, at(a1, grip_z + 0.004), PINCH, "let it down onto the drive", y1),
        Waypoint(0.6, at(a1, grip_z + 0.004), WIDE, "release", y1),
        Waypoint(1.2, at(a1, TRAVEL), WIDE, "up", y1),
        Waypoint(1.0, _park(arm), WIDE, "aside", 0.0),
    ]
    return arm, seg


SEG_LEN = len(_quarter(0.0)[1])


def _fill(start_yaw: float) -> None:
    """Write the quarter turns needed from the observed start direction into the plan's waypoints, in place. Slots
    not needed shrink to a moment's wait aside."""
    plans = _DEMO["plans"]
    quarter = round(start_yaw / (math.pi / 2)) * (math.pi / 2)  # the observed direction, to the nearest quarter turn
    need = round(((STOW - quarter) % (2 * math.pi)) / (math.pi / 2))
    for n in range(SLOTS):
        if n < need:
            arm, seg = _quarter(quarter + n * math.pi / 2)
            other = "right" if arm == "left" else "left"
            fills = {arm: seg, other: [Waypoint(w.duration, _park(other), WIDE, "wait aside", 0.0) for w in seg]}
        else:
            fills = {a: [Waypoint(0.01, _park(a), WIDE, "wait aside", 0.0)] * SEG_LEN for a in plans}
        for a in plans:
            for w, new in zip(plans[a][1 + n * SEG_LEN: 1 + (n + 1) * SEG_LEN], fills[a]):
                w.duration, w.pos, w.finger, w.label, w.yaw = new.duration, new.pos, new.finger, new.label, new.yaw
    _DEMO["filled"] = True


@parts.on_step
def _observe_lever(sim) -> None:
    """At the first control step, take the lever's handle direction from its pose, as the top camera shows it."""
    if not _DEMO["filled"] and _DEMO["plans"] is not None:
        _fill(lever_yaw(sim.states()["clamp_lever"]))


def demo():
    """Both arms first wait aside for 1.5 s. Then, for each quarter turn the lever needs: one arm pinches the handle,
    lifts the lever off the square drive against its spring, swings it a quarter turn counter-clockwise on the neck,
    lets the spring seat it and lets go, while the other arm waits aside. The number of quarter turns and the arm for
    each come from the handle's direction seen at the start (the first control step)."""
    plans = {a: [Waypoint(1.5, _park(a), WIDE, "wait aside", 0.0)] for a in ("left", "right")}
    for a in plans:
        for _ in range(SLOTS):
            plans[a] += [Waypoint(1.5, _park(a), WIDE, "placeholder", 0.0) for _ in range(SEG_LEN)]
        plans[a].append(Waypoint(3.0, _park(a), WIDE, "wait", 0.0))
    _DEMO.update(plans=plans, filled=False)
    if _offline():
        _fill(LEVER.yaw)  # offline checks have no simulator to look at: plan from the declared layout (lot A)
    return plans
