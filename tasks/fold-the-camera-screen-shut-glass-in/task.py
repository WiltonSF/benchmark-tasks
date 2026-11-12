"""On the station table, a black mirrorless camera stands on its base, either with its lens pointing away from the
robot or turned round with its lens toward the robot; which way it faces changes between trials. Its rear screen is
on a two-axis vari-angle mount: a side hinge on the camera's left edge swings the screen out to the side, and a
swivel turns the screen about its own width. The screen has a black glass face and a silver back. At the start the
screen is swung fully out to the camera's left and lies flat.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLACK, DARK, GLASS, SILVER, GREY = (
    (0.10, 0.10, 0.11), (0.20, 0.20, 0.22), (0.03, 0.04, 0.07), (0.78, 0.79, 0.81), (0.45, 0.46, 0.48))

# Camera, in its own frame (m): origin at the middle of the bottom edge of the body's back face, x forward (toward
# the lens), y to the camera's left as seen from behind, z up. The body's back face is the plane x = 0.
BODY = (0.040, 0.116, 0.070)
BODY_SHAPES = [
    parts.box(BODY, at=(BODY[0] / 2, 0.0, BODY[2] / 2), color=BLACK),
    parts.box((0.036, 0.050, 0.018), at=(0.018, 0.0, 0.079), color=BLACK),  # viewfinder hump: 88 mm tall in all
    parts.box((0.020, 0.026, 0.066), at=(0.050, -0.045, 0.033), color=BLACK),  # hand grip, camera's right
    parts.cylinder(0.031, 0.050, at=(0.065, 0.005, 0.038), color=DARK, axis="x"),  # kit lens, 62 mm across
    parts.cylinder(0.028, 0.004, at=(0.092, 0.005, 0.038), color=GREY, axis="x"),  # lens front ring
    parts.cylinder(0.005, 0.004, at=(0.050, -0.045, 0.068), color=GREY, axis="z"),  # shutter button
]

# The screen: 75 mm wide, 55 mm tall, 13 mm thick, glass on its outer face. Folded in the shooting position it lies
# 1 mm off the back face, from y -30 to +45 mm and z 8 to 63 mm. The side hinge (vertical) is 11 mm beyond its left
# edge, 7.5 mm behind the back face; the swivel turns it about its width (through its middle, 35.5 mm up).
SCREEN = (0.013, 0.075, 0.055)
AXIS_X, AXIS_Z = -0.0075, 0.0355
HINGE_Y = 0.056
SCREEN_Y = (-0.030, 0.045)
SCREEN_YC = sum(SCREEN_Y) / 2
FLIP_MAX = math.radians(175.0)  # side hinge: 0 folded against the body, 175 degrees swung out to the side
SWIVEL_MIN, SWIVEL_MAX = math.radians(-90.0), math.radians(180.0)  # 0 glass out (as when shooting), 180 glass in

BRACKET_SHAPES = [
    parts.cylinder(0.004, 0.040, at=(AXIS_X, HINGE_Y, AXIS_Z), color=DARK, axis="z"),  # hinge barrel
    parts.box((0.006, 0.008, 0.008), at=(AXIS_X, 0.049, AXIS_Z), color=DARK),  # arm to the swivel
]
SCREEN_SHAPES = [
    parts.box((0.011, SCREEN[1], SCREEN[2]), at=(AXIS_X, SCREEN_YC, AXIS_Z), color=DARK),  # frame, x -13 to -2 mm
    parts.box((0.001, 0.069, 0.049), at=(AXIS_X - 0.006, SCREEN_YC, AXIS_Z), color=GLASS),  # glass, outer face
    parts.box((0.001, 0.071, 0.051), at=(AXIS_X + 0.006, SCREEN_YC, AXIS_Z), color=SILVER),  # back, inner face
]
GLASS_NORMAL = np.array([-1.0, 0.0, 0.0])  # the glass faces out of the camera's back when folded at swivel 0

# Friction hinges: each joint's spring (N m/rad) pulls toward a rest point that follows the joint, so it holds
# below the breakaway torque and slips at it (side hinge 0.034 N m, swivel 0.011 N m).
FLIP_K, FLIP_SLIP = 1.5, math.radians(1.3)
SWIVEL_K, SWIVEL_SLIP = 0.5, math.radians(1.2)
CLICK = math.radians(4.0)  # within 4 degrees of folded, the side hinge clicks shut

CAMERA_XY = (0.36, 0.04)
CAMERA_MASS = 0.60  # body and lens; the screen and its bracket add 0.05 kg
CAMERA = parts.Part(
    "camera",
    pos=(*CAMERA_XY, TOP),
    mass=CAMERA_MASS,
    shapes=BODY_SHAPES,
    joints=[
        parts.Hinge("flip", shapes=BRACKET_SHAPES, pivot=(AXIS_X, HINGE_Y, AXIS_Z), axis=(0.0, 0.0, -1.0),
                    lower=0.0, upper=FLIP_MAX, mass=0.010, spring=FLIP_K, damping=0.01),
        parts.Hinge("swivel", shapes=SCREEN_SHAPES, pivot=(AXIS_X, 0.049, AXIS_Z), axis=(0.0, 1.0, 0.0),
                    lower=SWIVEL_MIN, upper=SWIVEL_MAX, mass=0.040, spring=SWIVEL_K, damping=0.002, parent="flip"),
    ],
)
parts.SELF_COLLIDE = False  # the housing is enforced through the joint limits (see _interlock)
OBJECTS = [CAMERA]

# Both lots: the screen swung out to its stop, lying flat with its glass up. Lot A (the screenshot): lens pointing
# away from the robot, screen on the robot's left. Lot B: the camera turned round, lens toward the robot, standing
# 20 mm further out and on the other side of the midline, so the screen lies on the robot's right.
START_FLIP = FLIP_MAX
START_SWIVEL = math.radians(90.0)
LOT_B_XY = (0.38, -0.04)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", camera=(*CAMERA_XY, 0.0)),  # x, y of the body's back-bottom centre (m), yaw (rad)
    lots.Lot("B", camera=(*LOT_B_XY, math.pi)),
)

# Housing interference: the screen can turn on its swivel only when it is swung out at least 75 degrees; it can
# fold in past 75 degrees only when it is square to the body (swivel within 8 degrees of 0 or 180).
SWIVEL_FREE = math.radians(75.0)
SQUARE = math.radians(8.0)

EPS = 1e-6
FOLDED = math.radians(3.0)  # folded flush: side hinge within 3 degrees of shut
GLASS_IN = math.radians(8.0)  # glass in: swivel within 8 degrees of 180
UPRIGHT = math.radians(5.0)  # the camera's base within 5 degrees of level
ON_TABLE = 0.003  # the camera's lowest point within 3 mm of the table
STILL = 0.05  # rad/s, each joint
CLEAR = 0.005  # released: no part of either arm (fingers, gripper, wrist, links) within 5 mm of the camera
HOLD = 1.0


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def glass_normal(cam) -> np.ndarray:
    """The world direction the glass faces."""
    if "swivel" not in cam.links:
        return _rot(cam.quat_xyzw) @ GLASS_NORMAL
    return _rot(cam.links["swivel"][1]) @ GLASS_NORMAL


def folded(cam) -> bool:
    return float(cam.joints["flip"]) <= FOLDED + EPS


def glass_in(cam) -> bool:
    return float(cam.joints["swivel"]) >= SWIVEL_MAX - GLASS_IN - EPS


def upright(cam) -> bool:
    tilt = math.acos(float(np.clip(_rot(cam.quat_xyzw)[2, 2], -1.0, 1.0)))
    return bool(tilt <= UPRIGHT + EPS and abs(float(cam.low[2]) - TOP) <= ON_TABLE + EPS)


# -- scoring --------------------------------------------------------------------------------------------------------
# In the simulator an on_step callback measures, every step, how close both grippers are to the camera, how fast the
# hinges turn and how long the end state has held. Until it has measured, nothing is credited (fail closed). Offline
# (near_misses.py sets parts.OFFLINE_CASES; no simulator) each case stands for a state already held 1 s, let go and still.
_sim = {"measured": False, "clear": False, "still": False, "since": None, "tier": 0.0, "t": 0.0,
        "targets": {}, "limits": None}


def _offline() -> bool:
    """True only for near_misses.py's offline cases: the flag it sets, and no simulator scene built in this process."""
    return bool(getattr(parts, "OFFLINE_CASES", False) and not parts._SPECS)


def _tier(objects) -> float:
    """The end state's score, before the release and hold checks."""
    cam = objects["camera"]
    if not upright(cam) or not glass_in(cam):
        return 0.0
    return 1.0 if folded(cam) else 0.5


def _let_go(objects) -> bool:
    if _offline():
        return bool(objects["camera"].resting())
    return bool(_sim["measured"] and _sim["clear"] and _sim["still"] and objects["camera"].resting())


def _held(objects, tier) -> bool:
    if tier <= 0.0 or not _let_go(objects):
        return False
    if _offline():
        return True
    since = _sim["since"]
    return bool(since is not None and _sim["tier"] == tier and _sim["t"] - since >= HOLD - 1e-6)


def success(objects):
    """The screen is folded flush with its glass facing the body, let go and still."""
    return bool(_tier(objects) == 1.0 and _let_go(objects))


def distance(objects):
    cam = objects["camera"]
    return float(abs(SWIVEL_MAX - cam.joints["swivel"]) + cam.joints["flip"])


def score(objects):
    """1.0 folded flush glass in; 0.5 turned glass in but not folded flush; each let go and held for 1 s. Else 0."""
    tier = _tier(objects)
    return tier if _held(objects, tier) else 0.0


# -- simulator hooks ------------------------------------------------------------------------------------------------


def _write(sim, names, values, limits=None):
    import torch

    base = sim.env.unwrapped
    cam = base.scene["camera"]
    ids = cam.find_joints(list(names), preserve_order=True)[0]
    dev = base.device
    if limits is not None:
        lim = torch.tensor([limits], dtype=torch.float32, device=dev)
        cam.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)
    if values is not None:
        cam.write_joint_position_to_sim_index(position=torch.tensor([values], device=dev), joint_ids=ids)
        cam.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, len(ids)), device=dev), joint_ids=ids)


def _interlock(sim, flip, swivel) -> None:
    """The housing: below 75 degrees out the swivel is held where it is; above it, the side hinge cannot fold in
    past 75 degrees unless the screen is square."""
    if flip < SWIVEL_FREE:
        lo, hi = max(SWIVEL_MIN, swivel - math.radians(0.5)), min(SWIVEL_MAX, swivel + math.radians(0.5))
        flip_lo = 0.0
    else:
        lo, hi = SWIVEL_MIN, SWIVEL_MAX
        square = min(abs(swivel), abs(SWIVEL_MAX - swivel)) <= SQUARE
        flip_lo = 0.0 if square else min(SWIVEL_FREE, flip)
    key = (round(lo, 4), round(hi, 4), round(flip_lo, 4))
    if key == _sim["limits"]:
        return
    if _sim["limits"] is not None and flip < SWIVEL_FREE and _sim["limits"][2] == 0.0 and flip_lo == 0.0 \
            and _sim["limits"][1] - _sim["limits"][0] < math.radians(1.5):
        return  # already held: keep the first hold rather than let it creep
    _sim["limits"] = key
    _write(sim, ("flip", "swivel"), None, limits=[[flip_lo, FLIP_MAX], [lo, hi]])


def _friction(sim, name, q, slip, shut=None) -> None:
    t = _sim["targets"].get(name, q)
    if shut is not None and q <= CLICK:
        t = shut
    elif q - t > slip:
        t = q - slip
    elif t - q > slip:
        t = q + slip
    if _sim["targets"].get(name) != t:
        _sim["targets"][name] = t
        sim.set_target("camera", name, t)


@parts.after_first_step
def _place_body(sim) -> None:
    """Stand the camera's body where this lot puts it, still. (Setting the joints of a loose articulation keeps its
    centre of mass where it was once the simulator steps, so the swung-out screen would otherwise shift the body about
    6 mm. This runs after each of the first two settle steps.)"""
    import torch

    base = sim.env.unwrapped
    cam = base.scene["camera"]
    dev = base.device
    x, y, yaw = lots.current().camera
    pose = torch.tensor([[x, y, TOP, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]], dtype=torch.float32, device=dev)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    cam.write_root_pose_to_sim_index(root_pose=pose)
    cam.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))


@parts.on_reset
def _new_trial(sim) -> bool:
    """Set the start: swung out to the stop, lying flat, glass up. (_place_body then stands the camera for the lot.)"""
    swivel = START_SWIVEL
    _sim.update(measured=False, clear=False, still=False, since=None, tier=0.0, t=0.0, targets={}, limits=None)
    _write(sim, ("flip", "swivel"), [START_FLIP, swivel], limits=[[SWIVEL_FREE, FLIP_MAX],
                                                                       [SWIVEL_MIN, SWIVEL_MAX]])
    _sim["limits"] = None
    for name, q in (("flip", START_FLIP), ("swivel", swivel)):
        _sim["targets"][name] = q
        sim.set_target("camera", name, q)
    return True  # settle and take a fresh first frame


@parts.on_step
def _physics(sim) -> None:
    from yam_arena import analysis

    st = sim.states()
    cam = st["camera"]
    flip, swivel = float(cam.joints["flip"]), float(cam.joints["swivel"])
    _interlock(sim, flip, swivel)
    _friction(sim, "flip", flip, FLIP_SLIP, shut=-math.radians(2.0))
    _friction(sim, "swivel", swivel, SWIVEL_SLIP)
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    qd = parts._np(base.scene["camera"].data.joint_vel)[0]
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n not in ("base", "base_link"):  # every moving link: finger tips, gripper, wrist and arm
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    poses = {"base": (np.asarray(cam.pos), _rot(cam.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in cam.links.items()})
    boxes = CAMERA.world_boxes(poses)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for _, c, r, s in boxes)
    _sim["still"] = bool(np.all(np.abs(qd) < STILL))
    _sim["measured"] = True
    _sim["t"] = float(sim.t)
    tier = _tier(st)
    if tier > 0.0 and _let_go(st):
        if _sim["since"] is None or _sim["tier"] != tier:
            _sim["since"], _sim["tier"] = float(sim.t), tier
    else:
        _sim["since"], _sim["tier"] = None, 0.0


# -- the demo -------------------------------------------------------------------------------------------------------
# Open loop: one fixed routine, played from each lot's published start (where the camera stands and which way it
# faces). In both lots it is the same routine in the camera's own frame; lot A uses the left arm, lot B the right.
TIP = 0.013  # finger tips below the TCP
FT = 0.0045  # half the closed fingers' thickness across the push, near the tips (fingers close across the push)
YAW = -math.pi / 2  # with the camera's lens pointing away (lot A): fingers close along x


def _camera_start():
    x, y, yaw = lots.current().camera
    return np.array([x, y, TOP]), yaw


def screen_pose(flip, swivel):
    """World (origin, rotation) of the screen's link frame, the camera standing where this lot publishes it."""
    pos, yaw = _camera_start()
    st = CAMERA.state(joints={"flip": flip, "swivel": swivel}, pos=pos,
                      quat_xyzw=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    p, q = st.links["swivel"]
    return np.asarray(p), _rot(q)


def on_screen(flip, swivel, point):
    """A point given in the camera frame of the folded, shooting-position screen, carried to (flip, swivel)."""
    p, r = screen_pose(flip, swivel)
    pivot = np.asarray(CAMERA.joints[1].pivot)
    return p + r @ (np.asarray(point, float) - pivot), r


def _cam(v):
    """A camera-frame direction in the world."""
    yaw = _camera_start()[1]
    return parts._rot((0.0, 0.0, 1.0), yaw) @ np.asarray(v, float)


def _tcp(tip):
    """The TCP that puts the finger tips' contact point (5 mm above their lowest edge) at `tip`."""
    return (float(tip[0]), float(tip[1]), float(tip[2]) + TIP - 0.005)


def _contact(flip, swivel, point, normal, gap):
    c, r = on_screen(flip, swivel, point)
    return c + (r @ np.asarray(normal, float)) * (FT + gap)


def _deg(a, b, n):
    return np.linspace(math.radians(a), math.radians(b), n)


LEAD = 10.0  # degrees the finger tips lead the screen while turning it
PUSH_Y = SCREEN_Y[1] - 0.030  # the swivel push presses 41 mm out from the side hinge
FOLD_Y = HINGE_Y - 0.050  # the folding push presses 50 mm out from the side hinge
GLASS_HIGH = (AXIS_X - 0.0065, PUSH_Y, AXIS_Z + 0.022)  # glass face, upper half (folded frame), 22 mm off the axis
BACK_FOLD = (AXIS_X + 0.0065, FOLD_Y, AXIS_Z - 0.018)  # back face, lower half, where the folding push presses
OUT, IN = (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)  # outward normals of the glass and the back faces (folded frame)


def demo():
    """One arm, fingers closed: the arm on the screen's side of the table. Press down the half of the flat screen
    that lies on the camera's back side until the glass faces the way the camera's back will, swing the screen round
    on its side hinge until it lies folded against the body, and move away."""
    _, yaw = _camera_start()
    arm = "left" if math.cos(yaw) > 0 else "right"
    grip = YAW + yaw

    def w(p, dur, label):
        return Waypoint(dur, _tcp(p), 0.0, label, grip)

    def sweep(flips, swivels, point, normal, dur, label, gap=-0.002):
        return [w(_contact(fl, sw, point, normal, gap), dur / len(flips), label) for fl, sw in zip(flips, swivels)]

    f, s0, n = START_FLIP, 90.0, 12
    start = _contact(f, math.radians(s0), GLASS_HIGH, OUT, 0.012)
    plan = [w(start + (0, 0, 0.03), 1.5, "over the screen's back-side half"), w(start, 1.0, "down to it")]
    plan += sweep([f] * n, _deg(s0 + LEAD, 150.0 + LEAD, n), GLASS_HIGH, OUT, 0.03 * (150.0 - s0), "press it down")
    out = _contact(f, math.radians(150.0), GLASS_HIGH, OUT, 0.0) + _cam((-0.015, 0.0, 0.0))
    plan += [w(out, 0.6, "back off"), w(out + (0, 0, 0.06), 1.0, "up")]
    # Fold: press the back of the upper half toward the camera's back side, then round onto the body.
    fold = _contact(f, SWIVEL_MAX, BACK_FOLD, IN, 0.025)
    plan += [w(fold + (0, 0, 0.05), 1.5, "beyond the screen"), w(fold, 1.0, "down beside it")]
    plan += [w(_contact(f, SWIVEL_MAX, BACK_FOLD, IN, -0.003), 1.2, "square it against its stop")]
    plan += sweep(_deg(175.0 - 5.0, -4.0, 18), [SWIVEL_MAX] * 18, BACK_FOLD, IN, 6.0, "swing it round onto the body",
                  gap=-0.003)
    end = _contact(0.0, SWIVEL_MAX, BACK_FOLD, IN, 0.025)
    side = end + _cam((0.0, 0.10, 0.0))  # out past the camera's left side, at the same height
    near = np.array([min(float(side[0]), 0.30), side[1], side[2]])  # toward the robot, beside the camera
    away = near + (0.0, 0.0, 0.06)
    plan += [w(end, 1.0, "back off"), w(side, 1.2, "out to the side"), w(near, 1.5, "toward the robot"),
             w(away, 1.0, "up"), w(away, 3.0, "wait")]
    return {arm: plan}
