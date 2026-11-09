"""On the station table, in front of the robot's right arm: a section of a white back door in its frame, facing the
robot. On it, a black lever handle with a white end stands out from the door on its spindle and points to the
robot's right, level. Below it and toward the door's far edge, on its own small plate, a key with a red bow (a white
stripe along it) is in the lock, bow pointing up. A steel bolt sits in the door's far edge, under a keep on the frame.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, FRAME, GREY, DARK = (0.92, 0.92, 0.90), (0.80, 0.80, 0.78), (0.55, 0.57, 0.60), (0.12, 0.12, 0.14)
RED, STEEL = (0.80, 0.12, 0.10), (0.68, 0.70, 0.72)

# Door frame: origin on the table at the middle of the door's face, x away from the robot (into the door), z up.
XD, YD = 0.345, -0.10
LEAF = (0.020, 0.300, 0.170)  # 20 mm thick, 300 wide, 170 tall: the lower part of a door, scaled down
SPINDLE = (0.0, 0.060, 0.120)  # where the handle's spindle leaves the door (y, z in the door frame)
KEY = (0.0, 0.120, 0.062)  # the lock cylinder, 60 mm toward the door's edge from the spindle and below it
LEVER_X = -0.040  # the lever's centre line stands 40 mm out from the door
LEVER_LEN, LEVER_T, LEVER_H = 0.110, 0.012, 0.016
BOW_X = -0.072  # the key's bow stands 72 mm out (a long shank, clear of the lever)
BOW = (0.006, 0.020, 0.045)  # bow: 6 thick, 20 wide, from 5 mm below to 40 mm above the cylinder axis
BOW_TIP = 0.040
HOOK_TRAVEL = 0.030

DOOR = parts.Part(
    "door",
    pos=(XD, YD, TOP),
    shapes=[
        parts.box(LEAF, at=(LEAF[0] / 2, 0.0, LEAF[2] / 2), color=WHITE),
        parts.box((0.040, 0.020, 0.200), at=(0.020, 0.160, 0.100), color=FRAME),  # frame stile, far edge
        parts.box((0.040, 0.030, 0.012), at=(0.000, 0.150, 0.186), color=STEEL),  # keep over the bolt
        parts.box((0.004, 0.036, 0.130), at=(-0.002, SPINDLE[1], 0.090), color=GREY),  # handle back plate
        parts.box((0.004, 0.036, 0.060), at=(-0.002, KEY[1], KEY[2]), color=GREY),  # the lock's escutcheon plate
        parts.cylinder(0.012, -LEVER_X - 0.006, at=((LEVER_X + 0.006) / 2 - 0.004, SPINDLE[1], SPINDLE[2]),
                       color=GREY, axis="x"),  # spindle rose
    ],
    joints=[
        parts.Hinge(
            "handle",
            shapes=[parts.box((LEVER_T, LEVER_LEN, LEVER_H),
                              at=(LEVER_X, SPINDLE[1] - LEVER_LEN / 2 + 0.008, SPINDLE[2]), color=DARK),
                    parts.box((LEVER_T + 0.001, 0.015, LEVER_H + 0.001),
                              at=(LEVER_X, SPINDLE[1] - LEVER_LEN + 0.008 + 0.0075, SPINDLE[2]), color=WHITE)],  # white end
            pivot=(LEVER_X, SPINDLE[1], SPINDLE[2]),
            axis=(-1.0, 0.0, 0.0),  # positive lifts the lever's free end
            lower=-0.6,
            upper=0.8,
            mass=0.03,
            spring=0.5,
            damping=0.05,
        ),
        parts.Hinge(
            "key",
            shapes=[
                parts.cylinder(0.005, -BOW_X - 0.006, at=((BOW_X + 0.003) / 2 - 0.004, KEY[1], KEY[2]), color=STEEL,
                               axis="x"),  # shank
                parts.box(BOW, at=(BOW_X, KEY[1], KEY[2] + BOW_TIP - BOW[2] / 2), color=RED),
                parts.box((0.001, 0.006, BOW[2]), at=(BOW_X - BOW[0] / 2 - 0.0005, KEY[1], KEY[2] + BOW_TIP - BOW[2] / 2),
                          color=WHITE),  # white stripe along the bow, on its face toward the robot
            ],
            pivot=(BOW_X, KEY[1], KEY[2]),
            axis=(-1.0, 0.0, 0.0),  # positive turns the bow from up toward the robot's left (the door's edge)
            lower=0.0,
            upper=1.6,
            mass=0.02,
            damping=0.02,
        ),
        parts.Slider(
            "hooks",
            shapes=[parts.box((0.012, 0.016, 0.040), at=(-0.002, 0.150, 0.150), color=STEEL)],
            axis=(0.0, 0.0, 1.0),
            travel=HOOK_TRAVEL,
            mass=0.02,
            spring=60.0,
            damping=1.0,
        ),
    ],
)

OBJECTS = [DOOR]

EPS = 1e-6
LIFTED = 0.5  # the handle counts as lifted at 0.5 rad (29 degrees) or more; its upper notch is at 0.7 rad
AT_NOTCH = 0.65  # partial credit needs the handle in its upper notch (0.65 rad, 37 degrees, or more)
DETENT_SWITCH = 0.35
HANDLE_NOTCH = 0.7
KEY_LOCKED = 1.4  # within 0.2 rad (11 degrees) of the 1.6 rad stop
KEY_BLOCKED = 0.15  # with the handle down the key turns only this far
KEY_HOLDS_HANDLE = 0.3  # once the key is past 0.3 rad the handle cannot be pressed down below 0.5 rad
CLEAR_GRIP = 0.005
STILL = 0.002  # rad per control step


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# Filled every control step by the hook below; offline (near_misses.py) the grippers count as clear and still.
_sim = {"clear": True, "still": True, "prev": None, "targets": {}, "limits": {}}


def _grade(objects) -> float:
    if not (_sim["clear"] and _sim["still"]):
        return 0.0
    q = objects["door"].joints
    if q["handle"] >= LIFTED - EPS and q["key"] >= KEY_LOCKED - EPS:
        return 1.0
    if q["handle"] >= AT_NOTCH - EPS:
        return 0.25
    return 0.0


def success(objects):
    """Locked: the handle lifted to 0.5 rad or more and the key turned to 1.4 rad or more; let go, still."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 locked; 0.25 the handle in its upper notch (hooks thrown) but the key not turned home; else 0. Let go,
    still."""
    return _grade(objects)


# --- simulator hooks ------------------------------------------------------------------------------------------------


def _set(sim, joint, value) -> None:
    if _sim["targets"].get(joint) != value:
        _sim["targets"][joint] = value
        sim.set_target("door", joint, value)


def _limit(sim, joint, lo, hi) -> None:
    import torch

    if _sim["limits"].get(joint) == (lo, hi):
        return
    _sim["limits"][joint] = (lo, hi)
    base = sim.env.unwrapped
    door = base.scene["door"]
    ids = door.find_joints([joint], preserve_order=True)[0]
    lim = torch.tensor([[[lo, hi]]], dtype=torch.float32, device=base.device)
    door.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_reset
def _reset(sim) -> None:
    _sim.update(clear=True, still=True, prev=None, targets={}, limits={})


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


@parts.on_step
def _lock(sim) -> None:
    """The multipoint gear: the handle has two notches (level and lifted); lifting it throws the hooks; the key turns
    only once the handle is lifted, and a turned key holds the handle up. Also measures how close the grippers are to
    the handle and the key, and whether they are still."""
    from yam_arena import analysis

    st = sim.states()
    door = st["door"]
    q = door.joints
    _set(sim, "handle", HANDLE_NOTCH if q["handle"] > DETENT_SWITCH else 0.0)
    _set(sim, "hooks", HOOK_TRAVEL * float(np.clip(q["handle"] / HANDLE_NOTCH, 0.0, 1.0)))
    _limit(sim, "key", 0.0, 1.6 if q["handle"] >= LIFTED - EPS else KEY_BLOCKED)
    _limit(sim, "handle", LIFTED if q["key"] > KEY_HOLDS_HANDLE else -0.6, 0.8)

    now = np.array([q["handle"], q["key"]])
    prev = _sim["prev"]
    _sim["still"] = bool(prev is None or np.max(np.abs(now - prev)) < STILL)
    _sim["prev"] = now
    poses = {k: (np.asarray(p), _rot(r)) for k, (p, r) in door.links.items() if k in ("handle", "key")}
    boxes = [(c, r, s) for _, c, r, s in DOOR.world_boxes(poses)]
    hulls = _hulls(sim.env.unwrapped)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR_GRIP for c, r, s in boxes)


# --- demo: right arm only -------------------------------------------------------------------------------------------
TIP = 0.013
GRIP_R = 0.070  # grip the lever 70 mm from the spindle
OPEN_ON_LEVER = -0.014
YAW = math.pi / 2  # fingers close along x, across the lever's 12 mm thickness


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def _lever_point(theta, r=GRIP_R):
    sy, sz = YD + SPINDLE[1], SPINDLE[2]
    return sy - r * math.cos(theta), sz + r * math.sin(theta)


def _bow_corner(a, u=-0.012, v=0.030):
    """World (y, z) of a point at (u across, v along) the bow when the key is turned a rad."""
    ky, kz = YD + KEY[1], KEY[2]
    return ky + u * math.cos(a) + v * math.sin(a), kz - u * math.sin(a) + v * math.cos(a)


def demo():
    """Right arm: pinch the lever 70 mm out, raise it along its arc to the upper notch, let go; then, with the
    fingers closed, push the key's bow round from up to its stop along its arc, and back away."""
    gx = XD + LEVER_X
    y0, z0 = _lever_point(0.0)
    tcp_dz = -LEVER_H / 2 + 0.004 + TIP  # tips 4 mm above the lever's underside
    plan = [
        Waypoint(2.5, _w(gx, y0, 0.18), OPEN_ON_LEVER, "over the lever", YAW),
        Waypoint(1.5, _w(gx, y0, z0 + tcp_dz), OPEN_ON_LEVER, "down to the lever", YAW),
        Waypoint(0.8, _w(gx, y0, z0 + tcp_dz), FINGER_CLOSED, "pinch it", YAW),
    ]
    for th in (0.15, 0.30, 0.45, 0.60, 0.75, 0.85):
        y, z = _lever_point(th)
        plan.append(Waypoint(0.6, _w(gx, y, z + tcp_dz), FINGER_CLOSED, f"lift the lever to {th:.2f} rad", YAW))
    y, z = _lever_point(0.85)
    plan += [
        Waypoint(0.6, _w(gx, y, z + tcp_dz), OPEN_ON_LEVER, "let go", YAW),
        Waypoint(1.0, _w(gx, y, 0.19), OPEN_ON_LEVER, "up", YAW),
    ]
    kx = XD + BOW_X
    cy, cz = _bow_corner(0.0)
    # The closed fingers' +y face is 18 mm from the TCP and their tips 13 mm below it.
    plan += [
        Waypoint(1.0, _w(kx, cy - 0.018, 0.15), FINGER_CLOSED, "over the key, fingers closed"),
        Waypoint(1.5, _w(kx, cy - 0.018, cz + TIP), FINGER_CLOSED, "down beside the bow"),
    ]
    for a in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.7, 1.8):
        cy, cz = _bow_corner(a)
        plan.append(Waypoint(0.9, _w(kx, cy - 0.018, cz + TIP), FINGER_CLOSED, f"turn the key to {a:.2f} rad"))
    plan += [
        Waypoint(1.0, _w(kx, cy - 0.018, 0.15), FINGER_CLOSED, "up"),
        Waypoint(1.5, _w(0.20, -0.25, 0.20), FINGER_CLOSED, "park"),
        Waypoint(3.0, _w(0.20, -0.25, 0.20), FINGER_CLOSED, "wait"),
    ]
    return {"right": plan}
