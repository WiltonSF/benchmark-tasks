"""On the station table, in front of the left arm: a grey bike-frame stand whose seat tube leans to one side, its
open top toward the robot's left. Nearer the robot, a loose silver seatpost with a black saddle block on top stands upright in a
yellow stand socket. A dark band is marked around the seatpost partway along it.
"""

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# World frame: x away from the arm bases, y to the robot's left, z up (m).
LEAN = math.radians(17.0)  # seat tube leans 17 degrees (a 73-degree seat angle), top toward the robot's left (+y)
# The frame stands side-on to the robot, so the lean shows from the top camera.
PIVOT = np.array([0.28, 0.17, TOP + 0.012])  # the tube's bottom, on the frame's base block
BORE, WALL = 0.032, 0.004  # square bore 32 mm inside, 4 mm walls
STOP_T = 0.005  # bottom stop plate inside the tube
DEPTH = 0.090  # from the stop's top to the mouth
TUBE_LEN = STOP_T + DEPTH

GREY, DARK, SILVER, BLACK, YELLOW = (0.45, 0.47, 0.50), (0.25, 0.26, 0.28), (0.78, 0.79, 0.81), (0.08, 0.08, 0.09), \
    (0.95, 0.80, 0.20)


def _b(size, at, color=GREY):
    return parts.box(size, at=at, color=color)


_off = (BORE + WALL) / 2
_zc = TUBE_LEN / 2
# Tube shapes in the part frame, written upright (joint 0). The tube's weight and the lug on its lean side tip it onto
# its upper stop at LEAN during the embodiment's 0.5 s settle at reset, before the first camera frame.
_TUBE = [
    _b((BORE + 2 * WALL, WALL, TUBE_LEN), (0.0, _off, _zc)),
    _b((BORE + 2 * WALL, WALL, TUBE_LEN), (0.0, -_off, _zc)),
    _b((WALL, BORE, TUBE_LEN), (_off, 0.0, _zc)),
    _b((WALL, BORE, TUBE_LEN), (-_off, 0.0, _zc)),
    _b((BORE, BORE, STOP_T), (0.0, 0.0, STOP_T / 2), DARK),  # bottom stop
    # A lug on the lean side of the tube, well above the table so it cannot touch it as the tube leans: its weight
    # tips the upright-written tube toward its stop at reset.
    _b((0.03, 0.03, 0.03), (0.0, 0.035, 0.055), DARK),
]
TUBE = parts.Hinge("tube", shapes=[_b(s.size, tuple(np.asarray(s.at) + PIVOT), s.color) for s in _TUBE],
                   pivot=tuple(PIVOT), axis=(-1.0, 0.0, 0.0), lower=0.0, upper=LEAN, mass=1.5, spring=0.0,
                   damping=0.05, max_effort=50.0)
FRAME = parts.Part("frame", pos=(0.0, 0.0, 0.0), shapes=[
    _b((0.10, 0.16, 0.010), (PIVOT[0], PIVOT[1] + 0.02, TOP + 0.005), DARK),  # base block
    _b((0.06, 0.012, 0.012), (PIVOT[0], PIVOT[1] - 0.03, TOP + 0.016), DARK),  # cradle behind the tube's foot
], joints=[TUBE])

# Seatpost: 28 mm round, 140 mm long, saddle block on top, dark band 65 mm from its bottom end. Origin at the
# post's bottom end, axis along its own z.
POST_R, POST_L = 0.014, 0.140
BAND = 0.065  # band centre above the post's bottom end; the band is 6 mm wide
BAND_TOP = BAND + 0.003  # its top edge: must be at or below the mouth
SADDLE = (0.016, 0.06, 0.012)  # along the frame (y); narrower than the fingers' grip gap (fingers close along x)
STAND_XY = (0.17, 0.25)
POST = parts.Part("seatpost", pos=(*STAND_XY, TOP), mass=0.15, shapes=[
    parts.cylinder(POST_R, BAND - 0.003, at=(0.0, 0.0, (BAND - 0.003) / 2), color=SILVER),
    parts.cylinder(POST_R, 0.006, at=(0.0, 0.0, BAND), color=BLACK),
    parts.cylinder(POST_R, POST_L - BAND - 0.003, at=(0.0, 0.0, (POST_L + BAND + 0.003) / 2), color=SILVER),
    parts.box(SADDLE, at=(0.0, 0.0, POST_L + SADDLE[2] / 2), color=BLACK),
])
# Stand socket: four walls 40 mm tall around a 36 mm square hole.
_S, _H, _W = 0.036, 0.040, 0.008
STAND = parts.Part("stand", pos=(*STAND_XY, TOP), shapes=[
    _b((_S + 2 * _W, _W, _H), (0.0, (_S + _W) / 2, _H / 2), YELLOW),
    _b((_S + 2 * _W, _W, _H), (0.0, -(_S + _W) / 2, _H / 2), YELLOW),
    _b((_W, _S, _H), ((_S + _W) / 2, 0.0, _H / 2), YELLOW),
    _b((_W, _S, _H), (-(_S + _W) / 2, 0.0, _H / 2), YELLOW),
])

OBJECTS = [FRAME, STAND, POST]


# ---- geometry helpers --------------------------------------------------------------------------------------------
def _rot(q):
    return parts._quat_to_matrix(q)


def tube_pose(objects):
    """(axis from the stop toward the mouth, mouth centre) of the seat tube as it stands."""
    st = objects["frame"]
    if "tube" in getattr(st, "links", {}):
        p, q = st.links["tube"]
        r = _rot(q)
        p = np.asarray(p, float)
    else:  # offline, a state without link poses: the tube on its stop
        r, p = parts._rot((-1.0, 0.0, 0.0), LEAN), PIVOT
    axis = r @ np.array([0.0, 0.0, 1.0])
    return axis, p + axis * TUBE_LEN


def post_axis(state):
    return _rot(state.quat_xyzw) @ np.array([0.0, 0.0, 1.0])


def insertion(objects):
    """(depth of the post's bottom end below the mouth along the tube, its distance off the tube's axis, angle in
    degrees between the post and the tube)."""
    axis, mouth = tube_pose(objects)
    s = objects["seatpost"]
    bottom = np.asarray(s.pos, float)
    rel = bottom - mouth
    depth = -float(rel @ axis)
    off = float(np.linalg.norm(rel - (rel @ axis) * axis))
    ang = math.degrees(math.acos(np.clip(float(post_axis(s) @ axis), -1.0, 1.0)))
    return depth, off, ang


# ---- scoring ---------------------------------------------------------------------------------------------------
ANGLE_MAX = 3.0
OFF_MAX = (BORE / 2 - POST_R) * math.sqrt(2) + 0.001  # inside the bore
CLEAR = 0.03  # released: no finger or gripper surface within 3 cm of the post

_GRIP = {"clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _post_points(state):
    p, r = np.asarray(state.pos, float), _rot(state.quat_xyzw)
    zs = np.linspace(0.0, POST_L + SADDLE[2], 16)
    a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    local = np.stack([np.tile(POST_R * np.cos(a), len(zs)), np.tile(POST_R * np.sin(a), len(zs)),
                      np.repeat(zs, len(a))], 1)
    return p + local @ r.T


def _robot_hulls(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        link_poses = {}
        for i, n in enumerate(asset.body_names):
            if n in _LINKS:
                t = np.eye(4)
                t[:3, :3], t[:3, 3] = _rot(poses[i, 3:]), poses[i, :3] - origin
                link_poses[n] = t
        out += analysis.world_hulls(link_poses)
    return out


def _clearance(hulls, pts) -> float:
    best = np.inf
    for _, h in hulls:
        d = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)  # a lower bound on each point's distance
        best = min(best, float(d.min()))
    return best


@parts.on_step
def _check_grip(sim):
    st = sim.states()
    _TUBE_HIST["min"] = min(_TUBE_HIST["min"], float(st["frame"].joints["tube"]))
    _GRIP["clear"] = _clearance(_robot_hulls(sim), _post_points(st["seatpost"])) >= CLEAR - EPS
    if os.environ.get("POST_DEBUG"):
        d, off, ang = insertion(st)
        with open(os.environ["POST_DEBUG"], "a") as f:
            f.write(json.dumps({"t": round(float(sim.t), 2), "depth": round(d, 4), "off": round(off, 4),
                                "ang": round(ang, 2), "tube": round(math.degrees(st["frame"].joints["tube"]), 2),
                                "clear": _GRIP["clear"]}) + "\n")


def released(objects) -> bool:
    """Measured in the simulator after every control step; offline (no simulator) taken as true."""
    return bool(_GRIP["clear"])


TUBE_MIN = LEAN - math.radians(2.0)  # the tube still on its stop (the frame is rigid in real life)


# The tube's lowest angle over the whole trial, reset at each episode reset (parts.py on_reset).
_TUBE_HIST = {"min": LEAN}


@parts.on_reset
def _reset_tube_history(sim):
    _TUBE_HIST["min"] = LEAN


def tube_on_stop(objects) -> bool:
    """The tube within 2 degrees of its stop now and at every step of the trial so far."""
    st = objects["frame"]
    now = getattr(st, "joints", {}).get("tube", LEAN)
    return bool(min(now, _TUBE_HIST["min"]) >= TUBE_MIN - EPS)


def success(objects):
    """Post in the tube on its own: band at or below the mouth, axis within 3 degrees of the tube's, still; the tube
    still at its 17-degree lean."""
    depth, off, ang = insertion(objects)
    return bool(tube_on_stop(objects) and depth >= BAND_TOP - EPS and off <= OFF_MAX + EPS and ang <= ANGLE_MAX + EPS and released(objects)
                and objects["seatpost"].resting())


def distance(objects):
    depth, off, _ = insertion(objects)
    return float(max(0.0, BAND_TOP - depth) + off)


def score(objects):
    """Pass/fail: 1.0 when success() holds, else 0."""
    return 1.0 if success(objects) else 0.0


# ---- demo: fixed waypoints; the lean uses tilt.py (fingertips, and the post's lower end, swing toward -y) --------
TIP = 0.013
GRIP_UP = 0.115  # TCP this far above the post's bottom end, just under the saddle
U = np.array([0.0, math.sin(LEAN), math.cos(LEAN)])  # tube axis, stop toward mouth
MOUTH = PIVOT + U * TUBE_LEN
PARK = (0.16, 0.32, TOP + 0.20)
LIFT_Z = TOP + 0.21  # carried upright: bottom end 95 mm above the table, clear of the stand and the mouth


# Measured on the GPU: leaning sideways with the fingers closing along x, the post turns about the grip axis and
# keeps the grip's lean (19 degrees for 19.5 asked), the TCP settles about 4 mm low, and the post sits about 3.5 mm
# toward the robot in the grip (one finger pushes harder). The demo leans the grip 17 degrees and aims 3.5 mm further
# from the robot and 5 mm high, so the post arrives on the tube's axis.
GRIP_LEAN = 17.0
AIM = np.array([0.0035, 0.0, 0.005])


def _along(depth):
    """TCP position with the post's bottom end ``depth`` below the mouth along the tube axis (negative: above)."""
    return tuple(MOUTH - U * depth + U * GRIP_UP + AIM)


def demo():
    """Lean the post 17 degrees to the tube's angle, lower it along the tube until the band is 15 mm in, let go.
    POST_DEMO=vertical replays the shortcut: the post kept upright, lowered over the mouth and let go."""
    import tilt  # demo only: a changed stack can break the scripted demo but never scoring or Dream-1

    sx, sy = STAND_XY
    gz = TOP + GRIP_UP
    Y = math.pi / 2  # fingers close along x, so the lean (in y) turns the post about the grip axis
    lean = dict(tilt=GRIP_LEAN, heading=-math.pi / 2, yaw=Y)
    W = tilt.Waypoint
    hold = -(POST_R - 0.003)
    w = [
        Waypoint(1.0, PARK, FINGER_OPEN, "start", Y),
        Waypoint(1.5, (sx, sy, gz + 0.03), FINGER_OPEN, "over the post", Y),
        Waypoint(1.2, (sx, sy, gz), FINGER_OPEN, "down", Y),
        Waypoint(0.8, (sx, sy, gz), hold, "grip under the saddle", Y),
        Waypoint(1.5, (sx, sy, LIFT_Z), hold, "lift out of the stand", Y),
    ]
    if os.environ.get("POST_DEMO") == "vertical":
        # Upright, the arm cannot reach high enough holding the post under its saddle, so this grips it lower
        # (TCP 75 mm above its bottom end), as a policy trying it upright would have to.
        low = 0.075
        mx, my, mz = MOUTH
        w = w[:2] + [Waypoint(1.2, (sx, sy, TOP + low), FINGER_OPEN, "down", Y),
                     Waypoint(0.8, (sx, sy, TOP + low), hold, "grip low", Y),
                     Waypoint(1.5, (sx, sy, mz + 0.035 + low), hold, "lift out of the stand", Y),
                     Waypoint(2.0, (mx, my, mz + 0.035 + low), hold, "upright, 35 mm over the mouth", Y),
                     Waypoint(1.0, (mx, my, mz + 0.035 + low), hold, "settle", Y),
                     Waypoint(2.5, (mx, my, mz - 0.030 + low), hold, "lower straight down", Y),
                     Waypoint(0.8, (mx, my, mz - 0.030 + low), FINGER_OPEN, "let go", Y),
                     Waypoint(1.2, (mx, my, mz + 0.03 + low), FINGER_OPEN, "up", Y),
                     Waypoint(1.5, PARK, FINGER_OPEN, "park", Y), Waypoint(4.0, PARK, FINGER_OPEN, "wait", Y)]
        return {"left": w}
    w += [
        W(2.0, _along(-0.015), hold, "lean to the tube's angle above the mouth", **lean),
        W(1.2, _along(-0.006), hold, "line up at the mouth", **lean),
        W(3.0, _along(BAND + 0.015), hold, "push in along the tube", **lean),
        W(0.8, _along(BAND + 0.015), FINGER_OPEN, "let go", **lean),
        W(1.5, _along(BAND + 0.015 - 0.07), FINGER_OPEN, "back out along the axis", **lean),
        Waypoint(1.5, PARK, FINGER_OPEN, "park", Y),
        Waypoint(4.0, PARK, FINGER_OPEN, "wait", Y),
    ]
    return {"left": w}
