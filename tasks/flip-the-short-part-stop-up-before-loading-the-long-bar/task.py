"""On the station table, between the arms: a cut-off fixture. A grey steel fence runs straight out from the robot,
with a fixed end stop across its far end on the robot's right side of the fence. A red hinged flip stop is mounted
on the top edge of the fence part way along; it can swing up and over the fence on its hinge, or hang down across
the strip of table in front of the fence face. A loose maple bar lies on the table to the right of the fixture.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
FENCE_Y = 0.02  # the fence face: the bar's lane is on its -y side
FENCE_X = (0.17, 0.45)
FENCE_T, FENCE_H = 0.020, 0.030
END_X = 0.42  # face of the end stop
END_W = 0.045  # the end stop reaches 45 mm out from the fence face
FLIP_X = (0.290, 0.300)  # the flip stop's faces; 120 mm from the end stop's face
FLIP_W, FLIP_LOW = 0.045, 0.004  # reach from the fence face; gap under it
PIVOT = (sum(FLIP_X) / 2, FENCE_Y, TOP + FENCE_H)  # hinge axis along x at the fence's top front edge
FLIP_UPPER = math.radians(135)  # leaning back over the fence on its upper stop
FLIP_UP = math.radians(125)  # past its balance point (about 120 deg): gravity holds it on the upper stop
BAR = (0.180, 0.030, 0.025)
BAR_START = (0.24, -0.20, TOP + BAR[2] / 2)

STEEL, RED, MAPLE = (0.45, 0.47, 0.50), (0.80, 0.12, 0.10), (0.86, 0.72, 0.50)

_fx, _fl = (FENCE_X[0] + FENCE_X[1]) / 2, FENCE_X[1] - FENCE_X[0]
_px, _py, _pz = PIVOT
_paddle_h = FENCE_H - FLIP_LOW
FIXTURE = parts.Part(
    "fixture", pos=(_px, FENCE_Y, TOP),
    shapes=[
        parts.box((_fl, FENCE_T, FENCE_H), at=(_fx - _px, FENCE_T / 2, FENCE_H / 2), color=STEEL),
        parts.box((FENCE_X[1] - END_X, END_W, FENCE_H),
                  at=((END_X + FENCE_X[1]) / 2 - _px, -END_W / 2, FENCE_H / 2), color=STEEL),
        parts.cylinder(0.004, 0.024, at=(0.0, 0.0, FENCE_H), color=STEEL, axis="x"),  # hinge barrel
    ],
    joints=[parts.Hinge(
        "flip", pivot=(0.0, 0.0, FENCE_H), axis=(-1.0, 0.0, 0.0), lower=0.0, upper=FLIP_UPPER, mass=0.04,
        damping=0.02,
        shapes=[parts.box((FLIP_X[1] - FLIP_X[0], FLIP_W, _paddle_h),
                          at=(0.0, -FLIP_W / 2, FLIP_LOW + _paddle_h / 2), color=RED)])],
)
LONG_BAR = Box("long_bar", pos=BAR_START, size=BAR, mass=0.10, color=MAPLE)
OBJECTS = [FIXTURE, LONG_BAR]

ON_TABLE = 0.001  # underside within 1 mm of the table
FLAT_DEG = 2.0
FENCE_TOL = 0.0015  # long side within 1.5 mm of the fence face at both ends
END_TOL = 0.001  # end face within 1 mm of the end stop
END_NEAR = 0.005  # partial credit: end within 5 mm
EPS = 1e-6


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _flat(bar) -> bool:
    """Lying on a 180 x 30 mm face, tilted under 2 degrees, underside within 1 mm of the table."""
    up = abs(_rot(bar.quat_xyzw)[2, 2])
    return bool(up >= math.cos(math.radians(FLAT_DEG)) - EPS and bar.low[2] - TOP <= ON_TABLE + EPS)


def _gaps(bar):
    """(largest gap between the bar's fence-side face and the fence face at its two ends,
    largest gap between its end face and the end stop at its two edges), in m, from the bar's true corners."""
    r = _rot(bar.quat_xyzw)
    c = np.asarray(bar.pos, float)
    u, n = r[:, 0], r[:, 1]  # long axis, width axis
    n = n if n[1] >= 0 else -n  # the long face toward the fence
    u = u if u[0] >= 0 else -u  # the end toward the end stop
    side = [c + n * BAR[1] / 2 + s * u * BAR[0] / 2 for s in (-1, 1)]
    end = [c + u * BAR[0] / 2 + s * n * BAR[1] / 2 for s in (-1, 1)]
    return max(FENCE_Y - p[1] for p in side), max(END_X - p[0] for p in end)


def _against_fence(bar) -> bool:
    fence_gap, _ = _gaps(bar)
    return bool(_flat(bar) and -0.001 <= fence_gap <= FENCE_TOL + EPS and bar.resting())


# Released: measured in the simulator after every control step, from both arms' finger and gripper collision hulls
# against points on the bar's surface. Offline (near_misses.py, no simulator) it is taken as true unless a case
# sets _GRIP["clear"].
CLEAR = 0.005  # no finger or gripper surface within 5 mm of the bar
SUCCESS_HOLD_S = 3.0
_GRIP = {"clear": True}
_LINKS = ("gripper", "tip_left", "tip_right")


def _surface_local(step=0.005) -> np.ndarray:
    """Points every 5 mm or less over the bar's six faces, in its own frame."""
    h = np.asarray(BAR) / 2
    grids = [np.linspace(-v, v, max(2, int(np.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for axis in range(3):
        a, b = [i for i in range(3) if i != axis]
        ga, gb = np.meshgrid(grids[a], grids[b], indexing="ij")
        for sign in (-1.0, 1.0):
            p = np.zeros((ga.size, 3))
            p[:, a], p[:, b], p[:, axis] = ga.ravel(), gb.ravel(), sign * h[axis]
            pts.append(p)
    return np.concatenate(pts)


_SURFACE = _surface_local()


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
    lo, hi = pts.min(0) - 0.02, pts.max(0) + 0.02
    best = np.inf
    for _, h in hulls:
        if np.any(h.vertices.max(0) < lo) or np.any(h.vertices.min(0) > hi):
            continue
        d = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)  # a lower bound on each point's distance
        best = min(best, float(d.min()))
    return best


@parts.on_step
def _check_grip(sim):
    b = sim.states()["long_bar"]
    pts = np.asarray(b.pos, float) + _SURFACE @ _rot(b.quat_xyzw).T
    _GRIP["clear"] = _clearance(_robot_hulls(sim), pts) >= CLEAR - EPS


def released(objects) -> bool:
    return bool(_GRIP["clear"])


def _stop_up(objects) -> bool:
    return bool(objects["fixture"].joints["flip"] >= FLIP_UP - EPS)


def _placed(objects) -> bool:
    """Flat against the fence, at rest, let go, and the flip stop up on its own (not resting on the bar)."""
    return bool(_against_fence(objects["long_bar"]) and released(objects) and _stop_up(objects))


def success(objects):
    """Loaded: flat, against the fence at both ends and against the end stop, at rest, let go, flip stop up."""
    return bool(_placed(objects) and _gaps(objects["long_bar"])[1] <= END_TOL + EPS)


def distance(objects):
    goal = np.array((END_X - BAR[0] / 2, FENCE_Y - BAR[1] / 2, TOP + BAR[2] / 2))
    return float(np.linalg.norm(np.asarray(objects["long_bar"].pos) - goal))


def score(objects):
    """1.0 loaded; 0.5 the same but with the end within 5 mm of the end stop; otherwise 0."""
    if success(objects):
        return 1.0
    return 0.5 if _placed(objects) and _gaps(objects["long_bar"])[1] <= END_NEAR + EPS else 0.0


# ---- scripted demo -------------------------------------------------------------------------------------------
TIP = 0.0129  # finger tips below the TCP
TRAVEL_Z = TOP + 0.10
TIP_FRONT = 0.0094  # closed finger tips' front face ahead of the TCP at push height (trace)
PUSH_OVER = 0.008  # pushes are commanded 8 mm past contact: the arm gives way (trace: 3 mm left the bar 1.7 mm short)
GRIP_REL = (-0.038, -0.022)  # where the left fingers hold the flip stop: (y, z) from the hinge axis
OVER = 175  # degrees commanded: the stop lags its grip on the way up (trace: 104 deg reached at 145 commanded)


def _on_arc(phi):
    """Left TCP holding the flip stop turned up by phi (rad)."""
    y, z = GRIP_REL
    c, s = math.cos(phi), math.sin(phi)
    return (_px, _py + y * c + z * s, _pz - y * s + z * c + TIP)


def demo():
    yaw = -math.pi / 2  # fingers close along x, along the hinge axis, so the stop turns between the pads
    g0 = _on_arc(0.0)
    left = [
        Waypoint(1.5, (g0[0], g0[1], TRAVEL_Z), FINGER_OPEN, "over the flip stop", yaw),
        Waypoint(1.5, g0, FINGER_OPEN, "down beside it", yaw),
        Waypoint(0.8, g0, FINGER_CLOSED, "grip its free end", yaw),
        *(Waypoint(0.7, _on_arc(math.radians(d)), FINGER_CLOSED, f"flip {d}", yaw) for d in range(15, 180, 15)),
        Waypoint(0.8, _on_arc(math.radians(OVER)), FINGER_CLOSED, "past its upper stop", yaw),
        Waypoint(0.6, _on_arc(math.radians(OVER)), FINGER_OPEN, "let go", yaw),
        Waypoint(1.0, (_px, _on_arc(math.radians(OVER))[1], TRAVEL_Z + 0.02), FINGER_OPEN, "up", yaw),
        Waypoint(1.5, (0.22, 0.25, TRAVEL_Z + 0.04), FINGER_OPEN, "park"),
    ]
    t_left = sum(w.duration for w in left)

    bx, by, _ = BAR_START
    grip_dx = 0.024  # grip off the bar's centre, away from the flip stop
    grip_z = TOP + 0.004 + TIP  # finger tips 4 mm above the table
    place = (END_X - 0.004 - BAR[0] / 2, FENCE_Y - 0.025 - BAR[1] / 2)  # 4 mm short of the end stop, 25 mm off the fence
    release = -0.022  # open only 7 mm a side so the far finger stays clear of the fence
    push_z = TOP + 0.006 + TIP
    face = place[1] - BAR[1] / 2  # the bar's -y face before the push
    right = [
        Waypoint(1.5, (bx + grip_dx, by, TRAVEL_Z), FINGER_OPEN, "over the bar"),
        Waypoint(1.5, (bx + grip_dx, by, grip_z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (bx + grip_dx, by, grip_z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (bx + grip_dx, by, TRAVEL_Z), FINGER_CLOSED, "lift"),
    ]
    right.append(Waypoint(max(0.5, t_left - sum(w.duration for w in right)), (bx + grip_dx, by, TRAVEL_Z),
                          FINGER_CLOSED, "wait for the flip stop"))
    right += [
        Waypoint(1.8, (place[0] + grip_dx, place[1], TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.5, (place[0] + grip_dx, place[1], grip_z + 0.006), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (place[0] + grip_dx, place[1], grip_z + 0.006), release, "release"),
        Waypoint(1.0, (place[0] + grip_dx, place[1], TRAVEL_Z), release, "up"),
    ]
    for k, x in enumerate((0.37, 0.26, 0.37)):
        start_y = face - 0.010 - 0.010  # finger face 10 mm off the bar
        right += [
            Waypoint(1.2, (x, start_y, TRAVEL_Z), FINGER_CLOSED, f"beside the bar {k}"),
            Waypoint(1.0, (x, start_y, push_z), FINGER_CLOSED, f"down {k}"),
            Waypoint(1.5, (x, FENCE_Y - BAR[1] - 0.010 + PUSH_OVER, push_z), FINGER_CLOSED, f"push onto the fence {k}"),
            Waypoint(0.6, (x, start_y, push_z), FINGER_CLOSED, f"back off {k}"),
            Waypoint(0.8, (x, start_y, TRAVEL_Z), FINGER_CLOSED, f"up {k}"),
        ]
    near_end = END_X - 0.004 - BAR[0]
    ey = FENCE_Y - BAR[1] / 2
    right += [
        Waypoint(1.2, (near_end - 0.020 - 0.012, ey, TRAVEL_Z), FINGER_CLOSED, "behind the bar"),
        Waypoint(1.0, (near_end - 0.020 - 0.012, ey, push_z), FINGER_CLOSED, "down behind it"),
        Waypoint(1.5, (END_X - BAR[0] - TIP_FRONT + 0.006, ey, push_z), FINGER_CLOSED, "push into the end stop"),
        Waypoint(0.6, (near_end - 0.020 - 0.015, ey, push_z), FINGER_CLOSED, "back off"),
        Waypoint(1.0, (near_end - 0.020 - 0.015, ey - 0.05, TRAVEL_Z), FINGER_CLOSED, "clear"),
        Waypoint(1.5, (0.20, -0.20, TRAVEL_Z + 0.04), FINGER_CLOSED, "park"),
        Waypoint(5.0, (0.20, -0.20, TRAVEL_Z + 0.04), FINGER_CLOSED, "wait, hands clear"),
    ]
    return {"left": left, "right": right}
