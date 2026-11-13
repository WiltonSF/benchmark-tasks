"""On the station table, a little to the left of the middle: a chrome bathroom-sink pop-up drain body standing upright
in a grey bench stand, its round flange on top and its pivot port, a short round spout with a white seat washer,
pointing at the robot. The pop-up stopper sits closed in the drain, its chrome head on the flange. Nearer the robot
and to the right, the steel pivot rod with its white pivot ball lies front to back in two notched rests, its inner
end toward the drain.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

CHROME, BRASS_DARK, WHITE = (0.80, 0.81, 0.83), (0.45, 0.42, 0.36), (0.93, 0.93, 0.90)
STEEL, GREY, DARK = (0.60, 0.60, 0.62), (0.55, 0.57, 0.60), (0.22, 0.23, 0.25)
STEM_C = (0.20, 0.20, 0.22)


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# -- drain body (fixed). Built in its own frame (axis vertical through the origin, the pivot port pointing along -y),
# then placed at (BX, BY) and turned so the port points along PORT_DIR (world, rad; 0 is away from the robot).
BX, BY = 0.33, 0.05
PORT_DIR = math.pi  # the port points at the robot
YAW_D = PORT_DIR + math.pi / 2  # the drain's turn
U = np.array([math.cos(PORT_DIR), math.sin(PORT_DIR), 0.0])  # outward along the port
RD = np.array([[math.cos(YAW_D), -math.sin(YAW_D), 0.0], [math.sin(YAW_D), math.cos(YAW_D), 0.0], [0.0, 0.0, 1.0]])
C0 = np.array([BX, BY, 0.0])
R_IN, R_OUT = 0.016, 0.020  # 32 mm bore, 40 mm outside
BASE_T = 0.008  # stand's base plate
HP = TOP + 0.054  # pivot port axis height
HF = TOP + 0.110  # flange top: the stopper's head rests here when closed
FLANGE_T, FLANGE_R = 0.004, 0.030
WIN = 0.006  # the opening through the wall is 12 mm tall
STAVES = 24
SEAT_IN, SEAT_OUT = R_OUT, R_OUT + 0.003  # seat plate (washer) from 20 to 23 mm off the axis
SEAT_HOLE = 0.0045  # 9 mm round hole through the seat
BORE = 0.008  # port bore 16 mm round
PORT_END = 0.040  # the port spout ends 40 mm from the drain axis


def _ring(rm, rc, n, yc, length, color, z=None):
    """n cylinders of radius rc along y, centred rm from a line along y at height z: a round hole of radius rm - rc."""
    z = HP if z is None else z
    return [parts.cylinder(rc, length, at=(rm * math.cos(2 * math.pi * k / n), yc, z + rm * math.sin(2 * math.pi * k / n)),
                           color=color, axis="y") for k in range(n)]


def _body_shapes():
    s = []
    z0, z1 = TOP + BASE_T, HF - FLANGE_T
    w = 2 * R_OUT * math.tan(math.pi / STAVES)
    rc = (R_IN + R_OUT) / 2
    for i in range(STAVES):
        a = -math.pi / 2 + i * 2 * math.pi / STAVES
        at_xy = (0.0 + rc * math.cos(a), 0.0 + rc * math.sin(a))
        cut = i in (0, 1, STAVES - 1)  # the three staves facing the port are cut by the opening
        spans = [(z0, HP - WIN), (HP + WIN, z1)] if cut else [(z0, z1)]
        for lo, hi in spans:
            s.append(parts.box((R_OUT - R_IN, w, hi - lo), at=(at_xy[0], at_xy[1], (lo + hi) / 2), color=CHROME,
                               quat=_qz(a)))
    fw = 2 * FLANGE_R * math.tan(math.pi / 12)
    for i in range(12):  # flange ring, 60 mm across, its hole the bore
        a = i * 2 * math.pi / 12
        rc_f = (R_IN + FLANGE_R) / 2
        s.append(parts.box((FLANGE_R - R_IN, fw, FLANGE_T), at=(0.0 + rc_f * math.cos(a), 0.0 + rc_f * math.sin(a),
                                                                  HF - FLANGE_T / 2), color=CHROME, quat=_qz(a)))
    # The kit has only solid cylinders, so each round hole along the port is a ring of overlapping cylinders
    # ("lobes") whose inner faces make the bore. Seat (washer): a 9 mm hole, 3 mm thick, 20 to 23 mm off the axis.
    yc = -(SEAT_IN + SEAT_OUT) / 2
    s += _ring(SEAT_HOLE + 0.002, 0.002, 14, yc, SEAT_OUT - SEAT_IN, WHITE)
    # Port spout: a 16 mm round bore from the seat out to 40 mm off the axis.
    s += _ring(BORE + 0.0022, 0.0022, 16, -(SEAT_OUT + PORT_END) / 2, PORT_END - SEAT_OUT, CHROME)
    # bench stand: base plate and a square clamp collar round the lower tube
    s.append(parts.box((0.10, 0.10, BASE_T), at=(0.0, 0.0, TOP + BASE_T / 2), color=DARK))
    ch, co = 0.022, 0.028
    zc = TOP + BASE_T + ch / 2
    s.append(parts.box((2 * co, co - R_OUT, ch), at=(0.0, 0.0 + (co + R_OUT) / 2, zc), color=GREY))
    s.append(parts.box((2 * co, co - R_OUT, ch), at=(0.0, 0.0 - (co + R_OUT) / 2, zc), color=GREY))
    s.append(parts.box((co - R_OUT, 2 * R_OUT, ch), at=(0.0 + (co + R_OUT) / 2, 0.0, zc), color=GREY))
    s.append(parts.box((co - R_OUT, 2 * R_OUT, ch), at=(0.0 - (co + R_OUT) / 2, 0.0, zc), color=GREY))
    return s


DRAIN = parts.Part("drain_body", pos=(BX, BY, 0.0), yaw=YAW_D, shapes=_body_shapes())

# -- pop-up stopper (loose). Part frame: origin at the centre of the head's underside, z up. In the frame the eye's
# hole runs along y; the stopper starts turned a quarter turn from facing the port, so the hole runs left to right.
HEAD_R, HEAD_T = 0.019, 0.009
KNOB_R, KNOB_H = 0.006, 0.012
FIN_SPAN, FIN_T, FIN_H = 0.0305, 0.002, 0.050
STEM_R = 0.0045
EYE_Z = -0.082  # eye centre below the head's underside: 26 mm below the port when closed
HOLE = 0.0045  # 9 mm round eye
TAB_W, TAB_T = 0.017, 0.005  # the eye's ring: 17 mm across, 5 mm thick along the hole
TAB_BAR = (TAB_W - 2 * HOLE) / 2  # 4 mm of ring round the hole
STOPPER_MASS = 0.055
START_YAW = YAW_D + math.pi / 2  # a quarter turn from facing the port


def _stopper_shapes():
    top_of_tab = EYE_Z + HOLE + TAB_BAR
    stem_len = -top_of_tab
    return [
        parts.cylinder(HEAD_R, HEAD_T, at=(0.0, 0.0, HEAD_T / 2), color=CHROME, axis="z"),
        parts.cylinder(KNOB_R, KNOB_H, at=(0.0, 0.0, HEAD_T + KNOB_H / 2), color=CHROME, axis="z"),
        parts.box((FIN_SPAN, FIN_T, FIN_H), at=(0.0, 0.0, -FIN_H / 2), color=STEM_C),
        parts.box((FIN_T, FIN_SPAN, FIN_H), at=(0.0, 0.0, -FIN_H / 2), color=STEM_C),
        parts.cylinder(STEM_R, stem_len, at=(0.0, 0.0, -stem_len / 2), color=STEM_C, axis="z"),
        *_ring(HOLE + 0.002, 0.002, 14, 0.0, TAB_T, STEM_C, z=EYE_Z),  # the eye: a 9 mm round hole
    ]


STOPPER = parts.Part("stopper", pos=(BX, BY, HF), yaw=START_YAW, mass=STOPPER_MASS, shapes=_stopper_shapes())

# -- pivot rod (loose). Part frame: origin at the pivot ball's centre, the rod along +y toward its inner tip.
ROD_R, ROD_L = 0.0025, 0.250
TIP = 0.042  # inner tip 42 mm in from the ball's centre
BALL_R, BALL_L = 0.007, 0.010  # the pivot ball, as a round barrel 14 mm across and 10 mm long
ROD_MASS = 0.040
SEATED = C0 + U * (SEAT_OUT + BALL_L / 2) + np.array([0.0, 0.0, HP])  # ball centre when its face is on the seat
_SHIFT = np.array([0.0, -0.10, 0.0])  # the rod starts 10 cm to the right of the port line
_S0 = SEATED + U * 0.060 + _SHIFT
ROD_START = (float(_S0[0]), float(_S0[1]), TOP + 0.030)
ROD = parts.Part(
    "pivot_rod", pos=ROD_START, yaw=YAW_D, mass=ROD_MASS,
    shapes=[parts.cylinder(ROD_R, ROD_L, at=(0.0, TIP - ROD_L / 2, 0.0), color=STEEL, axis="y"),
            parts.cylinder(BALL_R, BALL_L, at=(0.0, 0.0, 0.0), color=WHITE, axis="y")],
)

# -- rod rests (fixed): two notched blocks, the notch 6 mm wide
REST_TOP = ROD_START[2] - ROD_R
REST_YS = (TIP - 0.012, -0.180)  # along the rod from the ball's centre


def _rest_shapes():
    s = []
    x = 0.0
    for y in REST_YS:
        s.append(parts.box((0.020, 0.012, REST_TOP - TOP), at=(x, y, (REST_TOP + TOP) / 2 - ROD_START[2]), color=DARK))
        for sx in (-1, 1):
            s.append(parts.box((0.007, 0.012, 0.003), at=(x + sx * 0.0065, y, REST_TOP + 0.0015 - ROD_START[2]), color=DARK))
    return s


RESTS = parts.Part("rod_rests", pos=ROD_START, yaw=YAW_D, shapes=_rest_shapes())

OBJECTS = [DRAIN, STOPPER, ROD, RESTS]

# -- scoring ---------------------------------------------------------------------------------------------------
PAST_TAB = 0.002  # the tip at least 2 mm past the tab's far face
SEAT_GAP = 0.003  # ball within 3 mm of the seat
BALL_OFF = 0.002  # ball centre within 2 mm of the port axis
GUIDE_OFF, GUIDE_TILT, GUIDE_LIFT = 0.004, math.radians(6.0), 0.040
MIN_ALIGN = math.cos(math.radians(35.0))
RELEASE_GAP = 0.005
SUCCESS_HOLD_S = 1.0


def _eye_frame(st):
    r = _rot(st.quat_xyzw)
    return np.asarray(st.pos, float) + r @ np.array([0.0, 0.0, EYE_Z]), r


def _rod_axis(st):
    r = _rot(st.quat_xyzw)
    return np.asarray(st.pos, float), r[:, 1]


def captured(objects) -> bool:
    """The rod's axis passes through the eye's 9 mm opening (taken in the stopper's frame, at the tab's mid-plane),
    and the tip is at least 2 mm past the tab's far face while the ball is outside its near face."""
    c, r = _eye_frame(objects["stopper"])
    p, d = _rod_axis(objects["pivot_rod"])
    a = r[:, 1]
    k = float(d @ a)
    if abs(k) < MIN_ALIGN:
        return False
    sgn = 1.0 if k > 0 else -1.0
    s = float((c - p) @ a) / k  # along the rod from the ball's centre to the tab's mid-plane
    q = p + s * d - c
    inside = math.hypot(float(q @ r[:, 0]), float(q @ r[:, 2])) < HOLE - EPS
    tip_past = sgn * float((p + TIP * d - c) @ a) >= TAB_T / 2 + PAST_TAB - EPS
    ball_out = sgn * float((p - c) @ a) <= -(TAB_T / 2 + BALL_L / 2) + EPS
    return bool(inside and tip_past and ball_out and 0.0 < s < TIP)


def seat_gap(objects) -> float:
    """Gap from the ball's innermost point to the seat face (m); negative would mean past it."""
    p, d = _rod_axis(objects["pivot_rod"])
    p, d = RD.T @ (p - C0), RD.T @ d  # in the drain's frame: the port along -y
    ny = abs(float(d[1]))
    inner_y = p[1] + BALL_L / 2 * ny + BALL_R * math.sqrt(max(0.0, 1.0 - ny * ny))
    return float(-SEAT_OUT - inner_y)


def seated(objects) -> bool:
    p, _ = _rod_axis(objects["pivot_rod"])
    p = RD.T @ (p - C0)
    radial = math.hypot(p[0], p[2] - HP)
    return bool(seat_gap(objects) <= SEAT_GAP + EPS and radial <= BALL_OFF + EPS and p[1] < -SEAT_OUT)


def in_guide(objects) -> bool:
    st = objects["stopper"]
    p = np.asarray(st.pos, float)
    tilt = math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0)))
    return bool(math.hypot(p[0] - BX, p[1] - BY) <= GUIDE_OFF + EPS and tilt <= GUIDE_TILT + EPS
                and HF - 0.001 - EPS <= p[2] <= HF + GUIDE_LIFT + EPS)


# Release check: in the simulator an on_step hook measures, every step, the shortest distance from both arms' finger
# and gripper hulls to every shape of the rod and the stopper. Before the first measurement (no step since the reset)
# nothing is credited: the check fails closed. Offline (near_misses.py sets OFFLINE_FLAG; no simulator, no robot)
# each case stands for a state with the grippers away.
_GAP: dict[str, float] = {}
OFFLINE_FLAG = "STOPPER_TASK_OFFLINE_CHECK"
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_SPECS = {o.name: o for o in OBJECTS}


# The 1 s hold: the on_step hook also counts consecutive control steps on which the rod is through the eye, the
# stopper is in its guide and both parts are let go and still. Credit needs 20 such steps (1 s at 20 Hz). Before the
# first count (no step since the reset) nothing is credited; offline, near_misses.py's flag stands for a held state.
HOLD_STEPS = 20
_HOLD: dict[str, int] = {}


@parts.on_reset
def _clear(sim):
    _GAP.clear()
    _HOLD.clear()


@parts.on_step
def _record_gaps(sim):
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    states = sim.states()
    hands = []
    for arm in ("left_robot", "right_robot"):
        rob = base.scene[arm]
        poses = rob.data.body_link_pose_w
        poses = (poses.torch if hasattr(poses, "torch") else poses)[0].cpu().numpy().astype(float)
        links = {}
        for i, n in enumerate(rob.body_names):
            if n in _HAND_LINKS:
                t = np.eye(4)
                t[:3, :3] = _rot(poses[i, 3:])
                t[:3, 3] = poses[i, :3] - origin
                links[n] = t
        hands += analysis.world_hulls(links)
    for name in ("pivot_rod", "stopper"):
        st = states[name]
        boxes = _SPECS[name].world_boxes({"base": (np.asarray(st.pos, float), _rot(st.quat_xyzw))})
        _GAP[name] = min(float(analysis.hulls_to_box(hands, c, r, sz)[0]) for _, c, r, sz in boxes)
    held = captured(states) and in_guide(states) and _settled(states)
    _HOLD["n"] = _HOLD.get("n", 0) + 1 if held else 0


def _released(name) -> bool:
    if name not in _GAP:
        return os.environ.get(OFFLINE_FLAG) == "1"
    return _GAP[name] >= RELEASE_GAP - EPS


def _rod_still(st) -> bool:
    """Still: moving under 1 cm/s and swinging under 0.5 rad/s. Spin about the rod's own axis is left out: a round
    rod turning in place about its centreline moves nothing (in the simulator it can keep a slow roll in the port)."""
    _, d = _rod_axis(st)
    w = np.asarray(st.ang_vel, float)
    return bool(np.linalg.norm(st.lin_vel) < 0.01 and np.linalg.norm(w - (w @ d) * d) < 0.5)


def _settled(objects) -> bool:
    return bool(_rod_still(objects["pivot_rod"]) and objects["stopper"].resting()
                and _released("pivot_rod") and _released("stopper"))


def success(objects):
    """Reconnected: the rod through the stopper's eye, its ball on the seat, the stopper in the drain, let go, still."""
    return bool(captured(objects) and seated(objects) and in_guide(objects) and _settled(objects))


def _held() -> bool:
    if "n" not in _HOLD:
        return os.environ.get(OFFLINE_FLAG) == "1"
    return _HOLD["n"] >= HOLD_STEPS


def score(objects):
    """1.0 reconnected with the ball on its seat; 0.5 everything else holds but the ball is not seated; else 0. Each
    needs the state held for 1 s (20 control steps)."""
    if not (captured(objects) and in_guide(objects) and _settled(objects) and _held()):
        return 0.0
    return 1.0 if seated(objects) else 0.5


# -- demo ------------------------------------------------------------------------------------------------------
TIP_DZ = 0.013  # finger tips below the TCP
LIFT = HP - (HF + EYE_Z)  # 26 mm: the lift that brings the eye level with the port
L_GRIP_Z = HF + 0.002 + TIP_DZ  # tips 2 mm above the flange, pads on the head's rim
L_OPEN = -(HEAD_R + 0.008)
L_CLOSE = -(HEAD_R - 0.003)
L_DZ = 0.0025  # carry compensation, set from traces
R_DZ = 0.0023
R_DY = -0.0008
R_DX = 0.0
GRIP_OUT = 0.100  # the right arm grips the rod 100 mm out from the ball's centre
R_PINCH = 0.009  # TCP above the rod's axis
R_OPEN = -0.012
TURN = -math.pi / 2
R_YAW = math.pi / 2 - (math.pi / 2 - PORT_DIR) % math.pi  # fingers close across the rod, in (-90, 90] deg


def demo():
    """Left arm: pinch the stopper's head, lift it 26 mm and turn it a quarter turn, hold it there, then let go once
    the rod is in. Right arm: pick the rod up from its rests, line it up with the port and push it straight in until
    the ball reaches its seat, let go and move away. Open loop, from the published start layout."""
    hz = L_GRIP_Z
    left = [
        Waypoint(1.5, (BX, BY, hz + 0.03), L_OPEN, "over the stopper"),
        Waypoint(1.5, (BX, BY, hz), L_OPEN, "down round the head"),
        Waypoint(0.8, (BX, BY, hz), L_CLOSE, "pinch the head"),
        Waypoint(1.2, (BX, BY, hz + LIFT + L_DZ), L_CLOSE, "lift to the port"),
        Waypoint(1.5, (BX, BY, hz + LIFT + L_DZ), L_CLOSE, "quarter turn", TURN),
        Waypoint(5.5, (BX, BY, hz + LIFT + L_DZ), L_CLOSE, "hold while the rod goes in", TURN),
        Waypoint(0.6, (BX, BY, hz + LIFT + L_DZ), L_OPEN, "let go", TURN),
        Waypoint(1.5, (BX, BY + 0.07, hz + LIFT + L_DZ), L_OPEN, "slide off sideways, pads clear of the head", TURN),
        Waypoint(1.5, (BX - 0.08, BY + 0.10, hz + 0.03), L_OPEN, "away", TURN),
    ]
    sz = ROD_START[2]
    out = U[:2]  # outward along the port
    g0 = np.array(ROD_START[:2]) + out * GRIP_OUT  # grip point on the rod at its start
    zr = HP + R_PINCH + R_DZ
    fix = np.array([R_DX, R_DY])
    pre = SEATED[:2] + out * (0.062 + GRIP_OUT) + fix  # tip 6 mm short of the spout
    home = SEATED[:2] + out * (GRIP_OUT - 0.002) + fix  # pushed 2 mm past the seat
    back = home + out * 0.03
    away = home + out * 0.06 + np.array([-0.06, -0.06])

    def w(t, xy, z, f, label):
        return Waypoint(t, (float(xy[0]), float(xy[1]), z), f, label, R_YAW)

    right = [
        w(1.5, g0, sz + 0.06, R_OPEN, "over the rod"),
        w(1.2, g0, sz + R_PINCH, R_OPEN, "down"),
        w(0.8, g0, sz + R_PINCH, FINGER_CLOSED, "grip the rod"),
        w(1.2, g0, zr + 0.02, FINGER_CLOSED, "lift"),
        w(2.0, pre, zr, FINGER_CLOSED, "line up with the port"),
        w(0.5, pre, zr, FINGER_CLOSED, "wait for the stopper"),
        w(3.0, home, zr, FINGER_CLOSED, "push in to the seat"),
        w(0.6, home, zr, R_OPEN, "let go"),
        w(0.8, home, zr + 0.03, R_OPEN, "straight up"),
        w(1.0, back, zr + 0.05, R_OPEN, "back off"),
        w(5.0, away, zr + 0.06, R_OPEN, "away"),
    ]
    return {"left": left, "right": right}
