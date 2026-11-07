"""On the station table, a little in front of the robot on the midline: a spring winder. A round grey winding cone
lies flat on a low dark base and turns about a vertical shaft; a torsion spring under it pulls it back clockwise
(seen from above) against a stop. Four upright bars stand in sockets near the cone's rim, a quarter turn apart: one
blue, three orange. A dark hub covers the cone's middle. Two dark bridge bars on posts cross over the rim, a near one
on the robot's side and a far one opposite, just above the bars' tops. A red set pin with a round red knob passes
down through the near bridge; it starts raised.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
SUCCESS_HOLD_S = 3.0

DARK, GREY, ORANGE = (0.18, 0.19, 0.21), (0.62, 0.63, 0.65), (0.95, 0.45, 0.08)
RED, BLUE = (0.85, 0.10, 0.10), (0.15, 0.30, 0.85)

# Winder frame: origin on the table under the cone's axis.
CX, CY = 0.22, 0.0
BASE_H = 0.02  # base top
DISC_R, DISC_T = 0.135, 0.02  # cone disc, top at 0.04
DISC_TOP = BASE_H + DISC_T
PEG_R, PEG_H, PEG_RAD = 0.008, 0.04, 0.12  # bars: 16 mm across, 40 mm tall, on a 120 mm radius; tops at 0.08
NEAR_DEG = 200.0  # near bridge (degrees anticlockwise from straight ahead, x), toward the right arm
FAR_DEG = NEAR_DEG - 180.0
PIN_DEG = NEAR_DEG  # set pin in the near bridge
PIN_R = 0.005
CONTACT_DEG = math.degrees((PEG_R + PIN_R) / PEG_RAD)  # a bar resting on the pin is this far past the pin's angle
PEG0_DEG = PIN_DEG + CONTACT_DEG + 2.0  # the blue bar starts 2 degrees past the pin, the cone on its stop
PEG_DEGS = [(PEG0_DEG + 90.0 * k) % 360.0 for k in range(4)]
BLUE_BAR = 3  # the bar at 118 degrees, in plain view on the left at the start
BRIDGE_Z0, BRIDGE_T = 0.088, 0.010  # underside 8 mm above the bar tops
BRIDGE_R0, BRIDGE_R1 = 0.07, 0.15
PIN_TRAVEL = 0.017
PIN_TIP_UP = 0.085  # raised: tip 5 mm above the bar tops
KNOB_R, KNOB_H = 0.010, 0.015
KNOB_Z0 = 0.115  # raised knob's underside; pressed it sits 10 mm above the bridge's top
SPRING = 0.03  # N m/rad
UPPER = math.radians(300.0)


def _polar(deg, r=PEG_RAD):
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


PIN_XY = _polar(PIN_DEG)


def _bridge(deg):
    """A stepped bar along the radius at deg, built from 12 mm blocks so it can lie at any angle: 31 mm wide, from
    6.9 to 15.1 cm out, with a post under its outer end."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    n = int(round((BRIDGE_R1 - BRIDGE_R0) / 0.01))
    out = []
    for i in range(n):
        r = BRIDGE_R0 + 0.01 * (i + 0.5)
        for w in (-0.0095, 0.0, 0.0095):
            out.append(parts.box((0.012, 0.012, BRIDGE_T), at=(r * c - w * s, r * s + w * c,
                                                               BRIDGE_Z0 + BRIDGE_T / 2), color=DARK))
    post = 0.015
    out.append(parts.box((post, post, BRIDGE_Z0), at=((BRIDGE_R1 + post / 2) * c, (BRIDGE_R1 + post / 2) * s,
                                                     BRIDGE_Z0 / 2), color=DARK))
    return out


HUB_R = 0.07  # fixed hub over the cone's middle, up to the bridges' tops: nothing can press the disc inside 7 cm
BASE_SHAPES = [
    parts.box((0.28, 0.28, BASE_H), at=(0.0, 0.0, BASE_H / 2), color=DARK),
    parts.cylinder(HUB_R, BRIDGE_Z0 + BRIDGE_T - BASE_H, at=(0.0, 0.0, (BASE_H + BRIDGE_Z0 + BRIDGE_T) / 2),
                   color=DARK, axis="z"),
    *_bridge(NEAR_DEG),
    *_bridge(FAR_DEG),
]

CONE = parts.Hinge(
    "cone",
    shapes=[
        parts.cylinder(DISC_R, DISC_T, at=(0.0, 0.0, BASE_H + DISC_T / 2), color=GREY, axis="z"),
        *[parts.cylinder(PEG_R, PEG_H, at=(*_polar(d), DISC_TOP + PEG_H / 2), color=BLUE if k == BLUE_BAR else ORANGE,
                         axis="z") for k, d in enumerate(PEG_DEGS)],
    ],
    pivot=(0.0, 0.0, BASE_H),
    axis=(0.0, 0.0, 1.0),  # positive: anticlockwise seen from above (winding)
    lower=0.0,
    upper=UPPER,
    mass=0.3,
    spring=SPRING,
    damping=0.002,
)
PIN_LEN = KNOB_Z0 - PIN_TIP_UP
PIN = parts.Slider(
    "pin",
    shapes=[
        parts.cylinder(PIN_R, PIN_LEN, at=(*PIN_XY, PIN_TIP_UP + PIN_LEN / 2), color=RED, axis="z"),
        parts.cylinder(KNOB_R, KNOB_H, at=(*PIN_XY, KNOB_Z0 + KNOB_H / 2), color=RED, axis="z"),
    ],
    axis=(0.0, 0.0, -1.0),
    travel=PIN_TRAVEL,
    mass=0.03,
    spring=50.0,
    damping=1.0,
)
WINDER = parts.Part("winder", pos=(CX, CY, TOP), shapes=BASE_SHAPES, joints=[CONE, PIN])
OBJECTS = [WINDER]

PIN_HOLD_UP = -0.012  # unlatched, the spring holds the pin up against its top stop (0.6 N, it weighs 0.3 N)
PIN_LATCH = 0.014  # pressed this far, the pin's catch holds it down
PIN_FREE = 0.010  # pulled back up past here, the catch lets go
SLIP_DEG = 30.0
TIER_TOL = 3.0
CLEAR = 0.03  # released: no gripper or finger surface within 3 cm of the cone (disc and bars) or the pin's knob
TIERS = ((268.0, 1.0), (178.0, 0.5), (88.0, 0.25))  # the lock angles: a bar resting on the pin

# Whole-trial record, kept by the simulator's step callback (and by track() offline).
_H = {"max": 0.0, "slipped": False, "latched": False, "clear": True, "still": True, "last": None, "sim": False}


def _clear_history():
    _H.update(max=0.0, slipped=False, latched=False, clear=True, still=True, last=None)


def track(cone_rad: float, pin_m: float) -> None:
    """Update the whole-trial record from one step's joint positions (called after every control step)."""
    deg = math.degrees(cone_rad)
    _H["max"] = max(_H["max"], deg)
    if deg < _H["max"] - SLIP_DEG - EPS:
        _H["slipped"] = True
    if pin_m >= PIN_LATCH - EPS:
        _H["latched"] = True
    elif pin_m < PIN_FREE:
        _H["latched"] = False
    last = _H["last"]
    _H["still"] = last is not None and abs(deg - last) < 0.05  # under 1 degree per second at 20 Hz
    _H["last"] = deg


_VARIANT = os.environ.get("SPRING_DEMO", "")  # "letgo", "press": the replays described in the README
_LINKS = ("gripper", "tip_left", "tip_right")


def _rot(q):
    return parts._quat_to_matrix(q)


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


def _cyl_points(c, r, z0, z1):
    a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    zs = np.linspace(z0, z1, max(2, int((z1 - z0) / 0.005) + 1))
    return np.stack([np.tile(c[0] + r * np.cos(a), len(zs)), np.tile(c[1] + r * np.sin(a), len(zs)),
                     np.repeat(zs, len(a))], 1)


def _grip_points(cone_rad, pin_m):
    pts = []
    for d in PEG_DEGS:
        x, y = _polar(d + math.degrees(cone_rad))
        pts.append(_cyl_points((CX + x, CY + y), PEG_R, TOP + DISC_TOP, TOP + DISC_TOP + PEG_H))
    a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    for r in np.arange(HUB_R, DISC_R + 1e-9, 0.01):  # the disc's open top, in rings 1 cm apart
        pts.append(np.stack([CX + r * np.cos(a), CY + r * np.sin(a), np.full_like(a, TOP + DISC_TOP)], 1))
    pts.append(_cyl_points((CX, CY), DISC_R, TOP + BASE_H, TOP + DISC_TOP))  # its rim
    kz = TOP + KNOB_Z0 - pin_m
    pts.append(_cyl_points((CX + PIN_XY[0], CY + PIN_XY[1]), KNOB_R, kz, kz + KNOB_H))
    return np.concatenate(pts)


def _clearance(hulls, pts) -> float:
    best = np.inf
    for _, h in hulls:
        d = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)  # a lower bound on each point's distance
        best = min(best, float(d.min()))
    return best


@parts.on_step
def _step(sim):
    _H["sim"] = True
    w = sim.states()["winder"]
    cone, pin = float(w.joints["cone"]), float(w.joints["pin"])
    track(cone, pin)
    sim.set_target("winder", "pin", PIN_TRAVEL + 0.005 if _H["latched"] else PIN_HOLD_UP)
    _H["clear"] = _clearance(_robot_hulls(sim), _grip_points(cone, pin)) >= CLEAR - EPS


@parts.on_reset
def _reset(sim):
    _clear_history()
    sim.set_target("winder", "pin", PIN_HOLD_UP)


def _grade(objects) -> float:
    if not _H["sim"]:  # offline (near_misses.py): each call is one step of a trial; a start state is a new trial
        cone, pin = float(objects["winder"].joints["cone"]), float(objects["winder"].joints["pin"])
        if abs(cone) < 1e-9 and abs(pin) < 1e-9:
            _clear_history()
        track(cone, pin)
        _H["still"] = True
    if _H["slipped"]:
        return 0.0
    w = objects["winder"]
    if not (w.joints["pin"] >= PIN_LATCH - EPS and _H["clear"] and _H["still"]):
        return 0.0
    deg = math.degrees(w.joints["cone"])
    for target, value in TIERS:
        if abs(deg - target) <= TIER_TOL + EPS:
            return value
    return 0.0


def success(objects):
    """Wound three quarter turns and resting on the pressed set pin, let go and still; never slipped."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 at three quarter turns, 0.5 at two, 0.25 at one, each resting on the pressed pin with the grippers clear;
    0 if the cone ever ran back more than 30 degrees from the most it was wound."""
    return _grade(objects)


# -- scripted demo ----------------------------------------------------------------------------------------------

TIP = 0.013
GRIP_Z = TOP + DISC_TOP + 0.012 + TIP  # fingertips 12 mm above the disc
UP = TOP + 0.15
GRIP_OPEN = FINGER_OPEN  # fingers fully open (about 55 mm) to take a bar
COMP = {"left": (0.0, 0.0), "right": (-0.008, 0.0)}  # measured in a GPU trace: the right arm lands 8-10 mm far
ASIDE = {"left": (0.15, 0.25, TOP + 0.15), "right": (0.15, -0.25, TOP + 0.15)}


def _peg_world(deg):
    x, y = _polar(deg)
    return (CX + x, CY + y)


def _yaw(deg, ref=None):
    """Fingers closing along the bar's path (the tangent) at deg (yaw 0 closes them along world y), with the
    palm turned the way the arm's IK reaches on both sides of the cone (yaw = bar angle - 180 deg)."""
    y = deg - 180.0
    while y >= 190.0:
        y -= 360.0
    while y < -100.0:
        y += 360.0
    return math.radians(y)


def _drive(wps, start_deg, end_deg, label, step=8.0, dt=0.3, ref=None):
    """The gripped bar carried along its arc from start_deg to end_deg, the grip turning with it."""
    ref = (start_deg + end_deg) / 2 if ref is None else ref
    n = max(1, int(math.ceil(abs(end_deg - start_deg) / step)))
    for i in range(1, n + 1):
        d = start_deg + (end_deg - start_deg) * i / n
        wps.append(Waypoint(dt, (*_peg_world(d), GRIP_Z), FINGER_CLOSED, f"{label} {i}/{n}", _yaw(d)))


def _take(wps, deg, label, ref, side):
    xy, yaw = _peg_world(deg), _yaw(deg)
    xy = (xy[0] + COMP[side][0], xy[1] + COMP[side][1])
    wps += [
        Waypoint(1.2, (*xy, UP), GRIP_OPEN, f"over {label}", yaw),
        Waypoint(1.0, (*xy, GRIP_Z), GRIP_OPEN, f"down to {label}", yaw),
        Waypoint(0.6, (*xy, GRIP_Z), FINGER_CLOSED, f"grip {label}", yaw),
    ]


def _leave(wps, side, label):
    last = wps[-1]
    wps += [
        Waypoint(0.6, last.pos, GRIP_OPEN, f"let go of {label}", last.yaw),
        Waypoint(0.8, (last.pos[0], last.pos[1], last.pos[2] + 0.04), GRIP_OPEN, "up", last.yaw),
        Waypoint(1.0, ASIDE[side], FINGER_OPEN, "aside", 0.0),
    ]


def _t(wps):
    return sum(w.duration for w in wps)


def _hold(wps, until, label="hold"):
    if not wps:
        wps.append(Waypoint(until, ASIDE["left" if label == "left" else "right"], FINGER_OPEN, "wait"))
        return
    last = wps[-1]
    if until > _t(wps) + 1e-9:
        wps.append(Waypoint(until - _t(wps), last.pos, last.finger, "hold", last.yaw))


def _handover(giver, g_side, taker, t_side, take_deg, label, ref):
    """taker grips the bar at take_deg while giver holds; then giver lets go and moves aside."""
    start = _t(giver) - 2.8  # taker comes in during the giver's last moves
    _hold(taker, max(_t(taker), start), t_side)
    _take(taker, take_deg, label, ref, t_side)
    _hold(giver, _t(taker) + 0.8)
    _hold(taker, _t(giver))
    _leave(giver, g_side, "the bar")
    _hold(taker, _t(giver) - 1.4)


# Quarter-turn plan (bar angles in degrees): each arm works its own side of the bridges, the left arm from the far
# bridge round to the near one, the right arm from the near bridge round to the far one.
PLAN_END = 282.0  # wound past the 268 deg lock, so the pin can drop in clear of the bars
EASE_TO = 262.0
LEFT_END = NEAR_DEG - 30.0  # 170: where the left arm hands over
RIGHT_END = FAR_DEG + 360.0 - 50.0  # 330: where the right arm hands over (its IK reaches no further)


def demo():
    """Left arm takes the bar on its side and winds; the right arm takes a bar on its side before the left lets go,
    and winds on; the arms trade twice more. Past three quarter turns the left arm presses the set pin, then the
    right arm eases its bar back until a bar rests on the pin, and lets go."""
    left, right = [], []
    start = next(d for d in PEG_DEGS if FAR_DEG + 60.0 < d < LEFT_END)  # the bar on the left arm's side
    if _VARIANT == "press":  # shortcut replay: closed fingers pressed on the open disc top and dragged round
        r, z = 0.10, TOP + DISC_TOP - 0.004 + TIP
        p = lambda d: (CX + r * math.cos(math.radians(d)), CY + r * math.sin(math.radians(d)))
        left += [Waypoint(1.5, (*p(50.0), UP), FINGER_CLOSED, "over the disc", _yaw(50.0)),
                 Waypoint(1.0, (*p(50.0), z), FINGER_CLOSED, "press on the disc", _yaw(50.0))]
        for i in range(1, 9):
            d = 50.0 + 70.0 * i / 8
            left.append(Waypoint(0.35, (*p(d), z), FINGER_CLOSED, f"drag {i}/8", _yaw(d)))
        _hold(left, _t(left) + 2.0)
        _hold(right, _t(left))
        return {"left": left, "right": right}
    _take(left, start, "bar A", (start + LEFT_END) / 2, "left")
    _drive(left, start, LEFT_END, "wind")
    if _VARIANT == "letgo":  # failure replay: one arm winds as far as it can follow, then lets go to regrasp
        _leave(left, "left", "the bar")
        _take(left, start, "bar A again", (start + LEFT_END) / 2, "left")
        _hold(left, _t(left) + 2.0)
        _hold(right, _t(left))
        return {"left": left, "right": right}
    phi, stop = LEFT_END - start, LEFT_END
    # Each handover: the taker grips the bar a quarter turn ahead of the giver's.
    for k, (giver, g_side, taker, t_side, end) in enumerate([
            (left, "left", right, "right", RIGHT_END), (right, "right", left, "left", LEFT_END + 360.0),
            (left, "left", right, "right", None)]):
        a = stop + 90.0
        stop = end if end is not None else a + (PLAN_END - phi)
        _handover(giver, g_side, taker, t_side, a, f"bar {'BCD'[k]}", (a + stop) / 2)
        _drive(taker, a, stop, "wind")
        phi += stop - a
    # The left arm presses the pin while the right holds past three quarter turns.
    px, py = CX + PIN_XY[0], CY + PIN_XY[1]
    knob_top = TOP + KNOB_Z0 + KNOB_H
    _hold(left, _t(right))
    press = knob_top - PIN_TRAVEL - 0.008 + TIP
    left += [
        Waypoint(1.2, (px, py, knob_top + TIP + 0.02), FINGER_CLOSED, "over the pin"),
        Waypoint(0.8, (px, py, knob_top + TIP + 0.003), FINGER_CLOSED, "on the knob"),
        Waypoint(1.0, (px, py, press), FINGER_CLOSED, "press the pin down"),
        Waypoint(0.5, (px, py, press), FINGER_CLOSED, "hold"),
        Waypoint(0.8, (px, py, knob_top + TIP + 0.02), FINGER_CLOSED, "up"),
        Waypoint(1.0, ASIDE["left"], FINGER_OPEN, "aside"),
    ]
    _hold(right, _t(left) - 1.0)
    _drive(right, stop, stop - (PLAN_END - EASE_TO), "ease back onto the pin", step=4.0, dt=0.4,
           ref=(a + stop) / 2)
    _leave(right, "right", "the bar")
    return {"left": left, "right": right}
