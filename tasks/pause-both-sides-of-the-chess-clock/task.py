"""On the station table, between the arms: a mechanical chess clock in a wooden case, its two clock faces toward the
robot, each with a black minute hand and a small red flag at twelve. On top is a seesaw rocker lever with a black
button cap at each end; one cap is raised and the other pressed down. Time-setting knobs are on the back.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so the reset hooks run before the lots settle)
import lots  # noqa: E402
import tilt  # noqa: E402  (leaned fingertips for the scripted demo only)
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WOOD, DARK_WOOD = (0.55, 0.33, 0.17), (0.35, 0.20, 0.10)
BLACK, WHITE, RED, BRASS = (0.06, 0.06, 0.06), (0.95, 0.95, 0.92), (0.85, 0.10, 0.10), (0.75, 0.60, 0.28)

# Clock frame: origin at the centre of the case's underside; faces toward -x (the robot), +y is the robot's left.
CASE = (0.095, 0.160, 0.070)
FACE_R, FACE_Y, FACE_Z = 0.028, 0.040, 0.036
FRONT_X = -CASE[0] / 2
PIVOT_Z = CASE[2] + 0.005
CAP_Y, CAP_R, CAP_H = 0.050, 0.009, 0.010
BEAM = (0.014, 0.120, 0.006)
RISE = 0.006  # the raised cap stands 6 mm above level at its stop, the pressed one 6 mm below
STOP = math.asin(RISE / CAP_Y)  # 6.9 degrees
CAP_TOP_LEVEL = PIVOT_Z + BEAM[2] / 2 + CAP_H  # 88 mm above the table, with the rocker level

CLOCK_XY = (0.30, 0.0)

BODY = [
    parts.box(CASE, at=(0.0, 0.0, CASE[2] / 2), color=WOOD),
    parts.box((0.014, 0.014, 0.005), at=(0.0, 0.0, CASE[2] + 0.0025), color=BRASS),  # pivot post
    *[parts.cylinder(FACE_R, 0.001, at=(FRONT_X - 0.0005, s * FACE_Y, FACE_Z), color=WHITE, axis="x")
      for s in (1, -1)],
    *[parts.box((0.002, 0.004, 0.008), at=(FRONT_X - 0.0015, s * FACE_Y, FACE_Z + 0.020), color=RED)
      for s in (1, -1)],  # flags at twelve
    *[parts.cylinder(0.006, 0.010, at=(CASE[0] / 2 + 0.005, s * FACE_Y, FACE_Z), color=DARK_WOOD, axis="x")
      for s in (1, -1)],  # setting knobs on the back
]
ROCKER = parts.Hinge(
    "rocker",
    shapes=[parts.box(BEAM, at=(0.0, 0.0, PIVOT_Z), color=BRASS),
            *[parts.cylinder(CAP_R, CAP_H, at=(0.0, s * CAP_Y, PIVOT_Z + BEAM[2] / 2 + CAP_H / 2), color=BLACK)
              for s in (1, -1)]],
    pivot=(0.0, 0.0, PIVOT_Z), axis=(1.0, 0.0, 0.0),  # positive: the left cap (+y) up
    lower=-STOP, upper=STOP, mass=0.03, spring=0.5, damping=0.01,
)


def _hand(side: str, s: int) -> parts.Hinge:
    """A minute hand, pointing at twelve at joint 0; positive turns it clockwise as the robot sees the face."""
    return parts.Hinge(
        f"{side}_hand",
        shapes=[parts.box((0.0015, 0.0025, 0.022), at=(FRONT_X - 0.0025, s * FACE_Y, FACE_Z + 0.009), color=BLACK)],
        pivot=(FRONT_X - 0.0025, s * FACE_Y, FACE_Z), axis=(1.0, 0.0, 0.0), lower=-4 * math.pi,
        upper=4 * math.pi, mass=0.01, spring=0.05, damping=0.001,
    )


class _Clock(parts.Part):
    """Offline, the clock's state defaults to its start: the current lot's cap raised, both hands at twelve."""

    def state(self, joints=None, pos=None, quat_xyzw=None, lin_vel=None):
        start = {"rocker": (1 if lots.current().raised == "left" else -1) * STOP}
        return super().state(joints=start | dict(joints or {}), pos=pos, quat_xyzw=quat_xyzw, lin_vel=lin_vel)


CLOCK = _Clock("chess_clock", pos=(*CLOCK_XY, TOP), mass=0.45, shapes=BODY,
                   joints=[ROCKER, _hand("left", 1), _hand("right", -1)])
OBJECTS = [CLOCK]

# Lots: which player's button was left up (whose clock runs). The clock itself does not move; the zero move makes
# every lot settle again after the reset hook sets the rocker.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", raised="left", place={"chess_clock": CLOCK.pos}),
    lots.Lot("B", raised="right", place={"chess_clock": CLOCK.pos}),
)

EPS = 1e-6
LEVEL = math.asin(0.0015 / CAP_Y) + EPS  # each cap within 1.5 mm of level: tops within 3 mm of each other
MINUTE_HAND_RATE = 2 * math.pi / 3600  # rad/s: one turn an hour
MOVE, TILT = 0.010, math.radians(5.0)
CLEAR = 0.005
STILL = math.radians(0.1)
BREAKAWAY = math.radians(0.2)  # the rocker's static friction, as an angle of its holding drive (about 0.035 N at a cap)
HOLD_S = 2.0  # score() credits a pause only once it has held this long (counted in the simulator's step hook)
SUCCESS_HOLD_S = 2.1  # the episode ends a moment after score() has seen the full hold


def _start_sign() -> int:
    return 1 if lots.current().raised == "left" else -1


def running(rocker: float) -> str:
    """Which clock runs: the side whose cap is up past the level band; 'none' inside it."""
    return "left" if rocker > LEVEL else "right" if rocker < -LEVEL else "none"


def _tilt(q) -> float:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))


_record = {"fault": False}
_sim = {"on": False, "clear": None, "still": None, "prev": None, "t": None, "hold": 0.0, "held": 0.0}


def _is_start_offline(clock) -> bool:
    return bool(abs(clock.joints["rocker"] - _start_sign() * STOP) < 1e-9
                and np.allclose(clock.pos, lots.current().pos("chess_clock"), atol=1e-9))


def _update(objects) -> None:
    c = objects["chess_clock"]
    if not _sim["on"] and _is_start_offline(c):
        _record["fault"] = False
    other = "right" if _start_sign() > 0 else "left"
    start = np.asarray(lots.current().pos("chess_clock"), float)
    if (running(c.joints["rocker"]) == other or np.linalg.norm(np.asarray(c.pos) - start) > MOVE + EPS
            or _tilt(c.quat_xyzw) > TILT + EPS):
        _record["fault"] = True


def _paused(objects) -> bool:
    c = objects["chess_clock"]
    released = (not _sim["on"]) or (_sim["clear"] is True and _sim["still"] is True)
    return bool(not _record["fault"] and released and c.resting() and running(c.joints["rocker"]) == "none")


def success(objects):
    """Both clocks stopped (rocker level within 1.5 mm at each cap), let go and still; the other clock never ran."""
    _update(objects)
    return _paused(objects)


def distance(objects):
    return abs(objects["chess_clock"].joints["rocker"]) * CAP_Y


def score(objects):
    """Pass/fail: 1.0 once paused (as in success) has held for 2 s, otherwise 0."""
    _update(objects)
    held = (not _sim["on"]) or (_sim["held"] >= HOLD_S - EPS)
    return 1.0 if _paused(objects) and held else 0.0


# -- simulator hooks -------------------------------------------------------------------------------------------------
@parts.on_reset
def _reset(sim) -> None:
    _sim.update(on=True, clear=None, still=None, prev=None, t=None, left=0.0, right=0.0, hold=_start_sign() * STOP, held=0.0)
    _record["fault"] = False
    sim.set_target("chess_clock", "rocker", _sim["hold"])
    for side in ("left", "right"):
        sim.set_target("chess_clock", f"{side}_hand", 0.0)


@parts.on_step
def _step(sim) -> None:
    from yam_arena import analysis

    _sim["on"] = True
    st = sim.states()
    c = st["chess_clock"]
    r = float(c.joints["rocker"])
    if abs(r - _sim["hold"]) > BREAKAWAY:  # friction: pushed past its breakaway, the rocker's rest point follows it;
        _sim["hold"] = r  # otherwise it is held where it was left
    sim.set_target("chess_clock", "rocker", _sim["hold"])
    prev = _sim["prev"]
    _sim["still"] = prev is not None and abs(r - prev) < STILL
    _sim["prev"] = r
    t = float(sim.t)
    dt = 0.0 if _sim["t"] is None else max(0.0, t - _sim["t"])
    _sim["t"] = t
    side = running(r)
    if side != "none":  # the running clock's minute hand advances
        _sim[side] += MINUTE_HAND_RATE * dt
    for s in ("left", "right"):
        sim.set_target("chess_clock", f"{s}_hand", _sim[s])
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    hulls, bodies = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = parts._quat_to_matrix(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
                bodies += 1
    links = {"base": (np.asarray(c.pos), parts._quat_to_matrix(c.quat_xyzw))}
    links.update({k: (np.asarray(v[0]), parts._quat_to_matrix(v[1])) for k, v in c.links.items()})
    _sim["clear"] = bool(bodies == 6 and hulls and all(
        analysis.hulls_to_box(hulls, cc, rr, ss)[0] > CLEAR for _, cc, rr, ss in CLOCK.world_boxes(links)))
    _update(st)  # held time of an unbroken pause, for score()
    _sim["held"] = _sim["held"] + dt if _paused(st) else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
TIP = 0.013
LEAN = 30.0  # degrees: each arm's fingertips lean toward the midline, so the two elbows stay clear of each other
PRESS_DZ = 0.0  # commanded fingertip height relative to a level cap's top


def _tcp_for_tip(arm: str, tip):
    """The TCP that puts the closed fingertips at `tip` with the lean toward the midline."""
    s = 1 if arm == "left" else -1
    t = math.radians(LEAN)
    # fingertips sit TIP along the leaned approach axis: toward the midline (-s y) and down
    return (tip[0], tip[1] + s * TIP * math.sin(t), tip[2] + TIP * math.cos(t))


def demo():
    """Both arms at once, the same plan in every trial: each brings its closed fingertips down onto its own cap and
    stops at the height a level cap's top has, so the raised cap is pushed down and the low one rises against the
    other hand until both are level; both hold, then lift away together. No reading of which side is up."""
    plan = {}
    for arm, s in (("left", 1), ("right", -1)):
        lean = dict(tilt=LEAN, heading=-s * math.pi / 2)
        x, y = CLOCK_XY[0], CLOCK_XY[1] + s * CAP_Y
        above = _tcp_for_tip(arm, (x, y, TOP + CAP_TOP_LEVEL + RISE + 0.030))
        near = _tcp_for_tip(arm, (x, y, TOP + CAP_TOP_LEVEL + RISE + 0.003))
        press = _tcp_for_tip(arm, (x, y, TOP + CAP_TOP_LEVEL + PRESS_DZ))
        plan[arm] = [
            tilt.Waypoint(2.0, above, FINGER_CLOSED, "over its cap", **lean),
            tilt.Waypoint(1.0, near, FINGER_CLOSED, "down to the raised cap's height", **lean),
            tilt.Waypoint(2.0, press, FINGER_CLOSED, "both down to the level height", **lean),
            tilt.Waypoint(1.0, press, FINGER_CLOSED, "hold", **lean),
            tilt.Waypoint(1.5, above, FINGER_CLOSED, "both lift away", **lean),
            tilt.Waypoint(4.0, above, FINGER_CLOSED, "wait", **lean),
        ]
    return plan
