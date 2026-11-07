"""On the station table, in front of the right arm: a brass gate valve fixed to the table, its stem pointing up. On
top of the stem is a four-spoke star handle that turns flat, a short upright knob at the end of each spoke; one knob
is red. A small white tab on the table beside the valve marks a position on the handle's circle. A pipe rack, two
grey pipes on two posts, runs left to right just above the far part of the handle. On the valve's front, a position
indicator: a white scale post with a green band at the top and a red band at the bottom, and a black pointer, now at
the green band. The valve is open.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BRASS, STEEL = (0.78, 0.62, 0.28), (0.60, 0.62, 0.64)
BLACK, RED, WHITE = (0.12, 0.12, 0.13), (0.85, 0.12, 0.12), (0.95, 0.95, 0.95)
GREEN = (0.15, 0.65, 0.25)

# The valve's frame: origin on the table on the stem's axis. The part is turned -90 degrees, so its +x spoke (the red
# knob) starts at world angle 270 degrees (to the robot's right; angles from +x toward +y) and the spokes point at
# 0, 90, 180 and 270 degrees.
VX, VY = 0.30, -0.12
YAW = -math.pi / 2
HUB_Z = 0.100  # handle height above the table
SPOKE_R = 0.068  # axis to knob centre
BAR = (0.150, 0.014, 0.012)
KNOB_R, KNOB_H = 0.007, 0.024
KNOB_Z = HUB_Z + BAR[2] / 2 + KNOB_H / 2
TURNS = 1.5
CLOSED = TURNS * 2 * math.pi  # rad, clockwise seen from above, from fully open to the closed stop
DAMPING = 0.03  # N m s/rad

BODY = [
    parts.box((0.06, 0.06, 0.06), at=(0.0, 0.0, 0.03), color=BRASS),  # valve body
    parts.cylinder(0.035 / 2, 0.10, at=(0.0, 0.0, 0.03), color=BRASS, axis="x"),  # pipe ends either side
    parts.cylinder(0.010, 0.034, at=(0.0, 0.0, 0.077), color=STEEL),  # bonnet and stem, up to z 0.094
]
# The white tab on the table under the red knob's closed position. The red knob starts on the part's +x spoke and
# turns 1.5 turns clockwise, so it ends on the part's -x spoke.
MARK = parts.box((0.012, 0.012, 0.003), at=(-SPOKE_R, 0.0, 0.0015), color=WHITE)

# Position indicator on the valve's front (toward the robot; part -y): a white scale post with a green OPEN band at
# the top and a red SHUT band at the bottom, and a black pointer that the valve's stem drives down the post as the
# handle closes (1.5 turns = 45 mm). It sits below the handle, clear of the spokes and the fingers.
IND_Y = -0.040  # part frame
IND_TRAVEL = 0.045
IND_TOP = 0.085
IND_MASS, IND_SPRING = 0.02, 100.0
SCALE = [
    parts.box((0.006, 0.006, IND_TOP), at=(0.0, IND_Y - 0.003, IND_TOP / 2), color=WHITE),
    parts.box((0.0062, 0.0062, 0.012), at=(0.0, IND_Y - 0.003, IND_TOP - 0.010), color=GREEN),
    parts.box((0.0062, 0.0062, 0.012), at=(0.0, IND_Y - 0.003, IND_TOP - 0.010 - IND_TRAVEL), color=RED),
]
INDICATOR = parts.Slider(
    "indicator",
    shapes=[parts.box((0.014, 0.004, 0.004), at=(0.0, IND_Y - 0.008, IND_TOP - 0.010), color=BLACK)],
    axis=(0.0, 0.0, -1.0),
    travel=IND_TRAVEL,
    mass=IND_MASS,
    spring=IND_SPRING,
    damping=1.0,
)

HANDLE = parts.Hinge(
    "handle",
    pivot=(0.0, 0.0, HUB_Z),
    axis=(0.0, 0.0, -1.0),  # positive = clockwise seen from above = closing
    lower=0.0,
    upper=CLOSED,
    mass=0.15,
    damping=DAMPING,
    shapes=[
        parts.cylinder(0.016, 0.016, at=(0.0, 0.0, HUB_Z), color=BLACK),
        parts.box(BAR, at=(0.0, 0.0, HUB_Z), color=BLACK),
        parts.box((BAR[1], BAR[0], BAR[2]), at=(0.0, 0.0, HUB_Z), color=BLACK),
        parts.cylinder(KNOB_R, KNOB_H, at=(SPOKE_R, 0.0, KNOB_Z), color=RED),
        parts.cylinder(KNOB_R, KNOB_H, at=(-SPOKE_R, 0.0, KNOB_Z), color=BLACK),
        parts.cylinder(KNOB_R, KNOB_H, at=(0.0, SPOKE_R, KNOB_Z), color=BLACK),
        parts.cylinder(KNOB_R, KNOB_H, at=(0.0, -SPOKE_R, KNOB_Z), color=BLACK),
    ],
)
VALVE = parts.Part("valve", pos=(VX, VY, TOP), yaw=YAW, shapes=[*BODY, MARK, *SCALE], joints=[HANDLE, INDICATOR])


@parts.on_step
def _drive_indicator(sim):
    """The stem drives the pointer: its rest point follows the handle (less the pointer's own sag on its spring)."""
    a = float(sim.states()["valve"].joints["handle"])
    sim.set_target("valve", "indicator", IND_TRAVEL * a / CLOSED - IND_MASS * 9.81 / IND_SPRING)

# A pipe rack over the far part of the handle, 5 mm above the knobs: two 44 mm pipes side by side running left to right
# on two posts. A hand cannot reach the spokes under them, so the handle cannot be pushed round in one sweep.
PIPE_D = 0.044
RACK_X0 = VX + 0.030  # near edge of the near pipe
RACK_Z0 = KNOB_Z + KNOB_H / 2 + 0.005  # underside of the pipes
RACK_W = 0.30  # along y
PIPE_XS = (RACK_X0 + PIPE_D / 2, RACK_X0 + 1.5 * PIPE_D)
_post_x = (PIPE_XS[0] + PIPE_XS[1]) / 2
PIPE_RACK = parts.Part(
    "pipe_rack", pos=(_post_x, VY, TOP),
    shapes=[parts.cylinder(PIPE_D / 2, RACK_W, at=(x - _post_x, 0.0, RACK_Z0 + PIPE_D / 2), color=STEEL, axis="y")
            for x in PIPE_XS]
    + [parts.box((0.03, 0.03, RACK_Z0), at=(0.0, sgn * (RACK_W / 2 - 0.015), RACK_Z0 / 2), color=STEEL)
       for sgn in (1, -1)],
)
OBJECTS = [VALVE, PIPE_RACK]
SUCCESS_HOLD_S = 3.0  # runs on while the arm lifts away

SHUT_TOL = math.radians(5.0)  # closed: within 5 degrees of the stop
EPS = 1e-6


def turned(objects) -> float:
    """How far the handle has been turned closed from fully open (rad, clockwise seen from above)."""
    return float(objects["valve"].joints["handle"])


def success(objects):
    """Handle at its closed stop (within 5 degrees)."""
    return turned(objects) >= CLOSED - SHUT_TOL - EPS


def distance(objects):
    return max(0.0, CLOSED - turned(objects))


def score(objects):
    """1.0 at the closed stop (within 5 degrees); 0.5 at least one full turn closed; else 0."""
    a = turned(objects)
    if a >= CLOSED - SHUT_TOL - EPS:
        return 1.0
    return 0.5 if a >= 2 * math.pi - EPS else 0.0


# -- scripted demo: the right arm pushes the spokes round, one stroke per spoke ----------------------------------------
# Closed fingers go down in the gap between two spokes, sweep clockwise round the axis pushing the spoke ahead of them
# a little over a quarter turn, lift clear of the knobs, go back and repeat on the next spoke. Fixed waypoints only:
# the spokes come back to the same place after each stroke.
TIP = 0.013
PUSH_R = 0.045  # finger radius from the axis: inside the knobs, outside the hub
PUSH_Z = HUB_Z - 0.003 + TIP  # finger tips 3 mm below the spokes' centre line
CLEAR_Z = KNOB_Z + KNOB_H / 2 + 0.020 + TIP  # finger tips 2 cm above the knobs
START_DEG, END_DEG = 255.0, 110.0  # world angles of the finger, clockwise from START to END
STROKES = 7


def _at(deg, z):
    a = math.radians(deg)
    return (VX + PUSH_R * math.cos(a), VY + PUSH_R * math.sin(a), TOP + z)


def demo():
    w = [Waypoint(2.0, _at(START_DEG, CLEAR_Z), FINGER_CLOSED, "over the gap between two spokes")]
    for k in range(STROKES):
        w.append(Waypoint(0.8, _at(START_DEG, PUSH_Z), FINGER_CLOSED, f"stroke {k + 1}: down beside the spoke"))
        steps = 14
        for i in range(1, steps + 1):
            deg = START_DEG + (END_DEG - START_DEG) * i / steps
            w.append(Waypoint(0.2, _at(deg, PUSH_Z), FINGER_CLOSED, f"stroke {k + 1}: push"))
        w.append(Waypoint(0.6, _at(END_DEG, CLEAR_Z), FINGER_CLOSED, f"stroke {k + 1}: lift clear"))
        w.append(Waypoint(1.0, _at(START_DEG, CLEAR_Z), FINGER_CLOSED, f"stroke {k + 1}: back"))
    return {"right": w}
