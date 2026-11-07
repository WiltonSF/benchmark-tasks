"""On the station table: a dry heat block with two square wells sized for 50 mL tubes, one behind the other. The
far well holds a large orange tube; nothing stands in the near well. To the robot's right, a small bench rack holds
a narrower conical-bottom sample tube with a blue cap, a red band and a red tip. Two identical grey aluminium
reducing sleeves, each with a collar at its top, and a white PTFE well plug are on the bench or in the block.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import lots  # noqa: E402
import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

ALU, DARK_ALU = (0.72, 0.73, 0.75), (0.45, 0.46, 0.48)
ORANGE, BLUE, CLEAR = (0.95, 0.55, 0.15), (0.20, 0.40, 0.85), (0.85, 0.88, 0.90)
WHITE, RACK, RED = (0.96, 0.96, 0.94), (0.30, 0.55, 0.85), (0.80, 0.15, 0.15)


def _socket(name_prefix, cx, cy, bore, depth, floor, outer, height, color):
    """A fixed block with one square bore open at the top: a floor plate and four walls (Box objects)."""
    wx = (outer[0] - bore) / 2
    wy = (outer[1] - bore) / 2
    z0 = TOP
    boxes = [Box(f"{name_prefix}_floor", pos=(cx, cy, z0 + floor / 2), size=(outer[0], outer[1], floor), color=color)]
    h = height - floor
    zc = z0 + floor + h / 2
    boxes += [
        Box(f"{name_prefix}_near", pos=(cx - bore / 2 - wx / 2, cy, zc), size=(wx, outer[1], h), color=color),
        Box(f"{name_prefix}_far", pos=(cx + bore / 2 + wx / 2, cy, zc), size=(wx, outer[1], h), color=color),
        Box(f"{name_prefix}_left", pos=(cx, cy + bore / 2 + wy / 2, zc), size=(bore, wy, h), color=color),
        Box(f"{name_prefix}_right", pos=(cx, cy - bore / 2 - wy / 2, zc), size=(bore, wy, h), color=color),
    ]
    return boxes


# Heat block: 50 mm tall, two 30 mm square wells 45 mm deep (floor 5 mm thick), centres 40 mm apart along x.
WELL = 0.030
BLOCK_H, BLOCK_FLOOR = 0.050, 0.005
WELL_X, WELL_Y = 0.30, 0.0  # the empty well
FULL_X = WELL_X + 0.040  # the occupied well, behind it
BLOCK = _socket("block_empty", WELL_X, WELL_Y, WELL, BLOCK_H - BLOCK_FLOOR, BLOCK_FLOOR, (0.040, 0.060), BLOCK_H,
                DARK_ALU)
BLOCK += _socket("block_full", FULL_X, WELL_Y, WELL, BLOCK_H - BLOCK_FLOOR, BLOCK_FLOOR, (0.040, 0.060), BLOCK_H,
                 DARK_ALU)
BIG_TUBE = Cylinder("large_tube", pos=(FULL_X, WELL_Y, TOP + BLOCK_FLOOR + 0.050), radius=0.0145, height=0.100,
                    color=ORANGE)  # fixed: stands in the far well

# Sample tube (loose): a 16 mm body 85 mm long on a 6 mm tip 15 mm long (the conical end), blue cap at the top.
TUBE_R, TUBE_BODY_H = 0.008, 0.085
TIP_R, TIP_H = 0.003, 0.015
TUBE_LEN = TIP_H + TUBE_BODY_H
TUBE_MASS = 0.02

# Bench rack (fixed): an 18 mm bore, 40 mm deep above a 5 mm floor, with a 10 mm centring pocket for the tip.
RACK_X, RACK_Y = 0.22, -0.17
RACK_BORE, RACK_H, RACK_FLOOR = 0.018, 0.045, 0.005
SEAT = 0.010  # centring pocket: a 10 mm square, open in the middle of a 5 mm tall frame
SEAT_H = 0.005


def _seat_frame(prefix, cx, cy, z0, bore, color, mass=None):
    """Four boxes on a floor leaving a SEAT square pocket in the middle (fixed Boxes)."""
    w = (bore - SEAT) / 2
    zc = z0 + SEAT_H / 2
    return [
        Box(f"{prefix}_seat_n", pos=(cx - SEAT / 2 - w / 2, cy, zc), size=(w, bore, SEAT_H), color=color),
        Box(f"{prefix}_seat_f", pos=(cx + SEAT / 2 + w / 2, cy, zc), size=(w, bore, SEAT_H), color=color),
        Box(f"{prefix}_seat_l", pos=(cx, cy + SEAT / 2 + w / 2, zc), size=(SEAT, w, SEAT_H), color=color),
        Box(f"{prefix}_seat_r", pos=(cx, cy - SEAT / 2 - w / 2, zc), size=(SEAT, w, SEAT_H), color=color),
    ]


RACK_PARTS = _socket("rack", RACK_X, RACK_Y, RACK_BORE, RACK_H - RACK_FLOOR, RACK_FLOOR, (0.040, 0.040), RACK_H, RACK)
RACK_PARTS += _seat_frame("rack", RACK_X, RACK_Y, TOP + RACK_FLOOR, RACK_BORE, RACK)

# Tube shapes in its own frame: origin at the tip's lowest point. The narrow tip is red, and a red label band 50 to
# 70 mm up (above the rack's top) shows the body's width against the well's in the screenshot.
BAND_Z0, BAND_H = 0.050, 0.020
TUBE_SHAPES = [
    parts.cylinder(TIP_R, TIP_H, at=(0.0, 0.0, TIP_H / 2), color=RED),
    parts.cylinder(TUBE_R, TUBE_BODY_H - 0.012, at=(0.0, 0.0, TIP_H + (TUBE_BODY_H - 0.012) / 2), color=CLEAR),
    parts.cylinder(TUBE_R + 0.0003, BAND_H, at=(0.0, 0.0, BAND_Z0 + BAND_H / 2), color=RED),  # label band
    parts.cylinder(TUBE_R + 0.0005, 0.012, at=(0.0, 0.0, TUBE_LEN - 0.006), color=BLUE),  # cap
]
TUBE = parts.Part("sample_tube", pos=(RACK_X, RACK_Y, TOP + RACK_FLOOR + 0.0005), shapes=TUBE_SHAPES, mass=TUBE_MASS)

# Reducing sleeve (loose, aluminium): 22 mm square outside, 20 mm bore, 44 mm below its collar; the collar (36 mm
# square, 10 mm thick) is its top, wider than the well: it rests on the block's top and is what a gripper holds.
SL_OUT, SL_BORE = 0.022, 0.020
SL_BELOW, SL_ABOVE, COLLAR, COLLAR_T = 0.044, 0.0, 0.036, 0.010
SL_FLOOR = 0.003
SL_H = SL_BELOW + COLLAR_T + SL_ABOVE
SL_MASS = 0.04


def _sleeve_shapes():
    w = (SL_OUT - SL_BORE) / 2
    zc = SL_H / 2
    s = [
        parts.box((SL_OUT, SL_OUT, SL_FLOOR), at=(0.0, 0.0, SL_FLOOR / 2), color=ALU),
        parts.box((w, SL_OUT, SL_H), at=(-SL_BORE / 2 - w / 2, 0.0, zc), color=ALU),
        parts.box((w, SL_OUT, SL_H), at=(SL_BORE / 2 + w / 2, 0.0, zc), color=ALU),
        parts.box((SL_BORE, w, SL_H), at=(0.0, SL_BORE / 2 + w / 2, zc), color=ALU),
        parts.box((SL_BORE, w, SL_H), at=(0.0, -SL_BORE / 2 - w / 2, zc), color=ALU),
    ]
    # collar: a frame round the outside, 5 mm wide, from SL_BELOW to SL_BELOW + COLLAR_T
    cw = (COLLAR - SL_OUT) / 2
    cz = SL_BELOW + COLLAR_T / 2
    s += [
        parts.box((cw, COLLAR, COLLAR_T), at=(-SL_OUT / 2 - cw / 2, 0.0, cz), color=DARK_ALU),
        parts.box((cw, COLLAR, COLLAR_T), at=(SL_OUT / 2 + cw / 2, 0.0, cz), color=DARK_ALU),
        parts.box((SL_OUT, cw, COLLAR_T), at=(0.0, SL_OUT / 2 + cw / 2, cz), color=DARK_ALU),
        parts.box((SL_OUT, cw, COLLAR_T), at=(0.0, -SL_OUT / 2 - cw / 2, cz), color=DARK_ALU),
    ]
    # centring pocket on the floor
    w2 = (SL_BORE - SEAT) / 2
    sz = SL_FLOOR + SEAT_H / 2
    s += [
        parts.box((w2, SL_BORE, SEAT_H), at=(-SEAT / 2 - w2 / 2, 0.0, sz), color=ALU),
        parts.box((w2, SL_BORE, SEAT_H), at=(SEAT / 2 + w2 / 2, 0.0, sz), color=ALU),
        parts.box((SEAT, w2, SEAT_H), at=(0.0, SEAT / 2 + w2 / 2, sz), color=ALU),
        parts.box((SEAT, w2, SEAT_H), at=(0.0, -SEAT / 2 - w2 / 2, sz), color=ALU),
    ]
    return s


SLEEVE_X, SLEEVE_Y = 0.24, 0.20
SLEEVE = parts.Part("sleeve", pos=(SLEEVE_X, SLEEVE_Y, TOP + 0.0005), shapes=_sleeve_shapes(), mass=SL_MASS)
# A second, identical sleeve. Lot A: on the bench beyond the first. Lot B: already seated in the near well.
SLEEVE2_X, SLEEVE2_Y = 0.33, 0.20
SLEEVE2 = parts.Part("sleeve_2", pos=(SLEEVE2_X, SLEEVE2_Y, TOP + 0.0005), shapes=_sleeve_shapes(), mass=SL_MASS)

# Well plug (loose, PTFE): 28 mm across, 40 mm tall: fits the bare well. Nearer the left arm than the sleeve.
PLUG_R, PLUG_H = 0.014, 0.040
PLUG = Cylinder("well_plug", pos=(0.18, 0.12, TOP + PLUG_H / 2), radius=PLUG_R, height=PLUG_H, mass=0.03,
                color=WHITE)

OBJECTS = [*BLOCK, BIG_TUBE, *RACK_PARTS, TUBE, SLEEVE, SLEEVE2, PLUG]
SUCCESS_HOLD_S = 3.0  # runs on while the arm lifts away

RIM_Z = TOP + BLOCK_H  # top of the block: where the collar rests
# Lot B puts sleeve_2 in the near well, collar 0.5 mm above the rim, where a placed sleeve was traced to settle
# (3.3 mm beyond and 3.8 mm left of the well's centre), so the tube needs no sleeve fitted: the bench sleeve stays.
PRESEATED = (WELL_X + 0.0033, WELL_Y + 0.0038, RIM_Z - SL_BELOW + 0.0005)
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", fit_sleeve=True),
    lots.Lot("B", fit_sleeve=False, place={"sleeve_2": PRESEATED}),
)
SLEEVES = ("sleeve", "sleeve_2")

EPS = 1e-6
MAX_LEAN = 8.0  # degrees from vertical: a 16 mm tube can lean about 6.3 degrees into a corner of the 20 mm bore
ON = 0.001  # resting: within 1 mm


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def lean_deg(state) -> float:
    """Angle of a part's own up axis from vertical (degrees)."""
    return math.degrees(math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0))))


def _local(state, p):
    """World point of a part-frame point p."""
    return np.asarray(state.pos, float) + _rot(state.quat_xyzw) @ np.asarray(p, float)


def sleeve_seated(s) -> bool:
    """A sleeve (state) upright in the near well with its collar resting on the block top."""
    c = _local(s, (0.0, 0.0, 0.0))
    collar_bottom = s.pos[2] + SL_BELOW  # part origin is the sleeve's underside
    return bool(
        abs(c[0] - WELL_X) <= (WELL - SL_OUT) / 2 + 0.001 + EPS
        and abs(c[1] - WELL_Y) <= (WELL - SL_OUT) / 2 + 0.001 + EPS
        and lean_deg(s) <= 2.0 + EPS
        and abs(collar_bottom - RIM_Z) <= 0.002 + EPS
    )


def tube_in_sleeve(objects, sleeve="sleeve") -> bool:
    """Tube tip in the sleeve's centring pocket, resting on the sleeve floor, axis within MAX_LEAN of vertical."""
    t, s = objects["sample_tube"], objects[sleeve]
    tip = _local(t, (0.0, 0.0, 0.0))
    seat_floor = _local(s, (0.0, 0.0, SL_FLOOR))
    return bool(
        np.hypot(tip[0] - seat_floor[0], tip[1] - seat_floor[1]) <= SEAT / 2 + EPS
        and abs(tip[2] - seat_floor[2]) <= ON + EPS
        and lean_deg(t) <= MAX_LEAN + EPS
    )


def plug_clear(objects) -> bool:
    """The well plug is not in or over the empty well."""
    p = objects["well_plug"]
    return not (abs(p.pos[0] - WELL_X) < WELL / 2 + PLUG_R and abs(p.pos[1] - WELL_Y) < WELL / 2 + PLUG_R
                and p.low[2] < RIM_Z + 0.02)


def on_table(s) -> bool:
    """A sleeve's lowest point within 1 mm of the table top (standing or lying on the bench, not on the block)."""
    return bool(s.low[2] <= TOP + ON + EPS)


def _grade(objects) -> float:
    """The same rule in every lot: one sleeve seated in the near well with the tube standing in it, the other
    sleeve on the table, the plug clear, all four loose objects still."""
    still = all(objects[n].resting() for n in ("sample_tube", *SLEEVES, "well_plug"))
    if not (still and plug_clear(objects)):
        return 0.0
    for used, other in (SLEEVES, SLEEVES[::-1]):
        if sleeve_seated(objects[used]) and tube_in_sleeve(objects, used) and on_table(objects[other]):
            return 1.0
    return 0.0


def success(objects):
    """One sleeve seated in the near well, tube standing in it, the other sleeve on the table, all still."""
    return _grade(objects) == 1.0


def score(objects):
    """Pass/fail: 1.0 as success, else 0 (tube leaning in the bare well, a sleeve on another, plug in the well)."""
    return _grade(objects)


def distance(objects):
    t = objects["sample_tube"]
    return float(np.hypot(t.pos[0] - WELL_X, t.pos[1] - WELL_Y))


# -- scripted demo ------------------------------------------------------------------------------------------------
TIP = 0.013
UP = 0.15
TUBE_GRIP = 0.044  # finger tips this far below the tube's top: 5 mm above the sleeve's top once placed
UP_TUBE = 0.150
# Measured on the L40S: carrying a light part, the arm's TCP settles about 2.4 mm to the arm's outside and 4 mm low.
SAG_Z = 0.004
SAG_Y = {"left": 0.0024, "right": -0.0024}  # assumed mirror image for the right arm; check in a trace


def demo():
    # Lot B (a sleeve already seated in the well, as the cameras show): the left arm only moves to its parking spot
    # and the right arm racks the tube in that sleeve on the same waypoints. Lot A: the left arm fits the bench sleeve first.
    lot = lots.current()
    fit = lot.fit_sleeve
    # Left arm: grip the sleeve's top walls, carry it over the empty well, lower it to 4 mm above seating, let go.
    sl_top = TOP + SL_H
    g_sl = TOP + SL_BELOW + 0.003 + TIP  # tips 3 mm above the collar's underside: the pads hold its sides
    seat_drop = 0.004
    g_sl_place = RIM_Z + seat_drop + 0.003 + TIP + SAG_Z
    # Measured in a trace: the gripped sleeve sat 3.5 mm nearer the robot and 9.8 mm to the right of where a
    # straight-down carry put it (it settles off-centre between the pads), so the demo aims off by that much.
    lx, ly = WELL_X + 0.0035, WELL_Y - SAG_Y["left"] + 0.0122
    left = [
        Waypoint(1.5, (SLEEVE_X, SLEEVE_Y, TOP + UP), FINGER_OPEN, "over the sleeve"),
        Waypoint(1.2, (SLEEVE_X, SLEEVE_Y, g_sl), FINGER_OPEN, "down to its top"),
        Waypoint(0.8, (SLEEVE_X, SLEEVE_Y, g_sl), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (SLEEVE_X, SLEEVE_Y, TOP + UP), FINGER_CLOSED, "lift"),
        Waypoint(1.8, (lx, ly, TOP + UP), FINGER_CLOSED, "over the empty well"),
        Waypoint(2.0, (lx, ly, g_sl_place), FINGER_CLOSED, "lower into the well"),
        Waypoint(0.6, (lx, ly, g_sl_place), FINGER_OPEN, "let go"),
        Waypoint(1.0, (lx, ly, TOP + UP), FINGER_OPEN, "up"),
        Waypoint(1.5, (0.20, 0.25, TOP + UP), FINGER_OPEN, "park"),
    ]
    # Right arm: wait, then lift the tube out of the rack by its upper body and lower it into the sleeve.
    tube_top_rack = TOP + RACK_FLOOR + TUBE_LEN
    g_tube = tube_top_rack - TUBE_GRIP + TIP
    sleeve_seat = RIM_Z - SL_BELOW + SL_FLOOR  # sleeve floor top once seated
    release_gap = 0.005
    g_tube_place = sleeve_seat + release_gap + TUBE_LEN - TUBE_GRIP + TIP + SAG_Z
    # Traced: the tube was still swinging about 5 mm from the carry when its body reached the sleeve's rim (2 mm of
    # clearance a side) and in one lot it caught there, so the arm pauses, then pauses again with the tip 10 mm
    # above the sleeve's top before the last slow descent.
    g_above = RIM_Z + COLLAR_T + 0.010 + TUBE_LEN - TUBE_GRIP + TIP + SAG_Z
    # Measured in a trace: the seated sleeve comes to rest 3.3 mm beyond and 3.8 mm left of the well's centre (it has
    # 4 mm play a side), and the carried tube's tip arrives 6 mm nearer the robot and 6 mm right of where it is
    # aimed. The demo aims the tube at the sleeve's bore with both corrections.
    # Nov 7 traces with the settle pauses: the tip then arrived 2 to 2.5 mm beyond and 1.2 mm left of the sleeve's axis
    # in both lots (2 mm of play in the pocket), so the aim moves 2 mm nearer and 1 mm right.
    rx, ry = WELL_X + 0.0061 - 0.0020, WELL_Y - SAG_Y["right"] + 0.0057 - 0.0010
    right = [
        Waypoint(11.0, (0.20, -0.25, TOP + UP), FINGER_OPEN, "wait for the sleeve"),
        Waypoint(1.5, (RACK_X, RACK_Y, TOP + UP_TUBE), FINGER_OPEN, "over the tube"),
        Waypoint(1.2, (RACK_X, RACK_Y, g_tube), FINGER_OPEN, "down to the tube"),
        Waypoint(0.8, (RACK_X, RACK_Y, g_tube), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (RACK_X, RACK_Y, TOP + UP_TUBE), FINGER_CLOSED, "lift out of the rack"),
        Waypoint(2.0, (rx, ry, TOP + UP_TUBE), FINGER_CLOSED, "over the well"),
        Waypoint(1.0, (rx, ry, TOP + UP_TUBE), FINGER_CLOSED, "let the swing settle"),
        Waypoint(1.5, (rx, ry, g_above), FINGER_CLOSED, "lower until the tip is just above the sleeve"),
        Waypoint(1.0, (rx, ry, g_above), FINGER_CLOSED, "settle again"),
        Waypoint(2.0, (rx, ry, g_tube_place), FINGER_CLOSED, "lower into the sleeve"),
        Waypoint(0.8, (rx, ry, g_tube_place), FINGER_OPEN, "let go"),
        Waypoint(1.2, (rx, ry, TOP + UP_TUBE), FINGER_OPEN, "up"),
    ]
    if not fit:
        left = [Waypoint(1.5, (0.20, 0.25, TOP + UP), FINGER_OPEN, "park")]
    return {"left": left, "right": right}
