"""On the station table, which stands for the top of a vaccine fridge: a flat max/min thermometer unit in front of
the left arm, a log board to the robot's right, and a small white vial carton at the far left, which plays no part.

The thermometer unit is a dark grey slab. Along its far half runs a scale of five colour bands, left to right: blue,
green, yellow, orange, red. Beside the scale, on a white track, a thin grey index marks the present temperature, in
the green band; a black marker on the same track, to the right of the index, stands where the temperature reached
since the last reset. A round white RESET button sits on the unit's near edge. Pressing it brings the marker back to
the index.

The log board is a light grey slab with the same five bands, left to right, and a white blank zone at their left
end. A black pointer tab stands on the board's near edge in the blank zone and slides left and right along it.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots: its reset hook runs before the lots settle the scene)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
DARK = (0.22, 0.23, 0.25)
LIGHT = (0.70, 0.71, 0.73)
WHITE = (0.95, 0.95, 0.95)
GREY = (0.45, 0.46, 0.48)
BLACK = (0.04, 0.04, 0.04)
BANDS = (("blue", (0.10, 0.30, 0.90)), ("green", (0.10, 0.65, 0.20)), ("yellow", (0.95, 0.85, 0.10)),
         ("orange", (0.95, 0.50, 0.05)), ("red", (0.85, 0.08, 0.08)))
BAND = 0.028  # each band 28 mm along y
EPS = 1e-6

# -- the thermometer unit (fixed) ------------------------------------------------------------------------------------
# Part frame: origin on the table under the slab's centre. The scale runs along y, blue at +y (the robot's left).
THERMO_XY = (0.32, 0.10)
SLAB = (0.120, 0.190, 0.020)  # x, y, z
SCALE_X, TRACK_X, BUTTON_X = 0.030, 0.000, -0.040  # centre lines (part frame x)
SCALE_TOP = 0.070  # +y edge of the blue band; the red band ends 140 mm further, at -0.070
MARK = (0.018, 0.018, 0.010)  # the marker, x y z
INDEX_Y = 0.0395  # index centre; 5 mm wide
MARKER_Y0 = 0.028  # the marker's centre when reset: the middle of the green band, against the index
MARKER_TRAVEL = 0.100
MARKER_SPRING = 50.0  # N/m: holds the marker where it stands (a slider moved only by the reset)
BUTTON_R, BUTTON_H, BUTTON_TRAVEL = 0.012, 0.012, 0.008
BUTTON_SPRING = 120.0  # N/m
PRESSED = 0.005  # the reset acts once the cap is 5 mm down


def _band_y(k: int, top: float) -> float:
    return top - BAND * (k + 0.5)


THERMO = parts.Part(
    "thermometer", pos=(*THERMO_XY, TOP),
    shapes=[
        parts.box(SLAB, at=(0.0, 0.0, SLAB[2] / 2), color=DARK),
        *(parts.box((0.028, BAND, 0.002), at=(SCALE_X, _band_y(k, SCALE_TOP), SLAB[2] + 0.001), color=c)
          for k, (_, c) in enumerate(BANDS)),
        parts.box((0.024, 0.135, 0.002), at=(TRACK_X, -0.0075, SLAB[2] + 0.001), color=WHITE),
        parts.box((0.020, 0.005, 0.006), at=(TRACK_X, INDEX_Y, SLAB[2] + 0.002 + 0.003), color=GREY),
    ],
    joints=[
        parts.Slider("marker", shapes=[parts.box(MARK, at=(TRACK_X, MARKER_Y0, SLAB[2] + 0.002 + MARK[2] / 2),
                                                 color=BLACK)],
                     axis=(0.0, -1.0, 0.0), travel=MARKER_TRAVEL, mass=0.02, spring=MARKER_SPRING, damping=1.0),
        parts.Slider("reset", shapes=[parts.cylinder(BUTTON_R, BUTTON_H, at=(BUTTON_X, 0.0, SLAB[2] + BUTTON_H / 2),
                                                     color=WHITE, axis="z")],
                     axis=(0.0, 0.0, -1.0), travel=BUTTON_TRAVEL, mass=0.01, spring=BUTTON_SPRING, damping=1.0),
    ],
)

# -- the log board (fixed) -------------------------------------------------------------------------------------------
LOG_XY = (0.37, -0.14)
BOARD = (0.070, 0.200, 0.015)
LOG_BANDS_X, POINTER_X = 0.012, -0.020
BLANK = 0.034  # the blank zone, left of the bands
LOG_TOP = 0.090  # +y edge of the blank zone
LOG_SCALE_TOP = LOG_TOP - BLANK  # +y edge of the blue band
TAB = (0.018, 0.014, 0.025)  # the pointer tab, x y z
POINTER_Y0 = LOG_TOP - BLANK / 2  # the tab's centre at the start: the middle of the blank zone
POINTER_TRAVEL = 0.150

LOG = parts.Part(
    "log_board", pos=(*LOG_XY, TOP),
    shapes=[
        parts.box(BOARD, at=(0.0, 0.0, BOARD[2] / 2), color=LIGHT),
        parts.box((0.034, BLANK, 0.002), at=(LOG_BANDS_X, LOG_TOP - BLANK / 2, BOARD[2] + 0.001), color=WHITE),
        *(parts.box((0.034, BAND, 0.002), at=(LOG_BANDS_X, _band_y(k, LOG_SCALE_TOP), BOARD[2] + 0.001), color=c)
          for k, (_, c) in enumerate(BANDS)),
        parts.box((0.006, 0.176, 0.002), at=(POINTER_X, -0.002, BOARD[2] + 0.001), color=GREY),  # the pointer's rail
    ],
    joints=[
        parts.Slider("pointer", shapes=[parts.box(TAB, at=(POINTER_X, POINTER_Y0, BOARD[2] + 0.002 + TAB[2] / 2),
                                                  color=BLACK)],
                     axis=(0.0, -1.0, 0.0), travel=POINTER_TRAVEL, mass=0.03, spring=0.0, damping=1.0),
    ],
)

CARTON_XY = (0.45, 0.27)
CARTON = Box("vial_carton", pos=(*CARTON_XY, TOP + 0.015), size=(0.060, 0.040, 0.030), color=WHITE, mass=0.1)

OBJECTS = [THERMO, LOG, CARTON]
DEMO_TOUCHES = ("thermometer",)  # the left fingertips press the RESET cap down to the slab

BAND_NAMES = [n for n, _ in BANDS]


def _marker_q(band: str) -> float:
    """The marker's travel that puts its centre at the middle of ``band``."""
    return float(MARKER_Y0 - _band_y(BAND_NAMES.index(band), SCALE_TOP)) if band != "green" else 0.0


# Lot A (the screenshot): the stored maximum in the yellow band. B: red. C: orange. In every lot the maximum lies
# above the present reading (green), so logging the present reading is always wrong. Every lot moves the carton by
# nothing, so the scene settles and the first frame is fresh.
_STAY = {"vial_carton": CARTON.pos}
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", band="yellow", place=_STAY),
    lots.Lot("B", band="red", place=_STAY),
    lots.Lot("C", band="orange", place=_STAY),
)

RESET_TOL = 0.003  # reset: the marker within 3 mm of its place beside the index
CLEAR = 0.005  # released: no gripper or fingertip within 5 mm of the pointer, marker or button
STILL = 0.0003  # m per control step (6 mm/s): pointer, marker and button all still

# -- whole-trial record ----------------------------------------------------------------------------------------------
# "fault": RESET was pressed 5 mm or more while the pointer was not in the band the marker started in. Cleared only by
# a scene reset (the on_reset hook in the simulator; offline, the pointer back within 0.5 mm of its start, the button
# up, after a jump of over 20 mm in one control step, which a robot cannot do).
_record = {"fault": False, "prev": None}
# Measured every step in the simulator by the on_step hook. Offline (near_misses.py) there is no robot: clear and still.
_grip = {"clear": True, "still": True, "prev": None}
_latch = {"reset": False}


def _pointer_band(q: float) -> str | None:
    """The band the pointer tab's centre is in (each band's full 28 mm, edges included), or None."""
    y = POINTER_Y0 - q
    for k, name in enumerate(BAND_NAMES):
        c = _band_y(k, LOG_SCALE_TOP)
        if abs(y - c) <= BAND / 2 + EPS:
            return name
    return None


def _update(objects) -> None:
    ptr = objects["log_board"].joints["pointer"]
    btn = objects["thermometer"].joints["reset"]
    prev = _record["prev"]
    if prev is not None and abs(ptr) < 0.0005 and btn < 0.002 and abs(ptr - prev) > 0.020:
        _record["fault"] = False
    _record["prev"] = ptr
    if btn >= PRESSED - EPS and _pointer_band(ptr) != lots.current().band:
        _record["fault"] = True


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _scored_boxes(st):
    """World boxes of the pointer tab, the marker and the button cap."""
    out = []
    for part, joint in ((LOG, "pointer"), (THERMO, "marker"), (THERMO, "reset")):
        p, q = st[part.name].links[joint]
        out += [(c, r, s) for _, c, r, s in part.world_boxes({joint: (np.asarray(p), _rot(q))})]
    return out


@parts.on_reset
def _new_trial(sim) -> None:
    _record["fault"], _record["prev"] = False, None
    _grip["prev"] = None
    _latch["reset"] = False
    sim.set_target("thermometer", "marker", _marker_q(lots.current().band))


@parts.on_step
def _step(sim) -> None:
    from yam_arena import analysis

    st = sim.states()
    if st["thermometer"].joints["reset"] >= PRESSED - EPS and not _latch["reset"]:
        _latch["reset"] = True
        sim.set_target("thermometer", "marker", 0.0)  # the marker goes back to the index and stays there
    base = sim.env.unwrapped
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
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in _scored_boxes(st))
    q = np.array([st["log_board"].joints["pointer"], st["thermometer"].joints["marker"],
                  st["thermometer"].joints["reset"]])
    prev = _grip["prev"]
    _grip["still"] = prev is not None and bool(np.all(np.abs(q - prev) < STILL))
    _grip["prev"] = q


def _logged(objects) -> bool:
    return _pointer_band(objects["log_board"].joints["pointer"]) == lots.current().band


def _reset(objects) -> bool:
    return objects["thermometer"].joints["marker"] <= RESET_TOL + EPS


def _settled() -> bool:
    return _grip["clear"] and _grip["still"]


def success(objects):
    """The pointer in the band the marker stood in at the start, the marker back at the index, let go and still; the
    reset was never pressed before the log was right."""
    _update(objects)
    return bool(not _record["fault"] and _settled() and _logged(objects) and _reset(objects))


def distance(objects):
    target = _band_y(BAND_NAMES.index(lots.current().band), LOG_SCALE_TOP)
    return float(abs(POINTER_Y0 - objects["log_board"].joints["pointer"] - target))


def score(objects):
    """1.0 logged and reset; 0.5 logged, not reset; 0 otherwise, or if the reset came before the log."""
    _update(objects)
    if _record["fault"] or not _settled() or not _logged(objects):
        return 0.0
    return 1.0 if _reset(objects) else 0.5


# -- scripted demo ---------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
TRAVEL_Z = TOP + 0.14
LEAD = 0.015  # the closed fingers' right face is this far right (-y) of the TCP, plus the tab's half-width


def demo():
    """Right arm: closed fingers come down just left of the pointer tab, push it right to the middle of the band the
    cameras show the marker in, and lift away. Then the left arm presses RESET with closed fingers and lifts away."""
    band = lots.current().band
    lx, ly = LOG_XY
    px = lx + POINTER_X
    y0 = ly + POINTER_Y0 + LEAD + 0.012  # 12 mm clear of the tab
    y1 = ly + _band_y(BAND_NAMES.index(band), LOG_SCALE_TOP) + LEAD
    tz = TOP + BOARD[2] + 0.010 + TIP  # tips 10 mm above the board, 15 mm below the tab's top
    right = [
        Waypoint(1.5, (px, y0, TRAVEL_Z), FINGER_CLOSED, "over the board, left of the pointer tab"),
        Waypoint(1.2, (px, y0, tz), FINGER_CLOSED, "down beside the tab"),
        Waypoint(3.5, (px, y1, tz), FINGER_CLOSED, "push the pointer to the band"),
        Waypoint(0.8, (px, y1, tz), FINGER_CLOSED, "settle"),
        Waypoint(1.0, (px, y1, TRAVEL_Z), FINGER_CLOSED, "up"),
        Waypoint(1.0, (px - 0.10, y1 - 0.05, TRAVEL_Z), FINGER_CLOSED, "away"),
    ]
    bx, by = THERMO_XY[0] + BUTTON_X, THERMO_XY[1]
    touch = TOP + SLAB[2] + BUTTON_H + TIP  # TCP height at which the closed tips meet the cap
    left = [
        Waypoint(1.5, (bx - 0.06, by + 0.06, TRAVEL_Z), FINGER_CLOSED, "aside, waiting"),
        Waypoint(8.0, (bx - 0.06, by + 0.06, TRAVEL_Z), FINGER_CLOSED, "wait for the log"),
        Waypoint(1.5, (bx, by, touch + 0.015), FINGER_CLOSED, "over RESET"),
        Waypoint(1.2, (bx, by, touch - BUTTON_H - 0.004), FINGER_CLOSED, "press RESET"),
        Waypoint(0.6, (bx, by, touch - BUTTON_H - 0.004), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (bx, by, TRAVEL_Z), FINGER_CLOSED, "up"),
        Waypoint(1.0, (bx - 0.06, by + 0.06, TRAVEL_Z), FINGER_CLOSED, "away"),
    ]
    return {"left": left, "right": right}
