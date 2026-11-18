"""On the station table, in front of the robot and to its right: a robot controller board (green) on an aluminium
carrier plate, with a black 2 x 20 socket header down its middle. Nearer the robot and further right, a blue add-on
board stands on a black antistatic foam holder, its two rows of gold pins hanging down into the slot between the
holder's two foam blocks. The add-on board has a silver aluminium heat sink at its centre. Each board has a small
white mark at one corner of its header.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

GREEN, BLUE, BLACK = (0.10, 0.42, 0.20), (0.12, 0.30, 0.75), (0.05, 0.05, 0.06)
SILVER, GOLD, WHITE, FOAM = (0.75, 0.76, 0.78), (0.85, 0.68, 0.25), (0.95, 0.95, 0.95), (0.16, 0.16, 0.17)

# --- the header: 2 x 20 contacts at 2.54 mm pitch in both directions -------------------------------------------------
PITCH = 0.00254
N_PER_ROW = 20
ROW_X = (-PITCH / 2, PITCH / 2)  # the two rows, across x; contacts run along y
PIN_Y = tuple((i - (N_PER_ROW - 1) / 2) * PITCH for i in range(N_PER_ROW))
PIN_W = 0.00064  # 0.64 mm square pins
PIN_L = 0.006  # pins stand 6.0 mm below the add-on board
HDR_W, HDR_L, HDR_H = 2 * PITCH, N_PER_ROW * PITCH, 0.0085  # socket header body 5.08 x 50.8 x 8.5 mm

# --- the controller (fixed): carrier plate, PCB, header ---------------------------------------------------------------
CX, CY = 0.320, -0.100  # header centre, world
CARRIER_T, PCB_T = 0.006, 0.0016
PCB_TOP = TOP + CARRIER_T + PCB_T
HT = PCB_TOP + HDR_H  # the header's top face (the socket entries): 16.1 mm above the table
ENTRY = PIN_L  # the socket entry face is a slider that the pin tips stand on; it travels 6 mm down as the pins go in
GAP_BELOW = 0.0005  # the header body below the entry layer stops 0.5 mm short of the seated pin tips

# The press fit. The entry layer rides a straight-down slider on a stiff spring (K). An on_step rule (below) holds
# the spring's rest point where the layer is, so a resting or lightly pushed board stays put; only when the board's
# pins stand over their sockets and the push exceeds INSERT_N does the rest point follow the layer down (contact
# friction: it stays wherever it was pushed). Over the plastic between sockets the layer never yields.
K = 40000.0  # N/m
INSERT_N = 24.0  # insertion force for all 40 contacts (0.6 N each)
YIELD = INSERT_N / K  # 0.6 mm
BEND_N = 12.0  # pressing pins that stand on the plastic beside their sockets this hard bends them
BOTTOM = ENTRY - YIELD  # once the rest point is within YIELD of full depth, it goes to full depth (pins home)

ENTRY_LAYER = parts.Slider(
    "entry", axis=(0.0, 0.0, -1.0), travel=ENTRY, mass=0.05, spring=K, damping=60.0, max_effort=400.0,
    shapes=[parts.box((HDR_W, HDR_L, ENTRY), at=(0.0, 0.0, HT - TOP - ENTRY / 2), color=BLACK)],
)
_body_h = HDR_H - ENTRY - GAP_BELOW
CONTROLLER = parts.Part(
    "controller", pos=(CX, CY, TOP),
    shapes=[
        parts.box((0.070, 0.100, CARRIER_T), at=(0.0, 0.0, CARRIER_T / 2), color=SILVER),
        parts.box((0.056, 0.085, PCB_T), at=(0.0, 0.0, CARRIER_T + PCB_T / 2), color=GREEN),
        parts.box((HDR_W, HDR_L, _body_h), at=(0.0, 0.0, PCB_TOP - TOP + _body_h / 2), color=BLACK),
        # pin-1 mark: a white square on the PCB beside the header's +y end, on the +x side
        parts.box((0.008, 0.008, 0.0002), at=(0.009, HDR_L / 2 - 0.004, CARRIER_T + PCB_T + 0.0001), color=WHITE),
    ],
    joints=[ENTRY_LAYER],
)

# --- the add-on board (loose). Frame origin: the PCB's underside, over the centre of the pin field ---------------------
BOARD_W, BOARD_L, BOARD_T = 0.056, 0.065, 0.0016
SINK_W, SINK_H = 0.025, 0.012  # aluminium heat sink, 25 x 25 x 12 mm, glued at the centre
BOARD_MASS = 0.035
BOARD_SHAPES = [
    parts.box((BOARD_W, BOARD_L, BOARD_T), at=(0.0, 0.0, BOARD_T / 2), color=BLUE),
    parts.box((SINK_W, SINK_W, SINK_H), at=(0.0, 0.0, BOARD_T + SINK_H / 2), color=SILVER),
    *[parts.box((PIN_W, HDR_L, PIN_L), at=(x, 0.0, -PIN_L / 2), color=GOLD) for x in ROW_X],
    # pin-1 mark, matching the controller's: on the board's top, +x side, +y end
    parts.box((0.008, 0.008, 0.0002), at=(0.020, BOARD_L / 2 - 0.006, BOARD_T + 0.0001), color=WHITE),
]

# --- the foam holder (fixed): two foam blocks with a 10 mm slot between them for the pins ----------------------------
FX, FY = 0.220, -0.215  # holder centre, world
FOAM_H, SLOT = 0.020, 0.010
FOAM = parts.Part(
    "foam_holder", pos=(FX, FY, TOP),
    shapes=[parts.box((0.020, 0.060, FOAM_H), at=(s * (SLOT / 2 + 0.010), 0.0, FOAM_H / 2), color=FOAM)
            for s in (-1, 1)],
)
BOARD = parts.Part("addon_board", pos=(FX, FY, TOP + FOAM_H), shapes=BOARD_SHAPES, mass=BOARD_MASS)

OBJECTS = [CONTROLLER, FOAM, BOARD]

# --- geometry of the fit --------------------------------------------------------------------------------------------
CAPTURE = 0.0007  # a pin tip enters its socket's lead-in when within 0.7 mm of the socket centre, across and along
TILT_MAX = 3.4  # degrees: past this, 0.64 mm pins bind in their 1.0 mm socket holes and bend when pushed further
SEATED_TOL = 0.0003  # seated: the board's underside at most 0.3 mm above the header's top face
STARTED = 0.0020  # started: pins at least 2 mm into the correct sockets
ON_ENTRY = 0.0005  # the pin tips stand on the entry face when they are within 0.5 mm of it
CLEAR = 0.005  # let go: no part of either arm within 5 mm of the add-on board
STILL_STEPS = 20  # 1 s at 20 Hz
STILL_POS, STILL_DEG = 0.0002, 0.2  # still: within 0.2 mm and 0.2 degrees of the pose 1 control step earlier


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def fit(board) -> dict:
    """Where the add-on board's pin field is relative to the header, in the header's frame (x across the rows, y
    along them): the offset of its centre (m), its turn about the vertical (deg, 0 = pin 1 over pin 1), its tilt
    (deg from level) and the height of its underside above the header's top face (m)."""
    r = _rot(board.quat_xyzw)
    p = np.asarray(board.pos, float)
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    return {"dx": float(p[0] - CX), "dy": float(p[1] - CY), "yaw": yaw, "tilt": tilt, "h": float(p[2] - HT)}


def _lattice(d: float) -> tuple[int, float]:
    """Nearest whole number of pitches and what is left over (m)."""
    n = int(round(d / PITCH))
    return n, d - n * PITCH


def over_sockets(f) -> tuple[bool, tuple[int, int]]:
    """Do all the pin tips that are over the header stand within CAPTURE of a socket centre? Returns that and the
    offset in whole pitches (across, along). A board turned 180 degrees also lines up (the field is symmetric)."""
    yaw = (f["yaw"] + 90.0) % 180.0 - 90.0  # 0 and 180 degrees both put pins over sockets
    nx, rx = _lattice(f["dx"])
    ny, ry = _lattice(f["dy"])
    swing = (HDR_L / 2) * abs(math.sin(math.radians(yaw)))  # the end pins move this far across when turned
    along = (HDR_W / 2) * abs(math.sin(math.radians(yaw)))
    ok = abs(rx) + swing <= CAPTURE + EPS and abs(ry) + along <= CAPTURE + EPS
    return ok, (nx, ny)


def correct(f) -> bool:
    """Every pin over its own socket: no offset by a row or a pin, and pin 1 over pin 1 (not turned round)."""
    ok, n = over_sockets(f)
    return bool(ok and n == (0, 0) and abs(f["yaw"]) <= 90.0)


# --- the whole-trial record and the press-fit state. Kept in the parts module so near_misses.py (which imports this
# file separately) shares them. -----------------------------------------------------------------------------------
_grip = parts.__dict__.setdefault("_hdr_task_grip", {"clear": None})
_rec = parts.__dict__.setdefault("_hdr_task_record", {})


def _clear_record():
    _rec.clear()
    _rec.update(bent=False, set=0.0, prev=None, still=0, still_pose=None)


_clear_record()


def _bends(f, q: float, rest: float) -> bool:
    """Would this state bend pins? (1) Pins in their sockets (pushed 0.5 mm or more in) with the board tilted past
    TILT_MAX or moved off its sockets. (2) Pins standing on the plastic beside their sockets, pushed with BEND_N or
    more."""
    on, _ = over_sockets(f)
    inserted = q >= 0.0005 - EPS and f["h"] <= PIN_L - q + ON_ENTRY + EPS
    if inserted and (f["tilt"] > TILT_MAX + EPS or not on):
        return True
    standing = f["h"] <= PIN_L - q + ON_ENTRY + EPS and abs(f["dx"]) < HDR_W / 2 + PIN_W and abs(f["dy"]) < HDR_L
    return bool(standing and not on and K * (q - rest) >= BEND_N - EPS)


def _update(objects) -> None:
    """Once per control step (from score()): the bend record and the stillness count. Cleared on a scene reset: the
    add-on board back within 0.5 mm of its start, at rest, after a jump of over 20 mm in one control step."""
    b = objects["addon_board"]
    pos = np.asarray(b.pos, float)
    prev = _rec["prev"]
    if (prev is not None and np.linalg.norm(pos - np.asarray(BOARD.pos)) < 0.0005 and b.resting()
            and np.linalg.norm(pos - prev) > 0.020):
        _clear_record()
    _rec["prev"] = pos
    f = fit(b)
    q = float(objects["controller"].joints["entry"])
    if "offline_rest" in _grip:  # near_misses.py: no simulator, so it states the spring's rest point
        _rec["set"] = _grip["offline_rest"](objects)
    if _bends(f, q, _rec["set"]):
        _rec["bent"] = True
    now = (pos, _rot(b.quat_xyzw))
    was = _rec["still_pose"]
    same = (was is not None and np.linalg.norm(now[0] - was[0]) <= STILL_POS + EPS
            and math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(now[1].T @ was[1]) - 1) / 2)))) <= STILL_DEG)
    _rec["still"] = _rec["still"] + 1 if same else 0
    _rec["still_pose"] = now


def _clear(objects):
    """The arms' measured clearance (None until measured: no credit). near_misses.py stands in for it offline."""
    if "offline_clear" in _grip:
        _grip["clear"] = _grip["offline_clear"](objects)
    return _grip["clear"]


def seated(objects) -> bool:
    f = fit(objects["addon_board"])
    q = float(objects["controller"].joints["entry"])
    return bool(correct(f) and f["h"] <= SEATED_TOL + EPS and q >= ENTRY - SEATED_TOL - EPS
                and f["tilt"] <= TILT_MAX + EPS)


def started(objects) -> bool:
    f = fit(objects["addon_board"])
    q = float(objects["controller"].joints["entry"])
    return bool(correct(f) and q >= STARTED - EPS and f["h"] <= PIN_L - STARTED + ON_ENTRY + EPS
                and f["tilt"] <= TILT_MAX + EPS)


def _value(objects) -> float:
    if _rec["bent"]:
        return 0.0
    if _grip["clear"] is not True or _rec["still"] < STILL_STEPS:
        return 0.0
    if seated(objects):
        return 1.0
    if started(objects):
        return 0.5
    return 0.0


def success(objects):
    """The add-on board fully seated with every pin in its own socket, no pin bent, let go and still for 1 s."""
    _clear(objects)
    return bool(_value(objects) == 1.0)


def distance(objects):
    f = fit(objects["addon_board"])
    return float(math.hypot(f["dx"], f["dy"]) + max(f["h"], 0.0))


def score(objects):
    _clear(objects)
    _update(objects)
    return _value(objects)


# --- simulator callbacks: the press fit and the let-go measurement ---------------------------------------------------
@parts.on_step
def _press_fit(sim) -> None:
    st = sim.states()
    q = float(st["controller"].joints["entry"])
    f = fit(st["addon_board"])
    on, _ = over_sockets(f)
    standing = f["h"] <= PIN_L - q + ON_ENTRY + EPS
    lifted = f["h"] > PIN_L - q + 0.001  # pins withdrawn: the entry face springs back to the top
    rest = _rec["set"]
    if lifted:
        rest = 0.0
    elif standing and on and f["tilt"] <= TILT_MAX + EPS and K * (q - rest) >= INSERT_N - EPS:
        rest = min(q - YIELD, ENTRY)
        if rest >= BOTTOM - EPS:
            rest = ENTRY + 0.0005  # home: held against the full-depth stop
    _rec["set"] = rest
    sim.set_target("controller", "entry", rest)


FACE_STEP = 0.003  # arm hull faces are sampled about every 3 mm
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def _face_points(vertices, planes, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull about `step` apart, corners included, so a flat face lying on the board
    is caught even when all of its corners are far away."""
    pts = [np.asarray(vertices, float)]
    _, first = np.unique(np.round(planes / np.array([1e-4, 1e-4, 1e-4, 1e-5])).astype(np.int64), axis=0,
                         return_index=True)
    for n, d in zip(planes[np.sort(first), :3], planes[np.sort(first), 3]):
        on = vertices[np.abs(vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for a, b in zip(ring, np.roll(ring, -1, axis=0)):
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c)) / step)))
            for i in range(m + 1):
                for j in range(m + 1 - i):
                    pts.append((c + (a - c) * (i / m) + (b - c) * (j / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 3)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([np.asarray(vertices, float), pts[np.sort(keep)]])


_LINK_POINTS: dict = {}


def _link_points(name) -> list:
    if name not in _LINK_POINTS:
        from yam_arena import analysis

        out = []
        for h in analysis.link_hulls().get(name, []):
            pts = _face_points(h.vertices, h.planes)
            c = pts.mean(axis=0)
            out.append((c, float(np.linalg.norm(pts - c, axis=1).max()), pts))
        _LINK_POINTS[name] = out
    return _LINK_POINTS[name]


def board_boxes(board) -> list:
    return [(c, r, s) for _, c, r, s in BOARD.world_boxes({"base": (np.asarray(board.pos), _rot(board.quat_xyzw))})]


def _arms_clear(points, boxes) -> bool:
    """True only if every given arm hull point is more than CLEAR from every box."""
    from yam_arena import analysis

    if not len(points):
        return True
    pts = np.concatenate(points)
    for c, r, s in boxes:
        reach = float(np.linalg.norm(s)) / 2 + CLEAR + 1e-3
        near = pts[np.einsum("ij,ij->i", pts - c, pts - c) <= reach * reach]
        if len(near) and float(analysis.box_signed_distance(near, c, r, s).min()) <= CLEAR:
            return False
    return True


@parts.on_step
def _measure_grip(sim) -> None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = board_boxes(sim.states()["addon_board"])
    lo = np.min([c - np.linalg.norm(sz) / 2 for c, _, sz in boxes], axis=0) - CLEAR
    hi = np.max([c + np.linalg.norm(sz) / 2 for c, _, sz in boxes], axis=0) + CLEAR
    points, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ARM_LINKS:
                rot, pos = _rot(poses[i, 3:]), poses[i, :3] - origin
                hulls = _link_points(n)
                found += 1 if hulls else 0
                for c, rad, p in hulls:
                    cw = rot @ c + pos
                    if np.all(cw + rad >= lo) and np.all(cw - rad <= hi):
                        points.append(p @ rot.T + pos)
    # fail closed: unless every listed link of both arms was found, the state stays unknown (None, no credit)
    _grip["clear"] = _arms_clear(points, boxes) if found == 2 * len(ARM_LINKS) else None


@parts.on_reset
def _reset(sim) -> None:
    _grip["clear"] = None
    _clear_record()
    sim.set_target("controller", "entry", 0.0)


# --- demo: open loop from the published start layout ------------------------------------------------------------------
TIP = 0.013  # pad tips below the TCP
GRIP_ABOVE = BOARD_T + 0.0015 + TIP  # TCP above the board's underside: the pad tips 1.5 mm above the PCB's top
SQUEEZE = -0.0065  # pads commanded 13 mm apart on the 25 mm heat sink: 6 mm past each face
OPEN = -0.020  # pads 40 mm apart
TRAVEL_Z = TOP + 0.13
HOVER = 0.004  # pin tips 4 mm above the socket entries before going down
PRESS = 0.060  # commanded this far past the seated position: the soft arm then pushes about 30 N
EASE = 0.003  # after the press, eased back to 3 mm past seated before letting go
# Measured carrying error of the loaded right arm over the header (TCP target minus where the pin field ended up).
CARRY_FIX = (0.0, 0.0)
# Cheat and near-miss variants, for traces only (DEMO_VARIANT=...): the same plan with the board carried off its
# sockets before the press, or let go above the header.
VARIANTS = {"half_pitch_press": (PITCH / 2, 0.0), "one_row_over": (PITCH, 0.0), "one_pin_along": (0.0, PITCH)}


def demo():
    """The right arm grips the heat sink, lifts the add-on board off its foam, carries it over the header with the
    white marks on the same side, lowers its pins onto the socket entries, presses it home, eases off, lets go and
    moves away. The left arm stays parked."""
    import os

    variant = os.environ.get("DEMO_VARIANT", "")
    off = VARIANTS.get(variant, (0.0, 0.0))
    gx, gy, gz = FX, FY, TOP + FOAM_H + GRIP_ABOVE
    hx, hy = CX + CARRY_FIX[0] + off[0], CY + CARRY_FIX[1] + off[1]
    over = HT + PIN_L + HOVER + GRIP_ABOVE
    touch = HT + PIN_L + GRIP_ABOVE
    home = HT + GRIP_ABOVE
    w = [
        Waypoint(1.5, (gx, gy, TRAVEL_Z), OPEN, "over the add-on board", 0.0),
        Waypoint(1.5, (gx, gy, gz), OPEN, "down round the heat sink", 0.0),
        Waypoint(0.8, (gx, gy, gz), SQUEEZE, "grip", 0.0),
        Waypoint(1.0, (gx, gy, gz + 0.040), SQUEEZE, "lift the pins out of the foam", 0.0),
        Waypoint(2.0, (hx, hy, over + 0.030), SQUEEZE, "over the header", 0.0),
        Waypoint(1.0, (hx, hy, over), SQUEEZE, "pins just above the sockets", 0.0),
        Waypoint(1.0, (hx, hy, over), SQUEEZE, "steady", 0.0),
        Waypoint(1.0, (hx, hy, touch), SQUEEZE, "pin tips onto the socket entries", 0.0),
        Waypoint(3.0, (hx, hy, home - PRESS), SQUEEZE, "press home", 0.0),
        Waypoint(1.0, (hx, hy, home - PRESS), SQUEEZE, "hold", 0.0),
        Waypoint(1.5, (hx, hy, home - EASE), SQUEEZE, "ease off", 0.0),
        Waypoint(0.8, (hx, hy, home - EASE), OPEN, "let go", 0.0),
        Waypoint(1.2, (hx, hy, TRAVEL_Z), OPEN, "up", 0.0),
        Waypoint(1.5, (0.20, -0.30, TRAVEL_Z), OPEN, "away", 0.0),
        Waypoint(3.0, (0.20, -0.30, TRAVEL_Z), OPEN, "wait clear", 0.0),
    ]
    if variant == "drop":  # let go with the pins 4 mm above the sockets, then leave
        w = w[:7] + [Waypoint(0.8, (hx, hy, over), OPEN, "let go above the header", 0.0)] + w[12:]
    return {"right": w, "left": []}


DEMO_TOUCHES = ("controller",)  # the press is commanded past the seat; the pads stop on the board, not the header
