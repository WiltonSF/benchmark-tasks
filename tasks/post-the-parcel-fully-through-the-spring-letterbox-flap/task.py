"""On the station table, in front of the left arm: a section of a wooden garden gate, three cedar boards 19 mm thick
standing across the table, with a brass magazine-size mail slot through it. The slot's brass face plate (330 x 89 mm)
has a 286 x 62 mm opening whose floor is 140 mm above the table; a brass flap hangs in the opening on two pins along
its top edge, swings inward when something pushes it and is pulled shut by its own weight and a light spring. Behind
the gate a closed, grey sheet-steel catch box (rear door with a cam lock) collects whatever comes through, down a
steel chute sloping 35 degrees from the slot floor's inner edge; the only way in is the slot. A white corrugated mailer box, 162 x 76 x 32 mm, lies flat on the table in front of the gate,
its long side pointing at the slot, with a shipping label on top and tape across its tuck end.
"""

from __future__ import annotations

import math
import pathlib
import sys
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

# World frame: x away from the arm bases, y to the robot's left, z up (m). Heights below are above the table top.
# Part frame of the gate: x from the slot plate's outer face, y from the slot's centre line, z above the table.
YC = 0.12  # the slot's centre line
X_OUT = 0.360  # the slot plate's outer face (toward the robot)
PLATE_T = 0.003  # face plate thickness
BOARD_T = 0.019  # 3/4 in cedar boards
X_IN = X_OUT + PLATE_T + BOARD_T  # the gate's inner face: the end of the slot's throat
PLATE_W, PLATE_H = 0.330, 0.089  # Salsbury 4045 unit: 13 x 3-1/2 in
AP_W, AP_Z0, AP_Z1 = 0.286, 0.140, 0.202  # plate opening (width; floor and ceiling heights)
HOLE_W, HOLE_Z0, HOLE_Z1 = 0.286, 0.139, 0.203  # rough opening in the boards: 11-1/4 x 2-1/2 in
PLATE_ZC = (AP_Z0 + AP_Z1) / 2
GATE_W, GATE_H, CAP_T = 0.420, 0.264, 0.019  # three 140 mm boards, 264 mm tall, with a cap rail
SHEET = 0.002  # catch box sheet steel
BOX_D = 0.220  # catch box depth inside, behind the gate
BOX_W = 0.364  # catch box width inside
BOX_H = 0.262  # catch box roof (underside) above the table

CEDAR, SEAM = (0.66, 0.45, 0.30), (0.33, 0.22, 0.14)
BRASS, SCREW = (0.80, 0.64, 0.28), (0.55, 0.43, 0.18)
GREY, DARK, BLACK = (0.50, 0.52, 0.54), (0.30, 0.31, 0.33), (0.08, 0.08, 0.08)

# Flap: a 0.8 mm brass leaf with a rolled lower edge, on two pins at its top corners (11-3/8 in across the pins).
FLAP_LEAF_W, FLAP_T = 0.283, 0.0008
PIVOT_Z = AP_Z1 - 0.0015
FLAP_LOW = AP_Z0 + 0.001  # the leaf's lower edge when shut
FLAP_H = PIVOT_Z - FLAP_LOW  # 0.0605: hinge line to lower edge
BEAD_R = 0.0015
FLAP_X = PLATE_T / 2  # the leaf hangs in the plate's opening, flush with its middle
BRASS_DENSITY = 8500.0  # kg/m3
FLAP_SHUT_GAP = 0.005  # m: the flap's lower edge at most this far inside its shut position counts as shut
FLAP_SHUT = math.asin(FLAP_SHUT_GAP / FLAP_H)  # rad, about 0.083


def _b(size, centre, color):
    """A box in the gate's frame: x from the plate's outer face, y from the slot centre, z above the table."""
    return parts.box(size, at=centre, color=color)


def _frame(x0, t, w, z0, z1, hole_w, hz0, hz1, color, total_z0=None, total_z1=None):
    """A slab x0..x0+t, width w, from total_z0 to total_z1, with a hole hole_w wide from hz0 to hz1."""
    lo = z0 if total_z0 is None else total_z0
    hi = z1 if total_z1 is None else total_z1
    side = (w - hole_w) / 2
    xc = x0 + t / 2
    return [
        _b((t, w, hz0 - lo), (xc, 0.0, (lo + hz0) / 2), color),
        _b((t, w, hi - hz1), (xc, 0.0, (hz1 + hi) / 2), color),
        *(_b((t, side, hz1 - hz0), (xc, s * (hole_w + side) / 2, (hz0 + hz1) / 2), color) for s in (1, -1)),
    ]


_plate = _frame(0.0, PLATE_T, PLATE_W, 0, 0, AP_W, AP_Z0, AP_Z1, BRASS,
                total_z0=PLATE_ZC - PLATE_H / 2, total_z1=PLATE_ZC + PLATE_H / 2)
_screws = [parts.cylinder(0.004, 0.0015, at=(-0.00075, s * 0.154, PLATE_ZC + u * 0.034), color=SCREW, axis="x")
           for s in (1, -1) for u in (1, -1)]
_boards = _frame(PLATE_T, BOARD_T, GATE_W, 0, 0, HOLE_W, HOLE_Z0, HOLE_Z1, CEDAR, total_z0=0.0, total_z1=GATE_H)
# Board joints on the outer face (2 mm dark seams), above and below the plate only.
_seams = [_b((0.0005, 0.002, z1 - z0), (PLATE_T - 0.00025, s * 0.070, (z0 + z1) / 2), SEAM)
          for s in (1, -1) for z0, z1 in ((0.0, PLATE_ZC - PLATE_H / 2), (PLATE_ZC + PLATE_H / 2, GATE_H))]
_cap = [_b((BOARD_T, GATE_W, CAP_T), (PLATE_T + BOARD_T / 2, 0.0, GATE_H + CAP_T / 2), CEDAR)]
_plaque = [_b((0.002, 0.090, 0.050), (PLATE_T - 0.001, -0.150, 0.235), BLACK)]  # house-number plaque, to the right

_bx0, _bx1 = X_IN - X_OUT, X_IN - X_OUT + BOX_D  # catch box inside, front to back (gate frame)
_box = [
    *(_b((BOX_D, SHEET, BOX_H), ((_bx0 + _bx1) / 2, s * (BOX_W + SHEET) / 2, BOX_H / 2), GREY) for s in (1, -1)),
    _b((SHEET, BOX_W + 2 * SHEET, BOX_H), (_bx1 + SHEET / 2, 0.0, BOX_H / 2), GREY),  # back (rear door)
    _b((BOX_D + SHEET, BOX_W + 2 * SHEET, SHEET), ((_bx0 + _bx1 + SHEET) / 2, 0.0, BOX_H + SHEET / 2), GREY),  # roof
    _b((BOX_D, BOX_W, SHEET), ((_bx0 + _bx1) / 2, 0.0, SHEET / 2), GREY),  # floor
    # Rear door outline, its bottom hinge and a round cam lock (outside the back, facing away from the robot).
    _b((0.0005, BOX_W - 0.040, BOX_H - 0.050), (_bx1 + SHEET + 0.00025, 0.0, BOX_H / 2), DARK),
    parts.cylinder(0.004, BOX_W - 0.060, at=(_bx1 + SHEET + 0.004, 0.0, 0.020), color=DARK, axis="y"),
    parts.cylinder(0.0095, 0.006, at=(_bx1 + SHEET + 0.003, 0.0, BOX_H - 0.050), color=SCREW, axis="x"),
]

_leaf_zc = (PIVOT_Z + FLAP_LOW + BEAD_R) / 2
_flap_shapes = [
    _b((FLAP_T, FLAP_LEAF_W, PIVOT_Z - FLAP_LOW - BEAD_R), (FLAP_X, 0.0, _leaf_zc), BRASS),
    parts.cylinder(BEAD_R, FLAP_LEAF_W, at=(FLAP_X, 0.0, FLAP_LOW + BEAD_R), color=BRASS, axis="y"),  # rolled edge
    *(parts.cylinder(0.0025, 0.003, at=(FLAP_X, s * (FLAP_LEAF_W / 2 + 0.0015), PIVOT_Z), color=SCREW, axis="y")
      for s in (1, -1)),  # hinge pins into the frame
]
_flap_volume = (FLAP_T * FLAP_LEAF_W * (PIVOT_Z - FLAP_LOW - BEAD_R) + math.pi * BEAD_R ** 2 * FLAP_LEAF_W
                + 2 * math.pi * 0.0025 ** 2 * 0.003)
FLAP_MASS = round(BRASS_DENSITY * _flap_volume, 3)  # 0.133 kg: solid brass at the stated sizes

FLAP = parts.Hinge(
    "flap",
    shapes=_flap_shapes,
    pivot=(FLAP_X, 0.0, PIVOT_Z),
    axis=(0.0, -1.0, 0.0),  # positive swings the lower edge inward (+x)
    lower=0.0,  # shut, hanging in the opening
    upper=1.45,
    mass=FLAP_MASS,
    spring=0.005,  # N m/rad, plus the flap's weight
    damping=0.005,
)
# Chute: a 2 mm sheet-steel plate 120 mm long, fixed at 35 degrees below level from the lower edge of the slot's
# inner end, across the box's width. Posted items slide down it instead of tipping over the edge of the slot floor;
# a 32 mm parcel lying on it at 35 degrees clears the slot's ceiling, so it cannot wedge across the 22 mm throat.
# parts.py shapes cannot be tilted, so it is built as a link held hard against a fixed stop: a hinge at the slot
# floor's inner edge whose upper limit is the chute angle, driven past that limit by a stiff spring (set at every
# reset); it does not move under any load in this task.
CHUTE_ANGLE = math.radians(35.0)
CHUTE_L = 0.120
CHUTE = parts.Hinge(
    "chute",
    shapes=[_b((CHUTE_L, BOX_W - 0.002, SHEET), (_bx0 + CHUTE_L / 2, 0.0, HOLE_Z0 - SHEET / 2), GREY)],
    pivot=(_bx0, 0.0, HOLE_Z0),
    axis=(0.0, 1.0, 0.0),  # positive tips the far end down
    lower=0.0,
    upper=CHUTE_ANGLE,
    mass=0.68,  # 2 mm steel
    spring=50.0,
    damping=5.0,
)
CHUTE_HOLD = CHUTE_ANGLE + 0.4  # spring target beyond the stop: about 20 N m holding it on the stop
LETTERBOX = parts.Part(
    "letterbox", pos=(X_OUT, YC, TOP),
    shapes=[*_plate, *_screws, *_boards, *_seams, *_cap, *_plaque, *_box], joints=[FLAP, CHUTE],
    friction=(0.3, 0.25),
)

# Parcel: a 6 x 2-1/2 x 1 in (inside) crush-proof corrugated mailer, roll-end tuck-front, 200#/ECT-32-B white board.
# Outside 162 x 76 x 32 mm (double walls at the sides and the tuck end); 40 g empty, packed to 150 g.
PACKET_SIZE = (0.162, 0.076, 0.032)
L, W, H = PACKET_SIZE
OYSTER, LABEL, INK, TAPE, LINE = (0.90, 0.89, 0.85), (0.98, 0.98, 0.97), (0.10, 0.10, 0.10), (0.76, 0.62, 0.40), \
    (0.62, 0.61, 0.58)
_parcel = [
    parts.box(PACKET_SIZE, color=OYSTER),
    _b((0.095, 0.060, 0.0003), (-0.025, 0.0, H / 2 + 0.00015), LABEL),  # shipping label on the lid
    *(_b((0.0025, 0.028, 0.0002), (-0.060 + 0.006 * k, -0.012, H / 2 + 0.0004), INK) for k in range(6)),  # barcode
    _b((0.040, 0.030, 0.0002), (-0.010, 0.012, H / 2 + 0.0004), INK),  # address block
    _b((0.001, W, 0.0002), (L / 2 - 0.006, 0.0, H / 2 + 0.00015), LINE),  # lid fold over the tuck end
    _b((0.048, 0.048, 0.0002), (L / 2 - 0.024, 0.0, H / 2 + 0.0004), TAPE),  # tape over the tuck end...
    _b((0.0002, 0.048, H), (L / 2 + 0.0001, 0.0, 0.0), TAPE),  # ...and down its face
]
PACKET = parts.Part("packet", pos=(X_OUT - 0.060 - L / 2, YC, TOP + H / 2), shapes=_parcel,
                    mass=0.150, friction=(0.4, 0.3))

OBJECTS = [LETTERBOX, PACKET]


# -- release check: no gripper or finger surface within 5 mm of the parcel --------------------------------------------
CLEAR = 0.005
HAND_LINKS = ("gripper", "tip_left", "tip_right")
SAMPLE_STEP = 0.003  # m between surface samples on each collision-hull face


def sample_triangles(tris: np.ndarray, step: float = SAMPLE_STEP) -> np.ndarray:
    """Points on triangles (n, 3, 3), at most about ``step`` apart along each edge and across each face (corners
    included), so a broad face lying on the parcel is measured even with no hull vertex near it."""
    out = []
    for a, b, c in np.asarray(tris, float):
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / step)))
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = (i + j) <= n
        u, v = i[keep] / n, j[keep] / n
        out.append(a + u[:, None] * (b - a) + v[:, None] * (c - a))
    return np.concatenate(out) if out else np.zeros((0, 3))


@lru_cache(maxsize=1)
def hand_samples() -> dict[str, np.ndarray]:
    """Surface samples of every collision hull of the hand links, in each link's frame (read once from the URDF)."""
    import xml.etree.ElementTree as ET

    from yam_arena import ASSET_DIR, YAM_URDF
    from yam_arena.analysis import read_stl
    from yam_arena.urdf import rpy_matrix

    out: dict[str, list[np.ndarray]] = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        if link.get("name") not in HAND_LINKS:
            continue
        for col in link.findall("collision"):
            origin = col.find("origin")
            r, t = np.eye(3), np.zeros(3)
            if origin is not None:
                r = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                t = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
            tris = read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ r.T + t
            out.setdefault(link.get("name"), []).append(sample_triangles(tris))
    # Keep one sample per 1 mm cell: the meshes have many small triangles, and nearer samples add nothing.
    return {k: np.unique(np.round(np.concatenate(v), 3), axis=0) for k, v in out.items()}


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def parcel_gap(points_world: np.ndarray, packet) -> float:
    """Smallest distance (m) from any hand surface sample to any of the parcel's shapes; negative inside one."""
    from yam_arena.analysis import box_signed_distance

    boxes = PACKET.world_boxes({"base": (np.asarray(packet.pos, float), _rot(packet.quat_xyzw))})
    if len(points_world) == 0:
        return -math.inf
    return float(min(box_signed_distance(points_world, c, r, s).min() for _, c, r, s in boxes))


# Per-step record, kept on the shared parts module so every copy of this file in one process (the offline checker
# loads it once and near_misses.py imports it again) reads the same record. gap is None until measured: no credit.
_rec = parts.__dict__.setdefault("post_parcel_record", {})
_rec.update(gap=None)


def _hand_points(sim) -> np.ndarray | None:
    """Every hand surface sample of both arms in the world, or None unless all six hand links were read."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    samples = hand_samples()
    pts, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS and n in samples:
                pts.append(samples[n] @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    return np.concatenate(pts) if found == 2 * len(HAND_LINKS) else None


@parts.on_reset
def _new_trial(sim) -> None:
    _rec.update(gap=None)
    sim.set_target("letterbox", "chute", CHUTE_HOLD)


@parts.on_step
def _measure_release(sim) -> None:
    pts = _hand_points(sim)
    _rec["gap"] = None if pts is None else parcel_gap(pts, sim.states()["packet"])


def released() -> bool:
    return _rec["gap"] is not None and _rec["gap"] > CLEAR + EPS


# -- scoring --------------------------------------------------------------------------------------------------------


def _inside(p) -> bool:
    """The whole parcel inside the catch box: behind the gate's inner face (the throat's end), in front of the back
    wall, between the side walls and under the roof (bounding box, which is never smaller than the parcel)."""
    half = BOX_W / 2
    return bool(
        p.low[0] >= X_IN - EPS
        and p.high[0] <= X_IN + BOX_D + EPS
        and p.low[1] >= YC - half - EPS
        and p.high[1] <= YC + half + EPS
        and p.high[2] <= TOP + BOX_H + EPS
    )


def _flap_shut(objects) -> bool:
    return objects["letterbox"].joints["flap"] <= FLAP_SHUT + EPS


def success(objects):
    p = objects["packet"]
    return bool(_inside(p) and _flap_shut(objects) and p.resting() and released())


def score(objects):
    """1.0 inside the catch box with the flap shut, at rest and let go (no gripper or finger surface within 5 mm,
    measured every control step; no measurement scores 0); else 0."""
    return 1.0 if success(objects) else 0.0


# -- demo: the left arm grips the parcel's trailing end, loads it level into the slot until the hand meets the plate,
# and lets go; the parcel's centre is then past the throat's inner edge, so it tips onto the chute and slides into
# the catch box.
TIP = 0.013  # pad tips below the TCP
PAD_BACK = 0.013  # the pads reach about 13 mm behind the TCP (x) at the parcel's height
HAND_FRONT = 0.034  # the finger's widest part reaches 34 mm ahead of the TCP, above the slot
GRIP_Z = TOP + 0.003 + TIP  # pad tips 3 mm above the table
SAG_Z = 0.004  # the left arm settles about 4 mm low while carrying
BOTTOM = AP_Z0 + (AP_Z1 - AP_Z0 - H) / 2  # parcel underside centred in the opening: 155 mm
CARRY_Z = TOP + BOTTOM + 0.003 + TIP + SAG_Z
TRAVEL_Z = TOP + 0.19


def demo():
    rear = PACKET.pos[0] - L / 2
    gx = rear + PAD_BACK  # pads flush with the trailing end
    line_up = X_OUT - 0.010 - (L - PAD_BACK)  # leading end 10 mm short of the flap
    home = X_OUT - HAND_FRONT - 0.002  # hand 2 mm short of the plate
    y = YC
    left = [
        Waypoint(1.5, (gx, y, TRAVEL_Z), FINGER_OPEN, "over the trailing end"),
        Waypoint(1.5, (gx, y, GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (gx, y, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (gx, y, CARRY_Z), FINGER_CLOSED, "lift to slot height"),
        Waypoint(1.5, (line_up, y, CARRY_Z), FINGER_CLOSED, "line up"),
        Waypoint(3.0, (home, y, CARRY_Z), FINGER_CLOSED, "post it"),
        Waypoint(0.6, (home, y, CARRY_Z), FINGER_OPEN, "let go"),
        Waypoint(1.2, (home - 0.08, y, CARRY_Z), FINGER_OPEN, "back off"),
        Waypoint(1.0, (home - 0.08, y, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return {"left": left}
