"""On the station table, in front of the robot: a grey cast-iron housing block with a small round bore in its top
face. A bronze bushing has been started in the bore by hand, its top just below the face. Nearer the robot,
on the left and on the right, stand two dark tool stands, each holding one steel tool upright by its lower end: a
stepped driver (a square handle over a wide collar, with a short pilot below the collar) and a plain drift (the same
handle over a straight rod with a mushroomed end, and no collar). Which tool stands on which side changes between trials.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

IRON, DARK, STEEL = (0.33, 0.35, 0.37), (0.12, 0.13, 0.16), (0.70, 0.72, 0.74)
BRONZE, HOLE = (0.80, 0.52, 0.22), (0.18, 0.12, 0.06)

# --- housing: a fixed block whose part frame is on the table at its centre; the bore is on its vertical axis ------
HX, HY = 0.34, 0.0
HW, HH = 0.080, 0.040  # block width (x and y) and height: the face is 40 mm above the table
FACE = TOP + HH
BORE_R = 0.005  # 10 mm bore
BORE_DEPTH = 0.030
FLOOR_T = HH - BORE_DEPTH


def _slab(x0, x1, y0, y1, z0, z1, color=IRON):
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color)


def _bored_block(half_w, height, hole_r, z0=0.0, color=IRON, strips=10):
    """A square block with a round vertical hole on its axis, made of boxes: the hole is cut as `strips` strips
    across y, each as wide as the circle at the strip's inner edge, so the hole contains the circle."""
    z1 = z0 + height
    out = [_slab(-half_w, half_w, hole_r, half_w, z0, z1, color), _slab(-half_w, half_w, -half_w, -hole_r, z0, z1, color)]
    step = 2 * hole_r / strips
    for i in range(strips):
        y0, y1 = -hole_r + i * step, -hole_r + (i + 1) * step
        inner = min(abs(y0), abs(y1)) if y0 * y1 > 0 else 0.0
        w = math.sqrt(max(hole_r**2 - inner**2, 0.0))
        out += [_slab(-half_w, -w, y0, y1, z0, z1, color), _slab(w, half_w, y0, y1, z0, z1, color)]
    return out


# The bushing: a bronze sleeve 10 mm across and 15 mm tall, on a straight-down slider (the press fit). It starts
# with its top 1 mm below the face (PROUD is negative), so a pilot is already in the bore when it meets the bushing.
# A one-way ratchet (on_step below) moves the slider spring's rest point down to 0.5 mm short
# of the deepest point reached, so the bushing goes in only when pressed with more than about 1.5 N and never comes
# back up. The housing body is the slider's own body, so the bushing passes into it without colliding.
BUSH_R, BUSH_H = 0.005, 0.015
PROUD = -0.001  # the bushing's top above the face at the start
TRAVEL = 0.013  # to 14 mm below the face
K = 3000.0  # N/m
YIELD = 0.0005  # the spring gives way 0.5 mm past its set depth: 1.5 N
BUSH_MASS = 0.02
SAG = BUSH_MASS * 9.81 / K  # 0.065 mm under its own weight
_bz = HH + PROUD - BUSH_H / 2
SEAT = parts.Slider(
    "seat", axis=(0.0, 0.0, -1.0), travel=TRAVEL, mass=BUSH_MASS, spring=K, damping=10.0, max_effort=200.0,
    shapes=[parts.cylinder(BUSH_R, BUSH_H, at=(0.0, 0.0, _bz), color=BRONZE),
            parts.cylinder(0.0015, BUSH_H + 0.0002, at=(0.0, 0.0, _bz), color=HOLE)],  # the bushing's own bore
)
HOUSING = parts.Part(
    "housing", pos=(HX, HY, TOP),
    shapes=[_slab(-HW / 2, HW / 2, -HW / 2, HW / 2, 0.0, FLOOR_T), *_bored_block(HW / 2, BORE_DEPTH, BORE_R, FLOOR_T)],
    joints=[SEAT],
)

# --- tool stands: fixed dark blocks with a 12 mm square socket through them -----------------------------------
SX, SY = 0.22, 0.10  # stand centres at (SX, +SY) and (SX, -SY)
STAND_W, STAND_H, SOCKET = 0.032, 0.035, 0.014
STAND_TOP = TOP + STAND_H


def _stand(name, y):
    a, s = STAND_W / 2, SOCKET / 2
    return parts.Part(name, pos=(SX, y, TOP), shapes=[
        _slab(-a, a, s, a, 0.0, STAND_H, DARK), _slab(-a, a, -a, -s, 0.0, STAND_H, DARK),
        _slab(-a, -s, -s, s, 0.0, STAND_H, DARK), _slab(s, a, -s, s, 0.0, STAND_H, DARK)])


STAND_L, STAND_R = _stand("stand_left", SY), _stand("stand_right", -SY)

# --- tools: loose steel parts, frame origin at the tool's lower tip ----------------------------------------------
PILOT_R = 0.0035  # the driver's 7 mm pilot: 1.5 mm clear of the bore all round
REACH = 0.0054  # the pilot's length below the collar
COLLAR_R, COLLAR_T = 0.018, 0.005
HANDLE_W, HANDLE_L = 0.016, 0.045  # square handle
ROD_L = 0.060
ROD_R = 0.0045  # the drift's 9 mm rod: as wide as the sleeve's wall, too thick for its 3 mm bore
TIP_R, TIP_L = 0.006, 0.003  # its mushroomed end: 12 mm across, wider than the 10 mm bore
TOOL_MASS = 0.08
DRIVER_SHAPES = [
    parts.cylinder(PILOT_R, REACH, at=(0.0, 0.0, REACH / 2), color=STEEL),
    parts.cylinder(COLLAR_R, COLLAR_T, at=(0.0, 0.0, REACH + COLLAR_T / 2), color=STEEL),
    parts.box((HANDLE_W, HANDLE_W, HANDLE_L), at=(0.0, 0.0, REACH + COLLAR_T + HANDLE_L / 2), color=STEEL),
]
DRIFT_SHAPES = [
    parts.cylinder(TIP_R, TIP_L, at=(0.0, 0.0, TIP_L / 2), color=STEEL),
    parts.cylinder(ROD_R, ROD_L - TIP_L, at=(0.0, 0.0, TIP_L + (ROD_L - TIP_L) / 2), color=STEEL),
    parts.box((HANDLE_W, HANDLE_W, HANDLE_L), at=(0.0, 0.0, ROD_L + HANDLE_L / 2), color=STEEL),
]
HANDLE_TOP = {"driver": REACH + COLLAR_T + HANDLE_L, "drift": ROD_L + HANDLE_L}  # above the tool's tip
# Standing in a stand: the driver's collar rests on the stand's top; the drift's rod rests on the table in the socket.
# The driver's handle then starts 5 mm above the stand's top and the drift's 25 mm above it, over 25 mm of bare rod.
START_Z = {"driver": STAND_TOP - REACH, "drift": TOP}
DRIVER = parts.Part("driver", pos=(SX, -SY, START_Z["driver"]), shapes=DRIVER_SHAPES, mass=TOOL_MASS)
DRIFT = parts.Part("drift", pos=(SX, SY, START_Z["drift"]), shapes=DRIFT_SHAPES, mass=TOOL_MASS)
TOOLS = {"driver": DRIVER, "drift": DRIFT}

OBJECTS = [HOUSING, STAND_L, STAND_R, DRIVER, DRIFT]

# Lot A (the screenshot): the driver on the right-hand stand. Lot B: the tools swap stands.
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", driver_side="right", place={"driver": DRIVER.pos}),  # a zero move: the settle and a fresh first frame
    lots.Lot("B", driver_side="left", place={"driver": (SX, SY, START_Z["driver"]), "drift": (SX, -SY, TOP)}),
)

# --- scoring ----------------------------------------------------------------------------------------------------
EPS = 1e-6
DEPTH_LO, DEPTH_HI = 0.00425, 0.00575  # bushing top below the face: 5 mm +- 0.75 mm
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the bushing or of either tool
BUSH_STILL = 0.0005  # the bushing moved under 0.5 mm in the last control step (10 mm/s)
SUCCESS_HOLD_S = 1.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _overlap(a, b) -> bool:
    """Do two oriented boxes (centre, rotation, size) overlap? Separating-axis test."""
    (ca, ra, sa), (cb, rb, sb) = a, b
    ha, hb = np.asarray(sa) / 2, np.asarray(sb) / 2
    d = np.asarray(cb) - np.asarray(ca)
    axes = [ra[:, i] for i in range(3)] + [rb[:, i] for i in range(3)]
    axes += [np.cross(ra[:, i], rb[:, j]) for i in range(3) for j in range(3)]
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        if abs(ax @ d) > sum(ha[i] * abs(ax @ ra[:, i]) for i in range(3)) + sum(hb[i] * abs(ax @ rb[:, i])
                                                                                 for i in range(3)):
            return False
    return True


def _tool_boxes(name, st):
    return [(c, r, s) for _, c, r, s in TOOLS[name].world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]


# The bore below the face: any part of a tool in it (a pilot or rod still in the bore, or a tool resting on the
# bushing) means the tool has not been taken out.
BORE_BOX = (np.array([HX, HY, FACE - BORE_DEPTH / 2 - 0.0005]), np.eye(3), np.array([2 * BORE_R, 2 * BORE_R,
                                                                                     BORE_DEPTH]))


def tool_in_bore(name, st) -> bool:
    return any(_overlap(b, BORE_BOX) for b in _tool_boxes(name, st))


def depth(objects) -> float:
    """How far the bushing's top is below the housing face (m; negative: proud)."""
    return float(objects["housing"].joints["seat"]) - PROUD


# Simulator-only measurements, updated every control step by the on_step callback below. Offline (near_misses.py)
# there is no robot, so the grippers count as clear and the bushing as still.
_sim = {"clear": True, "still": True, "last_q": None, "set": 0.0}


def _settled(objects) -> bool:
    on_table = all(objects[n].low[2] > TOP - 0.005 for n in TOOLS)
    return bool(
        _sim["clear"] and _sim["still"] and on_table
        and all(objects[n].resting() for n in TOOLS)
        and not any(tool_in_bore(n, objects[n]) for n in TOOLS)
    )


def fitted(objects) -> bool:
    d = depth(objects)
    return bool(DEPTH_LO - EPS <= d <= DEPTH_HI + EPS)


def success(objects):
    """The bushing rests 4.25 to 5.75 mm below the face, both tools are out of the bore and let go, all still."""
    return bool(fitted(objects) and _settled(objects))


def distance(objects):
    return float(abs(depth(objects) - 0.005))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    return 1.0 if success(objects) else 0.0


# --- simulator callbacks: the ratchet and the release measurement -------------------------------------------------


@parts.on_reset
def _new_trial(sim) -> None:
    _sim.update(clear=True, still=True, last_q=None, set=0.0)


@parts.on_step
def _step(sim) -> None:
    from yam_arena import analysis

    st = sim.states()
    q = float(st["housing"].joints["seat"])
    if q > _sim["set"] + YIELD + EPS:
        _sim["set"] = q - YIELD
    sim.set_target("housing", "seat", _sim["set"])
    last = _sim["last_q"]
    _sim["still"] = last is None or abs(q - last) < BUSH_STILL
    _sim["last_q"] = q

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
    h = st["housing"]
    boxes = [(c, r, s) for _, c, r, s in HOUSING.world_boxes(
        {"seat": (np.asarray(h.links["seat"][0]), _rot(h.links["seat"][1]))})]
    for n in TOOLS:
        boxes += _tool_boxes(n, st[n])
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


# --- demo: the arm on the driver's side ---------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_BELOW_TOP = 0.024  # TCP 24 mm below the handle top: the tips stop 8 mm above the collar
HANDLE_GRIP = -0.005  # pads 10 mm apart on the 16 mm handle's flats: 3 mm past each side
OPEN = -0.020
TRAVEL_Z = TOP + 0.16
OVERPRESS = 0.010  # commanded past the collar landing: the soft arm then presses with about 2 N
DROP = 0.003  # the driver is let go with its collar 3 mm above the stand
# Measured carrying error of the loaded arm over the bore (TCP target minus where the pilot ended up), per arm.
CARRY_FIX = {"left": (0.0024, -0.0019), "right": (0.0020, 0.0017)}


def demo():
    """The arm on the driver's side (the stand whose tool has the wide collar, as the cameras show) lifts the driver
    out, presses it into the bore until its collar lands on the face, lifts it out and stands it back in its stand.
    The other arm stays parked."""
    lot = lots.current()
    arm = lot.driver_side
    other = "left" if arm == "right" else "right"
    sx, sy, sz = lot.pos("driver")
    dz = HANDLE_TOP["driver"] - GRIP_BELOW_TOP  # TCP above the driver's tip
    fx, fy = CARRY_FIX[arm]
    bx, by = HX + fx, HY + fy
    grip_z = sz + dz
    lift_z = grip_z + 0.03
    hover = FACE + 0.012 + dz  # pilot tip 12 mm above the face
    landed = FACE - REACH + dz  # collar on the face
    back = STAND_TOP - REACH + DROP + dz
    # The pads pinch along x: pinched along y, the driver leaned about y in its pads under the press.
    yaw = math.pi / 2 if arm == "right" else -math.pi / 2
    w = [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), OPEN, "over the driver", yaw),
        Waypoint(1.5, (sx, sy, grip_z), OPEN, "down to the handle", yaw),
        Waypoint(0.8, (sx, sy, grip_z), HANDLE_GRIP, "grip", yaw),
        Waypoint(1.0, (sx, sy, lift_z), HANDLE_GRIP, "lift out", yaw),
        Waypoint(2.0, (bx, by, hover), HANDLE_GRIP, "over the bore", yaw),
        Waypoint(1.0, (bx, by, hover), HANDLE_GRIP, "steady", yaw),
        Waypoint(2.5, (bx, by, landed + 0.002), HANDLE_GRIP, "pilot into the bore", yaw),
        Waypoint(3.0, (bx, by, landed - OVERPRESS), HANDLE_GRIP, "press: collar onto the face", yaw),
        Waypoint(1.0, (bx, by, landed - OVERPRESS), HANDLE_GRIP, "hold", yaw),
        Waypoint(1.5, (bx, by, hover), HANDLE_GRIP, "lift out of the bore", yaw),
        Waypoint(2.0, (sx, sy, lift_z), HANDLE_GRIP, "back over the stand", yaw),
        Waypoint(1.2, (sx, sy, back), HANDLE_GRIP, "lower into the socket", yaw),
        Waypoint(0.6, (sx, sy, back), OPEN, "let go", yaw),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), OPEN, "up", yaw),
        Waypoint(3.0, (sx - 0.05, sy, TRAVEL_Z), OPEN, "clear", yaw),
    ]
    park = {"left": (0.15, 0.25, TOP + 0.15), "right": (0.15, -0.25, TOP + 0.15)}
    return {arm: w, other: [Waypoint(sum(x.duration for x in w), park[other], OPEN, "park")]}
