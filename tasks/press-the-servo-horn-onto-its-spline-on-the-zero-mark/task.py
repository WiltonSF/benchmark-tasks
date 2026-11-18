"""On the station table, in front of the robot and to its right: a black hobby servo screwed to a small aluminium
base plate, its long side running away from the robot, with its splined output shaft at the far end of its top. A
white line is painted on the servo's top from the shaft towards the robot. Nearer the robot and further right, a
white single-arm servo horn lies on a black foam pad, hub down.
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

BLACK, WHITE, SILVER, FOAM = (0.05, 0.05, 0.06), (0.94, 0.94, 0.92), (0.75, 0.76, 0.78), (0.16, 0.16, 0.17)
STEEL = (0.55, 0.56, 0.58)

# --- the spline: 25 teeth, so the horn seats only every 360 / 25 = 14.4 degrees -------------------------------------
TEETH = 25
INDEX = 360.0 / TEETH
MARK_DEG = 180.0  # the painted zero mark points from the shaft towards the robot (world -x)

# --- the servo (fixed). Frame origin: on the table under the output shaft's axis ---------------------------------------
SX, SY = 0.300, -0.090
PLATE_T = 0.003
CASE_L, CASE_W, CASE_H = 0.0407, 0.0197, 0.037  # MG996R-size case; the shaft sits 10 mm from its far (+x) end
CASE_CX = 0.010 - CASE_L / 2  # case centre, relative to the shaft
BOSS_R, BOSS_H = 0.0065, 0.002
B = PLATE_T + CASE_H + BOSS_H  # the boss top, where a seated horn's hub stands: 42 mm above the table
SHAFT_R = 0.00295  # 5.9 mm across the spline
STANDOFF = 0.0035  # crest on crest the horn stands this far proud of seated (the original task's measured value)
PILOT_R, PILOT_TOP = 0.0016, 0.005  # the shaft's core, 3.2 mm across, up to 5 mm over the boss; it keeps the bore within 2 mm of the axis
COLLAR_W = 0.011  # the crest collar is an 11 mm square block, all but its corners hidden under the hub

# The press fit. The crests the horn's teeth stand on are a collar round the shaft on a straight-down slider with a
# stiff spring (K). An on_step rule (below) holds the spring's rest point where the collar is, so a resting or
# lightly pushed horn stays put. Only when the horn is centred over the shaft, level, turned to within CATCH of one
# of the 25 tooth positions, and pushed with at least SEAT_N does the rest point follow the collar down (it stays
# wherever it was pushed). Off a tooth position it never yields.
K = 40000.0  # N/m
SEAT_N = 32.0  # press force (the original task's measured value for this servo and horn)
YIELD = SEAT_N / K  # 0.8 mm
HOME = 0.0003  # within 0.3 mm of full depth, the crests go to it
COLLAR = parts.Slider(
    "crests", axis=(0.0, 0.0, -1.0), travel=STANDOFF, mass=0.05, spring=K, damping=60.0, max_effort=400.0,
    shapes=[parts.box((COLLAR_W, COLLAR_W, STANDOFF), at=(0.0, 0.0, B + STANDOFF / 2), color=BLACK)],
)
SERVO = parts.Part(
    "servo", pos=(SX, SY, TOP),
    shapes=[
        parts.box((0.070, 0.040, PLATE_T), at=(CASE_CX, 0.0, PLATE_T / 2), color=SILVER),
        parts.box((CASE_L, CASE_W, CASE_H), at=(CASE_CX, 0.0, PLATE_T + CASE_H / 2), color=BLACK),
        parts.box((0.054, CASE_W, 0.0025), at=(CASE_CX, 0.0, PLATE_T + 0.0275), color=BLACK),  # mounting ears
        parts.cylinder(BOSS_R, BOSS_H, at=(0.0, 0.0, PLATE_T + CASE_H + BOSS_H / 2), color=BLACK),
        parts.cylinder(PILOT_R, PILOT_TOP, at=(0.0, 0.0, B + PILOT_TOP / 2), color=STEEL),
        # the zero mark: a white paint line 3 mm wide, from 7 to 28 mm out from the shaft, towards the robot
        parts.box((0.021, 0.003, 0.0004), at=(-0.0175, 0.0, PLATE_T + CASE_H + 0.0002), color=WHITE),
    ],
    joints=[COLLAR],
)

# --- the horn (loose). Frame origin: the centre of the hub's underside; x along the arm --------------------------------
HUB_R, HUB_H, CAP_T = 0.0065, 0.0065, 0.0015  # 13 mm hub, 6.5 mm tall; its top 1.5 mm is solid (the screw boss)
BORE_R = 0.0033  # splined bore, 6.6 mm across the teeth
ARM_L, ARM_W, ARM_T = 0.030, 0.005, 0.002  # arm reaches 30 mm from the axis; its top is flush with the hub's
_WALL = 12  # the hub wall below the cap: 12 overlapping round lobes round the bore
_lobe_r = (HUB_R - BORE_R) / 2
HORN_SHAPES = [
    *[parts.box((2.6 * _lobe_r / 1.6 * 1.0, 2.6 * _lobe_r / 1.6, HUB_H - CAP_T),
                at=((BORE_R + _lobe_r) * math.cos(2 * math.pi * i / _WALL),
                    (BORE_R + _lobe_r) * math.sin(2 * math.pi * i / _WALL), (HUB_H - CAP_T) / 2), color=WHITE)
      for i in range(_WALL)],
    parts.cylinder(HUB_R, CAP_T, at=(0.0, 0.0, HUB_H - CAP_T / 2), color=WHITE),
    parts.box((ARM_L - HUB_R, ARM_W, ARM_T), at=((ARM_L + HUB_R) / 2, 0.0, HUB_H - ARM_T / 2), color=WHITE),
]
HORN_MASS = 0.020  # raised from about 3 g so PhysX can press it between a fingertip and the crests

# --- the foam pad (fixed) the horn lies on, hub down, its arm pointing away from the mark's direction --------------------
HX, HY = 0.210, -0.205
PAD_H = 0.025
HORN_START_DEG = 112.0  # the arm's direction at the start (world, from +x): 68 degrees from the mark, 4 off a tooth
PAD = parts.Part("foam_pad", pos=(HX, HY, TOP),
                 shapes=[parts.box((0.045, 0.045, PAD_H), at=(0.0, 0.0, PAD_H / 2), color=FOAM)])
HORN = parts.Part("horn", pos=(HX, HY, TOP + PAD_H), yaw=math.radians(HORN_START_DEG), shapes=HORN_SHAPES,
                  mass=HORN_MASS)

OBJECTS = [SERVO, PAD, HORN]

# --- geometry of the fit -------------------------------------------------------------------------------------------
CATCH = 2.5  # degrees: within this of a tooth position the horn's teeth find their slots
CENTRE = 0.002  # the bore within 2 mm of the shaft's axis: anywhere the shaft's core lets it sit
TILT_MAX = 4.0  # degrees: past this the bore binds on the shaft and does not go on
ON_COLLAR = 0.0005  # the hub stands on the crests when its underside is within 0.5 mm of them
LIFTED = 0.001  # more than 1 mm above them, it is off the teeth
STRIP_DEG = 4.0  # an engaged horn turned this far off its tooth position has stripped its teeth
SHIFT_MAX = 0.0023  # an engaged horn moved this far off the axis has been forced off the shaft sideways
SEATED_GAP = 0.0003  # seated: the hub at most 0.3 mm above the boss, the crests within 0.3 mm of full depth
SEATED_TILT = 2.0
STARTED = 0.001  # started: pressed at least 1 mm onto the teeth
CLEAR = 0.005  # let go: no part of either arm within 5 mm of the horn
STILL_STEPS = 20  # 1 s at 20 Hz
STILL_POS, STILL_DEG = 0.0002, 0.2


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def fit(horn) -> dict:
    """Where the horn is relative to the shaft: the hub underside's offset from the shaft axis (m) and height above
    the boss (m), its tilt (deg), the arm's angle from the zero mark (deg, -180..180) and the nearest tooth position
    (0 = on the mark) with the angle left over (deg)."""
    r = _rot(horn.quat_xyzw)
    p = np.asarray(horn.pos, float)
    arm = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    a = _wrap(arm - MARK_DEG)
    n = int(round(a / INDEX))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))
    return {"off": float(math.hypot(p[0] - SX, p[1] - SY)), "h": float(p[2] - (TOP + B)), "tilt": tilt, "a": a,
            "tooth": n % TEETH, "rem": a - n * INDEX}


def _crest(q: float) -> float:
    """Height of the crests (the collar's top) above the boss, with the collar pushed q down."""
    return STANDOFF - q


# --- the whole-trial record and the press-fit state, kept in the parts module so near_misses.py shares them -----------
_grip = parts.__dict__.setdefault("_horn_task_grip", {"clear": None})
_rec = parts.__dict__.setdefault("_horn_task_record", {})


def _clear_record():
    _rec.clear()
    _rec.update(damaged=False, tooth=None, set=0.0, prev=None, still=0, still_pose=None)


_clear_record()


def _update(objects) -> None:
    """Once per control step (from score()): which tooth the horn went on at, damage, and the stillness count.
    Cleared on a scene reset: the horn back within 0.5 mm of its start, at rest, after a jump of over 20 mm in one
    control step."""
    h = objects["horn"]
    pos = np.asarray(h.pos, float)
    prev = _rec["prev"]
    if (prev is not None and np.linalg.norm(pos - np.asarray(HORN.pos)) < 0.0005 and h.resting()
            and np.linalg.norm(pos - prev) > 0.020):
        _clear_record()
    _rec["prev"] = pos
    f = fit(h)
    q = float(objects["servo"].joints["crests"])
    if "offline_rest" in _grip:  # near_misses.py: no simulator, so it states the spring's rest point
        _rec["set"] = _grip["offline_rest"](objects)
    on_teeth = q >= 0.0005 - EPS and f["h"] <= _crest(q) + ON_COLLAR + EPS
    if f["h"] > _crest(q) + LIFTED + EPS:
        _rec["tooth"] = None  # pulled off the teeth: the next fit starts afresh
    elif on_teeth:
        if _rec["tooth"] is None:
            _rec["tooth"] = f["tooth"]
        off_tooth = abs(_wrap(f["a"] - _rec["tooth"] * INDEX))
        if off_tooth > STRIP_DEG + EPS or f["off"] > SHIFT_MAX + EPS:
            _rec["damaged"] = True
    now = (pos, _rot(h.quat_xyzw))
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


def _on_mark(objects) -> bool:
    f = fit(objects["horn"])
    return bool(_rec["tooth"] == 0 and abs(f["a"]) <= CATCH + EPS)


def seated(objects) -> bool:
    f = fit(objects["horn"])
    q = float(objects["servo"].joints["crests"])
    return bool(_on_mark(objects) and q >= STANDOFF - SEATED_GAP - EPS and f["h"] <= SEATED_GAP + EPS
                and f["tilt"] <= SEATED_TILT + EPS and f["off"] <= CENTRE + EPS)


def started(objects) -> bool:
    f = fit(objects["horn"])
    q = float(objects["servo"].joints["crests"])
    return bool(_on_mark(objects) and q >= STARTED - EPS and f["h"] <= _crest(q) + ON_COLLAR + EPS
                and f["tilt"] <= TILT_MAX + EPS and f["off"] <= CENTRE + EPS)


def _value(objects) -> float:
    if _rec["damaged"]:
        return 0.0
    if _grip["clear"] is not True or _rec["still"] < STILL_STEPS:
        return 0.0
    if seated(objects):
        return 1.0
    if started(objects):
        return 0.5
    return 0.0


def success(objects):
    """The horn fully home on the tooth position that puts its arm on the zero mark, let go and still for 1 s."""
    _clear(objects)
    return bool(_value(objects) == 1.0)


def distance(objects):
    f = fit(objects["horn"])
    return float(f["off"] + max(f["h"], 0.0) + 0.001 * abs(f["a"]))


def score(objects):
    _clear(objects)
    _update(objects)
    return _value(objects)


# --- simulator callbacks: the press fit and the let-go measurement ---------------------------------------------------
@parts.on_step
def _press_fit(sim) -> None:
    st = sim.states()
    q = float(st["servo"].joints["crests"])
    f = fit(st["horn"])
    standing = f["h"] <= _crest(q) + ON_COLLAR + EPS
    rest = _rec["set"]
    if f["h"] > _crest(q) + LIFTED + EPS:
        rest = 0.0  # off the teeth: the next horn stands on the crests again
    elif (standing and abs(f["rem"]) <= CATCH + EPS and f["off"] <= CENTRE + EPS and f["tilt"] <= TILT_MAX + EPS
          and K * (q - rest) >= SEAT_N - EPS):
        rest = min(q - YIELD, STANDOFF)
        if q >= STANDOFF - HOME - EPS:  # the hub is on the boss: the crests stay at full depth
            rest = STANDOFF + 0.0005  # home: held against the full-depth stop
    _rec["set"] = rest
    sim.set_target("servo", "crests", rest)


FACE_STEP = 0.003  # arm hull faces are sampled about every 3 mm
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def _face_points(vertices, planes, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull about `step` apart, corners included, so a flat face lying on the horn
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


def horn_boxes(horn) -> list:
    """(centre, rotation, size) of the boxes round every shape of the horn (a cylinder's box is a little bigger
    than the cylinder, so the 5 mm let-go rule is, if anything, stricter)."""
    return [(c, r, s) for _, c, r, s in HORN.world_boxes({"base": (np.asarray(horn.pos), _rot(horn.quat_xyzw))})]


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
    boxes = horn_boxes(sim.states()["horn"])
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
    sim.set_target("servo", "crests", 0.0)


# --- demo: open loop from the published start layout ------------------------------------------------------------------
TIP = 0.013  # pad tips below the TCP
GRIP_UP = 0.0015 + TIP  # TCP above the hub's underside while gripping: pad tips 1.5 mm above it
SQUEEZE = -0.0035  # pads commanded 7 mm apart on the 13 mm hub: 3 mm past each side
OPEN = -0.020  # pads 40 mm apart
CLOSED = 0.0
TRAVEL_Z = TOP + 0.13
PRESS = 0.075  # commanded this far below the seated cap: the soft arm then pushes about 35 N
CARRY_FIX = (0.0006, 0.00014)  # fixed carrying offset of the loaded right arm over the shaft, measured once in a trace
YAW_FIX = 2.0  # degrees: the horn turns this much the other way in the pads while carried, measured once in a trace
PICK_YAW = math.radians(HORN_START_DEG - 180.0)  # pads across the hub, the arm between them
# Cheat and near-miss variants, for traces only (DEMO_VARIANT=...): the same plan with the wrist turned so the horn
# goes on as picked, one tooth off, or turned back and forth while pressed.
VARIANTS = {"as_picked": HORN_START_DEG - 180.0, "one_tooth": INDEX}  # wrist yaw over the shaft, degrees


def demo():
    """The right arm grips the horn's hub, lifts it off the pad, carries it over the servo while turning the wrist
    so the arm lies along the painted line, lowers its bore over the shaft's tip onto the teeth and presses it home
    while still gripping. It opens slowly, rises straight up, closes its fingers, gives a second press with the
    fingertips, eases off and moves away. The left arm stays parked."""
    import os

    variant = os.environ.get("DEMO_VARIANT", "")
    place_yaw = math.radians(VARIANTS.get(variant, 0.0) + YAW_FIX)
    sx, sy = SX + CARRY_FIX[0], SY + CARRY_FIX[1]
    gz = TOP + PAD_H + GRIP_UP
    crest = TOP + B + STANDOFF  # hub underside on the crests
    seat = TOP + B  # the hub's underside when seated
    seat_top = seat + HUB_H  # the cap's top when seated
    w = [
        Waypoint(1.5, (HX, HY, TRAVEL_Z), OPEN, "over the horn", PICK_YAW),
        Waypoint(1.5, (HX, HY, gz), OPEN, "pads either side of the hub", PICK_YAW),
        Waypoint(0.8, (HX, HY, gz), SQUEEZE, "grip the hub", PICK_YAW),
        Waypoint(1.0, (HX, HY, gz + 0.040), SQUEEZE, "lift", PICK_YAW),
        Waypoint(2.5, (sx, sy, crest + GRIP_UP + 0.030), SQUEEZE, "over the shaft, arm turned onto the line",
                 place_yaw),
        Waypoint(1.0, (sx, sy, crest + GRIP_UP + 0.004), SQUEEZE, "just above the shaft's tip", place_yaw),
        Waypoint(1.5, (sx, sy, crest + GRIP_UP + 0.004), SQUEEZE, "steady", place_yaw),
        Waypoint(1.5, (sx, sy, crest + GRIP_UP + 0.0002), SQUEEZE, "bore over the tip, onto the teeth", place_yaw),
        Waypoint(3.0, (sx, sy, seat + GRIP_UP - PRESS), SQUEEZE, "press home, still gripping the hub", place_yaw),
        Waypoint(1.0, (sx, sy, seat + GRIP_UP - PRESS), SQUEEZE, "hold", place_yaw),
        Waypoint(1.5, (sx, sy, seat + GRIP_UP - 0.002), SQUEEZE, "ease off", place_yaw),
        Waypoint(2.0, (sx, sy, seat + GRIP_UP - 0.002), OPEN, "open slowly", place_yaw),
        Waypoint(1.5, (sx, sy, seat_top + TIP + 0.020), OPEN, "straight up along the shaft", place_yaw),
        Waypoint(0.8, (sx, sy, seat_top + TIP + 0.020), CLOSED, "close the fingers", place_yaw),
        Waypoint(1.0, (sx, sy, seat_top + TIP + 0.003), CLOSED, "fingertips onto the hub", place_yaw),
        Waypoint(2.0, (sx, sy, seat_top + TIP - PRESS), CLOSED, "second press with the fingertips", place_yaw),
        Waypoint(1.5, (sx, sy, seat_top + TIP + 0.002), CLOSED, "ease off", place_yaw),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), CLOSED, "up", place_yaw),
        Waypoint(1.5, (0.20, -0.30, TRAVEL_Z), OPEN, "away", 0.0),
        Waypoint(3.0, (0.20, -0.30, TRAVEL_Z), OPEN, "wait clear", 0.0),
    ]
    return {"right": w, "left": []}


DEMO_TOUCHES = ("servo",)  # the press is commanded past the seat; the fingertips stop on the horn, not the servo
