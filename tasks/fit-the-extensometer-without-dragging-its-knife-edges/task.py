"""A horizontal force test stand holds a white plastic dogbone test specimen in its two wedge grips. The specimen's
axis runs left to right in front of the robot, with its broad faces vertical: one face looks toward the robot and
the other toward the machine. Two short black ink gauge marks, 25 mm apart, are drawn on the specimen's upper edge
at the middle of its narrow section. A black clip-on extensometer (strain sensor) lies flat in its open carry case
on the bench to the robot's left, its two steel arms and knife edges pointing away from the robot and a thin wire
clip on each arm. Its cable runs to the machine.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from dataclasses import replace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (import before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402,F401  (kept for tilted grips; this demo grips straight down)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # score() itself already asks for a 1 s hold
EPS = 1e-6

SILVER, STEEL, BLACK, BLUE = (0.71, 0.73, 0.75), (0.55, 0.57, 0.60), (0.08, 0.09, 0.10), (0.06, 0.22, 0.46)
WHITE, INK, CASE_GREY, LABEL = (0.91, 0.90, 0.85), (0.02, 0.02, 0.02), (0.18, 0.18, 0.19), (0.80, 0.80, 0.78)

# --- specimen frame -------------------------------------------------------------------------------------------------
# ASTM D638 Type I: 165 x 19 x 3.2 mm, 57 x 13 mm narrow section, 76 mm shoulder radii, 115 mm between jaw mouths.
# Axis along world y; thickness along x (broad faces vertical); width along z.
X, Y = 0.330, 0.020  # specimen centre (m)
AXIS = TOP + 0.086  # height of the specimen's centre line
THICK, NARROW_W, END_W, NARROW_L, JAW_GAP = 0.0032, 0.013, 0.019, 0.057, 0.115
XF, XB = X - THICK / 2, X + THICK / 2  # front face (toward the robot) and back face
GAUGE = 0.025  # extensometer gauge length; the ink marks sit at Y +- GAUGE/2

# --- machine: Mark-10 ESM303H horizontal stand with two G1061 wedge grips (frame drawing estimates) ----------------
machine = [parts.box((.100, .7171, .100), (.190, 0, .065), BLUE),
           parts.box((.3411, .0319, .1684), (.06945, -.3562, .0842), SILVER),
           parts.box((.170, .0259, .1684), (.180, .3652, .0842), BLUE),
           parts.box((.030, .690, .012), (.127, 0, .122), SILVER),
           parts.box((.030, .690, .012), (.127, 0, .040), SILVER),
           parts.box((.0828, .026, .0711), (.130, .205, .116), SILVER),
           parts.box((.080, .105, .032), (.190, .205, .146), SILVER),
           parts.box((.120, .030, .010), (.190, .277, .220), SILVER),
           parts.box((.150, .090, .128), (.190, .277, .284), BLUE),
           parts.box((.100, .002, .048), (.190, .230, .300), BLACK),
           parts.cylinder(.012, .012, (.190, .222, .260), (.7, .06, .03), 'y'),
           parts.cylinder(.023, .034, (0, .1567, .086), SILVER, 'y'),
           parts.cylinder(.00635, .020, (0, .1297, .086), SILVER, 'y'),
           parts.cylinder(.00635, .020, (0, -.1297, .086), SILVER, 'y')]
# The short-grip setting puts the gauge region near the fixed end of the 750.3 mm frame, not at its middle.
FRAME_SHIFT = .20055
for i in (0, 1, 2, 3, 4, 7, 8, 9, 10):
    sh = machine[i]
    machine[i] = replace(sh, at=(sh.at[0], sh.at[1] + FRAME_SHIFT, sh.at[2]))
for i in (5, 6):
    sh = machine[i]
    machine[i] = replace(sh, at=(sh.at[0], .2007, sh.at[2]))
machine.extend([parts.box((.100, .020, .012), (.055, .18355, .086), SILVER),
                parts.box((.016, .030, .110), (.190, .277 + FRAME_SHIFT, .160), SILVER),
                parts.box((.010, .025, .025), (.135, .240, .100), BLACK),
                parts.cylinder(.004, .030, (.115, .240, .100), SILVER, 'x')])
for sy in (-1, 1):
    # G1061 wedge grip: two body halves either side of the jaw slot, two jaw faces clamping the specimen's
    # broad faces, the spring knob and its handle pointing toward the robot.
    cy = sy * (JAW_GAP / 2 + .0622 / 2)
    machine.extend([parts.box((.012, .0622, .0381), (-.01875, cy, .086), SILVER),
                    parts.box((.012, .0622, .0381), (.01875, cy, .086), SILVER),
                    parts.box((.008, .045, .0255), (-THICK / 2 - .004, cy + sy * .0086, .086), SILVER),
                    parts.box((.008, .045, .0255), (THICK / 2 + .004, cy + sy * .0086, .086), SILVER),
                    parts.cylinder(.004, .035, (-.042, cy, .086), SILVER, 'x'),
                    parts.cylinder(.011, .025, (-.063, cy, .086), BLACK, 'x')])
MACHINE = parts.Part("horizontal_test_machine", pos=(X, Y, TOP), shapes=machine, friction=(.4, .3))


def half_width(dy: float) -> float:
    """Half the specimen's width (z) at distance ``dy`` along its axis from the centre."""
    a = abs(dy)
    if a <= NARROW_L / 2:
        return NARROW_W / 2
    if a <= NARROW_L / 2 + .029:
        return min(END_W / 2, NARROW_W / 2 + (.076 - math.sqrt(max(0.0, .076 ** 2 - (a - NARROW_L / 2) ** 2))))
    return END_W / 2 if a <= .0825 else 0.0


coupon = [parts.box((THICK, NARROW_L, NARROW_W), (0, 0, 0), WHITE)]
for sign in (-1, 1):
    for k in range(16):
        yy = NARROW_L / 2 + (k + .5) * .0018125
        coupon.append(parts.box((THICK, .0018125, 2 * half_width(yy)), (0, sign * yy, 0), WHITE))
    coupon.append(parts.box((THICK, .025, END_W), (0, sign * .070, 0), WHITE))
    # ink gauge mark on the upper edge (0.1 mm proud), 25 mm apart, centred on the narrow section
    coupon.append(parts.box((THICK, .0006, .0001), (0, sign * GAUGE / 2, NARROW_W / 2 + .00005), INK))
SPECIMEN = parts.Part("type_i_specimen", pos=(X, Y, AXIS), shapes=coupon, friction=(.35, .3))
COUPON_BOXES = [(np.array((X, Y, AXIS)) + np.asarray(s.at), np.asarray(s.size) / 2) for s in coupon]

# --- Instron 2630-105 clip-on extensometer, lying flat (sensor frame: +x along the arms toward the knife edges,
# y along the gauge length, z up; origin at the housing's centre on the arms' plane) -------------------------------
HOUSING = (.044, .052, .039)  # catalogue core 100 x 52 x 39 mm with the arms; housing drawn as a box
L_TIP = .078  # knife-edge line, 78 mm in front of the housing centre (100 mm overall)
BLADE_HALF = .005  # each knife edge is 10 mm long (an estimate)
CLIP_Y = .0165  # each wire clip runs beside its arm, 4 mm outboard
WIRE_R = .0008
CLIP_TOP_Z = .0095  # clip wire over the specimen's upper edge (2.2 mm clearance when fitted centrally)
CLIP_BACK_X = L_TIP + THICK + .003 + WIRE_R  # rigid clip held open: 3 mm behind the back face when fitted
HOOK = (CLIP_BACK_X, -.003)  # (x, z) of each clip's hook end, behind the specimen

sensor = [parts.box(HOUSING, (0, 0, 0), BLACK),
          parts.box((.026, .016, .0004), (-.004, 0, HOUSING[2] / 2 + .0002), LABEL),
          parts.cylinder(.005, .0015, (.014, 0, HOUSING[2] / 2 + .00075), SILVER, 'z')]
for sign in (-1, 1):
    yy = sign * GAUGE / 2
    sensor += [parts.box((.051, .006, .006), (.0475, yy, 0), SILVER),  # measuring arm
               parts.box((.004, .006, .012), (.075, yy, 0), SILVER),  # knife-edge holder
               parts.box((.001, .0006, 2 * BLADE_HALF), (L_TIP - .0005, yy, 0), STEEL)]  # knife edge
    cy = sign * CLIP_Y
    path = [(.074, .004), (.074, CLIP_TOP_Z), (CLIP_BACK_X, CLIP_TOP_Z), HOOK, (CLIP_BACK_X + .010, HOOK[1]),
            (CLIP_BACK_X + .010, .004)]
    pts = [(px, cy, pz) for px, pz in path]
    sensor += [parts.wire(a, b, WIRE_R, SILVER) for a, b in zip(pts, pts[1:])]
    sensor += [parts.sphere(WIRE_R, p, SILVER) for p in pts[1:-1]]
    sensor.append(parts.box((.003, .0045, .003), (.074, sign * (GAUGE / 2 + .00525), .004), SILVER))  # clip mount

# --- the open carry case and the two layouts -----------------------------------------------------------------------
CASE_PLATE = .006
case_shapes = [parts.box((.235, .165, CASE_PLATE), (0, 0, 0), CASE_GREY),
               parts.box((.235, .008, .065), (0, .0785, .0355), CASE_GREY),  # open lid, standing at the back
               parts.box((.235, .006, .012), (0, -.0795, .009), CASE_GREY),  # front rim
               parts.box((.006, .153, .012), (-.1145, 0, .009), CASE_GREY),  # near rim
               parts.box((.006, .153, .012), (.1145, 0, .009), CASE_GREY)]  # far rim
CASE_A = (.185, .160, TOP + CASE_PLATE / 2)
REST_Z = TOP + CASE_PLATE + HOUSING[2] / 2  # sensor origin height when it lies in the case
SENSOR_IN_CASE = (-.022, -.010)
SENSOR_A = (CASE_A[0] + SENSOR_IN_CASE[0], CASE_A[1] + SENSOR_IN_CASE[1], REST_Z)
LOT_B_SHIFT = (-.035, -.040)
CASE_B = (CASE_A[0] + LOT_B_SHIFT[0], CASE_A[1] + LOT_B_SHIFT[1], CASE_A[2])
SENSOR_B = (SENSOR_A[0] + LOT_B_SHIFT[0], SENSOR_A[1] + LOT_B_SHIFT[1], REST_Z)

SENSOR = parts.Part("extensometer", pos=SENSOR_A, mass=.056, shapes=sensor, friction=(.5, .4))
CASE = parts.Part("open_storage_case", pos=CASE_A, mass=.45, shapes=case_shapes, friction=(.6, .5))
OBJECTS = [MACHINE, SPECIMEN, SENSOR, CASE]
# The case and the sensor in it move together between layouts; the machine and the gripped specimen stay put.
LOTS = lots.declare(OBJECTS,
                    lots.Lot("A", place={SENSOR.name: SENSOR_A, CASE.name: CASE_A}),
                    lots.Lot("B", place={SENSOR.name: SENSOR_B, CASE.name: CASE_B}))
DEMO_TOUCHES: set = set()

# Thin knife edges and wires need a contact offset smaller than their gaps.
parts.CONTACT_OFFSET = .0002
parts.REST_OFFSET = 0.
parts.SOLVER_ITERATIONS = (64, 8)
ROOT_PATHS: dict = {}
_author = getattr(parts, "_author_original", parts._author)
parts._author_original = _author


def _record_author(part, path, translation, orientation):
    ROOT_PATHS[part.name] = path
    return _author(part, path, translation, orientation)


parts._author = _record_author

# --- rules (stated estimates) ---------------------------------------------------------------------------------------
TOUCH_GAP = .0003  # m: a knife-edge point this close to the specimen touches it (the contact tolerance)
SEAT_N = .1  # N: least specimen contact force for the clips to take over
DRAG_LIMIT = .0005  # m: length of the track a touching knife edge may leave on the specimen
OVERLOAD_N = 8.0  # N: specimen or machine contact force that dents the specimen or blunts a knife edge
HAND_PULL_N = 2.0  # N: net hand force on a fitted extensometer that makes its knife edges slip (clip estimate)
HAND_CRUSH_N = 40.0  # N: grip force on the housing that counts as crushing it
CENTRE_Y, CENTRE_Z, LEAN_DEG = .002, .0015, 3.0  # gauge tolerance: along the axis, across the width, edge lean
HAND_GAP = .010  # m: every hand surface this far from the extensometer counts as clear
HOLD_S = 1.0
START_TOL, START_DEG = .004, 3.0
BLADE_PTS = np.array([(L_TIP, s * GAUGE / 2, z) for s in (-1, 1) for z in (-BLADE_HALF, 0.0, BLADE_HALF)])
HOOK_PTS = np.array([(HOOK[0], s * CLIP_Y, HOOK[1]) for s in (-1, 1)])

REC: dict = parts.__dict__.setdefault("_extensometer_record", {})


def reset_record() -> None:
    REC.clear()
    REC.update(known=False, zero=False, why="", checked=False, fit=None, drag=np.zeros(2), track=[None, None],
               tiers=[], t=None, physics=0, peak=dict(specimen=0.0, machine=0.0, hand=0.0, hand_net=0.0),
               step_peak=dict(specimen=0.0, machine=0.0, hand=0.0, hand_net=0.0))


reset_record()


def _R(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def specimen_distance(p: np.ndarray) -> np.ndarray:
    """Distance (m) from each point to the specimen's solid (0 inside)."""
    p = np.atleast_2d(p)
    d = np.full(len(p), np.inf)
    for c, h in COUPON_BOXES:
        d = np.minimum(d, np.linalg.norm(np.maximum(np.abs(p - c) - h, 0.0), axis=1))
    return d


def blade_points(pos, quat) -> np.ndarray:
    return BLADE_PTS @ _R(quat).T + np.asarray(pos)


def hook_points(pos, quat) -> np.ndarray:
    return HOOK_PTS @ _R(quat).T + np.asarray(pos)


def on_front_face(p) -> bool:
    """A point in front of the specimen's front face, within its width and between the jaw mouths."""
    dy = float(p[1] - Y)
    return abs(dy) <= JAW_GAP / 2 and abs(float(p[2] - AXIS)) <= half_width(dy) + EPS and float(p[0]) <= XF + TOUCH_GAP


def hooked(pos, quat) -> bool:
    """The extensometer upright, both clips' top wires above the specimen's upper edge, both hooks behind the back
    face and below that edge, and both knife edges in front of the front face: the clips straddle the specimen."""
    r = _R(quat)
    if r[2, 2] < math.cos(math.radians(45.0)):
        return False
    tops = np.array([((L_TIP + CLIP_BACK_X) / 2, s * CLIP_Y, CLIP_TOP_Z) for s in (-1, 1)]) @ r.T + np.asarray(pos)
    for h, top in zip(hook_points(pos, quat), tops):
        dy = float(h[1] - Y)
        if not (abs(dy) <= JAW_GAP / 2 and float(h[0]) >= XB - EPS and float(h[2] - AXIS) <= half_width(dy) + EPS
                and float(top[2] - AXIS) >= half_width(float(top[1] - Y)) - EPS):
            return False
    return all(float(b[0]) <= XF + TOUCH_GAP for b in blade_points(pos, quat))


def seated(pos, quat) -> bool:
    """Every point along both knife edges touches the front face (within TOUCH_GAP), inside its width, between the
    jaws."""
    b = blade_points(pos, quat)
    return bool(np.all(specimen_distance(b) <= TOUCH_GAP + EPS) and all(on_front_face(p) for p in b))


def fit_quality(pos, quat) -> dict:
    b = blade_points(pos, quat)
    mid = b.mean(axis=0)
    lean = max(math.degrees(math.atan2(abs(float(b[2 + 3 * k, 1] - b[3 * k, 1])), float(b[2 + 3 * k, 2] - b[3 * k, 2])))
               for k in (0, 1))
    off_y, off_z = float(mid[1] - Y), float(max(abs(b[1, 2] - AXIS), abs(b[4, 2] - AXIS)))
    return dict(off_y=off_y, off_z=off_z, lean=lean,
                ok=bool(abs(off_y) <= CENTRE_Y + EPS and off_z <= CENTRE_Z + EPS and lean <= LEAN_DEG + EPS))


def _zero(why: str) -> None:
    if not REC["zero"]:
        print(f"[extensometer] score 0 for this trial: {why}", file=sys.stderr, flush=True)
    REC.update(zero=True, why=REC["why"] or why)


def physics_update(pos, quat, f_specimen: float, f_machine: float, f_hand: float, f_hand_net: float) -> bool:
    """One physics substep of history from the extensometer's pose and the contact forces (N) on it: the drag
    record, the damage rules and the clip fit. Returns True once the clips hold the extensometer."""
    pk, sp = REC["peak"], REC["step_peak"]
    for k, v in (("specimen", f_specimen), ("machine", f_machine), ("hand", f_hand), ("hand_net", f_hand_net)):
        pk[k] = max(pk[k], float(v))
        sp[k] = max(sp[k], float(v))
    REC["physics"] += 1
    if f_machine > OVERLOAD_N + EPS:
        _zero(f"the extensometer struck the machine with {f_machine:.1f} N")
    if f_hand > HAND_CRUSH_N + EPS:
        _zero(f"the grip crushed the housing with {f_hand:.1f} N")
    if REC["fit"] is not None:
        if f_hand_net > HAND_PULL_N + EPS:
            _zero(f"a hand pushed or pulled the fitted extensometer with {f_hand_net:.1f} N")
        return True
    if f_specimen > OVERLOAD_N + EPS:
        _zero(f"the extensometer pressed the specimen with {f_specimen:.1f} N")
    # The track each knife edge leaves: from the substep it touches the specimen with any force until it leaves
    # the contact tolerance, the extent (y, z) of the positions its centre took along the face.
    b = blade_points(pos, quat)
    near = specimen_distance(b) <= TOUCH_GAP + EPS
    for k in (0, 1):
        centre = b[3 * k + 1][1:]
        if not bool(np.any(near[3 * k:3 * k + 3])):
            REC["track"][k] = None
            continue
        if REC["track"][k] is None:
            if f_specimen <= 0.0:
                continue
            REC["track"][k] = (centre.copy(), centre.copy())
        lo, hi = REC["track"][k]
        lo, hi = np.minimum(lo, centre), np.maximum(hi, centre)
        REC["track"][k] = (lo, hi)
        REC["drag"][k] = max(float(REC["drag"][k]), float(np.linalg.norm(hi - lo)))
    if float(np.max(REC["drag"])) > DRAG_LIMIT + EPS:
        _zero(f"a knife edge left a {1000 * float(np.max(REC['drag'])):.2f} mm track on the specimen")
    if not REC["zero"] and f_specimen >= SEAT_N - EPS and seated(pos, quat) and hooked(pos, quat):
        REC["fit"] = dict(pos=np.asarray(pos, float).copy(), quat=np.asarray(quat, float).copy(),
                          quality=fit_quality(pos, quat), force=float(f_specimen))
        print(f"[extensometer] clips fitted: {REC['fit']['quality']}", file=sys.stderr, flush=True)
        return True
    return False


def at_fit(s) -> bool:
    """The extensometer is where its clips took it (within 0.5 mm and 0.5 degrees)."""
    fit = REC["fit"]
    if fit is None:
        return False
    turn = 2 * math.degrees(math.acos(min(1.0, abs(float(np.dot(fit["quat"], np.asarray(s.quat_xyzw, float)))))))
    return bool(np.linalg.norm(np.asarray(s.pos) - fit["pos"]) <= .0005 + EPS and turn <= .5 + EPS)


def level(s, clear: bool, touching: bool) -> float:
    """The tier the extensometer shows now (whole-trial zero rules are applied by score())."""
    if REC["fit"] is not None:
        if not at_fit(s):
            return 0.0
        if not REC["fit"]["quality"]["ok"]:
            return 0.5
        return 1.0 if clear and not touching else 0.75
    if hooked(s.pos, s.quat_xyzw) and s.resting(.005, .1):
        return 0.25
    return 0.0


def control_update(o, t: float, clear: bool, touching: bool) -> float:
    """One control step: the drop rule and the tier history the 1 s hold reads."""
    s = o[SENSOR.name]
    if REC["fit"] is None and (float(s.low[2]) < TOP + .004 or tilt.tilt_of(s.quat_xyzw) > 45.0):
        _zero("the extensometer fell")
    lv = level(s, clear, touching)
    REC.update(clear=bool(clear), touching=bool(touching))
    REC["tiers"].append(lv)
    del REC["tiers"][:-int(round(HOLD_S * 20)) - 1]
    REC.update(t=t, known=True)
    for k in REC["step_peak"]:
        REC["step_peak"][k] = 0.0
    return lv


def score(objects) -> float:
    """The lowest tier shown over the last second (and now), or 0 under a whole-trial rule."""
    try:
        if not REC.get("known") or REC.get("zero"):
            return 0.0
        tiers = REC["tiers"]
        n = int(round(HOLD_S * 20))
        if len(tiers) < n:
            return 0.0
        now = level(objects[SENSOR.name], REC.get("clear", False), REC.get("touching", True))
        return float(min(min(tiers[-n:]), now))
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# --- simulator side ---------------------------------------------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_SIM: dict = {}
_FACE_PTS: dict = {}


def _arr(x) -> np.ndarray:
    x = x.torch if hasattr(x, "torch") else x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def _view(base):
    from isaaclab.sim.utils import get_current_stage
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics

    sv = PhysxManager.get_physics_sim_view()
    env = ROOT_PATHS[SENSOR.name].rsplit("/", 1)[0]
    hands = []

    def walk(prim):
        for c in prim.GetAllChildren():
            yield c
            yield from walk(c)

    for prim in get_current_stage().GetPrimAtPath(env).GetAllChildren():
        if "Arm" in prim.GetName():
            hands += [p.GetPath().pathString for p in walk(prim)
                      if p.GetName() in HAND_LINKS and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(hands) != 6:
        raise RuntimeError(f"found {len(hands)} hand links, expected 6")
    filters = [ROOT_PATHS[SPECIMEN.name], ROOT_PATHS[MACHINE.name], ROOT_PATHS[CASE.name], *hands]
    return sv.create_rigid_contact_view(ROOT_PATHS[SENSOR.name], filter_patterns=filters, max_contact_data_count=256)


def _pose(base):
    asset = base.scene[SENSOR.name]
    p = _arr(asset.data.root_link_pose_w)[0].astype(np.float64)
    origin = parts._np(base.scene.env_origins[0])
    return asset, p[:3] - origin, p[3:], origin


def _forces(base):
    if "view" not in _SIM:
        _SIM["view"] = _view(base)
    f = _arr(_SIM["view"].get_contact_force_matrix(dt=base.physics_dt)).reshape(-1, 3).astype(np.float64)
    if f.shape != (9, 3) or not np.isfinite(f).all():
        raise RuntimeError(f"contact data shape {f.shape}")
    mag = np.linalg.norm(f, axis=1)
    return float(mag[0]), float(mag[1]), float(mag[3:].max()), float(np.linalg.norm(f[3:].sum(axis=0)))


def _pin(asset, origin) -> None:
    """The fitted clips hold the extensometer where it was fitted (written after every physics substep)."""
    import torch

    fit = REC["fit"]
    pose = asset.data.root_link_pose_w
    pose = (pose.torch if hasattr(pose, "torch") else pose).clone()
    pose[0, :3] = torch.tensor(fit["pos"] + origin, dtype=pose.dtype, device=pose.device)
    pose[0, 3:] = torch.tensor(fit["quat"], dtype=pose.dtype, device=pose.device)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    vel = asset.data.default_root_vel
    vel = vel.torch if hasattr(vel, "torch") else vel
    asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(vel))


@parts.on_physics_step
def _on_physics(sim) -> None:
    try:
        base = sim.env.unwrapped
        asset, pos, quat, origin = _pose(base)
        fs, fm, fh, fhn = _forces(base)
        if physics_update(pos, quat, fs, fm, fh, fhn):
            _pin(asset, origin)
    except Exception as exc:  # noqa: BLE001  fail closed
        if not REC["zero"]:
            import traceback

            traceback.print_exc()
        _zero(f"physics witness error {type(exc).__name__}: {exc}")


def _hull_surface(h, spacing=0.003):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
    return np.unique(np.round(np.concatenate(out) / 0.0025).astype(np.int64), axis=0) * 0.0025


def hand_points(base) -> np.ndarray:
    """Points every 3 mm on both arms' gripper and finger collision hulls (env frame)."""
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                if n not in _FACE_PTS:
                    _FACE_PTS[n] = np.concatenate([_hull_surface(h) for h in analysis.link_hulls().get(n, [])])
                pts.append(_FACE_PTS[n] @ parts._quat_to_matrix(poses[i, 3:]).T + (poses[i, :3] - origin))
    if len(pts) != 6:
        raise RuntimeError("hand links not found")
    return np.concatenate(pts)


def hands_clear(points, s) -> bool:
    """Every hand point at least HAND_GAP from every shape of the extensometer."""
    from yam_arena import analysis

    if float(np.linalg.norm(np.maximum(np.maximum(s.low - points, points - s.high), 0.0), axis=1).min()) >= HAND_GAP:
        return True
    for _, c, r, size in SENSOR.world_boxes({"base": (s.pos, _R(s.quat_xyzw))}):
        if float(analysis.box_signed_distance(points, c, r, size).min()) < HAND_GAP - EPS:
            return False
    return True


def _start_ok(o) -> str:
    lot = lots.current()
    for spec in (SENSOR, CASE):
        err = float(np.linalg.norm(o[spec.name].pos[:2] - np.asarray(lot.pos(spec.name))[:2]))
        if err > START_TOL + EPS:
            return f"{spec.name} starts {err * 1000:.1f} mm from lot {lot.name}"
    r = _R(o[SENSOR.name].quat_xyzw)
    if math.degrees(abs(math.atan2(r[1, 0], r[0, 0]))) > START_DEG:
        return f"extensometer starts turned in lot {lot.name}"
    return ""


def show_cable(s) -> None:
    """The sensor lead (3 mm, drawn only: no collision or pull), from the housing's back to the machine."""
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, UsdGeom, Vt

    p = np.asarray(s.pos) + _R(s.quat_xyzw) @ np.array((-HOUSING[0] / 2, 0, 0))
    pts = [tuple(p), tuple(p + (-.030, .010, -.012)), (.080, .260, TOP + .008), (.150, .330, TOP + .008),
           (.250, .300, TOP + .008), (.330, .250, TOP + .030), (.390, .270, TOP + .080), (.480, .250, TOP + .110)]
    curve = UsdGeom.BasisCurves.Define(get_current_stage(), "/World/ExtensometerLead")
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr([len(pts)])
    curve.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, v)) for v in pts]))
    curve.CreateWidthsAttr([.003])
    curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curve.CreateDisplayColorAttr([Gf.Vec3f(.07, .07, .07)])


@parts.on_reset
def _on_reset(sim) -> None:
    reset_record()
    _SIM.pop("rows", None)


@parts.on_step
def _on_step(sim) -> None:
    row = {}
    try:
        base = sim.env.unwrapped
        o = sim.states()
        t = float(sim.t)
        s = o[SENSOR.name]
        if not REC["checked"]:
            REC["checked"] = True
            # the physics record from the lot's settle steps is discarded: the trial starts here
            REC.update(drag=np.zeros(2), track=[None, None], peak={k: 0.0 for k in REC["peak"]})
            why = _start_ok(o)
            print(f"[extensometer] trial start lot {lots.current().name}: sensor {np.round(s.pos, 4).tolist()} "
                  f"case {np.round(o[CASE.name].pos, 4).tolist()}", file=sys.stderr, flush=True)
            if why:
                _zero("start: " + why)
        if REC["physics"] == 0:
            raise RuntimeError("no physics substeps observed")
        pts = hand_points(base)
        clear = hands_clear(pts, s)
        touching = REC["step_peak"]["hand"] > 0.0
        sp = dict(REC["step_peak"])
        lv = control_update(o, t, clear, touching)
        try:
            show_cable(s)
        except Exception:  # noqa: BLE001  the drawn lead never affects a trial
            pass
        b = blade_points(s.pos, s.quat_xyzw)
        row = dict(t=t, lot=lots.current().name, pos=s.pos.tolist(), quat=s.quat_xyzw.tolist(),
                   tilt=tilt.tilt_of(s.quat_xyzw), blades=b.tolist(), gaps=specimen_distance(b).tolist(),
                   hooks=hook_points(s.pos, s.quat_xyzw).tolist(), hooked=hooked(s.pos, s.quat_xyzw),
                   force=sp, drag=REC["drag"].tolist(), fit=None if REC["fit"] is None else REC["fit"]["quality"],
                   clear=clear, touching=touching, level=lv, score=score(o), zero=REC["zero"], why=REC["why"],
                   vel=s.lin_vel.tolist(), low=s.low.tolist())
    except Exception as exc:  # noqa: BLE001  fail closed: an unreadable trial scores 0
        if not REC["zero"]:
            import traceback

            traceback.print_exc()
        _zero(f"scorer error {type(exc).__name__}: {exc}")
        row = {"t": float(getattr(sim, "t", 0.0)), "error": str(exc)}
    if os.environ.get("TRACE_OUT"):
        try:
            from yam_arena.constants import TCP_OFFSET

            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
                row[arm] = (pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)).tolist()
                row[arm + "_rot"] = parts._quat_to_matrix(pose[3:]).tolist()
                row[arm + "_fingers"] = parts._np(rob.data.joint_pos)[0, -2:].tolist()
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("extensometer.jsonl").open("a") as f:
                f.write(json.dumps(row, default=float) + "\n")
        except Exception as exc:  # noqa: BLE001  logging never affects the trial
            if not _SIM.get("logerr"):
                _SIM["logerr"] = True
                print(f"[extensometer] trace logging skipped: {exc}", file=sys.stderr, flush=True)


# --- scripted demo: open loop from the visible layout -----------------------------------------------------------------
GRIP_X = .003  # m in front of the housing centre: over the extensometer's centre of mass
GRIP_Z = .003  # TCP above the arms' plane (the pads straddle the housing's mid-height)
GRIP_FINGER = -(HOUSING[1] / 2 - .002)  # pads pressed 2 mm into the 52 mm housing: about 10 N each
CARRY_DZ = .050  # carried with the arms 50 mm above the specimen's centre line
STANDOFF = .0015  # knife edges this far in front of the face while the clips go over the edge
PRELOAD = .003  # the hand is commanded this far past first touch (the arm gives about 0.2 N per mm)
FIT_FIX = (0.0, -.0008, .0018)  # measured correction for the loaded arm's sag (m): it dips 1 mm as the push starts
FIT_YAW = math.radians(-0.85)  # measured: the loaded hand turns the knife edges 0.85 deg about the vertical


def demo():
    lot = lots.current()
    p = np.asarray(lot.pos(SENSOR.name), float)
    grip = p + np.array((GRIP_X, 0.0, GRIP_Z))
    fix = np.asarray(FIT_FIX)

    def at(dx, dz):
        """TCP target with the knife-edge line ``dx`` past the front face and the arms ``dz`` above the axis."""
        return np.array((XF - L_TIP + dx + GRIP_X, Y, AXIS + dz + GRIP_Z)) + fix

    def wp(t, pos, f, label, yaw=0.0):
        return Waypoint(float(t), tuple(float(v) for v in pos), float(f), label, float(yaw))

    up = np.array((0.0, 0.0, 1.0))
    L = [wp(2.0, grip + up * .080, FINGER_OPEN, "over the housing"),
         wp(1.5, grip, FINGER_OPEN, "down beside the housing"),
         wp(1.0, grip, GRIP_FINGER, "grip the housing"),
         wp(2.0, (grip[0], grip[1], AXIS + CARRY_DZ + GRIP_Z), GRIP_FINGER, "lift from the case"),
         wp(3.0, at(-STANDOFF, CARRY_DZ), GRIP_FINGER, "clips above the specimen's upper edge", FIT_YAW),
         wp(3.0, at(-STANDOFF, 0.0), GRIP_FINGER, "lower the clips over the edge, knife edges clear of the face",
            FIT_YAW),
         wp(1.0, at(-STANDOFF, 0.0), GRIP_FINGER, "settle", FIT_YAW),
         wp(2.0, at(PRELOAD, 0.0), GRIP_FINGER, "ease the knife edges onto the face", FIT_YAW),
         wp(1.0, at(PRELOAD, 0.0), GRIP_FINGER, "hold while the clips take it", FIT_YAW),
         wp(1.0, at(PRELOAD, 0.0), FINGER_OPEN, "let go of the housing", FIT_YAW),
         wp(2.0, at(PRELOAD, 0.0) + up * .080, FINGER_OPEN, "hand straight up", FIT_YAW),
         wp(2.0, (.200, .200, TOP + .200), FINGER_OPEN, "hand clear"),
         wp(4.0, (.200, .200, TOP + .200), FINGER_OPEN, "hands clear")]
    R = [Waypoint(sum(w.duration for w in L), (.140, -.220, TOP + .120), FINGER_OPEN, "clear of the machine")]
    return {"left": L, "right": R}
