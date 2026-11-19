"""On the station table, to the robot's right: a handheld game console stands upright in its black TV dock, its
screen toward the robot. A blue controller is attached to the console's left end; its right end is bare, showing a
lipped rail channel that runs down the end from the top. A red controller lies loose on the table nearer the robot,
on its side.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s hold is enforced inside score()
EPS = 1e-6
MM = 1e-3

BODY, SCREEN = (0.12, 0.12, 0.13), (0.03, 0.05, 0.10)
JOY_L, JOY_R, STICK = (0.05, 0.62, 0.86), (0.92, 0.27, 0.22), (0.15, 0.15, 0.16)
DOCK, PLUG, RAIL = (0.10, 0.10, 0.11), (0.80, 0.81, 0.83), (0.55, 0.56, 0.58)

# --- the console: Nintendo Switch (HAC-001) body 173 x 102 x 13.9 mm, 297 g, with the left Joy-Con (HAC-015, 49 g)
# attached. Frame: origin at the body's centre; x its thickness (the screen faces -x), y its width (+y is the left
# end as the player holds it), z its height (the port edge at z = -51 mm). -------------------------------------------
CT, CH, TAB_W, JW = 13.9 * MM, 102.0 * MM, 173.0 * MM, 33.0 * MM  # JW: a Joy-Con's width outside the body
HX, HZ, TY = CT / 2, CH / 2, TAB_W / 2
CON_MASS = 0.297 + 0.049
PORT_W, PORT_T, PORT_D, PORT_R = 10.0 * MM, 3.6 * MM, 7.0 * MM, 1.0 * MM  # USB-C port opening (estimate)
GLASS_W, GLASS_H = 137.0 * MM, 77.2 * MM  # 6.2 in screen glass, centred on the front face

# --- the rail (estimates). The console's right end carries a lipped channel open at the top and closed 3 mm above
# the bottom; the Joy-Con's slider is a T: a stem through the lips and a head behind them. --------------------------
PLAY = 0.35 * MM  # clearance each side, head and stem
STEM_W, STEM_D = 4.0 * MM, 1.2 * MM  # stem: across the thickness (x), depth (y)
HEAD_X0, HEAD_X1, HEAD_D = -4.0 * MM, 4.0 * MM, 1.7 * MM  # head: 8 mm across, 1.7 mm deep
SLIDE_D = STEM_D + HEAD_D  # 2.9 mm: the slider's depth into the console (35.9 mm Joy-Con = 33 + 2.9)
STOP_H = 3.0 * MM  # the channel's closed bottom
EDGE_R = 1.0 * MM  # the channel mouth's top edges, rounded
FLOOR_R = 3.0 * MM  # the console's top corner over the channel floor, rounded
# The sprung catch: a 3 mm round nub on the Joy-Con's rail face, 6 mm below its top, springs into a 5 mm notch in
# the console's lip once the Joy-Con is fully down. Between the notch and the console's top the lip is a 3.5 mm bar
# that pushes the nub in on the way down.
NUB_R, NUB_OUT, NUB_X, NUB_Z = 1.5 * MM, 0.8 * MM, -5.5 * MM, 45.0 * MM
NOTCH_LO, NOTCH_HI = 42.5 * MM, 47.5 * MM  # console frame z
LATCH_MASS, LATCH_K, LATCH_PRELOAD, LATCH_C = 0.010, 2500.0, 0.5 * MM, 3.0
LATCH_TRAVEL = 1.0 * MM

# --- the loose right Joy-Con (HAC-016): 102 x 35.9 x 13.9 mm body, 52 g. Frame J (as seated): origin at the body's
# centre, axes as the console's (x: its stick faces -x; +y toward its rail face; z: its top end up). ----------------
JHY, JHZ = JW / 2, CH / 2
JOY_MASS = 0.052
STICK_Z = -11.0 * MM  # the stick's centre, 62 mm below the top end
LIP_IN = STEM_D - PLAY  # 0.85 mm: the lips' depth
YB = -TY + SLIDE_D + PLAY  # the channel floor (console frame y)

# --- the dock: Nintendo Switch dock (HAC-007), 173 x 104 x 54 mm, 327 g. Frame: origin at its footprint's middle;
# x its depth (front at -x), y its length, z up. ---------------------------------------------------------------------
DL, DD, DH = 173.0 * MM, 54.0 * MM, 104.0 * MM
DOCK_MASS = 0.327
FLOOR = 45.0 * MM  # the slot floor above the table (estimate)
SLOT_W = 16.0 * MM  # the slot, front to back (estimate)
FRONT_TOP = 75.0 * MM  # the front wall's top (estimate); the back wall is the dock's full 104 mm
MOUTH_R = 4.0 * MM
PLUG_T, PLUG_W, PLUG_H = 2.4 * MM, 8.25 * MM, 6.5 * MM  # USB Type-C plug


def _box(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2), color=color)


def _cyl(r, h, at, color, axis):
    return parts.cylinder(float(r), float(h), at=tuple(float(v) for v in at), color=color, axis=axis)


def _console_shapes():
    s = []
    zb, zp, r = -HZ, -HZ + PORT_D, PORT_R
    # the body from the channel floor to its left end, around the USB-C port in the middle of the lower edge
    s.append(_box((-HX, YB + FLOOR_R, zp), (HX, TY, HZ), BODY))
    s.append(_box((-HX, YB, zp), (HX, YB + FLOOR_R, HZ - FLOOR_R), BODY))
    s.append(_cyl(FLOOR_R, CT, (0.0, YB + FLOOR_R, HZ - FLOOR_R), BODY, "x"))
    s.append(_box((-HX, TY, zb), (HX, TY + JW, HZ), JOY_L))  # the attached left Joy-Con
    for sign in (1, -1):
        y0, y1 = sign * (PORT_W / 2 + r), (TY if sign > 0 else YB)
        s.append(_box((-HX, min(y0, y1), zb), (HX, max(y0, y1), zp), BODY))
        y0, y1 = sign * PORT_W / 2, sign * (PORT_W / 2 + r)
        s.append(_box((-HX, min(y0, y1), zb + r), (HX, max(y0, y1), zp), BODY))
        s.append(_cyl(r, CT, (0.0, sign * (PORT_W / 2 + r), zb + r), BODY, "x"))
    for sign in (1, -1):
        x0, x1 = sign * (PORT_T / 2 + r), sign * HX
        s.append(_box((min(x0, x1), -PORT_W / 2, zb), (max(x0, x1), PORT_W / 2, zp), BODY))
        x0, x1 = sign * PORT_T / 2, sign * (PORT_T / 2 + r)
        s.append(_box((min(x0, x1), -PORT_W / 2, zb + r), (max(x0, x1), PORT_W / 2, zp), BODY))
        s.append(_cyl(r, PORT_W, (sign * (PORT_T / 2 + r), 0.0, zb + r), BODY, "y"))
    # the channel's closed bottom
    z_stop = -HZ + STOP_H
    s.append(_box((-HX, -TY, -HZ), (HX, YB, z_stop), RAIL))
    # cavity walls (behind the lips, either side of the head) and lips (either side of the stem); each inner top edge
    # rounded. The lip on the screen side (-x) has the catch's notch, with the 3.5 mm bar above it.
    y_lip, y_cav = (-TY, -TY + LIP_IN), (-TY + LIP_IN, YB)
    walls = ((y_cav, -HX, HEAD_X0 - PLAY), (y_cav, HEAD_X1 + PLAY, HX),
             (y_lip, -HX, -STEM_W / 2 - PLAY), (y_lip, STEM_W / 2 + PLAY, HX))
    for k, ((ya, yb), xa, xb) in enumerate(walls):
        inner = xb if xa < 0 else xa  # the edge beside the slot
        sgn = -1.0 if xa < 0 else 1.0
        xo = inner + sgn * EDGE_R  # the round's centre, toward the outside
        lo_x, hi_x = (xa, xo) if xa < 0 else (xo, xb)
        spans = [(z_stop, HZ)]
        if k == 2:  # the -x lip: notch
            spans = [(z_stop, NOTCH_LO), (NOTCH_HI, HZ)]
        for za, zb2 in spans:
            s.append(_box((lo_x, ya, za), (hi_x, yb, zb2), RAIL))
            top_inner = zb2 - EDGE_R if zb2 == HZ else zb2
            ia, ib = (xo, inner) if xa < 0 else (inner, xo)
            s.append(_box((min(ia, ib), ya, za), (max(ia, ib), yb, top_inner), RAIL))
            if zb2 == HZ:
                s.append(_cyl(EDGE_R, yb - ya, (xo, (ya + yb) / 2, HZ - EDGE_R), RAIL, "y"))
    # the screen glass, 0.1 mm proud of the front face
    s.append(_box((-HX - 0.1 * MM, -GLASS_W / 2, -GLASS_H / 2), (-HX + 0.2 * MM, GLASS_W / 2, GLASS_H / 2), SCREEN))
    return s


def _joycon_shapes_j():
    """The Joy-Con's shapes in frame J (seated orientation): body, slider (stem and keyed head), stick."""
    s = []
    r = FLOOR_R
    # body, its lower inner edge rounded (it lands on the console's top edge if it comes in too far toward it)
    s.append(_box((-HX, -JHY, -JHZ), (HX, JHY - r, JHZ), JOY_R))
    s.append(_box((-HX, JHY - r, -JHZ + r), (HX, JHY, JHZ), JOY_R))
    s.append(_cyl(r, CT, (0.0, JHY - r, -JHZ + r), JOY_R, "x"))
    # slider: stem and head, their lower ends rounded; both end STOP_H above the Joy-Con's bottom
    z0 = -JHZ + STOP_H
    sr = STEM_W / 2
    s.append(_box((-sr, JHY, z0 + sr), (sr, JHY + STEM_D, JHZ), RAIL))
    s.append(_cyl(sr, STEM_D, (0.0, JHY + STEM_D / 2, z0 + sr), RAIL, "y"))
    hr, hc = (HEAD_X1 - HEAD_X0) / 2, (HEAD_X0 + HEAD_X1) / 2
    s.append(_box((HEAD_X0, JHY + STEM_D, z0 + hr), (HEAD_X1, JHY + SLIDE_D, JHZ), RAIL))
    s.append(_cyl(hr, HEAD_D, (hc, JHY + STEM_D + HEAD_D / 2, z0 + hr), RAIL, "y"))
    # the stick on the front face: a 4 mm shaft and a 16 mm cap, 8 mm proud in all
    s.append(_cyl(4.0 * MM, 5.0 * MM, (-HX - 2.5 * MM, 0.0, STICK_Z), STICK, "x"))
    s.append(_cyl(8.0 * MM, 3.0 * MM, (-HX - 6.5 * MM, 0.0, STICK_Z), STICK, "x"))
    return s


def _latch_shapes_j():
    """The catch (frame J): a hidden 10 g carrier inside the body and the round nub on the rail face."""
    zc = NUB_Z
    return [_box((-6.0 * MM, 8.0 * MM, zc - 5.0 * MM), (-2.5 * MM, JHY - 2.0 * MM, zc + 5.0 * MM), JOY_R),
            _cyl(NUB_R, 2.0 * MM, (NUB_X, JHY + NUB_OUT - NUB_R, zc), RAIL, "x")]


# The Joy-Con lies on its rail face. Part frame L (its lying pose, lot A): J's x -> +y, J's y -> -z, J's z -> -x.
LIE = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # columns: J's x, y, z in L


def _turn(shape, m=LIE):
    at = m @ np.asarray(shape.at, float)
    if shape.kind == "box":
        size = np.abs(m) @ np.asarray(shape.size, float)
        return parts.box(tuple(float(v) for v in size), at=tuple(float(v) for v in at), color=shape.color)
    ax = "xyz"[int(np.argmax(np.abs(m[:, "xyz".index(shape.axis)])))]
    return parts.cylinder(shape.size[0], shape.size[1], at=tuple(float(v) for v in at), color=shape.color, axis=ax)


def _dock_shapes():
    s = []
    hx, hy = DD / 2, DL / 2
    w, r = SLOT_W / 2, MOUTH_R
    s.append(_box((-hx, -hy, 0.0), (hx, hy, FLOOR), DOCK))
    for sign, top in ((-1, FRONT_TOP), (1, DH)):
        a, b = sign * w, sign * hx
        s.append(_box((min(a, b), -hy, FLOOR), (max(a, b), hy, top - r), DOCK))
        a = sign * (w + r)
        s.append(_box((min(a, b), -hy, top - r), (max(a, b), hy, top), DOCK))
        s.append(_cyl(r, DL, (sign * (w + r), 0.0, top - r), DOCK, "y"))
    t = PLUG_T / 2
    tip = FLOOR + PLUG_H - t
    s.append(_box((-t, -PLUG_W / 2, FLOOR - 0.5 * MM), (t, PLUG_W / 2, tip), PLUG))
    s.append(_cyl(t, PLUG_W - 2 * t, (0.0, 0.0, tip), PLUG, "y"))
    for sign in (1, -1):
        s.append(_cyl(t, PLUG_T, (0.0, sign * (PLUG_W / 2 - t), tip), PLUG, "x"))
    return s


# --- layout -----------------------------------------------------------------------------------------------------------
DOCK_XY = (0.310, -0.120)
CON_START = (DOCK_XY[0], DOCK_XY[1], TOP + FLOOR + HZ + 0.2 * MM)  # seated on the plug, the console's port on it
JOY_LIE_Z = TOP + SLIDE_D + JHY + 0.3 * MM  # J's origin when the Joy-Con lies on its slider
JOY_START = {"A": (0.195, -0.165, JOY_LIE_Z), "B": (0.175, -0.255, JOY_LIE_Z)}
JOY_YAW = {"A": 0.0, "B": -math.pi / 2}  # lot A: top end toward the robot; lot B: top end to the robot's left

CONSOLE = parts.Part("console", pos=CON_START, shapes=tuple(_console_shapes()), mass=CON_MASS, friction=(0.5, 0.4))
DOCK_PART = parts.Part("dock", pos=(*DOCK_XY, TOP + 0.2 * MM), shapes=tuple(_dock_shapes()), mass=DOCK_MASS,
                       friction=(0.7, 0.6))
LATCH = parts.Slider("latch", shapes=tuple(_turn(s) for s in _latch_shapes_j()),
                     axis=tuple(float(v) for v in LIE @ np.array([0.0, -1.0, 0.0])), travel=LATCH_TRAVEL,
                     mass=LATCH_MASS, spring=LATCH_K, damping=LATCH_C, max_effort=20.0)
JOYCON = parts.Part("joycon", pos=JOY_START["A"], shapes=tuple(_turn(s) for s in _joycon_shapes_j()),
                    mass=JOY_MASS - LATCH_MASS, friction=(0.5, 0.4), joints=(LATCH,))

OBJECTS = [DOCK_PART, CONSOLE, JOYCON]
LOTS = lots.declare(
    OBJECTS,
    *[lots.Lot(k, place={"joycon": JOY_START[k]}, yaw={"joycon": JOY_YAW[k]}) for k in ("A", "B")],
)

# --- scoring geometry ------------------------------------------------------------------------------------------------
ON_RAIL = 0.6 * MM  # the slider's head between the channel's walls: J within 0.6 mm of the seated line, x and y
SEAT = 1.0 * MM  # seated: within 1 mm of the bottom stop
ENGAGED = 30.0 * MM  # on the rail: at least 30 mm of slider inside the channel
SEATED_FROM_TOP = CH - STOP_H  # 99 mm of slider is inside the channel when seated
LATCHED = 0.25 * MM  # the nub out in its notch: pushed in no more than 0.25 mm
PARALLEL, PARALLEL_RAIL = 2.0, 3.0  # degrees between the Joy-Con's frame and the console's
CON_SEAT, CON_UPRIGHT = 3.0 * MM, 3.0  # the console's lower edge within 3 mm of the slot floor, upright within 3 deg
LIFTED, CON_TIP = 5.0 * MM, 10.0  # never: the console's lower edge more than 5 mm above the floor or tipped 10 deg
DROP_SPEED = 0.6  # never: the Joy-Con moving down faster than 0.6 m/s (a free fall of about 18 mm)
STILL_STEPS = 20  # 1 s at 20 Hz
STILL_POS, STILL_DEG = 0.2 * MM, 0.2
CLEAR = 5.0 * MM  # let go: no arm surface within 5 mm of the Joy-Con, console or dock
TOUCH = 1.0 * MM  # never: a gripper or fingertip within 1 mm of the screen glass


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _angle(a, b) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(a.T @ b)) - 1.0) / 2.0))))


def joy_frame(joy):
    """(origin, rotation) of the Joy-Con's frame J in the world."""
    return np.asarray(joy.pos, float), _rot(joy.quat_xyzw) @ LIE


def fit(con, joy) -> dict:
    """The Joy-Con relative to its seated place on the console, in the console's frame (m, degrees)."""
    rc, pc = _rot(con.quat_xyzw), np.asarray(con.pos, float)
    pj, rj = joy_frame(joy)
    d = rc.T @ (pj - pc) - np.array([0.0, -TY - JHY, 0.0])
    up = rc[:, 2]
    return {"x": float(d[0]), "y": float(d[1]), "z": float(d[2]), "angle": _angle(rc, rj),
            "latch": float(joy.joints.get("latch", math.inf)),
            "con_tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))}


def console_in_dock(con, dock) -> dict:
    rd = _rot(dock.quat_xyzw)
    rc = _rot(con.quat_xyzw)
    low = rd.T @ (np.asarray(con.pos, float) + rc @ np.array([0.0, 0.0, -HZ]) - np.asarray(dock.pos, float))
    up = rd.T @ rc[:, 2]
    return {"x": float(low[0]), "y": float(low[1]), "z": float(low[2]),
            "tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))}


def seated(f) -> bool:
    return bool(abs(f["x"]) <= ON_RAIL + EPS and abs(f["y"]) <= ON_RAIL + EPS and abs(f["z"]) <= SEAT + EPS
                and f["latch"] <= LATCHED + EPS and f["angle"] <= PARALLEL + EPS)


def on_rail(f) -> bool:
    return bool(abs(f["x"]) <= ON_RAIL + EPS and abs(f["y"]) <= ON_RAIL + EPS
                and -SEAT - EPS <= f["z"] <= SEATED_FROM_TOP - ENGAGED + EPS and f["angle"] <= PARALLEL_RAIL + EPS)


def docked(c) -> bool:
    return bool(abs(c["z"] - FLOOR) <= CON_SEAT + EPS and c["tilt"] <= CON_UPRIGHT + EPS
                and abs(c["x"]) <= SLOT_W / 2 + EPS and abs(c["y"]) <= 5.0 * MM + EPS)


# --- the whole-trial record (kept in the parts module so near_misses.py, which imports this file separately, shares
# it), cleared only by a scene reset ----------------------------------------------------------------------------------
_grip = parts.__dict__.setdefault("_joycon_task_grip", {"clear": None, "touch": None})
_rec = parts.__dict__.setdefault("_joycon_task_record", {})


def _clear_record():
    _rec.clear()
    _rec.update(screen=False, lifted=False, dropped=False, unknown=False, prev=None, still=0, last=None)


_clear_record()


def _update(objects) -> None:
    """Once per control step (success() and score() get the same states; the first call updates)."""
    if _rec.get("last") is objects:
        return
    _rec["last"] = objects
    con, dock, joy = objects["console"], objects["dock"], objects["joycon"]
    pc, rc = np.asarray(con.pos, float), _rot(con.quat_xyzw)
    pj, rj = np.asarray(joy.pos, float), _rot(joy.quat_xyzw)
    if not (np.isfinite(pc).all() and np.isfinite(pj).all() and np.isfinite(np.asarray(dock.pos, float)).all()):
        _rec["unknown"] = True
        return
    if "offline_touch" in _grip:  # near_misses.py: no robot offline, so it states the touches
        _grip["touch"] = _grip["offline_touch"](objects)
    touch = _grip["touch"]
    if touch is None:
        _rec["unknown"] = True
    else:
        _rec["screen"] |= bool(touch.get("screen"))
    c = console_in_dock(con, dock)
    if c["z"] - FLOOR > LIFTED + EPS or c["tilt"] > CON_TIP + EPS:
        _rec["lifted"] = True
    if -float(joy.lin_vel[2]) > DROP_SPEED + EPS:
        _rec["dropped"] = True
    prev = _rec["prev"]
    if prev is not None:
        same = (np.linalg.norm(pc - prev[0]) <= STILL_POS + EPS and np.linalg.norm(pj - prev[2]) <= STILL_POS + EPS
                and _angle(rc, prev[1]) <= STILL_DEG + EPS and _angle(rj, prev[3]) <= STILL_DEG + EPS)
        _rec["still"] = _rec["still"] + 1 if same else 0
    _rec["prev"] = (pc, rc, pj, rj)


def _clear(objects):
    """The arms' measured clearance (None until measured: no credit). near_misses.py stands in for it offline."""
    if "offline_clear" in _grip:
        _grip["clear"] = _grip["offline_clear"](objects)
    return _grip["clear"]


def _value(objects) -> float:
    if _rec["unknown"] or _rec["screen"] or _rec["lifted"] or _rec["dropped"]:
        return 0.0
    if _grip["clear"] is not True or _rec["still"] < STILL_STEPS:
        return 0.0
    con, dock, joy = objects["console"], objects["dock"], objects["joycon"]
    if not docked(console_in_dock(con, dock)):
        return 0.0
    f = fit(con, joy)
    if seated(f):
        return 1.0
    if on_rail(f):
        return 0.5
    return 0.0


def success(objects):
    """The Joy-Con slid fully down the console's rail with its catch in the notch, let go and still for 1 s."""
    _clear(objects)
    _update(objects)
    return bool(_value(objects) == 1.0)


def distance(objects):
    f = fit(objects["console"], objects["joycon"])
    return float(abs(f["x"]) + abs(f["y"]) + abs(f["z"]))


def score(objects):
    _clear(objects)
    _update(objects)
    v = _value(objects)
    if os.environ.get("TRACE_OUT"):  # optional per-step log for debugging runs; no effect on the score
        _log_step(objects, v)
    return v


def _log_step(objects, v) -> None:
    import json

    f = fit(objects["console"], objects["joycon"])
    c = console_in_dock(objects["console"], objects["dock"])
    row = {"score": v, "lot": lots.current().name, "clear": _grip.get("clear"), "still": _rec["still"],
           "dist_mm": [round(d * 1000, 2) for d in _grip.get("dist", ())],
           "vz": round(float(objects["joycon"].lin_vel[2]), 3),
           **{k: _rec[k] for k in ("screen", "lifted", "dropped", "unknown")},
           **{k: round(val * 1000, 2) if k in ("x", "y", "z", "latch") else round(val, 2) for k, val in f.items()},
           "con_z": round((c["z"] - FLOOR) * 1000, 2), "con_tilt": round(c["tilt"], 2)}
    with pathlib.Path(os.environ["TRACE_OUT"]).with_name("joycon.jsonl").open("a") as fh:
        fh.write(json.dumps(row) + "\n")


# --- simulator: the arms' clearance and touches, from face-sampled link hulls ----------------------------------------
FACE_STEP = 3.0 * MM
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")
HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _face_points(vertices, planes, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull about `step` apart, corners included, so a flat face lying on a part is
    caught even when all of its corners are far away."""
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


def envelope(spec, state) -> list:
    """The box around every shape of a part, as one (centre, rotation, size): never closer to a point than the part."""
    lo, hi = spec._local_box()
    r = _rot(state.quat_xyzw)
    return [(np.asarray(state.pos) + r @ ((lo + hi) / 2), r, hi - lo)]


def glass_box(con) -> tuple:
    r = _rot(con.quat_xyzw)
    c = np.asarray(con.pos) + r @ np.array([-HX + 0.05 * MM, 0.0, 0.0])
    return c, r, np.array([0.3 * MM, GLASS_W, GLASS_H])


def _min_distance(points, boxes) -> float:
    """The smallest distance from any of the points to any of the boxes (m); inf if none is near."""
    from yam_arena import analysis

    if not len(points):
        return math.inf
    pts = np.concatenate(points)
    best = math.inf
    for c, r, s in boxes:
        reach = float(np.linalg.norm(s)) / 2 + CLEAR + 2 * MM
        near = pts[np.einsum("ij,ij->i", pts - c, pts - c) <= reach * reach]
        if len(near):
            best = min(best, float(analysis.box_signed_distance(near, c, r, s).min()))
    return best


@parts.on_step
def _measure_arms(sim) -> None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    st = sim.states()
    con, dock, joy = st["console"], st["dock"], st["joycon"]
    scored = envelope(CONSOLE, con) + envelope(DOCK_PART, dock) + envelope(JOYCON, joy)
    lo = np.min([c - np.linalg.norm(sz) / 2 for c, _, sz in scored], axis=0) - CLEAR - 2 * MM
    hi = np.max([c + np.linalg.norm(sz) / 2 for c, _, sz in scored], axis=0) + CLEAR + 2 * MM
    arm_pts, hand_pts, found = [], [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n not in ARM_LINKS:
                continue
            hulls = _link_points(n)
            found += 1 if hulls else 0
            rot, pos = _rot(poses[i, 3:]), poses[i, :3] - origin
            for c, rad, p in hulls:
                cw = rot @ c + pos
                if np.any(cw + rad < lo) or np.any(cw - rad > hi):
                    continue
                w = p @ rot.T + pos
                arm_pts.append(w)
                if n in HAND_LINKS:
                    hand_pts.append(w)
    if found != 2 * len(ARM_LINKS) or not all(np.isfinite(p).all() for p in arm_pts):
        _grip["clear"], _grip["touch"] = None, None  # fail closed: unknown arms, no credit
        return
    d_arm = _min_distance(arm_pts, scored)
    d_glass = _min_distance(hand_pts, [glass_box(con)])
    _grip["clear"] = d_arm > CLEAR + EPS
    _grip["touch"] = {"screen": d_glass <= TOUCH + EPS}
    _grip["dist"] = (d_arm, d_glass)


@parts.on_reset
def _reset(sim) -> None:
    _grip["clear"], _grip["touch"] = None, None
    _clear_record()
    sim.set_target("joycon", "latch", -LATCH_PRELOAD)  # the catch's spring is preloaded


# --- demo: open loop from the published layout (the Joy-Con's place and turn in each lot are visible in the top
# camera); one tilted grip with the right arm ---------------------------------------------------------------------------
YAW = math.pi / 2  # the stock grip turned so the fingers close along x: across the Joy-Con's 13.9 mm thickness
OPEN, SQUEEZE = -0.020, -0.0045  # pads 40 mm apart; commanded 9 mm apart on the 13.9 mm body
GRIP_J = np.array([0.0, -JHY + 12.0 * MM, JHZ - 30.0 * MM])  # the TCP in frame J: 12 mm in from the outer edge,
# 30 mm below the top end, between the stick and the top
FINAL_ROT = tilt.rotation(YAW, 45.0, math.pi / 2)  # leaning 45 degrees toward the console (+y)
INWARD = 0.5 * MM  # aim this far toward the console, inside the 0.35 mm play plus the rounded edges
ENTRY = 104.0 * MM  # the Joy-Con's height above seated before the slide: its slider's end 5 mm above the console
PRESS = 8.0 * MM  # commanded below seated, so the soft arm pushes the catch home however the Joy-Con sits in
# the pads (it settled 2 mm low to 6 mm high in trace 1); the pads slip on the Joy-Con once it is on the stop
TIP_BELOW = 13.0 * MM  # the closed fingertips reach this far below the TCP
TOP_PRESS = 6.0 * MM  # then the closed fingertips press the top end this far below seated
DOWN = tilt.rotation(YAW, 0.0, 0.0)
CARRY_FIX = (0.0008, 0.0)  # measured carrying error of the loaded right arm (x, y), applied at the console:
# trace 1 held the Joy-Con 0.8 mm toward the screen side of its line in both lots


def _held(duration, pos, finger, label, rot) -> tilt.Waypoint:
    w = tilt.Waypoint(duration, tuple(float(v) for v in pos), finger, label, YAW, tilt=1.0)
    w.yaw = tilt._TiltedYaw(YAW, rot)
    return w


def demo():
    """The right arm takes the Joy-Con by its outer edge near the top with a tilted grip, lifts it and turns it
    upright with its rail toward the console, lowers its slider into the top of the console's channel and pushes it
    down, lets go, then presses the Joy-Con's top end down with closed fingertips until the catch clicks and backs
    away. The left arm stays parked."""
    lot = lots.current()
    p0 = np.asarray(lot.pos("joycon"), float)
    yaw = lot.yaw_of("joycon")
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    start_j = rz @ LIE  # frame J of the lying Joy-Con
    turn = start_j  # world <- J at the start; FINAL is J aligned with the world
    g0 = p0 + turn @ GRIP_J
    rot0 = turn @ FINAL_ROT
    approach0 = rot0[:, 2]  # the gripper's approach axis (its z)
    lift_rot = turn @ FINAL_ROT
    seat_j = np.array([CON_START[0] + CARRY_FIX[0], CON_START[1] - TY - JHY + INWARD + CARRY_FIX[1], CON_START[2]])
    g_seat = seat_j + GRIP_J
    g_entry = g_seat + np.array([0.0, 0.0, ENTRY])
    back = -FINAL_ROT[:, 2]  # straight back along the approach
    lifted = g0 + np.array([0.0, 0.0, 0.070])
    top = np.array([seat_j[0], seat_j[1] - INWARD, seat_j[2] + JHZ])  # the seated Joy-Con's top end, mid-width
    w = [
        _held(2.0, g0 - approach0 * 0.040, OPEN, "over the Joy-Con's outer edge", rot0),
        _held(1.5, g0, OPEN, "pads round the edge", rot0),
        _held(0.8, g0, SQUEEZE, "grip", rot0),
        _held(0.5, g0, SQUEEZE, "settle", rot0),
        _held(1.5, lifted, SQUEEZE, "lift", lift_rot),
        _held(3.0, np.array([0.250, -0.250, TOP + 0.150]), SQUEEZE, "turn upright, rail to the console", FINAL_ROT),
        _held(2.0, g_entry + np.array([-0.020, -0.010, 0.0]), SQUEEZE, "up beside the console", FINAL_ROT),
        _held(1.5, g_entry, SQUEEZE, "over the channel", FINAL_ROT),
        _held(1.0, g_entry, SQUEEZE, "steady", FINAL_ROT),
        _held(5.0, g_seat + np.array([0.0, 0.0, 0.010]), SQUEEZE, "slide down the rail", FINAL_ROT),
        _held(2.0, g_seat - np.array([0.0, 0.0, PRESS]), SQUEEZE, "push the catch home", FINAL_ROT),
        _held(1.0, g_seat - np.array([0.0, 0.0, PRESS]), SQUEEZE, "hold", FINAL_ROT),
        _held(0.8, g_seat - np.array([0.0, 0.0, PRESS]), OPEN, "let go", FINAL_ROT),
        _held(1.2, g_seat + back * 0.040, OPEN, "back off", FINAL_ROT),
        _held(1.5, top + np.array([0.0, 0.0, TIP_BELOW + 0.030]), FINGER_CLOSED, "over its top end", DOWN),
        _held(1.5, top + np.array([0.0, 0.0, TIP_BELOW - TOP_PRESS]), FINGER_CLOSED, "press it down to the click",
              DOWN),
        _held(1.0, top + np.array([0.0, 0.0, TIP_BELOW - TOP_PRESS]), FINGER_CLOSED, "hold", DOWN),
        _held(1.0, top + np.array([0.0, 0.0, TIP_BELOW + 0.040]), FINGER_CLOSED, "up", DOWN),
        _held(1.5, top + np.array([-0.060, 0.0, TIP_BELOW + 0.060]), FINGER_CLOSED, "away", DOWN),
        _held(2.5, top + np.array([-0.060, 0.0, TIP_BELOW + 0.060]), FINGER_CLOSED, "wait clear", DOWN),
    ]
    return {"right": w, "left": []}


DEMO_TOUCHES = ()
