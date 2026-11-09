"""On the station table, a little to the robot's right of centre: the grey body of a bagless vacuum cleaner lying on
its side, with its clear dust cup docked in a dark tray beside it. Three bright orange balls of dirt lie in the cup,
under a grille across its top. The cup has a flange round its bottom, a dark handle along its right-hand rim and a
red button on top of the handle. A yellow catch on the vacuum body reaches over the cup's left rim; a yellow post on
top of the body works it. A small black bin stands in front of the vacuum, to the robot's right.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BODY_GREY, DARK, YELLOW = (0.45, 0.47, 0.52), (0.18, 0.19, 0.22), (0.95, 0.78, 0.10)
CLEAR, FLANGE, HANDLE, RED = (0.80, 0.84, 0.88, 0.35), (0.55, 0.58, 0.62), (0.25, 0.12, 0.35), (0.85, 0.10, 0.08)
BIN_BLACK, DIRT = (0.08, 0.08, 0.09), (1.00, 0.45, 0.00)  # dirt balls bright orange, to show on camera

# --- the vacuum body: a fixed tray (the cradle) and a block beside it, with the sprung catch ---------------------
# Body frame: origin on the table under the middle of the tray, x away from the robot, z up.
XC, YC = 0.33, -0.025
FLOOR_T = 0.005  # tray floor thickness: a docked cup's underside is 5 mm up
GUIDE_H = 0.015  # the tray's walls rise to 15 mm
BLOCK_Y0 = 0.054  # the block's face toward the cup
BODY_H = 0.092
BLOCK_W = 0.130  # the block runs 130 mm to the robot's left of the tray
POST_Y, POST_H = 0.165, 0.032  # the catch's post: 165 mm left of the tray's centre, 32 mm tall
TONGUE = (0.040, 0.034, 0.006)  # the catch's tongue: 40 mm wide, 34 mm out from the block face, over the cup's rim
TONGUE_Z0 = 0.086  # its underside: 1 mm over a docked cup's rim
CATCH_TRAVEL = 0.022

BODY = parts.Part(
    "vacuum_body",
    pos=(XC, YC, TOP),
    shapes=[
        parts.box((0.128, 0.120, FLOOR_T), at=(0.0, -0.003, FLOOR_T / 2), color=DARK),  # tray floor
        parts.box((0.006, 0.120, GUIDE_H), at=(-0.061, -0.003, GUIDE_H / 2), color=DARK),  # tray front wall
        parts.box((0.006, 0.120, GUIDE_H), at=(0.061, -0.003, GUIDE_H / 2), color=DARK),  # tray back wall
        parts.box((0.128, 0.006, GUIDE_H), at=(0.0, -0.060, GUIDE_H / 2), color=DARK),  # tray side wall
        parts.box((0.120, BLOCK_W, BODY_H), at=(0.0, BLOCK_Y0 + BLOCK_W / 2, BODY_H / 2), color=BODY_GREY),  # motor block
        parts.box((0.040, 0.060, 0.004), at=(-0.035, BLOCK_Y0 + 0.040, BODY_H + 0.002), color=DARK),  # vent strip
    ],
    joints=[parts.Slider(
        "catch",
        shapes=[
            parts.box(TONGUE, at=(0.0, BLOCK_Y0 - TONGUE[1] / 2, TONGUE_Z0 + TONGUE[2] / 2), color=YELLOW),
            parts.box((TONGUE[0], 0.010, TONGUE[2] + 0.001), at=(0.0, BLOCK_Y0 - TONGUE[1] + 0.005, TONGUE_Z0 + TONGUE[2] / 2),
                      color=(0.05, 0.05, 0.06)),  # black tip, over the rim
            parts.box((0.016, POST_Y - BLOCK_Y0, 0.006), at=(0.0, (POST_Y + BLOCK_Y0) / 2, TONGUE_Z0 + 0.003),
                      color=YELLOW),  # in the block
            parts.box((0.020, 0.010, POST_H), at=(0.0, POST_Y, BODY_H + POST_H / 2), color=YELLOW),  # the post
        ],
        axis=(0.0, 1.0, 0.0),  # back into the block, away from the cup
        travel=CATCH_TRAVEL,
        mass=0.03,
        spring=40.0,
        damping=1.0,
    )],
)

# --- the dust cup: loose, with a sprung bottom flap and a release button -------------------------------------------
# Cup frame: origin at the middle of its underside. Walls 70 x 70 x 80 mm (5 mm thick), a 15 mm flange round the
# bottom, a grille of two bars across the top, the handle bar on the -y wall's top with the button on it.
CUP_IN = 0.030  # inner half-width
CUP_H = 0.080
FLANGE_R = 0.050
HANDLE_Y = -0.032
HANDLE_Z0, HANDLE_Z1 = 0.080, 0.110
BUTTON_TRAVEL = 0.006
BUTTON_TOP = HANDLE_Z1 + 0.008

CUP = parts.Part(
    "dust_cup",
    pos=(XC, YC, TOP + FLOOR_T),
    mass=0.15,
    shapes=[
        parts.box((0.100, 0.015, 0.004), at=(0.0, 0.0425, 0.002), color=FLANGE),  # flange first: the part's colour
        parts.box((0.070, 0.005, CUP_H), at=(0.0, 0.0325, CUP_H / 2), color=CLEAR),
        parts.box((0.070, 0.005, CUP_H), at=(0.0, -0.0325, CUP_H / 2), color=CLEAR),
        parts.box((0.005, 0.060, CUP_H), at=(0.0325, 0.0, CUP_H / 2), color=CLEAR),
        parts.box((0.005, 0.060, CUP_H), at=(-0.0325, 0.0, CUP_H / 2), color=CLEAR),
        parts.box((0.100, 0.015, 0.004), at=(0.0, -0.0425, 0.002), color=FLANGE),
        parts.box((0.015, 0.070, 0.004), at=(0.0425, 0.0, 0.002), color=FLANGE),
        parts.box((0.015, 0.070, 0.004), at=(-0.0425, 0.0, 0.002), color=FLANGE),
        parts.box((0.060, 0.009, 0.003), at=(0.0, 0.0115, CUP_H - 0.0015), color=FLANGE),  # grille
        parts.box((0.060, 0.009, 0.003), at=(0.0, -0.0115, CUP_H - 0.0015), color=FLANGE),
        parts.box((0.050, 0.012, HANDLE_Z1 - HANDLE_Z0), at=(0.0, HANDLE_Y, (HANDLE_Z0 + HANDLE_Z1) / 2), color=HANDLE),
    ],
    joints=[
        parts.Hinge(
            "flap",
            shapes=[parts.box((0.060, 0.060, 0.005), at=(0.0, 0.0, 0.0025), color=FLANGE)],
            pivot=(CUP_IN, 0.0, 0.0025),
            axis=(0.0, -1.0, 0.0),  # the free edge (toward the robot) swings down
            lower=0.0,
            upper=1.2,
            mass=0.03,
            spring=1.0,
            damping=0.08,
        ),
        parts.Slider(
            "release",
            shapes=[parts.box((0.016, 0.008, 0.008), at=(0.0, HANDLE_Y, HANDLE_Z1 + 0.004), color=RED)],
            axis=(0.0, 0.0, -1.0),
            travel=BUTTON_TRAVEL,
            mass=0.015,
            spring=200.0,  # stiff enough that its own weight sags it well under 1 mm
            damping=0.5,
        ),
    ],
)

# --- the bin: an open box 92 mm square and 60 mm tall, 8 mm walls (76 mm opening) ----------------------------------
XB, YB = 0.22, -0.16
BIN_H, BIN_IN, BIN_WALL = 0.060, 0.038, 0.008
BIN = parts.Part(
    "bin",
    pos=(XB, YB, TOP),
    shapes=[
        parts.box((0.092, 0.092, 0.004), at=(0.0, 0.0, 0.002), color=BIN_BLACK),
        parts.box((0.092, BIN_WALL, BIN_H - 0.004), at=(0.0, BIN_IN + BIN_WALL / 2, 0.004 + (BIN_H - 0.004) / 2),
                  color=BIN_BLACK),
        parts.box((0.092, BIN_WALL, BIN_H - 0.004), at=(0.0, -BIN_IN - BIN_WALL / 2, 0.004 + (BIN_H - 0.004) / 2),
                  color=BIN_BLACK),
        parts.box((BIN_WALL, 2 * BIN_IN, BIN_H - 0.004), at=(BIN_IN + BIN_WALL / 2, 0.0, 0.004 + (BIN_H - 0.004) / 2),
                  color=BIN_BLACK),
        parts.box((BIN_WALL, 2 * BIN_IN, BIN_H - 0.004), at=(-BIN_IN - BIN_WALL / 2, 0.0, 0.004 + (BIN_H - 0.004) / 2),
                  color=BIN_BLACK),
    ],
)

# --- the dirt: three 16 mm balls on the flap, in the grille's gaps ------------------------------------------------
BALL_R = 0.008
BALL_XY = ((-0.014, -0.021), (0.012, 0.0), (-0.010, 0.021))
BALLS = [
    Sphere(f"dirt_{i + 1}", pos=(float(XC + dx), float(YC + dy), float(TOP + FLOOR_T + 0.005 + BALL_R)),
           radius=BALL_R, mass=0.010, color=DIRT)
    for i, (dx, dy) in enumerate(BALL_XY)
]
BALL_NAMES = tuple(b.name for b in BALLS)

OBJECTS = [BODY, CUP, BIN, *BALLS]

EPS = 1e-6
SEAT_Z = 0.001  # docked: the cup's underside within 1 mm of the tray floor
SEAT_DX, SEAT_DY = 0.008, 0.007  # and its centre within 8 mm (front to back) and 7 mm (sideways) of the tray's centre
UPRIGHT = math.radians(3.0)
UPRIGHT_LOOSE = math.radians(10.0)
CAUGHT = 0.001  # the catch within 1 mm of its rest, over the rim
FLAP_SHUT = math.radians(3.0)
ON = 0.003  # a ball resting on a surface: its underside within 3 mm of it
CLEAR_GRIP = 0.005  # released: no part of either gripper within 5 mm of the cup, the balls or the catch
PRESS = 0.004  # the button counts as pressed past 4 mm
FLAP_OPEN_TARGET = 1.0
FLAP_LATCH = 0.035  # the flap latches once within 0.035 rad (2 degrees) of shut, with the button let go
CAM_ZONE = 0.030  # a cup coming down within 30 mm over the catch pushes the tongue back
CAM_ALIGN = 0.010


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(state) -> float:
    return math.acos(float(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def _in_cup(ball, cup) -> bool:
    """The ball's centre inside the cup's walls, between its floor and its top (the cup's own frame)."""
    local = _rot(cup.quat_xyzw).T @ (np.asarray(ball.pos) - np.asarray(cup.pos))
    return bool(abs(local[0]) <= CUP_IN + EPS and abs(local[1]) <= CUP_IN + EPS and -EPS <= local[2] <= CUP_H + EPS)


def _in_bin(ball) -> bool:
    """The ball's centre inside the bin's inner walls and below its rim."""
    p = ball.pos
    return bool(abs(p[0] - XB) <= BIN_IN + EPS and abs(p[1] - YB) <= BIN_IN + EPS and p[2] <= TOP + BIN_H + EPS)


def _docked(cup) -> bool:
    return bool(
        abs(cup.pos[0] - XC) <= SEAT_DX + EPS and abs(cup.pos[1] - YC) <= SEAT_DY + EPS
        and abs(cup.pos[2] - (TOP + FLOOR_T)) <= SEAT_Z + EPS and _tilt(cup) <= UPRIGHT + EPS
    )


def _over_tray(p) -> bool:
    return bool(abs(p[0] - XC) <= 0.064 and YC - 0.063 <= p[1] <= YC + BLOCK_Y0)


def _spilled(ball, cup) -> bool:
    """A ball out of the cup and the bin that has landed: on the table, the tray floor or below the table top, or
    lying still anywhere else (the vacuum body, the bin's rim, the cup's outside)."""
    if _in_cup(ball, cup) or _in_bin(ball):
        return False
    low = ball.pos[2] - BALL_R
    if low <= TOP + ON + EPS or ball.pos[2] < TOP:
        return True
    if _over_tray(ball.pos) and low <= TOP + FLOOR_T + ON + EPS:
        return True
    return bool(ball.resting())


# Whole-trial record: has any ball been spilled? Cleared only by a scene reset: every ball back within 0.5 mm of its
# start and at rest, after a jump of over 20 mm in one control step (a robot cannot move them back that way).
_record = {"spill": False, "prev": None}
# In the simulator, an on_step callback fills these every step; offline (near_misses.py) there is no robot, so the
# grippers count as clear.
_sim = {"clear": True, "below": True, "flap_latched": True, "targets": {}, "flap_upper": None}
BALL_START = {b.name: np.asarray(b.pos, float) for b in BALLS}


def _update(objects) -> None:
    pos = {n: np.asarray(objects[n].pos, float) for n in BALL_NAMES}
    prev = _record["prev"]
    if prev is not None:
        home = all(np.linalg.norm(pos[n] - BALL_START[n]) < 0.0005 and objects[n].resting() for n in BALL_NAMES)
        jumped = max(np.linalg.norm(pos[n] - prev[n]) for n in BALL_NAMES) > 0.020
        if home and jumped:
            _record["spill"] = False
    _record["prev"] = pos
    cup = objects["dust_cup"]
    if any(_spilled(objects[n], cup) for n in BALL_NAMES):
        _record["spill"] = True


def _grade(objects) -> float:
    _update(objects)
    if _record["spill"] or not _sim["clear"]:
        return 0.0
    cup, body = objects["dust_cup"], objects["vacuum_body"]
    balls = [objects[n] for n in BALL_NAMES]
    if not (cup.resting() and all(b.resting() for b in balls)) or cup.pos[2] < TOP - 0.010:
        return 0.0
    # A ball counts as emptied only if it is out of the cup: a cup dropped upside down into the bin, its dirt still
    # resting on the grille, has emptied nothing.
    if all(_in_bin(b) and not _in_cup(b, cup) for b in balls):
        if _docked(cup) and body.joints["catch"] <= CAUGHT + EPS and cup.joints["flap"] <= FLAP_SHUT + EPS:
            return 1.0
        return 0.5
    if all(_in_cup(b, cup) for b in balls) and not _docked(cup) and _tilt(cup) <= UPRIGHT_LOOSE + EPS:
        return 0.25
    return 0.0


def success(objects):
    """All three balls in the bin; the cup docked in its tray with the catch back over its rim and its flap shut;
    let go and still; no ball ever spilled."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 as success; 0.5 all three balls in the bin, the cup anywhere else; 0.25 the cup out of its tray, upright,
    all three balls still in it. Released and still. 0 if a ball was ever spilled."""
    return _grade(objects)


def distance(objects):
    return float(np.mean([np.linalg.norm(objects[n].pos[:2] - np.array([XB, YB])) for n in BALL_NAMES]))


# --- simulator hooks ------------------------------------------------------------------------------------------------


def _set(sim, part, joint, value) -> None:
    key = (part, joint)
    if _sim["targets"].get(key) != value:
        _sim["targets"][key] = value
        sim.set_target(part, joint, value)


def _flap_limit(sim, upper) -> None:
    import torch

    if _sim["flap_upper"] == upper:
        return
    _sim["flap_upper"] = upper
    base = sim.env.unwrapped
    cup = base.scene["dust_cup"]
    ids = cup.find_joints(["flap"], preserve_order=True)[0]
    lim = torch.tensor([[[0.0, upper]]], dtype=torch.float32, device=base.device)
    cup.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_reset
def _reset(sim) -> None:
    _sim.update(clear=True, below=True, flap_latched=True, targets={}, flap_upper=None)
    _flap_limit(sim, FLAP_LATCH)


def _hulls(base):
    from yam_arena import analysis

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
    return hulls


def _link_poses(part, state):
    poses = {"base": (np.asarray(state.pos), _rot(state.quat_xyzw))}
    poses.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in state.links.items()})
    return poses


@parts.on_step
def _mechanisms(sim) -> None:
    """The cup's release opens its flap while pressed; let go, the spring shuts it and it latches. The body's catch
    tongue is pushed back by a cup coming down onto it (its sloped face) and springs over the rim once the cup is
    home. Also measures how close the grippers are to the scored parts."""
    from yam_arena import analysis

    st = sim.states()
    cup, body = st["dust_cup"], st["vacuum_body"]
    pressed = cup.joints["release"] >= PRESS - EPS
    flap = cup.joints["flap"]
    if pressed:
        _sim["flap_latched"] = False
    elif flap <= FLAP_LATCH:
        _sim["flap_latched"] = True
    _set(sim, "dust_cup", "flap", FLAP_OPEN_TARGET if pressed else 0.0)
    _flap_limit(sim, FLAP_LATCH if _sim["flap_latched"] else CUP.joints[0].upper)

    rim = cup.pos[2] + CUP_H
    if rim <= TOP + TONGUE_Z0 + 0.0002:
        _sim["below"] = True
    elif rim > TOP + TONGUE_Z0 + TONGUE[2] + 0.002:
        _sim["below"] = False
    aligned = abs(cup.pos[0] - XC) <= CAM_ALIGN and abs(cup.pos[1] - YC) <= CAM_ALIGN and _tilt(cup) < 0.2
    cam = (not _sim["below"]) and aligned and rim < TOP + TONGUE_Z0 + TONGUE[2] + CAM_ZONE
    _set(sim, "vacuum_body", "catch", CATCH_TRAVEL if cam else 0.0)

    boxes = [(c, r, s) for _, c, r, s in CUP.world_boxes(_link_poses(CUP, cup))]
    boxes += [(c, r, s) for k, c, r, s in BODY.world_boxes(_link_poses(BODY, body)) if k == "catch"]
    boxes += [(np.asarray(st[n].pos), np.eye(3), np.full(3, 2 * BALL_R)) for n in BALL_NAMES]
    hulls = _hulls(sim.env.unwrapped)
    _sim["clear"] = all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR_GRIP for c, r, s in boxes)


# --- demo -----------------------------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_TIPS = 0.090  # tips 10 mm down the handle bar (cup frame)
WIDE = -0.020
ON_BAR = FINGER_CLOSED  # close fully on the 12 mm bar


def _w(x, y, z):
    return (float(x), float(y), float(TOP + z))


def demo():
    """Left arm: push the catch's post back and hold it while the right arm lifts the cup out, then let go. Right
    arm: lift the cup by its handle, set its flange on the bin, let go, press the red button for 2 s (the flap opens
    and the dirt drops), let go of it, grip the handle again and lower the cup back into its tray."""
    hx, hy = XC, YC + HANDLE_Y
    rx, ry = hx + 0.006, hy + 0.003  # the loaded arm lands the cup about 6 mm short and 3 mm right (measured)
    dock = FLOOR_T + GRIP_TIPS + TIP  # TCP height with the cup docked
    lift = dock + 0.035  # underside 40 mm up: clear of the tray walls and the tongue
    carry = 0.072 + GRIP_TIPS + TIP  # underside 72 mm up, over the 60 mm bin
    bx, by = XB, YB + HANDLE_Y
    on_bin = BIN_H + GRIP_TIPS + TIP
    press_from = BIN_H + BUTTON_TOP + 0.007 + TIP
    press_to = BIN_H + BUTTON_TOP - 0.007 + TIP

    px = XC
    py0 = YC + POST_Y - 0.005 - 0.003 - 0.018  # finger faces 3 mm short of the post (17 cm from the right arm)
    pz = BODY_H + 0.010 + TIP  # tips 10 mm over the block's top, on the post's lower half
    left = [
        Waypoint(2.0, _w(px, py0, 0.16), FINGER_CLOSED, "over the catch post"),
        Waypoint(1.2, _w(px, py0, pz), FINGER_CLOSED, "down beside the post"),
        Waypoint(1.5, _w(px, py0 + 0.032, pz), FINGER_CLOSED, "push the catch back"),
        Waypoint(2.6, _w(px, py0 + 0.032, pz), FINGER_CLOSED, "hold it while the cup is lifted"),
        Waypoint(1.0, _w(px, py0 + 0.032, 0.16), FINGER_CLOSED, "up: the catch springs back"),
        Waypoint(1.5, _w(0.20, 0.25, 0.16), FINGER_CLOSED, "park"),
    ]
    right = [
        Waypoint(2.5, _w(hx, hy, 0.16), WIDE, "over the handle"),
        Waypoint(1.5, _w(hx, hy, dock), WIDE, "down to the handle"),
        Waypoint(0.8, _w(hx, hy, dock), ON_BAR, "grip the handle"),
        Waypoint(0.6, _w(hx, hy, dock), ON_BAR, "wait for the catch"),
        Waypoint(1.5, _w(hx, hy, lift), ON_BAR, "lift out of the tray"),
        Waypoint(1.2, _w(hx, hy, lift), ON_BAR, "wait for the left arm"),
        Waypoint(1.0, _w(hx, hy, carry), ON_BAR, "lift clear"),
        Waypoint(2.5, _w(bx, by, carry), ON_BAR, "carry over the bin"),
        Waypoint(1.2, _w(bx, by, on_bin + 0.006), ON_BAR, "lower onto the bin"),
        Waypoint(0.6, _w(bx, by, on_bin + 0.006), WIDE, "let go"),
        Waypoint(1.0, _w(bx, by, press_from + 0.01), WIDE, "up"),
        Waypoint(0.6, _w(bx, by, press_from + 0.01), FINGER_CLOSED, "close the fingers"),
        Waypoint(0.8, _w(bx, by, press_from), FINGER_CLOSED, "over the button"),
        Waypoint(1.0, _w(bx, by, press_to), FINGER_CLOSED, "press the button"),
        Waypoint(2.0, _w(bx, by, press_to), FINGER_CLOSED, "hold: the dirt drops"),
        Waypoint(1.0, _w(bx, by, press_from + 0.01), FINGER_CLOSED, "let go of the button"),
        Waypoint(0.6, _w(bx, by, press_from + 0.01), WIDE, "open"),
        Waypoint(1.2, _w(bx, by, on_bin), WIDE, "down to the handle"),
        Waypoint(0.8, _w(bx, by, on_bin), ON_BAR, "grip the handle"),
        Waypoint(1.0, _w(bx, by, carry), ON_BAR, "lift"),
        Waypoint(2.5, _w(hx, hy, carry), ON_BAR, "carry over the tray"),
        Waypoint(2.0, _w(rx, ry, dock + 0.006), ON_BAR, "lower into the tray"),
        Waypoint(0.6, _w(rx, ry, dock + 0.006), WIDE, "let go"),
        Waypoint(1.2, _w(hx, hy, 0.16), WIDE, "up"),
        Waypoint(1.2, _w(0.20, -0.25, 0.16), WIDE, "park"),
        Waypoint(3.0, _w(0.20, -0.25, 0.16), WIDE, "wait"),
    ]
    return {"left": left, "right": right}
