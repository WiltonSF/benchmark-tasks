"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py). Each case:
(label, {object: spec}, expected score, expected success); a spec is (pos, size or None, velocity or None[, quat])
or a part's ready state (CUP.state(...)). Cases run in order and share the whole-trial spill record, so a spill stays
until a scene reset (an empty case after the balls were moved).
"""

import math

from task import (BALL_R, BIN_H, BIN_IN, BODY, BODY_H, CUP, FLOOR_T, TOP, XB, XC, YB, YC)

DOCK_Z = TOP + FLOOR_T


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def cup(x=XC, y=YC, z=DOCK_Z, flap=0.0, quat=None, vel=None, release=0.0):
    return {"dust_cup": CUP.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel, joints={"flap": flap, "release": release})}


def catch(q):
    return {"vacuum_body": BODY.state(joints={"catch": q})}


def balls_in_bin(dx=0.0, vel=None):
    spots = ((-0.02, -0.02), (0.02, -0.01), (0.0, 0.022))
    out = {}
    for i, (sx, sy) in enumerate(spots):
        out[f"dirt_{i + 1}"] = ((XB + sx + (dx if i == 0 else 0.0), YB + sy, TOP + 0.004 + BALL_R), None,
                                vel if i == 0 else None)
    return out


def balls_in_cup_at(x, y, z):
    spots = ((-0.014, -0.021), (0.012, 0.0), (-0.010, 0.021))
    return {f"dirt_{i + 1}": ((x + sx, y + sy, z + 0.005 + BALL_R), None, None) for i, (sx, sy) in enumerate(spots)}


DONE = {**cup(), **balls_in_bin()}
OFF = (0.24, -0.02)  # a free spot on the table, in front of the tray
ON_BIN_Z = TOP + BIN_H
SCENE_RESET = {}

CASES = [
    ("emptied, cup docked and caught, flap shut", DONE, 1.0, True),
    ("catch 1 mm out (limit)", {**DONE, **catch(0.001)}, 1.0, True),
    ("catch 2 mm out: not caught", {**DONE, **catch(0.002)}, 0.5, False),
    ("catch held back fully", {**DONE, **catch(0.010)}, 0.5, False),
    ("flap 3 degrees open (limit)", {**cup(flap=math.radians(3.0)), **balls_in_bin()}, 1.0, True),
    ("flap 4 degrees open", {**cup(flap=math.radians(4.0)), **balls_in_bin()}, 0.5, False),
    ("cup 1 mm above the tray floor (limit)", {**cup(z=DOCK_Z + 0.001), **balls_in_bin()}, 1.0, True),
    ("cup 2 mm above the tray floor", {**cup(z=DOCK_Z + 0.002), **balls_in_bin()}, 0.5, False),
    ("cup 8 mm off in x (limit)", {**cup(x=XC + 0.008), **balls_in_bin()}, 1.0, True),
    ("cup 9 mm off in x", {**cup(x=XC - 0.009), **balls_in_bin()}, 0.5, False),
    ("cup 7 mm off in y (limit)", {**cup(y=YC - 0.007), **balls_in_bin()}, 1.0, True),
    ("cup 8 mm off in y", {**cup(y=YC - 0.008), **balls_in_bin()}, 0.5, False),
    ("cup tilted 4 degrees in the tray", {**cup(quat=tilt_x(4.0)), **balls_in_bin()}, 0.5, False),
    ("cup still moving", {**cup(vel=(0.02, 0.0, 0.0)), **balls_in_bin()}, 0.0, False),
    ("a ball still rolling in the bin", {**cup(), **balls_in_bin(vel=(0.03, 0.0, 0.0))}, 0.0, False),
    ("ball at the bin's inner wall", {**cup(), **balls_in_bin(dx=-(BIN_IN - 0.02))}, 1.0, True),
    ("emptied, cup left on the bin", {**cup(XB, YB, ON_BIN_Z), **balls_in_bin()}, 0.5, False),
    ("emptied, cup left on the table", {**cup(*OFF, TOP), **balls_in_bin()}, 0.5, False),
    ("emptied, cup on its side on the table", {**cup(*OFF, TOP + 0.05, quat=tilt_x(90)), **balls_in_bin()}, 0.5,
     False),
    ("emptied, cup knocked off the table", {**cup(*OFF, TOP - 0.30), **balls_in_bin()}, 0.0, False),
    ("cup dropped upside down into the bin, dirt still on its grille",
     {**cup(XB, YB, TOP + 0.004 + 0.080, quat=(1.0, 0.0, 0.0, 0.0)),
      **{f"dirt_{i + 1}": ((XB + dx, YB + dy, TOP + 0.004 + 0.080 - 0.0785 + 0.0015 + BALL_R), None, None)
         for i, (dx, dy) in enumerate(((-0.014, -0.021), (0.012, 0.0), (-0.010, 0.021)))}}, 0.0, False),
    ("cup lifted out, set on the table, dirt still in it", {**cup(*OFF, TOP), **balls_in_cup_at(*OFF, TOP)}, 0.25,
     False),
    ("cup set on the bin, dirt still in it", {**cup(XB, YB, ON_BIN_Z), **balls_in_cup_at(XB, YB, ON_BIN_Z)}, 0.25,
     False),
    ("cup on the table tilted 15 degrees, dirt in it", {**cup(*OFF, TOP + 0.01, quat=tilt_x(15)),
                                                         **balls_in_cup_at(*OFF, TOP + 0.01)}, 0.0, False),
    ("cup docked, catch held back, dirt still in it", {**catch(0.010)}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("ball in flight between cup and bin: no spill",
     {"dirt_1": ((XB, YB, TOP + 0.075), None, (0.0, 0.0, -0.4))}, 0.0, False),
    ("then all in the bin, cup home: 1.0", DONE, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Spills from here on; each is followed by a scene reset.
    ("one ball on the table", {"dirt_1": ((0.24, -0.08, TOP + BALL_R), None, None)}, 0.0, False),
    ("then picked up into the bin, cup home: still 0", DONE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, emptied and home: 1.0", DONE, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("flap opened in the tray: a ball on the tray floor",
     {**cup(flap=1.0), "dirt_2": ((XC + 0.045, YC - 0.03, TOP + FLOOR_T + BALL_R), None, None)}, 0.0, False),
    ("then emptied and home: still 0", DONE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("a ball resting on the vacuum body", {"dirt_3": ((XC, YC + 0.09, TOP + BODY_H + BALL_R), None, None)}, 0.0,
     False),
    ("then emptied and home: still 0", DONE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("a ball resting on the bin's rim", {"dirt_1": ((XB, YB + BIN_IN + 0.004, TOP + BIN_H + BALL_R), None, None)},
     0.0, False),
    ("then emptied and home: still 0", DONE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("a ball on the floor", {"dirt_2": ((0.24, -0.08, TOP - 0.40), None, None)}, 0.0, False),
    ("then emptied and home: still 0", DONE, 0.0, False),
]
