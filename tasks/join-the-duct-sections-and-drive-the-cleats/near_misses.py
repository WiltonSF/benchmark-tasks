"""Near-successes, unintended strategies and fixed habits, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: spec}, expected score, expected success). Specs are ready states from each part's own
kinematics (Part.state(...)). Offline there are no grippers, so held states show up only through the table and
stillness checks here; the release check is exercised in the GPU trace.
"""

import math

from task import CLEAT_Y, CLEATS, DH, DL, DUCT_B, FL_T, FLANGE_MID, TOP, XJ, Y0

HX = XJ - DL / 2  # the loose section's centre when home
MOVING = (0.0, 0.0, -0.05)


def _qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _qy(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def b(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, quat=None, vel=None):
    """The loose section with its leading end dx from the joint (negative: short), turned yaw deg."""
    q = quat if quat is not None else _qz(yaw)
    return {"loose_duct": DUCT_B.state(pos=(HX + dx, Y0 + dy, TOP + dz), quat_xyzw=q, lin_vel=vel)}


def cleat(side, dx=0.0, dy=0.0, dz=0.0, quat=None, vel=None):
    """A cleat driven over the joint's flanges on ``side`` (slot centred on the joint), shifted as given."""
    return {f"{side}_cleat": CLEATS[side].state(pos=(XJ + dx, CLEAT_Y[side] + dy, TOP + dz), quat_xyzw=quat,
                                                  lin_vel=vel)}


BOTH = {**cleat("left"), **cleat("right")}


def swapped(cleat_name, side, dx=0.0):
    """Cleat ``cleat_name`` turned round (180 deg) and driven over the flanges on the other ``side``."""
    return {cleat_name: CLEATS["left" if cleat_name == "left_cleat" else "right"].state(
        pos=(XJ + dx, CLEAT_Y[side], TOP), quat_xyzw=_qz(180.0))}


SWAPPED = {**swapped("left_cleat", "right"), **swapped("right_cleat", "left")}


def b_pivot(yaw_deg, touching):
    """The section turned yaw_deg with the far flange on side ``touching`` against the fixed one (the other short)."""
    a = math.radians(yaw_deg)
    s = 1.0 if touching == "left" else -1.0
    # far flange face on that side: centre x + DL/2 cos(a) - s FLANGE_MID sin(a) = XJ
    cx = XJ - (DL / 2 * math.cos(a) - s * FLANGE_MID * math.sin(a))
    return {"loose_duct": DUCT_B.state(pos=(cx, Y0, TOP), quat_xyzw=_qz(yaw_deg))}

CASES = [
    ("home, both cleats on", {**b(), **BOTH}, 1.0, True),
    ("home, both cleats on, a cleat still sliding down", {**b(), **cleat("left", vel=MOVING), **cleat("right")}, 0.0,
     False),
    ("home, one cleat on", {**b(), **cleat("right")}, 0.6, False),
    ("home, no cleats", b(), 0.25, False),
    ("cleats swapped: each turned round and driven on the other side", {**b(), **SWAPPED}, 1.0, True),
    ("one cleat swapped onto the other side, the other left on the table",
     {**b(), **swapped("left_cleat", "right")}, 0.6, False),
    ("cleat on its own side but turned round (slot facing away from the duct)",
     {**b(), **cleat("left", quat=_qz(180.0)), **cleat("right")}, 0.6, False),
    ("turned 0.4 deg, right flanges touching, left gap 0.94 mm", {**b_pivot(0.4, "right"), **BOTH}, 1.0, True),
    ("turned 0.5 deg, right flanges touching, left gap 1.18 mm", {**b_pivot(0.5, "right"), **BOTH}, 0.0, False),
    ("turned 0.5 deg the other way, left flanges touching, right gap 1.18 mm", {**b_pivot(-0.5, "left"), **BOTH},
     0.0, False),
    ("0.9 mm short (flange gap 0.9 mm), both on", {**b(dx=-0.0009), **cleat("left", dx=-0.0005),
                                                   **cleat("right", dx=-0.0005)}, 1.0, True),
    ("1.1 mm short (gap over 1 mm), both on", {**b(dx=-0.0011), **cleat("left", dx=-0.0006),
                                               **cleat("right", dx=-0.0006)}, 0.0, False),
    ("home but 1.9 mm off centre", {**b(dy=0.0019), **BOTH}, 1.0, True),
    ("home but 2.1 mm off centre", {**b(dy=0.0021), **BOTH}, 0.0, False),
    ("centre home but 1.4 deg skew: one flange pair 1.7 mm apart", {**b(yaw=1.4), **BOTH}, 0.0, False),
    ("home, 1.6 deg skew", {**b(yaw=1.6), **BOTH}, 0.0, False),
    ("habit: pushed straight from the start, stopped short and turned", b(dx=-0.040, dy=0.025, yaw=8.0), 0.0, False),
    ("habit: lot A's shift and turn used in lot B (50 mm off, turned 16 deg)", b(dx=-0.010, dy=-0.050, yaw=-16.0), 0.0,
     False),
    ("riding up on the lips (tilted 3 deg)", {**b(dz=0.003, quat=_qy(-3.0)), **BOTH}, 0.0, False),
    ("habit: cleat set on top of the flanges", {**b(), **cleat("left", dz=0.041), **cleat("right")}, 0.6, False),
    ("both cleats on the flange tops", {**b(), **cleat("left", dz=0.041), **cleat("right", dz=0.041)}, 0.25, False),
    ("cleat over both flanges but 3.1 mm high", {**b(), **cleat("left", dz=0.0031), **cleat("right")}, 0.6, False),
    ("cleat 2.9 mm high", {**b(), **cleat("left", dz=0.0029), **cleat("right")}, 1.0, True),
    ("cleat slot 1.9 mm off the joint's middle: both flanges still in the 8 mm slot",
     {**b(), **cleat("left", dx=0.0019), **cleat("right")}, 1.0, True),
    ("cleat slot 2.1 mm off the joint's middle: the loose flange in the lead-in only",
     {**b(), **cleat("left", dx=0.0021), **cleat("right")}, 0.6, False),
    ("cleat taking 5.1 mm of the flanges", {**b(), **cleat("left", dy=0.0024), **cleat("right")}, 1.0, True),
    ("cleat taking only 4.9 mm of the flanges", {**b(), **cleat("left", dy=0.0026), **cleat("right")}, 0.6, False),
    ("cleat leaning 6 deg", {**b(), **cleat("left", quat=_qy(6.0)), **cleat("right")}, 0.6, False),
    ("cleat knocked over on the table", {**b(), **cleat("left", dz=0.0, quat=_qy(90.0)), **cleat("right")}, 0.6,
     False),
    ("section short by 10 mm, cleats on the fixed flange only",
     {**b(dx=-0.010), **cleat("left", dx=FL_T), **cleat("right", dx=FL_T)}, 0.0, False),
    ("section held 2 mm up off the table, cleats on", {**b(dz=0.002), **BOTH}, 0.0, False),
    ("section pushed past home into the fixed flanges (1 mm overlap)", {**b(dx=0.001), **BOTH}, 0.0, False),
    ("nothing moved", {}, 0.0, False),
]


def _run() -> int:
    import task

    base = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(base) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
