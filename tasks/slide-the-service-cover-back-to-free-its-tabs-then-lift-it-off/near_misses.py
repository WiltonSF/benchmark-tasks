"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(lip on the far edge, so the cover frees by sliding toward the robot, along -x). Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state from the part's own geometry. Cases run in order and share
the whole-trial record, so a fault stays until a scene reset (an empty case after the cover was moved away).
"""

import math

from task import CHASSIS, CHASSIS_XY, COVER, MAT_TOP, MAT_XY, SEAT, TOP

CX, CY = CHASSIS_XY
MX, MY = MAT_XY


def q_y(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def q_z(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def seated(slide=0.0, dy=0.0, dz=0.0, quat=None, vel=None):
    """The cover in the chassis, slid `slide` m away from the lip (toward the robot in lot A)."""
    return {"cover": COVER.state(pos=(CX - slide, CY + dy, TOP + SEAT + dz), quat_xyzw=quat, lin_vel=vel)}


def on_mat(x=MX, y=MY, gap=0.0, quat=None, vel=None):
    return {"cover": COVER.state(pos=(x, y, MAT_TOP + gap), quat_xyzw=quat, lin_vel=vel)}


SCENE_RESET = {}

CASES = [
    ("cover in place at the start", {}, 0.0, False),
    ("slid 13 mm: tabs still 1 mm under the lip", seated(0.013), 0.0, False),
    ("slid 14 mm: tabs just clear of the lip", seated(0.014), 0.5, False),
    ("slid 20 mm, flat in its seat", seated(0.020), 0.5, False),
    ("slid 20 mm, still moving", seated(0.020, vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("slid 20 mm, lifted 3 mm (held up)", seated(0.020, dz=0.003), 0.0, False),
    ("slid 20 mm, tilted 4 degrees in its seat", seated(0.020, dz=0.004, quat=q_y(4)), 0.0, False),
    ("wedged under the lip, front raised 8 degrees", seated(0.0, dz=0.010, quat=q_y(8)), 0.0, False),
    ("lifted 2 mm straight up, tabs against the lip", seated(0.0, dz=0.002), 0.0, False),
    ("flat on the mat, centred", on_mat(), 1.0, True),
    ("flat on the mat, turned 90 degrees", on_mat(quat=q_z(90)), 1.0, True),
    ("on the mat, 2 mm up (settling)", on_mat(gap=0.002), 1.0, True),
    ("held 3 mm above the mat", on_mat(gap=0.003), 0.0, False),
    ("on the mat, sliding", on_mat(vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("on the mat, edge exactly on the mat's edge", on_mat(x=MX + 0.110 - 0.075), 1.0, True),
    ("1 mm past the mat's edge", on_mat(x=MX + 0.110 - 0.075 + 0.001), 0.0, False),
    ("on the mat, tilted 5 degrees", on_mat(gap=0.075 * math.sin(math.radians(5)), quat=q_y(5)), 1.0, True),
    ("on the mat, tilted 6 degrees (leaning)", on_mat(gap=0.075 * math.sin(math.radians(6)), quat=q_y(6)), 0.0, False),
    ("lying on the table beside the mat", {"cover": COVER.state(pos=(MX, MY - 0.17, TOP))}, 0.0, False),
    ("upside down on the mat", {"cover": COVER.state(pos=(MX, MY, MAT_TOP + 0.037), quat_xyzw=(1.0, 0.0, 0.0, 0.0))},
     0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("chassis shoved 4 mm", {"chassis": CHASSIS.state(pos=(CX + 0.004, CY, TOP))}, 0.0, False),
    ("then the cover on the mat: still 0", {"chassis": CHASSIS.state(), **on_mat()}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, cover on the mat: 1.0", on_mat(), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("chassis tipped 4 degrees", {"chassis": CHASSIS.state(pos=(CX, CY, TOP + 0.003), quat_xyzw=q_y(4))}, 0.0, False),
    ("then freed in its seat: still 0", {"chassis": CHASSIS.state(), **seated(0.020)}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("chassis nudged 3 mm, cover on the mat: 1.0",
     {"chassis": CHASSIS.state(pos=(CX + 0.003, CY, TOP)), **on_mat()}, 1.0, True),
]


if __name__ == "__main__":
    import task

    start = {o.name: o.state() if hasattr(o, "state") else None for o in task.OBJECTS}
    start["mat"] = None
    bad = 0
    for label, changes, want, want_ok in CASES:
        st = dict(start, **changes)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "" if (got == want and ok == want_ok) else "   <-- MISMATCH"
        bad += bool(flag)
        print(f"{got:4.2f}  {label}{flag}")
    raise SystemExit(1 if bad else 0)
