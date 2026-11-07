"""Near-successes, wrong outcomes and fixed habits, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). The tube's and the sleeve's states are parts.py
Part states: pos is the part's origin (the tube's tip, the sleeve's underside). The plug is a plain cylinder.
"""

import math

from task import (COLLAR_T, PLUG_H, PRESEATED, RACK_FLOOR, RACK_X, RACK_Y, RIM_Z, SL_BELOW, SL_FLOOR, SLEEVE,
                  SLEEVE2, SLEEVE_X, SLEEVE_Y, TOP, TUBE, WELL_X, WELL_Y, BLOCK_FLOOR)

SEATED_Z = RIM_Z - SL_BELOW  # sleeve underside when its collar rests on the rim
SEAT_Z = SEATED_Z + SL_FLOOR  # top of the sleeve floor


def lean(deg, axis="x"):
    a = math.radians(deg) / 2
    if axis == "xy":  # toward a corner of the square bore
        k = math.sin(a) / math.sqrt(2)
        return (k, -k, 0.0, math.cos(a))
    return (math.sin(a), 0.0, 0.0, math.cos(a)) if axis == "x" else (0.0, math.sin(a), 0.0, math.cos(a))


def sleeve(x=WELL_X, y=WELL_Y, z=SEATED_Z, deg=0.0):
    return SLEEVE.state(pos=(x, y, z), quat_xyzw=lean(deg))


def tube(x=WELL_X, y=WELL_Y, z=SEAT_Z, deg=0.0, vel=None, axis="x"):
    return TUBE.state(pos=(x, y, z), quat_xyzw=lean(deg, axis), lin_vel=vel)


DONE = {"sleeve": sleeve(), "sample_tube": tube()}
PLUG_IN_WELL = ((WELL_X, WELL_Y, TOP + BLOCK_FLOOR + PLUG_H / 2), None, None)

CASES = [
    ("sleeve seated, tube upright in it", DONE, 1.0, True),
    ("tube leaning 4 degrees in the sleeve", {**DONE, "sample_tube": tube(deg=4.0)}, 1.0, True),
    ("tube leaning 6.3 degrees into a corner of the sleeve's bore (its natural lean)",
     {**DONE, "sample_tube": tube(deg=6.3, axis="xy")}, 1.0, True),
    ("tube leaning exactly 8 degrees", {**DONE, "sample_tube": tube(deg=8.0, axis="y")}, 1.0, True),
    ("tube leaning 8.1 degrees", {**DONE, "sample_tube": tube(deg=8.1, axis="y")}, 0.0, False),
    ("tube tip 1 mm off the seat (just resting)", {**DONE, "sample_tube": tube(z=SEAT_Z + 0.001)}, 1.0, True),
    ("tube held 3 mm above the seat", {**DONE, "sample_tube": tube(z=SEAT_Z + 0.003)}, 0.0, False),
    ("tube tip outside the centring pocket", {**DONE, "sample_tube": tube(x=WELL_X + 0.0065, deg=3.0)}, 0.0, False),
    ("tube still moving", {**DONE, "sample_tube": tube(vel=(0.0, 0.0, -0.05))}, 0.0, False),
    ("collar 3 mm above the rim (sleeve not seated)", {"sleeve": sleeve(z=SEATED_Z + 0.003),
                                                       "sample_tube": tube(z=SEAT_Z + 0.003)}, 0.0, False),
    ("sleeve tilted 3 degrees", {"sleeve": sleeve(deg=3.0), "sample_tube": tube()}, 0.0, False),
    ("habit: tube straight into the bare well, leaning 16 degrees (traced)",
     {"sample_tube": tube(z=TOP + BLOCK_FLOOR, deg=16.0)}, 0.0, False),
    ("tube upright in the bare well (if it could balance)", {"sample_tube": tube(z=TOP + BLOCK_FLOOR)}, 0.0, False),
    ("sleeve seated, tube left in its rack", {"sleeve": sleeve(),
                                             "sample_tube": tube(RACK_X, RACK_Y, TOP + RACK_FLOOR)}, 0.0, False),
    ("tube in the sleeve, sleeve standing on the table", {"sleeve": sleeve(SLEEVE_X, SLEEVE_Y, TOP),
                                                         "sample_tube": tube(SLEEVE_X, SLEEVE_Y, TOP + SL_FLOOR)},
     0.0, False),
    ("lot B done: tube in the pre-seated sleeve_2, bench sleeve left on the table",
     {"sleeve_2": SLEEVE2.state(pos=PRESEATED[:2] + (SEATED_Z,)),
      "sample_tube": tube(PRESEATED[0], PRESEATED[1], SEAT_Z)}, 1.0, True),
    ("lot B habit, always fit a sleeve: bench sleeve set on the seated one's collar, tube left in the rack",
     {"sleeve_2": SLEEVE2.state(pos=PRESEATED[:2] + (SEATED_Z,)),
      "sleeve": sleeve(PRESEATED[0], PRESEATED[1], SEATED_Z + SL_BELOW + COLLAR_T)}, 0.0, False),
    ("lot B habit: bench sleeve on the seated one, tube balanced in its bore",
     {"sleeve_2": SLEEVE2.state(pos=PRESEATED[:2] + (SEATED_Z,)),
      "sleeve": sleeve(PRESEATED[0], PRESEATED[1], SEATED_Z + SL_BELOW + COLLAR_T),
      "sample_tube": tube(PRESEATED[0], PRESEATED[1], SEATED_Z + SL_BELOW + COLLAR_T + SL_FLOOR)}, 0.0, False),
    ("lot B habit: bench sleeve put on the block's top beside the well, tube in the seated one",
     {"sleeve_2": SLEEVE2.state(pos=PRESEATED[:2] + (SEATED_Z,)),
      "sleeve": sleeve(WELL_X, WELL_Y + 0.040, RIM_Z),
      "sample_tube": tube(PRESEATED[0], PRESEATED[1], SEAT_Z)}, 0.0, False),
    ("habit, nearest first: the plug in the well, tube on the table", {"well_plug": PLUG_IN_WELL}, 0.0, False),
    ("habit, use everything: plug on the block over the well, sleeve and tube done",
     {**DONE, "well_plug": ((WELL_X, WELL_Y + 0.025, RIM_Z + PLUG_H / 2), None, None)}, 0.0, False),
]


if __name__ == "__main__":
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    start = {o.name: o.state() if hasattr(o, "state") else None for o in task.OBJECTS}
    plug = task.PLUG
    half = np.array((plug.radius, plug.radius, plug.height / 2))

    def plain(pos, vel=None):
        p = np.asarray(pos, float)
        return ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3) if vel is None else np.asarray(vel, float),
                           np.zeros(3), p - half, p + half)

    bad = 0
    for label, changes, want, want_ok in CASES:
        st = {n: s for n, s in start.items() if s is not None}
        st["well_plug"] = plain(plug.pos)
        for name, spec in changes.items():
            st[name] = plain(spec[0]) if isinstance(spec, tuple) else spec
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_ok) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    raise SystemExit(bad)
