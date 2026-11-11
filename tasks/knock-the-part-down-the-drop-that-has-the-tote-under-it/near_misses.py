"""Near-successes and unintended strategies, scored offline with task.py's own score() and success(), in lot A (the
tote in the right bore). Each case: (label, {object: spec}, expected score, expected success); a spec is
(centre, size or None, velocity or None[, quat_xyzw]) or a ready part state (TOTE.state(...)). Cases run in order
and share the whole-trial record, so a fault stays until a scene reset (an empty case after the part was moved).
"""

import math

from task import BORE_FLOOR, FLOOR, PART_H, PART_R, TOP, TOTE, TOTE_AT, TOTE_FLOOR, TOTE_INNER, X0, Y0

TX, TY, TZ = TOTE_AT["right"]
ON_TOTE_FLOOR = TZ + TOTE_FLOOR + PART_H / 2
LYING = ON_TOTE_FLOOR - PART_H / 2 + PART_R  # the part lying on its side on the tote floor


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def part(x, y, z, vel=None, quat=None):
    return {"part": ((x, y, z), None, vel, *([quat] if quat else []))}


IN_TOTE = part(TX, TY, ON_TOTE_FLOOR)
RESET = {}
BARE_Z = TOP + BORE_FLOOR + PART_H / 2  # on a bare bore's black floor

CASES = [
    ("part at its start in the channel", {}, 0.0, False),
    ("part upright in the tote", IN_TOTE, 1.0, True),
    ("part lying on its side in the tote", part(TX, TY, LYING, quat=tilt_x(90)), 1.0, True),
    ("part leaning 30 degrees against the tote wall", part(TX, TY - 0.004, ON_TOTE_FLOOR - PART_H / 2 + PART_H / 2 * math.cos(math.radians(30)) + PART_R * math.sin(math.radians(30)), quat=tilt_x(30)), 1.0,
     True),
    ("part tipped 50 degrees, its box reaching 5 mm below the tote floor",
     part(TX, TY, ON_TOTE_FLOOR - PART_H / 2 + 0.012, quat=tilt_x(50)), 1.0, True),
    ("part centre at the tote's inner wall", part(TX, TY - TOTE_INNER, ON_TOTE_FLOOR), 1.0, True),
    ("part in the tote, still moving", part(TX, TY, ON_TOTE_FLOOR, vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("part pushed part-way, still on the channel floor at the drop edge", part(X0, Y0 - 0.030, TOP + FLOOR + PART_H / 2),
     0.0, False),
    ("part pushed the wrong way, on the floor past the left drop-through",
     part(X0, Y0 + 0.085, TOP + FLOOR + PART_H / 2), 0.0, False),
    ("part caught on the tote's rim, its top above the channel floor",
     part(TX, TY, TOP + FLOOR - 0.003 + PART_H / 2 - 0.004), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("habit: always the right knob (part down the bare left bore)", part(X0, Y0 + 0.055, BARE_Z), 0.0, False),
    ("then the part found in the tote: still 0", IN_TOTE, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("habit: always the near knob (part down the bare far bore)", part(X0 + 0.055, Y0, BARE_Z), 0.0, False),
    ("then the part found in the tote: still 0", IN_TOTE, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("part lodged 6 mm down the left drop-through", part(X0, Y0 + 0.055, TOP + FLOOR - 0.006 + PART_H / 2), 0.0, False),
    ("then in the tote: still 0", IN_TOTE, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("part 4 mm down the left drop-through edge: no fault", part(X0, Y0 + 0.035, TOP + FLOOR - 0.004 + PART_H / 2),
     0.0, False),
    ("then in the tote: 1.0", IN_TOTE, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("tote lifted 6 mm (pulled up through its drop-through)", {"tote": TOTE.state(pos=(TX, TY, TZ + 0.006))}, 0.0,
     False),
    ("then the tote back and the part in it: still 0", {"tote": TOTE.state(), **IN_TOTE}, 0.0, False),
    ("scene reset", {"tote": TOTE.state()}, 0.0, False),
    ("tote shifted 4 mm in its bore, part in it: 1.0", {"tote": TOTE.state(pos=(TX, TY - 0.004, TZ)), **IN_TOTE}, 1.0,
     True),
    ("scene reset", {"tote": TOTE.state()}, 0.0, False),
    ("tote moved to the left bore and the part dropped there", {
        "tote": TOTE.state(pos=TOTE_AT["left"]), **part(TOTE_AT["left"][0], TOTE_AT["left"][1], ON_TOTE_FLOOR)},
     0.0, False),
]


def _run() -> int:
    """Score every case in order with task.py's own score() and success(); print each and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(name, spec):
        if isinstance(spec, ObjectState):
            return spec
        pos, size, vel, *quat = spec
        o = next(o for o in task.OBJECTS if o.name == name)
        size = np.array([2 * o.radius, 2 * o.radius, o.height]) if size is None else np.asarray(size, float)
        q = np.asarray(quat[0] if quat else (0.0, 0.0, 0.0, 1.0), float)
        r = task._rot(q)
        half = np.abs(r) @ (size / 2)
        p = np.asarray(pos, float)
        return ObjectState(p, q / np.linalg.norm(q), np.zeros(3) if vel is None else np.asarray(vel, float),
                           np.zeros(3), p - half, p + half)

    start = {"sorting_nest": task.NEST.state(), "tote": task.TOTE.state(), "part": state("part", (task.PART.pos, None, None))}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start) | {k: state(k, v) for k, v in changes.items()}
        ok = bool(task.success(st))
        got = float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
