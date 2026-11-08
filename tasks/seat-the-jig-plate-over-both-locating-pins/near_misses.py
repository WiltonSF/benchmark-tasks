"""Near-successes and unintended strategies, scored offline by factory/check.py, in lot A (round pin on the left).
Cases run in order and share the trial record (the fixture must never move), so a fault stays until a scene reset
(an empty case after the plate was moved)."""

import math

import numpy as np

from task import (BASE_H, BASE_X, BASE_Y, FIXTURE, HOLES, PIN_H, PIN_MID, PIN_Y, PLATE, PLATE_T, PLATE_W, PLAY,
                  RISER_SIZE, RISER_X, RISER_Y, TOP)

MID = PLATE_T / 2
BASE_TOP = TOP + BASE_H


def quat(yaw_deg=0.0, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)):
    """Turn yaw_deg about z, then tilt_deg about a world axis."""
    def q(axis, deg):
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        s = math.sin(math.radians(deg) / 2)
        return np.array([a[0] * s, a[1] * s, a[2] * s, math.cos(math.radians(deg) / 2)])

    def mul(p, r):
        x1, y1, z1, w1 = p
        x2, y2, z2, w2 = r
        return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])

    return tuple(mul(q(tilt_axis, tilt_deg), q((0, 0, 1), yaw_deg)))


def _r(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def plate(dx=0.0, dy=0.0, up=0.0, yaw=180.0, tilt=0.0, tilt_axis=(1.0, 0.0, 0.0), vel=None, z=None, centre=None,
          fix=(0.0, 0.0)):
    """The plate turned (yaw, tilt; yaw 180 is the way that fits in lot A), its holes' midpoint dx, dy from the pins'
    midpoint (or its own middle at centre=(x, y)), its middle at height z (default: flat on the fixture, up higher).
    fix: the fixture's shift, which moves the pins' midpoint too."""
    q = quat(yaw, tilt, tilt_axis)
    if centre is None:
        off = _r(q) @ np.array([0.0, (HOLES[0] + HOLES[1]) / 2, 0.0])
        xy = (BASE_X + fix[0] + dx - off[0], BASE_Y + PIN_MID + fix[1] + dy - off[1])
    else:
        xy = centre
    pos = (*xy, (BASE_TOP + MID + up) if z is None else z)
    return PLATE.state(pos=pos, quat_xyzw=q, lin_vel=vel)


def fixture(dx=0.0, dy=0.0, yaw=0.0):
    return FIXTURE.state(pos=(BASE_X + dx, BASE_Y + dy, TOP), quat_xyzw=quat(yaw))


# Riding on one pin's top: that end's underside sits on the pin top (12 mm up), the other end down on the fixture,
# pins 90 mm apart, plate tilted about x by asin(12 / 90).
RIDE = math.degrees(math.asin(PIN_H / (2 * PIN_Y)))


def riding(sign, yaw=180.0):
    # Centre raised by half the lift, tilted so the end over the pin at +sign*PIN_Y is up.
    return plate(up=PIN_H / 2, tilt=sign * RIDE, yaw=yaw)


def edge_up(deg):
    """Tilted deg about the long axis, the low long edge resting on the fixture."""
    t = math.radians(deg)
    return plate(z=BASE_TOP + PLATE_W / 2 * math.sin(t) + MID * math.cos(t), tilt=deg, tilt_axis=(0.0, 1.0, 0.0))


SEATED = {"jig_plate": plate()}
RESET = {}

CASES = [
    ("start: plate on its riser", {}, 0.0, False),
    ("seated: big hole on the round pin", SEATED, 1.0, True),
    ("turned end for end, flat on the fixture (cannot happen: the round pin is under the small hole's rim)",
     {"jig_plate": plate(yaw=0.0)}, 0.0, False),
    ("turned end for end, resting on the round pin's top", {"jig_plate": riding(1, yaw=0.0)}, 0.0, False),
    ("turned end for end, resting on both pin tops", {"jig_plate": plate(yaw=0.0, up=PIN_H)}, 0.0, False),
    ("seated upside down (turned over about its long axis)",
     {"jig_plate": plate(tilt=180.0, tilt_axis=(0.0, 1.0, 0.0))}, 1.0, True),
    ("turned over about its short axis: holes swap ends", {"jig_plate": plate(tilt=180.0)}, 0.0, False),
    ("habit: plate centred on the fixture base", {"jig_plate": plate(centre=(BASE_X, BASE_Y))}, 0.0, False),
    ("habit: plate's middle on the pins' midpoint (10 mm out)",
     {"jig_plate": plate(centre=(BASE_X, BASE_Y + PIN_MID))}, 0.0, False),
    ("habit: set down as it lay (15 degrees off), not turned, on the fixture", {"jig_plate": plate(yaw=15.0)}, 0.0,
     False),
    ("seated, 3 mm off forward (the round pin touches its hole's wall)", {"jig_plate": plate(dx=PLAY)}, 1.0, True),
    ("seated, 3 mm off sideways", {"jig_plate": plate(dy=-PLAY)}, 1.0, True),
    ("seated, 3 mm off both ways", {"jig_plate": plate(dx=-PLAY, dy=PLAY)}, 1.0, True),
    ("3.1 mm off forward", {"jig_plate": plate(dx=PLAY + 0.0001)}, 0.0, False),
    ("3.1 mm off sideways", {"jig_plate": plate(dy=PLAY + 0.0001)}, 0.0, False),
    ("seated, turned 3.5 degrees about the holes' midpoint", {"jig_plate": plate(yaw=183.5)}, 1.0, True),
    ("turned 4 degrees", {"jig_plate": plate(yaw=184.0)}, 0.0, False),
    ("riding on the round pin's top, other end on the fixture", {"jig_plate": riding(1)}, 0.0, False),
    ("riding on the diamond pin's top", {"jig_plate": riding(-1)}, 0.0, False),
    ("plate 1.5 mm up (within tolerance)", {"jig_plate": plate(up=0.0015)}, 1.0, True),
    ("plate held 1.6 mm up", {"jig_plate": plate(up=0.0016)}, 0.0, False),
    ("plate held 5 mm up, pins in the holes", {"jig_plate": plate(up=0.005)}, 0.0, False),
    ("seated but still moving", {"jig_plate": plate(vel=(0.0, 0.0, -0.05))}, 0.0, False),
    ("one long edge on the fixture, the other 1.2 mm up (1 degree)", {"jig_plate": edge_up(1.0)}, 1.0, True),
    ("one long edge on the fixture, the other 2.4 mm up (2 degrees)", {"jig_plate": edge_up(2.0)}, 0.0, False),
    ("plate flat on the table in front of the fixture",
     {"jig_plate": plate(centre=(BASE_X - 0.10, BASE_Y), z=TOP + MID)}, 0.0, False),
    ("plate back on its riser", {"jig_plate": PLATE.state(pos=(RISER_X, RISER_Y, TOP + RISER_SIZE[2] + MID))},
     0.0, False),
    ("plate flat on the fixture beside the pins (none through)", {"jig_plate": plate(dy=0.07, yaw=90.0)}, 0.0,
     False),
    ("plate standing on edge against the fixture",
     {"jig_plate": plate(centre=(BASE_X - 0.04, BASE_Y), z=TOP + 0.035, tilt=90.0, tilt_axis=(0.0, 1.0, 0.0))},
     0.0, False),
    ("fixture nudged 4 mm, plate seated on its pins",
     {"fixture": fixture(dx=0.004), "jig_plate": plate(fix=(0.004, 0.0))}, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("fixture shoved 6 mm, plate seated on its pins",
     {"fixture": fixture(dy=0.006), "jig_plate": plate(fix=(0.0, 0.006))}, 0.0, False),
    ("then the fixture pushed back and the plate seated: still 0", SEATED, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, seated: 1.0", SEATED, 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("fixture turned 4 degrees", {"fixture": fixture(yaw=4.0)}, 0.0, False),
    ("then turned back and the plate seated: still 0", SEATED, 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("fixture turned 2.5 degrees: no fault", {"fixture": fixture(yaw=2.5)}, 0.0, False),
    ("then seated: 1.0", SEATED, 1.0, True),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task

    start = {o.name: (o.state() if hasattr(o, "state") else None) for o in task.OBJECTS}
    start = {k: v for k, v in start.items() if v is not None}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


# Fixed habits, scored in every lot with lots.use(): each must average 0.5 or less over the four lots. Each puts the
# plate flat on the fixture with its holes' midpoint on that lot's pins' midpoint (so only the way round decides),
# except the replays, which repeat lot A's final pose or lot A's motion (the same turn, to the same place).
def _seat_at(lot_pins_mid, yaw):
    import task

    q = quat(math.degrees(yaw))
    off = _r(q) @ np.array([0.0, (HOLES[0] + HOLES[1]) / 2, 0.0])
    return task.PLATE.state(pos=(lot_pins_mid[0] - off[0], lot_pins_mid[1] - off[1], BASE_TOP + MID), quat_xyzw=q)


def _pins_mid(lot):
    fx, fy, _ = lot.pos("fixture")
    a = lot.yaw_of("fixture")
    return np.array([fx, fy]) + np.array([-math.sin(a), math.cos(a)]) * PIN_MID


def _habit_poses(lot, lot_a):
    p0 = lot.yaw_of("jig_plate")
    a_turn = math.pi  # lot A's plate ends at yaw 180 degrees
    return {
        "always the lot-A way round (small hole to the right)": _seat_at(_pins_mid(lot), math.pi),
        "turn anticlockwise until along the bar": _seat_at(_pins_mid(lot), math.pi * math.ceil(p0 / math.pi + 1e-9)),
        "turn clockwise until along the bar": _seat_at(_pins_mid(lot), math.pi * math.floor(p0 / math.pi - 1e-9)),
        "replay lot A's final pose": _seat_at(_pins_mid(lot_a), a_turn),
        "replay lot A's motion (same turn, same place)":
            _seat_at(_pins_mid(lot_a), p0 + a_turn - lot_a.yaw_of("jig_plate")),
        "the right way round (the intended answer)": _seat_at(_pins_mid(lot), lot.yaw_of("fixture") + math.pi),
    }


HABIT_LIMIT = 0.5


def _habits() -> int:
    import lots
    import task

    names = [lot.name for lot in lots.all_lots()]
    lot_a = lots.all_lots()[0]
    rows = {}
    for lot in lots.all_lots():
        lots.use(lot.name)
        fix = task.FIXTURE.state(pos=lot.pos("fixture"), quat_xyzw=quat(math.degrees(lot.yaw_of("fixture"))))
        for label, pl in _habit_poses(lot, lot_a).items():
            task._H.update(fault=False, prev=None)
            rows.setdefault(label, []).append(float(task.score({"fixture": fix, "jig_plate": pl})))
    lots.use(None)
    bad = 0
    for label, sc in rows.items():
        mean = sum(sc) / len(sc)
        want_ok = mean == 1.0 if label.startswith("the right way") else mean <= HABIT_LIMIT + 1e-9
        bad += not want_ok
        per = " ".join(f"{n}={v:.0f}" for n, v in zip(names, sc))
        print(f"{'ok  ' if want_ok else 'FAIL'} mean {mean:4.2f}  ({per})  habit: {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run() + _habits())
