"""Near-successes and wrong outcomes, scored offline by the factory checker.

Each case: (label, {object: ObjectState}, expected score, expected success). The valve's joint "handle" is how far
the handle has been turned closed from fully open (rad, clockwise seen from above); the closed stop is at 1.5 turns.
"""

import math

from task import VALVE

TURN = 2 * math.pi
DEG = math.radians(1)


def at(a):
    return {"valve": VALVE.state(joints={"handle": a})}


CASES = [
    ("at the closed stop", at(1.5 * TURN), 1.0, True),
    ("4 degrees short of the stop", at(1.5 * TURN - 4 * DEG), 1.0, True),
    ("exactly 5 degrees short", at(1.5 * TURN - 5 * DEG), 1.0, True),
    ("6 degrees short", at(1.5 * TURN - 6 * DEG), 0.5, False),
    ("half a turn short (red knob at the mark, one turn done)", at(1.0 * TURN), 0.5, False),
    ("just under one turn", at(TURN - DEG), 0.0, False),
    ("three quarters of a turn", at(0.75 * TURN), 0.0, False),
    ("just under three quarters", at(0.75 * TURN - DEG), 0.0, False),
    ("half a turn (red knob also at the mark)", at(0.5 * TURN), 0.0, False),
    ("one stroke: a quarter turn", at(0.25 * TURN), 0.0, False),
    ("untouched, fully open", at(0.0), 0.0, False),
    # Fixed habits: turn it the other way ("open"), which the back stop prevents, so the handle stays fully open;
    # stop when the red knob first reaches the white tab (half a turn); one long sweep, which the pipe rack limits to
    # under about 210 degrees; push until the arm stalls, which leaves it at the stop (the intended outcome).
    ("habit: turned anticlockwise, held at the open stop", at(0.0), 0.0, False),
    ("habit: stopped when the red knob first reaches the tab", at(0.5 * TURN), 0.0, False),
    ("habit: one long sweep of 210 degrees", at(math.radians(210)), 0.0, False),
]


if __name__ == "__main__":
    import task

    bad = 0
    for label, s, want, want_ok in CASES:
        ok = bool(task.success(s))
        got = 1.0 if ok else float(task.score(s))
        flag = "ok  " if (got, ok) == (want, want_ok) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    raise SystemExit(bad)
