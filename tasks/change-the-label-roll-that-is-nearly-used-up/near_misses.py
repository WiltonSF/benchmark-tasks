"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A (the
nearly used-up roll on the left printer, the spare in front of it, the bin in front of the centre printer). Each case:
(label, {object: spec}, expected score, expected success); a spec is a ready state from the part's own geometry
(ROLL.state(...)). Cases run in order and share the whole-trial record, so a fault stays until a scene reset (an empty
case after a roll was moved). Habits in lots B and C are checked in habit_check() below (run this file).
"""

import math

from task import BIN, BIN_FLOOR, BIN_IN, FULL_A, FULL_B, LOW, ROLL_H, SPARE, TOP, cup, front

L, C, R = cup("left"), cup("centre"), cup("right")
BX, BY, BZ = front("centre")  # the bin in lot A


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def spare_at(x, y, z, quat=None, vel=None):
    return {"spare_roll": SPARE.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)}


def low_at(x, y, z, quat=None):
    return {"low_roll": LOW.state(pos=(x, y, z), quat_xyzw=quat)}


SEATED = spare_at(*L)
IN_BIN = low_at(BX, BY, TOP + BIN_FLOOR)
LOW_ON_TABLE = low_at(0.20, 0.30, TOP)
LOW_SIDE_IN_BIN = low_at(BX, BY - ROLL_H / 2, TOP + BIN_FLOOR + 0.025, quat=tilt_x(-90))
LOW_SIDE_ON_TABLE = low_at(0.22, 0.30, TOP + 0.025, quat=tilt_x(-90))
DONE = {**SEATED, **IN_BIN}
SCENE_RESET = {}

CASES = [
    ("changed: spare on the left spindle, old roll in the bin", DONE, 1.0, True),
    ("old roll lying on its side in the bin", {**SEATED, **LOW_SIDE_IN_BIN}, 1.0, True),
    ("bin nudged 20 mm, old roll still in it",
     {**SEATED, "grey_bin": BIN.state(pos=(BX + 0.02, BY, BZ)), **low_at(BX + 0.02, BY, TOP + BIN_FLOOR)}, 1.0, True),
    ("spare 1 mm above the deck (exact limit)", {**spare_at(L[0], L[1], L[2] + 0.001), **IN_BIN}, 1.0, True),
    ("spare 1.5 mm above the deck: held or caught", {**spare_at(L[0], L[1], L[2] + 0.0015), **IN_BIN}, 0.0, False),
    ("spare 3 mm off the spindle's axis (against it)", {**spare_at(L[0] + 0.003, L[1], L[2]), **IN_BIN}, 1.0, True),
    ("spare 4 mm off axis (exact limit)", {**spare_at(L[0], L[1] - 0.004, L[2]), **IN_BIN}, 1.0, True),
    ("spare 5 mm off axis: not on the spindle", {**spare_at(L[0] + 0.005, L[1], L[2]), **IN_BIN}, 0.0, False),
    ("spare beside the spindle on the deck", {**spare_at(L[0] - 0.03, L[1], L[2]), **IN_BIN}, 0.0, False),
    ("spare sitting on top of the spindle", {**spare_at(L[0], L[1], L[2] + 0.028), **IN_BIN}, 0.0, False),
    ("spare caught on the spindle, leaning 8 degrees",
     {**spare_at(L[0], L[1], L[2] + 0.043 * math.sin(math.radians(8)), quat=tilt_x(8)), **IN_BIN}, 0.0, False),
    ("spare seated but still moving", {**spare_at(*L, vel=(0.0, 0.0, -0.05)), **IN_BIN}, 0.0, False),
    ("spare seated, old roll set down on the table", {**SEATED, **LOW_ON_TABLE}, 0.5, False),
    ("spare seated, old roll lying on its side on the table", {**SEATED, **LOW_SIDE_ON_TABLE}, 0.5, False),
    ("spare seated, old roll 3 mm up (held over the table)", {**SEATED, **low_at(0.20, 0.30, TOP + 0.003)}, 0.0,
     False),
    ("spare seated, old roll on the bin's rim", {**SEATED, **low_at(BX, BY + BIN_IN / 2 + 0.004, TOP + 0.040)},
     0.0, False),
    ("spare seated, old roll on a printer's deck", {**SEATED, **low_at(C[0] - 0.035, C[1] - 0.02, C[2])}, 0.0,
     False),
    ("spare seated, old roll fallen off the table", {**SEATED, **low_at(0.20, 0.30, TOP - 0.30)}, 0.0, False),
    ("old roll in the bin, spare still on the table", IN_BIN, 0.0, False),
    ("spare stacked on the old roll on its spindle", spare_at(L[0], L[1], L[2] + ROLL_H), 0.0, False),
    ("spare put in the bin, old roll left", spare_at(BX, BY, TOP + BIN_FLOOR), 0.0, False),
    ("nothing done", {}, 0.0, False),
    ("centre roll nudged 3 mm on its spindle, job done",
     {**DONE, "full_roll_a": FULL_A.state(pos=(C[0] + 0.003, C[1], C[2]))}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("right roll lifted 12 mm on its spindle", {"full_roll_b": FULL_B.state(pos=(R[0], R[1], R[2] + 0.012))}, 0.0,
     False),
    ("then put back and the job done: still 0", DONE, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, the job done: 1.0", DONE, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("centre roll lifted 9 mm and dropped back", {"full_roll_a": FULL_A.state(pos=(C[0], C[1], C[2] + 0.009))},
     0.0, False),
    ("  then the job done: 1.0", DONE, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("wrong printer: centre roll binned, spare on the centre spindle",
     {"full_roll_a": FULL_A.state(pos=(BX, BY, TOP + BIN_FLOOR)), **spare_at(*C)}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("all three changed", {**DONE, "full_roll_a": FULL_A.state(pos=(0.2, 0.3, TOP)),
                           "full_roll_b": FULL_B.state(pos=(0.2, -0.3, TOP))}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
]


def habit_check():
    """Each fixed habit, scored in all three lots: every one must average 1/3 or less."""
    import lots
    import task

    def end_state(lot, col):
        """The end state if the robot changes the roll on printer `col`: whatever stands there goes in the bin, the
        spare goes on that spindle."""
        b = lot.pos("grey_bin")
        on_col = next(n for n in ("low_roll", "full_roll_a", "full_roll_b")
                      if abs(lot.pos(n)[1] - cup(col)[1]) < 1e-6)
        part = {"low_roll": LOW, "full_roll_a": FULL_A, "full_roll_b": FULL_B}[on_col]
        st = {o.name: o.state(pos=lot.pos(o.name)) for o in task.OBJECTS if o.mass is not None}
        st.update({o.name: o.state() for o in task.OBJECTS if o.mass is None})
        start = dict(st)
        st[on_col] = part.state(pos=(b[0], b[1], b[2] + BIN_FLOOR))
        st["spare_roll"] = SPARE.state(pos=cup(col))
        return start, st

    def col_of(y):
        return min(("left", "centre", "right"), key=lambda c: abs(cup(c)[1] - y))

    habits = {
        "always left": lambda lot: "left",
        "always centre": lambda lot: "centre",
        "always right": lambda lot: "right",
        "printer behind the spare": lambda lot: col_of(lot.pos("spare_roll")[1]),
        "printer behind the bin": lambda lot: col_of(lot.pos("grey_bin")[1]),
        "printer with nothing in front": lambda lot: ({"left", "centre", "right"} - {
            col_of(lot.pos("spare_roll")[1]), col_of(lot.pos("grey_bin")[1])}).pop(),
        "the thin roll (intended)": lambda lot: lot.low,
    }
    for name, pick in habits.items():
        scores = []
        for lot in lots.all_lots():
            lots.use(lot.name)
            task._record.update(fault=False, prev=None)
            start, end = end_state(lot, pick(lot))
            task.score(start)
            scores.append(task.score(end))
        print(f"{name:32s} {scores} mean {sum(scores) / len(scores):.2f}")
    lots.use(None)


if __name__ == "__main__":
    habit_check()
