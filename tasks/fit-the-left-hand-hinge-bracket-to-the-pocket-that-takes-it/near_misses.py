"""Near-successes, fixed habits and unintended strategies, scored offline by the factory checker.

CASES are scored in lot A (left-hand bracket at the middle tray place), as factory/check.py does. A wrong-hand or
wrong-way bracket in a pocket rests on the peg; it is built here 6 mm proud (level, and tilted), as the simulator
would leave it. The release check (no gripper within 5 mm) runs only in the simulator and counts as met here.

Habits across lots (habit_check, run on import, so check.py fails if it breaks): for every fixed rule "the bracket
at tray place i goes to the left pocket, the one at place j to the right" (i != j), the end state that rule leaves
is built in each lot (a right-hand bracket in the left pocket, or the left-hand one in the right pocket, stands
proud on the peg) and scored; none may average more than 1/3.
"""

import itertools
import math

import lots
from task import BASE, BRACKETS, FLOOR, FRAME_X, PEG, POCKET, POCKET_Y, TRAY_Y

B = BRACKETS
SEAT_Z = FLOOR + BASE[2] / 2


def q(yaw_deg=0.0, tilt_deg=0.0):
    """Turned yaw about z, then tilted about the bracket's own y axis."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    ct, st = math.cos(math.radians(tilt_deg) / 2), math.sin(math.radians(tilt_deg) / 2)
    # q_yaw * q_tilt(y)
    return (-sy * st, cy * st, sy * ct, cy * ct)


def seated(name, side, dz=0.0, dx=0.0, dy=0.0, yaw=0.0, tilt=0.0, vel=None):
    return B[name].state(pos=(FRAME_X + dx, POCKET_Y[side] + dy, SEAT_Z + dz), quat_xyzw=q(yaw, tilt), lin_vel=vel)


def proud(name, side, yaw=0.0, tilt=0.0):
    """On the peg: base underside 6 mm up."""
    return seated(name, side, dz=PEG[2], yaw=yaw, tilt=tilt)


L, R1, R2 = "bracket_l", "bracket_r1", "bracket_r2"
CASES = [
    ("both pockets: left-hand left, right-hand right", {L: seated(L, "left"), R2: seated(R2, "right")}, 1.0, True),
    ("the other right-hand bracket in the right pocket", {L: seated(L, "left"), R1: seated(R1, "right")}, 1.0, True),
    ("left-hand seated 1.0 mm high", {L: seated(L, "left", dz=0.001), R2: seated(R2, "right")}, 1.0, True),
    ("left-hand seated 1.1 mm high (right pocket done)",
     {L: seated(L, "left", dz=0.0011), R2: seated(R2, "right")}, 0.0, False),
    ("left-hand shifted 4 mm toward the robot in its pocket (the clearance)",
     {L: seated(L, "left", dx=-0.004), R2: seated(R2, "right")}, 1.0, True),
    ("left-hand tilted 2.5 deg", {L: seated(L, "left", tilt=2.5), R2: seated(R2, "right")}, 0.0, False),
    ("both done but a bracket still moving",
     {L: seated(L, "left"), R2: seated(R2, "right", vel=(0.0, 0.0, -0.02))}, 0.0, False),
    ("only the left pocket: left-hand seated", {L: seated(L, "left")}, 0.5, False),
    ("left pocket seated, a right-hand bracket turned round resting proud on the right pocket's peg",
     {L: seated(L, "left"), R2: proud(R2, "right", yaw=180, tilt=7)}, 0.5, False),
    ("only the right pocket: right-hand seated (the easy one)", {R2: seated(R2, "right")}, 0.0, False),
    ("habit: right-hand bracket into the left pocket, on its peg",
     {R1: proud(R1, "left"), R2: seated(R2, "right")}, 0.0, False),
    ("habit: right-hand bracket into the left pocket, turned round, on its peg, tilted",
     {R1: proud(R1, "left", yaw=180, tilt=7), R2: seated(R2, "right")}, 0.0, False),
    ("left-hand turned round in the left pocket, on its peg", {L: proud(L, "left", yaw=180), R2: seated(R2, "right")},
     0.0, False),
    ("left-hand into the right pocket, on its peg; right-hand into the left, on its peg",
     {L: proud(L, "right"), R1: proud(R1, "left")}, 0.0, False),
    ("impossible offline state: right-hand flat in the left pocket through the peg",
     {R1: seated(R1, "left"), R2: seated(R2, "right")}, 0.0, False),
    ("impossible offline state: left-hand turned round, flat through the peg",
     {L: seated(L, "left", yaw=180), R2: seated(R2, "right")}, 0.0, False),
    ("both on the frame top, not in the pockets",
     {L: seated(L, "left", dz=POCKET[2]), R2: seated(R2, "right", dz=POCKET[2])}, 0.0, False),
    ("do nothing", {}, 0.0, False),
]


def habit_check():
    """Score every fixed place rule in each lot. Returns [(label, [score per lot], mean)], worst first."""
    import task

    out = []
    try:
        for i, j in itertools.permutations(range(3), 2):
            scores = []
            for lot in lots.all_lots():
                lots.use(lot.name)
                at = {round(lot.pos(n)[1], 3): n for n in BRACKETS}
                left_n, right_n = at[round(TRAY_Y[i], 3)], at[round(TRAY_Y[j], 3)]
                st = {o.name: o.state() for o in task.OBJECTS}
                for n, side in ((left_n, "left"), (right_n, "right")):
                    hand = task.HANDS[n]
                    st[n] = seated(n, side) if hand == side else proud(n, side)
                scores.append(1.0 if task.success(st) else float(task.score(st)))
            out.append((f"place {i} -> left pocket, place {j} -> right pocket", scores, sum(scores) / len(scores)))
    finally:
        lots.use(None)
    return sorted(out, key=lambda r: -r[2])


_RESULTS = habit_check()
for _label, _s, _m in _RESULTS:
    assert _m <= 1 / 3 + 1e-9, f"habit '{_label}' averages {_m:.2f} over the lots: {_s}"

if __name__ == "__main__":
    for label, s, m in _RESULTS:
        print(f"{m:.2f}  {s}  {label}")
