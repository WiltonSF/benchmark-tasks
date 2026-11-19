"""Near misses and unintended strategies for taking the hard tag off the cuff.

The score rests on a whole-trial record (pin travel counted only while the dome sits in the detacher's well, the
let-go, the 1 s still hold, unknown states), so each case drives the scorer's own recorder (`task.record`, which the
simulator calls every control step) through a sequence of tag and cuff poses and hand points, then reads the score.

Poses are built from the task's own kinematics: the tag either seated flat in the well or lying off it, and the cuff
placed so that its pin is drawn a given distance along the tag's pin axis."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402

DT = 0.05
B = task.B
FAR = np.array([[0.05, 0.45, B + 0.30], [0.05, -0.45, B + 0.30]])
FAILS: list[str] = []
I = (0.0, 0.0, 0.0, 1.0)
SEAT = (task.D[0], task.D[1], B)  # the tag flat on the counter, dome in the well
OFF_WELL = (task.D[0] + 0.0, task.D[1] - 0.120, B)  # the tag flat on the counter beside the cuff (not in the well)
SOMEWHERE_UP = (task.D[0], task.D[1], B + 0.040)  # the tag lifted 40 mm out of the well, level


def tag_at(pos, q=I):
    return task.TAG.state(pos=pos, quat_xyzw=q)


def cuff_drawn(tag_st, drawn, aside=(0.0, 0.0, 0.0)):
    """The cuff with its pin drawn ``drawn`` metres out along the tag's axis (cuff level), then moved by ``aside``."""
    r = task._rot(tag_st.quat_xyzw)
    hinge = np.asarray(tag_st.pos, float) + r @ np.array([0.0, 0.0, task.HZ + drawn]) + np.asarray(aside, float)
    return task.CUFF.state(pos=hinge, quat_xyzw=tag_st.quat_xyzw)


def world(tag_pos, drawn, aside=(0.0, 0.0, 0.0), q=I):
    t = tag_at(tag_pos, q)
    return {"counter": task.COUNTER.state(), "tag": t, "cuff": cuff_drawn(t, drawn, aside)}


def run(steps, state, hands=FAR, reset=True):
    if reset:
        task.reset_record()
    for k in range(steps):
        s = state(k) if callable(state) else state
        h = hands(k) if callable(hands) else hands
        task.record(s, h, DT)


def pull(tag_pos, a, b, n=10, hands=FAR, reset=False, aside=(0.0, 0.0, 0.0)):
    """Draw the pin from a to b metres in n steps, the tag at ``tag_pos``."""
    run(n, lambda k: world(tag_pos, a + (b - a) * (k + 1) / n, aside), hands=hands, reset=reset)


AWAY = (0.0, -0.07, -0.025)  # the cuff carried 70 mm aside and set down: pin out and clear
FULL = 0.025


def demo_like(still=24, hands=FAR):
    run(5, world(SEAT, 0.0))  # seated, nothing drawn
    pull(SEAT, 0.0, FULL)  # drawn 25 mm while seated
    run(still, world(SEAT, FULL, AWAY), hands=hands, reset=False)


def near(tag_pos, dz):
    """A hand point dz metres above the tag's top face at its free end (plus the far points)."""
    return np.vstack([FAR, [tag_pos[0], tag_pos[1] + 0.040, tag_pos[2] + task.TAG_H + dz]])


def lower_table(part_name, recover=False):
    """A real resting height on the tabletop outside the raised counter, after valid removal."""
    demo_like()
    states = world(SEAT, FULL, AWAY)
    part = task.TAG if part_name == "tag" else task.CUFF
    previous = states[part_name]
    pos = np.asarray(previous.pos, float).copy()
    pos[0] = 0.46  # the entire part is beyond the raised board's x=0.40 m edge
    pos[2] += task.TOP - float(task.corners(part, previous)[:, 2].min())
    states[part_name] = part.state(pos=tuple(pos), quat_xyzw=previous.quat_xyzw)
    assert abs(float(task.corners(part, states[part_name])[:, 2].min()) - task.TOP) < 1e-10
    run(1 if recover else 24, states, reset=False)
    assert task._REC["failed"] == "tag or cuff off the counter", "table contact must latch the fall itself"
    if recover:
        run(24, world(SEAT, FULL, AWAY), reset=False)


def cutoff(z, failed):
    demo_like()
    states = world(SEAT, FULL, AWAY)
    old = states["tag"]
    pos = np.asarray(old.pos).copy()
    pos[0] = 0.46
    pos[2] += z - task.corners(task.TAG, old)[:, 2].min()
    states["tag"] = tag_at(tuple(pos))
    run(1, states, reset=False)
    assert bool(task._REC["failed"]) is failed
    run(24, world(SEAT, FULL, AWAY), reset=False)


def tilted_world(drawn, angle):
    states = world(SEAT, drawn)
    cuff = states["cuff"]
    states["cuff"] = task.CUFF.state(joints={"hem": angle}, pos=cuff.pos, quat_xyzw=I)
    return states


def tilted_tip_only():
    angle = 0.7
    # The tip CENTRE is 1 mm above the 20 mm body top, but its tilted radius/shaft is not 5 mm clear.
    target_hinge_z = B + task.TAG_H + 0.001 - task.PIN_TIP * math.cos(angle)
    drawn = target_hinge_z - (B + task.HZ)
    run(5, world(SEAT, 0.0))
    pull(SEAT, 0.0, 0.008)  # establish genuine partial withdrawal before tilting the exposed pin
    run(10, lambda k: tilted_world(0.008 + (drawn - 0.008) * (k + 1) / 10, angle), reset=False)
    states = tilted_world(drawn, angle)
    assert task.pin_tip(states["cuff"])[2] > B + task.TAG_H + 0.0005
    assert not task.pin_out(states["tag"], states["cuff"])


def reinsert_after_removal():
    demo_like()
    run(5, world(SEAT, FULL), reset=False)
    pull(SEAT, FULL, 0.0)


def finish_visible_removal_outside():
    run(5, world(SEAT, 0.0))
    pull(SEAT, 0.0, task.DEPTH0)
    run(3, world(SOMEWHERE_UP, task.DEPTH0), reset=False)
    pull(SOMEWHERE_UP, task.DEPTH0, FULL)
    run(24, world(SEAT, FULL, AWAY), reset=False)


def approach_without_reinsertion():
    demo_like()
    near_state = world(SEAT, FULL, (0.0, -0.0264, -0.025))
    assert abs(task.whole_pin_gap(near_state["tag"], near_state["cuff"]) - 0.0049) < 1e-9
    run(1, near_state, reset=False)
    run(24, world(SEAT, FULL, AWAY), reset=False)


def approach_at_gap(gap):
    demo_like()
    # Tag's front edge is -14 mm; the pin head has a 7.5 mm radius.
    state = world(SEAT, FULL, (0.0, -0.0215 - gap, -0.025))
    assert not task.pin_in_channel(state["tag"], state["cuff"])
    run(1, state, reset=False)
    run(24, world(SEAT, FULL, AWAY), reset=False)


def bad_removal_then_free_approach():
    finish_visible_removal_outside()
    state = world(SEAT, FULL, (0.0, -0.0264, -0.025))
    run(1, state, reset=False)
    run(24, world(SEAT, FULL, AWAY), reset=False)


def bad_removal_then_genuine_retry():
    finish_visible_removal_outside()
    run(1, world(SEAT, FULL), reset=False)
    pull(SEAT, FULL, 0.0)
    pull(SEAT, 0.0, FULL)
    run(24, world(SEAT, FULL, AWAY), reset=False)



def preclearance_exterior_launder():
    # A shaft exits outside engagement but never reaches the 5 mm display-clearance flag.
    # Free lowering beside the tag must not reset its extraction reference or permit a new false pull.
    run(5, world(SEAT, 0.0))
    run(3, world(SOMEWHERE_UP, 0.0), reset=False)
    pull(SOMEWHERE_UP, 0.0, 0.017)
    for state in (world(SOMEWHERE_UP, 0.017, (0.0286, 0.0, 0.0)),
                  world(SOMEWHERE_UP, 0.0, (0.0286, 0.0, 0.0)),
                  world(SEAT, 0.0, (0.0286, 0.0, 0.0))):
        assert not task.pin_out(state["tag"], state["cuff"])
        assert not task.pin_in_channel(state["tag"], state["cuff"])
        run(3, state, reset=False)
    pull(SEAT, 0.0, FULL, aside=(0.0286, 0.0, 0.0))
    run(24, world(SEAT, FULL, AWAY), reset=False)


def valid_exit_free_approach_before_clearance():
    run(5, world(SEAT, 0.0))
    pull(SEAT, 0.0, 0.017)
    run(1, world(SEAT, 0.017, (0.0286, 0.0, 0.0)), reset=False)
    run(1, world(SEAT, 0.0, (0.0286, 0.0, 0.0)), reset=False)
    pull(SEAT, 0.0, FULL, aside=(0.0286, 0.0, 0.0))
    run(24, world(SEAT, FULL, AWAY), reset=False)



def angled_complete_extraction():
    # A physically channel-fitting 40-degree tilt can fully extract the shaft with <16 mm head rise.
    # The readable whole-pin gap, rather than a concealed straight-line depth, proves final removal.
    run(5, world(SEAT, 0.0))
    pull(SEAT, 0.0, 0.015)
    def tilted_at_tip(angle, tip_z):
        rot = task._rot(task._qx(angle))
        tip = np.array([SEAT[0], SEAT[1], tip_z])
        hinge = tip - rot @ np.array([0.0, 0.0, task.PIN_TIP])
        state = world(SEAT, 0.0)
        state["cuff"] = task.CUFF.state(pos=hinge, joints={"hem": angle}, quat_xyzw=I)
        return state
    run(14, lambda k: tilted_at_tip(0.7 * (k + 1) / 14, B + 0.019), reset=False)
    run(20, lambda k: tilted_at_tip(0.7, B + 0.019 + 0.015 * (k + 1) / 20), reset=False)
    run(24, world(SEAT, FULL, AWAY), reset=False)


def diagonal_motion():
    demo_like()
    states = world(SEAT, FULL, AWAY)
    delta = np.array([0.0019, 0.0019, 0.0])
    states["tag"] = tag_at(np.asarray(states["tag"].pos) + delta)
    states["cuff"] = task.CUFF.state(pos=np.asarray(states["cuff"].pos) + delta, quat_xyzw=I)
    run(1, states, reset=False)


def case(label, setup, want):
    setup()
    got = task.score(None)
    good = abs(got - want) <= 1e-9
    if not good:
        FAILS.append(f"{label}: got {got}, want {want}")
    print(f"{'ok  ' if good else 'FAIL'} {label}: {got}")


HIST_CASES = [
    ("start layout, nothing done", lambda: run(30, {"counter": task.COUNTER.state(), "tag": task.TAG.state(),
                                                    "cuff": task.CUFF.state()}), 0.0),
    ("tag seated, pin drawn 25 mm in the well, cuff set aside, hands clear, still 1.2 s", lambda: demo_like(), 1.0),
    ("the same, still only 0.5 s", lambda: demo_like(still=10), 0.75),
    ("the same, a hand 3 mm over the tag", lambda: demo_like(hands=near(SEAT, 0.003)), 0.75),
    ("the same, a hand 6 mm over the tag", lambda: demo_like(hands=near(SEAT, 0.006)), 1.0),
    ("pin out but the cuff still held (hand 2 mm off the cuff)", lambda: demo_like(
        hands=np.vstack([FAR, [SEAT[0], SEAT[1] - 0.07 - 0.052, B - 0.025 + 0.0235 + 0.002]])), 0.5),
    ("whole pin 5 mm above the visible tag rim, cuff still raised", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, FULL),
        run(24, world(SEAT, task.DEPTH0 + 0.0005 + 0.005), reset=False)),
     0.5),
    ("whole pin 10 mm clear but cuff suspended: no returned-cuff tier", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, FULL),
        run(24, world(SEAT, task.DEPTH0 + 0.0005 + task.CLEAR_TAG), reset=False)), 0.5),
    ("drawn 4.9 mm in the well, then stopped", lambda: (run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.0049)), 0.0),
    ("drawn 5.0 mm in the well, then stopped", lambda: (run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.005)), 0.25),
    ("drawn 15.4 mm in the well: not out", lambda: (run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.0154)), 0.25),
    ("pull without the detacher: tag off the well, pin drawn all the way", lambda: (
        run(5, world(OFF_WELL, 0.0)), pull(OFF_WELL, 0.0, FULL),
        run(24, world(OFF_WELL, FULL, AWAY), reset=False)), 0.0),
    ("late pull: seated, the tag lifted out of the well, then the pin drawn", lambda: (
        run(5, world(SEAT, 0.0)), run(5, world(SOMEWHERE_UP, 0.0), reset=False), pull(SOMEWHERE_UP, 0.0, FULL),
        run(24, world(SOMEWHERE_UP, FULL, AWAY), reset=False)), 0.0),
    ("late pull, then the tag put back in the well with the pin already out", lambda: (
        run(5, world(SEAT, 0.0)), run(5, world(OFF_WELL, 0.0), reset=False), pull(OFF_WELL, 0.0, FULL),
        run(5, world(SEAT, FULL), reset=False), run(24, world(SEAT, FULL, AWAY), reset=False)), 0.0),
    ("drawn 8 mm in the well, the tag taken out, the rest drawn outside", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.008), run(3, world(SOMEWHERE_UP, 0.008), reset=False),
        pull(SOMEWHERE_UP, 0.008, FULL), run(24, world(SOMEWHERE_UP, FULL, AWAY), reset=False)), 0.25),
    ("drawn 8 mm in the well, out, reseated and finished in the well", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.008), run(3, world(SOMEWHERE_UP, 0.008), reset=False),
        run(3, world(SEAT, 0.008), reset=False), pull(SEAT, 0.008, FULL),
        run(24, world(SEAT, FULL, AWAY), reset=False)), 1.0),
    ("drawn 10 mm in the well, then pushed 8 mm back in", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.010), pull(SEAT, 0.010, 0.002)), 0.0),
    ("drawn 10 mm in the well, out of the well, pushed back 6 mm", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, 0.010), run(2, world(SOMEWHERE_UP, 0.010), reset=False),
        pull(SOMEWHERE_UP, 0.010, 0.004)), 0.0),
    ("dome 4.5 mm off the well's axis (limit): counts as seated", lambda: (
        run(5, world((SEAT[0] + 0.0045, SEAT[1], B), 0.0)), pull((SEAT[0] + 0.0045, SEAT[1], B), 0.0, FULL),
        run(24, world((SEAT[0] + 0.0045, SEAT[1], B), FULL, AWAY), reset=False)), 1.0),
    ("dome 4.6 mm off the well's axis: not seated", lambda: (
        run(5, world((SEAT[0] + 0.0046, SEAT[1], B), 0.0)), pull((SEAT[0] + 0.0046, SEAT[1], B), 0.0, FULL),
        run(24, world((SEAT[0] + 0.0046, SEAT[1], B), FULL, AWAY), reset=False)), 0.0),
    ("tag leaning 9 degrees over the well, pin drawn: not seated", lambda: run(
        10, lambda k: world(SEAT, FULL * (k + 1) / 10, q=task._qx(math.radians(9)))), 0.0),
    ("full run, but the tag knocked off the counter once", lambda: (
        demo_like(), run(1, world((task.D[0], task.D[1], B - 0.05), FULL, AWAY), reset=False),
        run(24, world(SEAT, FULL, AWAY), reset=False)), 0.0),
    ("removed tag rests on lower tabletop outside the counter", lambda: lower_table("tag"), 0.0),
    ("removed cuff rests on lower tabletop outside the counter", lambda: lower_table("cuff"), 0.0),
    ("tag touches lower tabletop then returns to counter: fall stays latched",
     lambda: lower_table("tag", recover=True), 0.0),
    ("cuff touches lower tabletop then returns to counter: fall stays latched",
     lambda: lower_table("cuff", recover=True), 0.0),
    ("fall cutoff: tag 0.1 mm below fails after recovery", lambda: cutoff(task.OFF - 0.0001, True), 0.0),
    ("fall cutoff: exact limit survives after return", lambda: cutoff(task.OFF, False), 1.0),
    ("fall cutoff: tag 0.1 mm above survives after return", lambda: cutoff(task.OFF + 0.0001, False), 1.0),
    ("sideways buried tip beyond old 3 mm axis gate creates no withdrawal", lambda: (
        run(5, world(SEAT, 0.0)), run(24, world(SEAT, 0.0, (0.0031, 0.0, 0.0)), reset=False)), 0.0),
    ("hem rotation +40 degrees alone creates no withdrawal", lambda: run(24, tilted_world(0.0, 0.7)), 0.0),
    ("hem rotation -40 degrees alone creates no withdrawal", lambda: run(24, tilted_world(0.0, -0.7)), 0.0),
    ("tip centre above tag but whole tilted shaft not clear is not removal", tilted_tip_only, 0.25),
    ("whole pin 4.9 mm clear of visible rim is not removal", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, task.DEPTH0 + 0.0005 + 0.0049)), 0.25),
    ("whole pin 5.0 mm clear of visible rim is complete removal", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, task.DEPTH0 + 0.0005 + 0.005)), 0.5),
    ("whole pin 5.1 mm clear of visible rim is complete removal", lambda: (
        run(5, world(SEAT, 0.0)), pull(SEAT, 0.0, task.DEPTH0 + 0.0005 + 0.0051)), 0.5),
    ("already-removed initial pin cannot manufacture extraction history",
     lambda: run(30, world(SEAT, FULL, AWAY)), 0.0),
    ("genuine extraction followed by ordinary lowering aside retains completion", lambda: demo_like(), 1.0),
    ("actual reinsertion after removal revokes completion", reinsert_after_removal, 0.0),
    ("16 mm seated then complete removal outside and reseat cannot earn removal tier",
     finish_visible_removal_outside, 0.25),
    ("separated 4.9 mm approach and retreat preserves prior valid removal",
     approach_without_reinsertion, 1.0),
    ("separated 0.1 mm approach and retreat is not reinsertion", lambda: approach_at_gap(0.0001), 1.0),
    ("pin-head touch on outside edge is not shaft reinsertion", lambda: approach_at_gap(0.0), 1.0),
    ("bad outside removal cannot be laundered by a free near approach", bad_removal_then_free_approach, 0.25),
    ("fully reinserted then genuinely extracted in the well can retry", bad_removal_then_genuine_retry, 1.0),
    ("outside shaft exit before5mm then exterior lowering/reseat cannot create new withdrawal",
     preclearance_exterior_launder, 0.0),
    ("valid seated shaft exit survives free exterior lowering before first5mm clearance",
     valid_exit_free_approach_before_clearance, 1.0),
    ("genuine channel-fitting angled extraction is complete despite head rise below16mm",
     angled_complete_extraction, 1.0),
    ("1.9 mm plus 1.9 mm diagonal corner motion exceeds 2 mm Euclidean stillness",
     diagonal_motion, 0.75),
    ("cuff returned but tag still suspended cannot earn full score", lambda: (
        demo_like(), run(24, {"counter": task.COUNTER.state(), "tag": tag_at((task.D[0], task.D[1], B + 0.03)),
                             "cuff": world(SEAT, FULL, AWAY)["cuff"]}, reset=False)), 0.75),
    ("hand state unknown for one step", lambda: (
        demo_like(), run(1, world(SEAT, FULL, AWAY), hands=None, reset=False)), 0.0),
    ("object state unknown for one step", lambda: (demo_like(), run(1, None, reset=False)), 0.0),
]

for _label, _setup, _want in HIST_CASES:
    case(_label, _setup, _want)
print(f"{len(HIST_CASES) - len(FAILS)}/{len(HIST_CASES)} as expected")
if FAILS:
    raise SystemExit(f"{len(FAILS)} near-miss case(s) failed: {FAILS}")

demo_like()  # leave a valid full-score history for check.py's end-state case
CASES = [
    ("end state of a valid history: pin out in the well, cuff aside, hands clear, still", {}, 1.0, True),
]
