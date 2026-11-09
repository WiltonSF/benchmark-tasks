"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A (the
stored maximum in the yellow band). Each case: (label, {object: ready state}, expected score, expected success).
Cases run in order and share the whole-trial record, so a fault stays until a scene reset (an empty case after the
pointer was moved). Offline there is no robot, so "let go" and "still" hold in every case.
"""

from task import BAND, BAND_NAMES, LOG, LOG_SCALE_TOP, POINTER_Y0, PRESSED, RESET_TOL, THERMO, _band_y, _marker_q

STORED = _marker_q("yellow")  # lot A's marker travel


def ptr_in(band, off=0.0):
    """Pointer travel putting the tab's centre `off` from the middle of `band` (+ toward the red end)."""
    return POINTER_Y0 - _band_y(BAND_NAMES.index(band), LOG_SCALE_TOP) + off


def case(ptr=0.0, marker=STORED, button=0.0):
    return {"log_board": LOG.state(joints={"pointer": ptr}),
            "thermometer": THERMO.state(joints={"marker": marker, "reset": button})}


YELLOW = ptr_in("yellow")
RESET = {}  # every object at its start: with the jump below, a scene reset

CASES = [
    ("start: nothing done", case(), 0.0, False),
    ("pointer in yellow, not reset", case(YELLOW), 0.5, False),
    ("pointer in yellow, 13.9 mm toward orange, not reset", case(ptr_in("yellow", BAND / 2 - 0.0001)), 0.5, False),
    ("pointer at the yellow/orange edge, not reset", case(ptr_in("yellow", BAND / 2)), 0.5, False),
    ("pointer 0.1 mm into orange, not reset", case(ptr_in("yellow", BAND / 2 + 0.0001)), 0.0, False),
    ("pointer in green, not reset", case(ptr_in("green")), 0.0, False),
    ("pointer in yellow, RESET pressed 5 mm: the log came first", case(YELLOW, marker=0.0, button=PRESSED), 1.0,
     True),
    ("then let up: logged and reset", case(YELLOW, marker=0.0), 1.0, True),
    ("marker 3 mm from the index: reset", case(YELLOW, marker=RESET_TOL), 1.0, True),
    ("marker 3.1 mm from the index: not reset", case(YELLOW, marker=RESET_TOL + 0.0001), 0.5, False),
    ("then the pointer moved on to orange: 0", case(ptr_in("orange"), marker=0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("RESET pressed first, pointer in the blank zone", case(0.0, marker=0.0, button=PRESSED), 0.0, False),
    ("then the pointer set to yellow: still 0", case(YELLOW, marker=0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("after the reset, logged then reset: 1.0", case(YELLOW, marker=0.0, button=0.006), 1.0, True),
    ("scene reset", RESET, 0.0, False),
    ("RESET pressed with the pointer in green (the present reading)", case(ptr_in("green"), marker=0.0,
                                                                            button=0.006), 0.0, False),
    ("then corrected to yellow: still 0", case(YELLOW, marker=0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("RESET 4.9 mm down with the pointer in the blank zone: not a reset", case(0.0, button=PRESSED - 0.0001), 0.0,
     False),
    ("then logged to yellow: 0.5, no fault", case(YELLOW), 0.5, False),
    ("scene reset", RESET, 0.0, False),
    ("RESET pressed with the pointer passing through blue on its way", case(ptr_in("blue"), marker=0.0,
                                                                             button=0.006), 0.0, False),
    ("then on to yellow: still 0", case(YELLOW, marker=0.0), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("pointer pushed to red (always the top), reset", case(ptr_in("red"), marker=0.0, button=0.006), 0.0, False),
    ("scene reset", RESET, 0.0, False),
    ("marker at the index after release with the pointer in yellow (pushed by hand, its spring returns it; only RESET keeps it there)",
     case(YELLOW, marker=0.0), 1.0, True),
]
