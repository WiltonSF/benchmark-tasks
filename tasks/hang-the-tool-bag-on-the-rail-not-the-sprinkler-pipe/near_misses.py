"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py), in lot A
(pipe nearest the bag, rail to the right). Each case: (label, {object: spec}, expected score, expected success); a
spec is a ready state from the part's own geometry (BAG.state(...)). Cases run in order and share the whole-trial
record, so a fault stays until a scene reset (an empty case after the bag was moved).
"""

import math

from task import BAG, BAR_TOP, BAR_UNDER, FAR_SPOT, NEAR_SPOT, PIPE, POST_REACH, TOP

RX, RY, _ = FAR_SPOT  # the rail in lot A
PX, PY, _ = NEAR_SPOT  # the pipe in lot A


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def bag_at(x, y, gap=0.0, quat=None, vel=None):
    """The bag with its crossbar's underside `gap` above a bar's top at (x, y)."""
    return {"tool_bag": BAG.state(pos=(x, y, TOP + BAR_TOP - BAR_UNDER + gap), quat_xyzw=quat, lin_vel=vel)}


ON_RAIL = bag_at(RX, RY)
SCENE_RESET = {}

CASES = [
    ("hung on the rail, centred", ON_RAIL, 1.0, True),
    ("hung on the rail, 3 mm gap (still settling)", bag_at(RX, RY, gap=0.003), 1.0, True),
    ("held 4 mm above the rail", bag_at(RX, RY, gap=0.004), 0.0, False),
    ("hung on the rail near a post", bag_at(RX, RY + 0.060), 1.0, True),
    ("hung on the rail at the 66 mm post limit", bag_at(RX, RY + POST_REACH), 1.0, True),
    ("1 mm past the 66 mm post limit", bag_at(RX, RY + POST_REACH + 0.001), 0.0, False),
    ("past the rail's post", bag_at(RX, RY + 0.085), 0.0, False),
    ("rail at the opening's edge (hook 20 mm off)", bag_at(RX - 0.020, RY), 1.0, True),
    ("rail outside the opening (hook 21 mm off)", bag_at(RX - 0.021, RY), 0.0, False),
    ("hung on the rail, swinging", bag_at(RX, RY, vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("bag on the table at its start", {}, 0.0, False),
    ("bag set on the rail stand's base plate", {"tool_bag": BAG.state(pos=(RX + 0.06, RY, TOP))}, 0.0, False),
    ("bag lying on the table beside the rail", {"tool_bag": BAG.state(pos=(RX - 0.08, RY, TOP))}, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("hung on the sprinkler pipe, beside the head", bag_at(PX, PY - 0.045), 0.0, False),
    ("then moved to the rail: still 0", ON_RAIL, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, hung on the rail: 1.0", ON_RAIL, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("hook resting on the pipe for a moment while held", bag_at(PX, PY + 0.045, gap=0.002), 0.0, False),
    ("then hung on the rail: still 0", ON_RAIL, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("bag pushed against the sprinkler head", {"tool_bag": BAG.state(pos=(PX - 0.04, PY, TOP + 0.040))}, 0.0,
     False),
    ("then hung on the rail: still 0", ON_RAIL, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("bag carried past the pipe 4 mm over its top: no fault", bag_at(PX, PY - 0.045, gap=0.004), 0.0, False),
    ("then hung on the rail: 1.0", ON_RAIL, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("bag lifted 6 mm, its body touching the pipe's side", {"tool_bag": BAG.state(pos=(PX - 0.048, PY - 0.05,
                                                                                      TOP + 0.075))}, 0.0, False),
    ("then hung on the rail: still 0", ON_RAIL, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("bag's body 3 mm in front of the deflector, not touching: no fault",
     {"tool_bag": BAG.state(pos=(PX - 0.016 - 0.040 - 0.003, PY, TOP + 0.040))}, 0.0, False),
    ("then hung on the rail: 1.0", ON_RAIL, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("pipe stand shoved 11 mm", {"sprinkler_pipe": PIPE.state(pos=(PX - 0.011, PY, TOP))}, 0.0, False),
    ("then hung on the rail: still 0", ON_RAIL, 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("pipe stand nudged 9 mm, then hung on the rail: 1.0",
     {"sprinkler_pipe": PIPE.state(pos=(PX - 0.009, PY, TOP)), **ON_RAIL}, 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("pipe stand tipped 6 degrees", {"sprinkler_pipe": PIPE.state(pos=(PX, PY, TOP + 0.002), quat_xyzw=tilt_x(6))},
     0.0, False),
    ("then hung on the rail: still 0", ON_RAIL, 0.0, False),
]
