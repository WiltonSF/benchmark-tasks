"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py). Each case:
(label, {object: ready state}, expected score, expected success). The door is fixed; only its joints change.
"""

from task import DOOR, KEY_BLOCKED


def door(handle=0.0, key=0.0, hooks=None):
    h = 0.030 * max(0.0, min(1.0, handle / 0.7)) if hooks is None else hooks
    return {"door": DOOR.state(joints={"handle": handle, "key": key, "hooks": h})}


CASES = [
    ("locked: handle in its upper notch, key at its stop", door(0.7, 1.6), 1.0, True),
    ("handle at 0.5 rad (limit), key at 1.4 rad (limit)", door(0.5, 1.4), 1.0, True),
    ("key 1.39 rad: not home", door(0.7, 1.39), 0.25, False),
    ("handle in its notch, key not turned", door(0.7, 0.0), 0.25, False),
    ("handle at 0.65 rad (notch limit), key not turned", door(0.65, 0.0), 0.25, False),
    ("handle brushed up to 0.64 rad, not in its notch", door(0.64, 0.0), 0.0, False),
    ("handle lifted, key half turned", door(0.7, 0.8), 0.25, False),
    ("handle 0.49 rad, key not turned", door(0.49, 0.0), 0.0, False),
    ("handle pressed down (the opening move)", door(-0.6, 0.0), 0.0, False),
    ("key tried with the handle down: stopped at its interlock", door(0.0, KEY_BLOCKED), 0.0, False),
    ("handle lifted a little, key at its interlock", door(0.3, KEY_BLOCKED), 0.0, False),
    ("handle level, key at its stop (cannot happen: the key is blocked)", door(0.0, 1.6), 0.0, False),
]
