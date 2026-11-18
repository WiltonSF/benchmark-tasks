"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case also
writes into the task's per-trial record what the step callback would have measured: the closest gripper face to the
paper and guides, whether everything was still for 1 s with both latches let go, and whether the arms were readable.
A case is a function of that record, so the checker's single scoring call per case sees the right one.
"""

import math

import numpy as np

import lots
import task
from task import A4, BUNDLES, CASSETTE, FLOOR, LEGAL, T, TOP, WIDE, XC, YF

lots.use("A")
W_SNUG = (WIDE - A4[0]) / 2  # each width guide's travel to touch an A4 stack centred in the tray
L_SNUG = LEGAL - A4[1]


def _q(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


class Case(dict):
    """A state dict that loads its record into the task when the checker reads it."""

    def __init__(self, states, rec):
        super().__init__(states)
        self.rec = rec

    def items(self):
        _load(self.rec)
        return super().items()

    def __iter__(self):
        _load(self.rec)
        return super().__iter__()


def _load(rec):
    task.fresh()
    task._rec.update(rec)


def state(width=0.0, length=0.0, press=(0.0005, 0.0005, 0.0005), latches=(0.0, 0.0), bundles=None, gap=0.05,
          held=True, blind=False):
    """Guides set so each pad face is `width`/`length` m off an A4 stack squared in the feed corner, pads pressed
    `press` (near, far, length); bundle overrides as {index: (dx, dy, dz, yaw_deg, tilt_deg)}."""
    pn, pf, pl = press
    joints = {"width_near": W_SNUG - width + pn, "width_far": W_SNUG - width + pf, "length": L_SNUG - length + pl,
              "pad_near": pn, "pad_far": pf, "pad_length": pl, "latch_width": latches[0], "latch_length": latches[1]}
    out = {"cassette": CASSETTE.state(joints=joints)}
    for i, b in enumerate(BUNDLES):
        dx, dy, dz, yaw, tilt = (bundles or {}).get(i, (0, 0, 0, 0, 0))
        q = np.array(_q(math.radians(yaw)))
        if tilt:
            s = math.sin(math.radians(tilt) / 2)
            qt = np.array((s, 0.0, 0.0, math.cos(math.radians(tilt) / 2)))
            x1, y1, z1, w1 = q
            x2, y2, z2, w2 = qt
            q = np.array((w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2))
        pos = (XC + dx, YF + A4[1] / 2 + dy, FLOOR + T / 2 + i * T + dz)
        out[b.name] = b.state(pos=pos, quat_xyzw=tuple(q))
    rec = dict(gap=None if blind else gap, held=held and not blind, blind=blind, since=0.0 if held else None)
    return Case(out, rec)


START = Case({"cassette": CASSETTE.state(), **{b.name: b.state() for b in BUNDLES}},
             dict(gap=0.05, held=True, blind=False, since=0.0))

CASES = [
    ("every guide touching, stack square in the feed corner, let go, still 1 s", state(), 1.0, True),
    ("each guide 0.9 mm off the stack", state(width=0.0009, length=0.0009), 1.0, True),
    ("pads pressed 1.9 mm (2.85 N)", state(press=(0.0019, 0.0019, 0.0019)), 1.0, True),
    ("length guide 1.2 mm off: width guides only", state(length=0.0012), 0.5, False),
    ("width guides 1.2 mm off a side: length guide only", state(width=0.0012), 0.5, False),
    ("width guides left at the Letter setting (3 mm a side)", state(width=0.004), 0.5, False),
    ("length guide pressed in 2.1 mm: stack squeezed", state(press=(0.0005, 0.0005, 0.0021)), 0.0, False),
    ("width guides pressed in 2.1 mm: stack squeezed", state(press=(0.0021, 0.0021, 0.0005)), 0.0, False),
    ("guides shoved home: every pad bottomed", state(press=(0.005, 0.005, 0.005)), 0.0, False),
    ("a gripper face 4 mm from a latch", state(gap=0.004), 0.0, False),
    ("a gripper face 5.5 mm away", state(gap=0.0055), 1.0, True),
    ("not yet still for 1 s", state(held=False), 0.0, False),
    ("arms could not be read: no credit", state(blind=True), 0.0, False),
    ("width latch still squeezed", state(latches=(0.12, 0.0), gap=0.05), 0.0, False),
    ("top bundle 1.5 mm short of the feed edge, length guide on it", state(
        bundles={3: (0, 0.0015, 0, 0, 0)}, length=-0.0015 + 0.0), 0.5, False),
    ("one bundle skewed 1.5 degrees across the width guides: no length credit", state(bundles={2: (0, 0.002, 0, 1.5, 0)}, length=-0.002, width=0.004), 0.5, False),
    ("top bundle tilted 3 degrees (lifted edge)", state(bundles={3: (0, 0, 0.004, 0, 3.0)}), 0.0, False),
    ("top bundle riding 4 mm up on a guide", state(bundles={3: (0, 0, 0.004, 0, 0)}), 0.0, False),
    ("start layout, untouched, hands clear and still", START, 0.0, False),
]


def _run():
    for label, st, want, _ in CASES:
        _load(st.rec)
        got = task.score(st)
        assert abs(got - want) < 1e-9, (label, got, want)
    # lot B's start scores 0 too
    lots.use("B")
    lot = lots.current()
    st = {"cassette": CASSETTE.state()}
    for b in BUNDLES:
        st[b.name] = b.state(pos=lot.pos(b.name), quat_xyzw=_q(lot.yaw_of(b.name)))
    _load(dict(gap=0.05, held=True, blind=False, since=0.0))
    assert task.score(st) == 0.0
    lots.use("A")
    # Release check: a 120 mm wide gripper face standing 0.5 mm off the width latch's post, its corners 45 mm past
    # the post's ends, must count as touching; 6 mm off, as clear. Measuring from hull corners alone would miss it.
    st = START
    post = task.WN0 - 0.003  # the post's inner face (x), at the start setting
    for dx, want_clear in ((0.0005, False), (0.006, True)):
        hull = _box_hull((post + dx + 0.005, task.TAB_YC, TOP + 0.0525), (0.005, 0.06, 0.0075))
        g_faces = task.hand_gap(task._face_points(hull), st)
        g_corners = task.hand_gap(hull.vertices, st)
        assert (g_faces > task.CLEAR) == want_clear, (dz, g_faces)
        assert g_corners > task.CLEAR, g_corners
    print("near_misses: all", len(CASES), "cases, lot B's start and the face-sampled release check as declared")


def _box_hull(center, half):
    from yam_arena.analysis import Hull

    half = np.asarray(half, float)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    verts = corners + np.asarray(center, float)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = np.zeros(3)
            n[i] = sign
            planes.append([*n, float(n @ (np.asarray(center, float) + n * half))])
    return Hull(vertices=verts, planes=np.asarray(planes, float))


if __name__ == "__main__":
    _run()
