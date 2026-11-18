"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial whose end state
has held, let go and still, for 1 s (the grippers 50 mm away). _run() then drives the step-side function observe()
directly to check the 1 s hold, stillness, the let-go distance (with face sampling), the whole-trial fall rule, the
missing-measurement rule and lot B, and raises if any scores wrongly.
"""

import math

import numpy as np

import lots
import task
from task import BOARD_W, CLAMP, CLAMP_START, LEFT, LEFT_C, RIGHT, RIGHT_C, SLIDE, TOP
from yam_arena.analysis import Hull

lots.use("A")


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) * math.sin(a)
    return (float(v[0]), float(v[1]), float(v[2]), math.cos(a))


def scene(n_force=100.0, gap=0.0, left_dx=0.0, right_dx=0.0, right_dz=0.0, right_q=None, edge_off=0.0,
          far_off=0.0, clamp_dx=SLIDE, clamp_q=None, shift=(0.0, 0.0)):
    """The boards on the bench (the right one moved to leave ``gap``), and the clamp with its fixed pad on the left
    board's outer edge (``edge_off`` m off it), compressed for ``n_force`` N, and its far pad ``far_off`` m off the
    right board's outer edge."""
    sx, sy = shift
    pad = n_force / task.K_PAD
    left = LEFT.state(pos=(LEFT_C[0] + left_dx + sx, LEFT_C[1] + sy, TOP))
    ry = LEFT_C[1] - BOARD_W - gap
    right = RIGHT.state(pos=(RIGHT_C[0] + right_dx + sx, ry + sy, TOP + right_dz), quat_xyzw=right_q)
    y0 = LEFT_C[1] + BOARD_W / 2 + (task.PAD_FACE - pad) + edge_off  # clamp origin: pad face on the left edge
    u_right = y0 - (ry - BOARD_W / 2)
    bar = task.FAR_FACE - (u_right + far_off)
    clamp = CLAMP.state(pos=(CLAMP_START[0] + clamp_dx + sx, y0 + sy, TOP), quat_xyzw=clamp_q or CLAMP.state().quat_xyzw,
                        joints={"pad": pad, "bar": bar, "plate": -0.02, "trigger": 0.0})
    return {"clamp": clamp, "board_left": left, "board_right": right}


def preset():
    task.reset_record()
    task._rec.update(steps=50, gaps=(0.05, 0.05, 0.05), since=9.0, level=1.0, now=1.0, t=10.0)


preset()
START = {"clamp": CLAMP.state(), "board_left": LEFT.state(), "board_right": RIGHT.state()}
CASES = [
    ("clamped at 100 N, joint shut, ends and tops level, let go, 1 s", scene(), 1.0, True),
    ("exactly 60 N", scene(n_force=60.0), 1.0, True),
    ("59 N: closed but under the force", scene(n_force=59.0), 0.5, False),
    ("pads touching only (0 N): the jaw pushed shut by hand", scene(n_force=0.0), 0.5, False),
    ("joint open 0.5 mm exactly", scene(gap=0.0005), 1.0, True),
    ("joint open 0.6 mm", scene(gap=0.0006), 0.25, False),
    ("joint open 3 mm, both pads on the edges", scene(gap=0.003), 0.25, False),
    ("ends 3 mm apart exactly", scene(right_dx=0.003), 1.0, True),
    ("ends 4 mm apart", scene(right_dx=0.004), 0.5, False),
    ("right board 1.5 mm proud (riding up)", scene(right_dz=0.0015), 0.5, False),
    ("right board 3 degrees twisted (its corner past the far pad)", scene(right_q=quat((0, 0, 1), 3.0)), 0.0, False),
    ("fixed pad 2 mm off its edge (pressing something else)", scene(edge_off=0.002), 0.25, False),
    ("far pad 2 mm short of its edge", scene(far_off=0.002), 0.25, False),
    ("clamp not slid on: jaws short of the boards", scene(clamp_dx=0.0), 0.0, False),
    ("clamp 35 mm short: pads bear on exactly 15 mm", scene(clamp_dx=SLIDE - 0.035), 1.0, True),
    ("clamp 36 mm short: pads bear on 14 mm", scene(clamp_dx=SLIDE - 0.036), 0.0, False),
    ("clamp 12 mm farther on: board ends at the bar exactly", scene(clamp_dx=SLIDE + 0.012), 1.0, True),
    ("clamp 14 mm farther on: boards over the bar", scene(clamp_dx=SLIDE + 0.014), 0.0, False),
    ("clamp lying 10 degrees tilted", scene(clamp_q=quat((0.7071, 0.7071, 0), 10.0)), 0.0, False),
    ("start layout (do nothing)", START, 0.0, False),
]


def _states(**kw):
    out = scene()
    out.update(kw)
    return out


def box_hull(center, half):
    half = np.asarray(half, float)
    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + np.asarray(center)
    planes = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            n = np.zeros(3)
            n[i] = sign
            planes.append([*n, sign * center[i] + half[i]])
    return Hull(vertices=v, planes=np.array(planes))


def _check(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{label}: score {got}, expected {want}")


def _run() -> None:
    dt = 0.05
    end = scene()
    far = (0.05, 0.05, 0.05)

    task.reset_record()
    for k in range(20):
        task.observe(end, far, k * dt)
    _check("0.95 s into the hold", task.score(end), 0.0)
    task.observe(end, far, 20 * dt)
    _check("1.0 s into the hold", task.score(end), 1.0)
    task.observe(end, (0.004, 0.05, 0.05), 21 * dt)
    _check("a finger 4 mm from the clamp", task.score(end), 0.0)
    for k in range(22, 50):
        task.observe(end, (0.05, 0.05, 0.0049), k * dt)
    _check("a finger 4.9 mm from the right board: not let go", task.score(end), 0.0)
    for k in range(50, 80):
        task.observe(end, (0.0051, 0.0051, 0.0051), k * dt)
    _check("every finger just over 5 mm away", task.score(end), 1.0)

    # Held in the gripper: never credited, even at full force.
    task.reset_record()
    for k in range(40):
        task.observe(end, (0.0, 0.05, 0.05), k * dt)
    _check("clamped at 100 N but still held", task.score(end), 0.0)

    # Stillness: a board sliding 3 mm back and forth within the second does not count.
    task.reset_record()
    for k in range(30):
        s = scene(left_dx=0.003 * (k % 2), right_dx=0.003 * (k % 2))
        task.observe(s, far, k * dt)
    _check("panel sliding 3 mm to and fro", task.score(s), 0.0)

    # The 1.0 must hold for the whole second: at 100 N for 0.5 s, then the force falls to 45 N.
    task.reset_record()
    for k in range(10):
        task.observe(end, far, k * dt)
    low = scene(n_force=45.0)
    for k in range(10, 40):
        task.observe(low, far, k * dt)
    _check("force fell to 45 N", task.score(low), 0.5)

    # Whole-trial rules.
    task.reset_record()
    task.observe(_states(board_right=RIGHT.state(pos=(RIGHT_C[0], RIGHT_C[1], TOP - 0.06))), far, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("a board knocked off the bench once", task.score(end), 0.0)
    task.reset_record()
    task.observe(_states(board_left=LEFT.state(quat_xyzw=quat((1, 0, 0), 35.0))), far, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("a board flipped up past 30 degrees once", task.score(end), 0.0)
    task.reset_record()
    task.observe(end, None, 0.0)
    for k in range(1, 40):
        task.observe(end, far, k * dt)
    _check("grippers unreadable at one step", task.score(end), 0.0)
    task.reset_record()
    _check("no step recorded", task.score(end), 0.0)

    # Let-go is measured on sampled hull faces: a 40 x 120 mm face 0.5 mm over the left board's top, across it, with
    # every corner 30 mm beyond its edges.
    b = end["board_left"]
    top = float(b.high[2])
    h = box_hull((b.pos[0], b.pos[1], top + 0.0005 + 0.01), (0.020, 0.060, 0.010))
    corners_only = task.gap_to(LEFT, b, h.vertices)
    sampled = task.gap_to(LEFT, b, task._face_points(h))
    if not (corners_only > task.CLEAR and sampled < task.CLEAR):
        raise AssertionError(f"face sampling: corners {corners_only:.4f} m, sampled {sampled:.4f} m")

    # Lot B: the same clamped panel shifted with the lot scores 1.0 there.
    lots.use("B")
    task.reset_record()
    endb = scene(shift=task.LOT_B)
    for k in range(25):
        task.observe(endb, far, k * dt)
    _check("lot B: clamped panel", task.score(endb), 1.0)
    lots.use("A")
    preset()


_run()
