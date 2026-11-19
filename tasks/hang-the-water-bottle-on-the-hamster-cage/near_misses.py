"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so each case first
replays the step-side function observe() for 1.2 s of control steps with the given end state and the hands 50 mm away
(unless the case says otherwise), and then scores it. _run() afterwards drives observe() directly to check the 1 s
hold, the shoved-cage rule over the whole trial, unknown states, face sampling and lot B, and raises if anything
scores wrongly.
"""

import math

import numpy as np

import lots
import parts
import task
from task import BOTTLE, CAGE, CAGE_A, HX, R_HUNG, STAND, WIRE_R, Z_HOOK

lots.use("A")
Y_A = task.hang_y(CAGE_A[1]) - CAGE_A[1]  # the gap the demo uses, cage frame


def _ry(deg):
    return parts._rot((0, 1, 0), math.radians(deg))


def hung(y=Y_A, dx=0.0, dz=0.0, swing=0.0, cage=CAGE_A, rot=None):
    """The bottle hung in the cage frame at gap y, moved (dx, dz), swung ``swing`` degrees about the high wire
    (positive: lower end away from the cage)."""
    piv = np.array([HX, y, Z_HOOK])
    o = np.array([task.HUNG_X, y, task.HUNG_Z])
    r = _ry(swing) @ (R_HUNG if rot is None else rot)
    o = piv + _ry(swing) @ (o - piv) + np.array([dx, 0.0, dz])
    return BOTTLE.state(pos=tuple(np.asarray(cage) + o), quat_xyzw=tuple(parts._matrix_to_quat(r)))


def lean(seat_dx):
    """The hung bottle turned about its lower edge on the bars until the hook's seat point is ``seat_dx`` farther from
    the cage (as it settles after release)."""
    corner = np.array([task.HUNG_X, Y_A, task.HUNG_Z]) + R_HUNG @ task.CORNER_L
    seat = np.array([task.HUNG_X, Y_A, task.HUNG_Z]) + R_HUNG @ task.SEAT_L
    ang = -math.degrees(math.atan2(seat_dx, seat[2] - corner[2]))
    o = corner + _ry(ang) @ (np.array([task.HUNG_X, Y_A, task.HUNG_Z]) - corner)
    return BOTTLE.state(pos=tuple(np.asarray(CAGE_A) + o), quat_xyzw=tuple(parts._matrix_to_quat(_ry(ang) @ R_HUNG)))


def replay(objects, gap=0.05, seconds=1.2):
    """Fill the trial record as the simulator would after ``seconds`` at 20 Hz with these states."""
    task.reset_record()
    for k in range(int(round(seconds * 20)) + 1):
        task.observe(objects, gap, k / 20.0)
    return task.score(objects)


def task_start():
    return {o.name: o.state() for o in task.OBJECTS}


def case(label, bt, want, gap=0.05, cage=None):
    objs = {"bottle": bt} | ({"cage": cage} if cage is not None else {})
    replay({**task_start(), **objs}, gap=gap)
    return (label, objs, want, want >= 1.0)


class _Lazy(list):
    """The checker scores each case right after building it, so each case is replayed just before it is checked."""

    def __init__(self, makers):
        super().__init__(range(len(makers)))
        self._makers = makers

    def __iter__(self):
        for m in self._makers:
            yield m()

    def __getitem__(self, i):
        return self._makers[i]()


DOOR_MID = (task.DOOR_Y[0] + task.DOOR_Y[1]) / 2
MAKERS = [
    lambda: case("hung: hook on the high wire, tube through, hands clear, 1 s still", hung(), 1.0),
    lambda: case("hook 1.5 mm above the wire (exact limit)", hung(dz=0.0015), 1.0),
    lambda: case("hook 2 mm above the wire (held up, not resting)", hung(dz=0.0020), 0.0),
    lambda: case("settled back on its lower edge: wire 2 mm from the hook's lip end (exact limit)",
                 lean(0.006), 1.0),
    lambda: case("leaned back further: wire 1.5 mm from the lip end", lean(0.0065), 0.0),
    lambda: case("hooked first and let go: tip resting on the low wire outside the bars", hung(swing=6.8), 0.0),
    lambda: case("hooked first, swung part way in by hand: tip 1 mm outside", hung(swing=4.0), 0.0),
    lambda: case("swung out 1 degree: lower end 3.6 mm off the bars, tip still inside", hung(swing=1.0), 1.0),
    lambda: case("swung out 3 degrees: lower end 6.9 mm off the bars, not resting on them", hung(swing=3.0), 0.0),
    lambda: case("hooked over the top frame wire: tip 50 mm too high", hung(dz=task.Z_TOP - Z_HOOK), 0.0),
    lambda: case("hook on the door's top strip, tip 34 mm too high",
                 hung(y=DOOR_MID, dz=task.DOOR_Z[1] + task.STRIP[1] / 2 + task.STRIP[1] / 2 - Z_HOOK - WIRE_R), 0.0),
    lambda: case("at the door, where the high wire stops: bottle down on the low wire", hung(y=DOOR_MID, dz=-0.012),
                 0.0),
    lambda: case("tube in, hook 2 mm low (wedged, not on top of the wire)", hung(dz=-0.002), 0.0),
    lambda: case("spout up (bottle upside down) at the hung place", hung(rot=R_HUNG @ np.diag([1.0, -1.0, -1.0])), 0.0),
    lambda: case("still standing in the drip stand", BOTTLE.state(), 0.0),
    lambda: case("hung but a fingertip 4 mm from it (still held)", hung(), 0.0, gap=0.004),
    lambda: case("hung, nearest hand point 5.5 mm away", hung(), 1.0, gap=0.0055),
]
CASES = _Lazy(MAKERS)


def _run():
    start = task_start()
    good = {**start, "bottle": hung()}
    assert replay(good, seconds=0.9) == 0.0, "0.9 s of stillness must not score"
    assert replay(good, seconds=1.0) == 1.0, "1.0 s of stillness must score"
    # the cage shoved more than 10 mm at any step ends credit for the trial, even if it goes back
    for d, want in ((0.0105, 0.0), (0.0095, 1.0)):
        task.reset_record()
        moved = {**good, "cage": CAGE.state(pos=tuple(np.add(CAGE_A, (d, 0, 0))))}
        for k in range(40):
            task.observe(moved if k == 3 else good, 0.05, k / 20.0)
        assert task.score(good) == want, f"cage shoved {d * 1000:.1f} mm must score {want}"
    # tipped 3.5 degrees at one step
    task.reset_record()
    tipped = {**good, "cage": CAGE.state(pos=CAGE_A, quat_xyzw=tuple(parts._matrix_to_quat(_ry(3.5))))}
    for k in range(40):
        task.observe(tipped if k == 3 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a cage tipped 3.5 degrees must end credit"
    # movement during the hold restarts it
    task.reset_record()
    for k in range(25):
        task.observe({**good, "bottle": hung(dx=-0.0015) if k == 15 else hung()}, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "a 1.5 mm movement 0.45 s before the end must restart the hold"
    # unknown hands or bottle at any step: no credit
    task.reset_record()
    for k in range(30):
        task.observe(good, None if k == 3 else 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown hand state must give no credit"
    task.reset_record()
    bad = {**good, "bottle": BOTTLE.state(pos=(float("nan"), 0.0, 0.0))}
    for k in range(30):
        task.observe(bad if k == 2 else good, 0.05, k / 20.0)
    assert task.score(good) == 0.0, "an unknown bottle state must give no credit"
    # the hand gap: a point 0.5 mm off the body's flat side, and one inside the hook, are caught
    b = good["bottle"]
    rb = parts._quat_to_matrix(b.quat_xyzw)
    p_side = np.asarray(b.pos) + rb @ np.array([-(task.BODY_D / 2 + 0.0005), 0.0, 0.08])
    g = task.hand_gap(good, np.array([p_side]))
    assert 0.0004 < g < 0.0006, g
    p_hook = np.asarray(b.pos) + rb @ ((task.HOOK_OUT + task.HOOK_IN) / 2)
    assert task.hand_gap(good, np.array([p_hook])) < 0.0, "a point inside the hook must read as touching it"
    from yam_arena.analysis import Hull
    cube = np.array([[x, y, z] for x in (0, 0.03) for y in (0, 0.03) for z in (0, 0.03)], float)
    planes = np.array([[1, 0, 0, 0.03], [-1, 0, 0, 0], [0, 1, 0, 0.03], [0, -1, 0, 0], [0, 0, 1, 0.03], [0, 0, -1, 0]],
                      float)
    face = task._hull_face_points(Hull(vertices=cube, planes=planes))
    assert np.min(np.linalg.norm(face - np.array([0.015, 0.015, 0.03]), axis=1)) < 0.003, "faces must be sampled"
    # lot B: the cage moved, so lot A's hanging place is now the door; the demo's lot B place scores
    lots.use("B")
    cb = lots.current().place["cage"]
    yb = task.hang_y(cb[1]) - cb[1]
    at_b = {**good, "cage": CAGE.state(pos=cb), "stand": STAND.state(pos=lots.current().place["stand"]),
            "bottle": hung(y=yb, cage=cb)}
    assert replay(at_b) == 1.0, "lot B's hanging place must score"
    replay_a = {**at_b, "bottle": hung(y=Y_A + CAGE_A[1] - cb[1], cage=cb)}
    assert task.hung(replay_a) is False, "lot A's world place is lot B's door: no high wire there"
    lots.use("A")
    task.reset_record()


_run()
