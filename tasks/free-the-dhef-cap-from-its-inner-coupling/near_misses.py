"""Near misses and unintended strategies for the DHEF cap task, scored by the offline task checker.

The end states are built from the parts' own kinematics (the cap's carrier pose, how far its tip has rolled back,
the rod's position), with the whole-trial record the simulator callback keeps set as each case says: hands clear and
the scene still for the 1 s hold unless the case is about that. The tests at the bottom also run when this file is
imported: they check the declared mechanisms (stick-slip rolling and rod seals, stretch past full eversion) and the
puncture rule's geometry, so a change that breaks them fails the offline check.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import task  # noqa: E402

AX = task.AXIS_W
CAP0 = np.asarray(task.CAP_W, float)
YAWQ = (0.0, 0.0, math.sin(task.YAW / 2), math.cos(task.YAW / 2))


def turn(angle):
    """The cap's orientation turned ``angle`` about its own axis from the start (negative: toward open)."""
    r = task.CAP.rotation @ task.parts._rot((1.0, 0.0, 0.0), angle)
    return tuple(float(v) for v in task.parts._matrix_to_quat(r))


def cap(drawn=0.0, evert=None, angle=-task.TURN, pos=None, quat=None):
    """The cap drawn ``drawn`` m forward along the gripper's axis, its tip rolled back ``evert`` m (default: as far as
    the carrier was drawn, the rod holding the tip where it was)."""
    q = drawn if evert is None else evert
    p = CAP0 + drawn * AX if pos is None else np.asarray(pos, float)
    return task.CAP.state(joints={"evert": q}, pos=p, quat_xyzw=quat if quat is not None else turn(angle))


def off_drawn(sep):
    """How far the carrier is drawn (rod forward, cap fully everted) for the stud head's back face to stand ``sep``
    m in front of the coupling's front face."""
    return task.S_FULL + (task.U_CPL1 - task.U_HEAD0) + sep


def lying(x, y, angle=0.0, lift=0.0):
    """The cap lying on its catch ring (axis level along world y, as the hand carries it), the ring's lowest face
    ``lift`` m above the mat's top."""
    p = (x, y, task.MAT_TOP + task.R_RING_OUT + lift)
    return task.CAP.state(joints={"evert": task.S_FULL}, pos=p, quat_xyzw=turn(angle))


MAT_A = task.MAT.state()
MX, MY = task.MAT_A[0], task.MAT_A[1]


def flags(**kw):
    base = dict(known=True, broken=False, punctured=False, stretched=0.0, dropped=False, clear=True,
                hold=task.HOLD_STEPS)
    base.update(kw)
    return base


# (label, {object: state}, expected score, expected success, whole-trial record)
_RAW = [
    ("start, hands clear", {}, 0.0, False, flags()),
    ("catch turned to open but not drawn off", {"cap": cap(0.0)}, 0.0, False, flags()),
    ("ring drawn 9.9 mm forward", {"cap": cap(0.0099)}, 0.0, False, flags()),
    ("ring drawn 10 mm forward (catch off): 0.25", {"cap": cap(0.010)}, 0.25, False, flags()),
    ("drawn 60 mm, cap half rolled", {"cap": cap(0.060)}, 0.25, False, flags()),
    ("rolled to 122.4 mm: tip 19.9 mm behind the ring", {"cap": cap(0.1224)}, 0.25, False, flags()),
    ("rolled to 122.5 mm: tip 20 mm behind the ring: 0.5", {"cap": cap(task.Q_EVERTED)}, 0.5, False, flags()),
    ("fully everted, still coupled", {"cap": cap(task.S_FULL)}, 0.5, False, flags()),
    ("fully everted but a hand still within 30 mm", {"cap": cap(task.S_FULL)}, 0.0, False, flags(clear=False)),
    ("fully everted, still for only 0.95 s", {"cap": cap(task.S_FULL)}, 0.0, False, flags(hold=task.HOLD_STEPS - 1)),
    ("stud 4.9 mm out of the coupling", {"cap": cap(off_drawn(0.0049), task.S_FULL)}, 0.5, False, flags()),
    ("stud 5 mm out of the coupling, cap left hanging: 0.75", {"cap": cap(off_drawn(0.005), task.S_FULL)}, 0.75,
     False, flags()),
    ("cap off and lying on the table beside the mat", {"cap": lying(0.215, 0.05)}, 0.75, False, flags()),
    ("cap lying on the mat: 1.0", {"cap": lying(MX, MY + 0.02)}, 1.0, True, flags()),
    ("cap on the mat, ring turned any way", {"cap": lying(MX, MY + 0.02, angle=1.0)}, 1.0, True, flags()),
    ("cap on the mat but 4 mm above it (still held up)", {"cap": lying(MX, MY + 0.02, lift=0.004)}, 0.75, False,
     flags()),
    ("cap on the mat, ring 1 mm over its edge", {"cap": lying(MX + task.MAT_SIZE[0] / 2 - task.R_RING_OUT + 0.001,
                                                              MY + 0.02)}, 0.75, False, flags()),
    ("cap on the mat, ring just inside its edge", {"cap": lying(MX + task.MAT_SIZE[0] / 2 - task.R_RING_OUT - 0.0005,
                                                                MY + 0.02)}, 1.0, True, flags()),
    ("cap on the mat, a hand still within 30 mm", {"cap": lying(MX, MY + 0.02)}, 0.0, False, flags(clear=False)),
    ("cap on the mat after a finger went through the silicone", {"cap": lying(MX, MY + 0.02)}, 0.0, False,
     flags(punctured=True)),
    ("cap on the mat after a part left the table", {"cap": lying(MX, MY + 0.02)}, 0.0, False, flags(dropped=True)),
    ("cap on the mat, contacts unreadable", {"cap": lying(MX, MY + 0.02)}, 0.0, False, flags(broken=True)),
    ("cap on the mat, no step seen", {"cap": lying(MX, MY + 0.02)}, 0.0, False, flags(known=False)),
    ("everted cap with the rod pushed 40 mm back", {"cap": cap(task.S_FULL - 0.040, task.S_FULL),
                                                    "dhef": task.DHEF.state(joints={"rod": 0.040})}, 0.5, False,
     flags()),
    ("rod pushed 40 mm back, coupling drawn off a held cap",
     {"cap": cap(task.S_FULL), "dhef": task.DHEF.state(joints={"rod": (task.U_CPL1 - task.U_HEAD0) + 0.005})},
     0.75, False, flags()),
]


# The checker reads CASES rows as (label, {object: spec}, score, success); the record has to be in place when score()
# runs, so each row's states are wrapped in a mapping that sets it on first access.
class _Setter(dict):
    def __init__(self, changes, record):
        super().__init__(changes)
        self._record = record

    def items(self):
        task._S.update(self._record)
        return super().items()


CASES = [(label, _Setter(changes, rec), want, ok) for label, changes, want, ok, rec in _RAW]


# -- tests that run on import ---------------------------------------------------------------------------------------
def _tests():
    # stick-slip: the target stays while the joint moves less than the band, then trails it by the band
    b = task.F_ROLL / task.EVERT.spring
    assert task.stick_slip(0.0005, 0.0, b) == 0.0
    assert abs(task.stick_slip(0.010, 0.0, b) - (0.010 - b)) < 1e-12
    assert abs(task.stick_slip(0.010, 0.020, b) - (0.010 + b)) < 1e-12
    assert task.stick_slip(0.0, 0.0002, b) == 0.0002
    # rolling force never above F_ROLL before full eversion; stretch force = K_STRETCH x stretch past it
    t = 0.0
    for q in np.linspace(0.0, task.S_FULL, 200):
        t = task.evert_target(q, t)
        assert task.EVERT.spring * (q - t) <= task.F_ROLL + 1e-9
    for s in (0.002, 0.008):
        t = task.evert_target(task.S_FULL + s, t)
        assert abs(task.EVERT.spring * (task.S_FULL + s - t) - task.K_STRETCH * s) < 1e-6
    # puncture geometry: a point about 4 mm inside the cap wall at the start counts; one on the catch ring's outside does
    # not; at full eversion, points on the releasing ring (where the left hand pinches it) do not
    c0 = task.CAP.state()
    inside = task.to_world(task.cyl(task.U_RIM + 0.040, task.R_CAP_RIM - 0.008, 0.0))
    ring_out = task.to_world(task.cyl((task.U_RING0 + task.U_RING1) / 2, task.R_RING_OUT, 0.0))
    assert task.puncture_depth(np.array([inside]), c0) >= task.PUNCTURE
    assert task.puncture_depth(np.array([ring_out]), c0) == 0.0
    ce = cap(task.S_FULL)
    sleeve = [task.to_world(task.cyl(u, task.R_SLEEVE, th)) for u in np.linspace(0.183, 0.209, 14)
              for th in np.linspace(0, 2 * math.pi, 12)]
    assert task.puncture_depth(np.array(sleeve), ce) == 0.0
    tip = task.to_world(task.cyl(task.U_NECK1 + 0.002, 0.004, 0.0))
    assert task.puncture_depth(np.array([tip]), ce) >= task.PUNCTURE
    # locked, every lug lies behind the lip: radially inside its outer edge and angularly inside one of its arcs
    # (so a pull before the catch is turned meets the lip); turned 45 deg, every lug lines up with a gap
    assert task.R_LUG0 < task.R_LIP and task.U_LUG1 < task.U_LIP0
    for th in task.LOCK_TH:
        gaps = [abs((th - g + math.pi) % (2 * math.pi) - math.pi) for g in task.OPEN_TH]
        assert min(gaps) - task.GAP_W / 2 >= task.LUG_W / 2, (th, gaps)
    for th in task.OPEN_TH:
        assert min(abs((th - g + math.pi) % (2 * math.pi) - math.pi) for g in task.OPEN_TH) < 1e-9
    assert task.GAP_W - task.LUG_W >= math.radians(20)  # 10 deg either way for the turn
    # at the start the stud's head sits 14.5 mm inside the coupling
    st = {o.name: o.state() for o in task.OBJECTS}
    sep = float((task.head_back_world(st["cap"]) - task.coupling_front_world(st["dhef"])) @ AX)
    assert abs(sep + (task.U_CPL1 - task.U_HEAD0)) < 1e-9, sep


_tests()
