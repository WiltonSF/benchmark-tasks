"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). States come from Part.state(pos=...,
quat_xyzw=...): the stopper's origin is the centre of its head's underside, the rod's origin the pivot ball's centre.
Offline there is no robot, so the release check passes; _run() below also checks the release rule with a gripper
placed near a part, and that an unmeasured gripper fails closed.
"""

import math
import os
from dataclasses import replace

import numpy as np

import task
from task import BX, BY, EYE_Z, HF, HP, RD, ROD, SEATED, STOPPER, U, YAW_D

os.environ[task.OFFLINE_FLAG] = "1"  # the task credits these offline cases as let go


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def qx(a):
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def stopper(yaw=math.pi, eye_z=HP, dx=0.0, dy=0.0, tilt=0.0, vel=None):
    """The stopper turned `yaw` from the drain's frame (pi or 0: its eye facing the port; pi/2: as it starts), with
    its eye's centre at height `eye_z`; dx is sideways across the port, tilt about that sideways axis."""
    off = RD @ np.array([dx, dy, 0.0])
    q = q_mul(q_mul(qz(YAW_D), qx(tilt)), qz(yaw))
    return {"stopper": STOPPER.state(pos=(BX + off[0], BY + off[1], eye_z - EYE_Z), quat_xyzw=q, lin_vel=vel)}


def rod(out=0.0, tilt_deg=0.0, dx=0.0, dz=0.0, vel=None):
    """The rod along the port axis, its ball `out` metres short of seated; tilt_deg lowers the inner tip."""
    p = SEATED + U * out + RD @ np.array([dx, 0.0, dz])
    q = q_mul(qz(YAW_D), qx(-math.radians(tilt_deg)))
    return {"pivot_rod": ROD.state(pos=tuple(float(v) for v in p), quat_xyzw=q, lin_vel=vel)}


def spun(case, w):
    """The rod of a case turning at angular velocity w (world, rad/s)."""
    return {**case, "pivot_rod": replace(case["pivot_rod"], ang_vel=np.asarray(w, float))}


def tip_down_eye(tilt_deg):
    """Eye height where a rod tilted tilt_deg (tip down; negative: tip up) meets the eye's top edge."""
    t = math.radians(tilt_deg)
    axis_z = HP - float(np.linalg.norm(SEATED[:2] - np.array([BX, BY]))) * math.tan(t)
    return axis_z + task.ROD_R - task.HOLE


CASES = [
    ("rod through the eye, horizontal, ball seated", {**stopper(), **rod()}, 1.0, True),
    ("stopper turned the other way (hole axis flipped)", {**stopper(yaw=0.0), **rod()}, 1.0, True),
    ("rest pose: rod tilted 13 deg, eye resting on it", {**stopper(eye_z=tip_down_eye(13)), **rod(tilt_deg=13)},
     1.0, True),
    ("let go: rod's outer end dropped 14 deg, tip holding the stopper up",
     {**stopper(eye_z=tip_down_eye(-14)), **rod(tilt_deg=-14, out=0.0015)}, 1.0, True),
    ("ball 1.0 mm off its seat", {**stopper(), **rod(out=0.001)}, 1.0, True),
    ("ball 3.0 mm off its seat", {**stopper(), **rod(out=0.003)}, 1.0, True),
    ("ball 3.5 mm off its seat: through the eye only", {**stopper(), **rod(out=0.0035)}, 0.5, False),
    ("ball 9 mm short, rod still through the eye", {**stopper(), **rod(out=0.009)}, 0.5, False),
    ("rod tip only 1 mm past the tab", {**stopper(), **rod(out=0.0105)}, 0.0, False),
    ("rod short of the eye, ball 25 mm out", {**stopper(), **rod(out=0.025)}, 0.0, False),
    ("rod seated but passing under the eye (stopper resting on it)",
     {**stopper(eye_z=HP + task.ROD_R + task.HOLE + task.TAB_BAR), **rod()}, 0.0, False),
    ("rod seated, axis 4.6 mm below the eye's centre", {**stopper(eye_z=HP + 0.0046), **rod()}, 0.0, False),
    ("rod seated, axis 4.4 mm below the eye's centre", {**stopper(eye_z=HP + 0.0044), **rod()}, 1.0, True),
    ("rod seated beside the eye (stopper not turned)", {**stopper(yaw=math.pi / 2), **rod()}, 0.0, False),
    ("rod seated beside the tab, 11 mm off", {**stopper(dx=0.011), **rod()}, 0.0, False),
    ("stopper turned 30 deg from the port", {**stopper(yaw=math.pi + math.radians(30)), **rod()}, 1.0, True),
    ("stopper turned 60 deg from the port", {**stopper(yaw=math.pi + math.radians(60)), **rod()}, 0.0, False),
    ("rod seated, stopper left closed and turned", {**rod()}, 0.0, False),
    ("rod at its start, stopper lifted and turned", {**stopper()}, 0.0, False),
    ("rod rolling 1.2 rad/s about its own axis: still in place", spun({**stopper(), **rod()}, -1.2 * U), 1.0, True),
    ("rod swinging 0.6 rad/s about the vertical", spun({**stopper(), **rod()}, (0.0, 0.0, 0.6)), 0.0, False),
    ("everything still swinging", {**stopper(vel=(0.0, 0.0, 0.05)), **rod()}, 0.0, False),
    ("stopper pulled 41 mm above the flange", {**stopper(eye_z=HF + 0.041 + EYE_Z), **rod()}, 0.0, False),
    ("stopper tilted 6.5 deg in the bore", {**stopper(tilt=math.radians(6.5)), **rod()}, 0.0, False),
    ("stopper head 3.9 mm off centre, toward the port", {**stopper(dy=-0.0039), **rod()}, 1.0, True),
    ("stopper tilted 5.5 deg in the bore", {**stopper(tilt=math.radians(5.5)), **rod()}, 1.0, True),
    ("stopper head 4.1 mm off centre", {**stopper(dx=0.0041), **rod(dx=0.0)}, 0.0, False),
    ("ball 3 mm off the port axis: through the eye only", {**stopper(), **rod(dx=0.003)}, 0.5, False),
]


def _release_cases(start) -> int:
    bad = 0
    full = {**start, **stopper(), **rod()}
    checks = [
        ("gripper 4.9 mm from the rod", {"pivot_rod": 0.0049, "stopper": 0.05}, 0.0),
        ("gripper 5.0 mm from the rod", {"pivot_rod": 0.005, "stopper": 0.05}, 1.0),
        ("gripper still on the stopper's head", {"pivot_rod": 0.05, "stopper": 0.0}, 0.0),
    ]
    for label, gaps, want in checks:
        task._GAP.clear()
        task._GAP.update(gaps)
        got = 1.0 if task.success(full) else float(task.score(full))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator: the 1 s hold is counted
    for label, n, want in (("held 19 steps (0.95 s): no credit", 19, 0.0), ("held 20 steps (1 s): credited", 20, 1.0)):
        task._GAP.clear()
        task._GAP.update({"pivot_rod": 0.05, "stopper": 0.05})
        task._HOLD.clear()
        task._HOLD["n"] = n
        got = float(task.score(full))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._HOLD.clear()
    task._GAP.clear()
    task._GAP.update({"pivot_rod": 0.05, "stopper": 0.05})
    got = float(task.score(full))
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  hold not yet counted: fails closed")
    task._GAP.clear()
    got = float(task.score(full))
    os.environ[task.OFFLINE_FLAG] = "1"
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  no gripper measurement yet: fails closed")
    return bad


def _run() -> int:
    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad + _release_cases(start)


if __name__ == "__main__":
    raise SystemExit(_run())
