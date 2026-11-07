"""Near-successes and unintended strategies, scored offline by factory/check.py. No record is kept over the trial,
so cases are independent. Offline there are no grippers, so every case counts as released; the 5 mm hands-off test
runs only in the simulator."""

import math

import numpy as np

from task import B, DOOR, DOOR_H, DOOR_W0, DOOR_W1, HX, HY, PIN_H, RACK_FLOOR, RACK_X, RACK_Y, SLOT_W


def quat(yaw_deg=0.0, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)):
    """Turn yaw_deg about z, then tilt_deg about a world axis."""
    def q(axis, deg):
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        s = math.sin(math.radians(deg) / 2)
        return np.array([a[0] * s, a[1] * s, a[2] * s, math.cos(math.radians(deg) / 2)])

    def mul(p, r):
        x1, y1, z1, w1 = p
        x2, y2, z2, w2 = r
        return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])

    return tuple(mul(q(tilt_axis, tilt_deg), q((0, 0, 1), yaw_deg)))


MID_Y = (DOOR_W0 + DOOR_W1) / 2  # the panel's middle, along the door from the axis


def hung(deg, rise=0.0, tilt=0.0, vel=None):
    """On its pins, deg open, lifted rise above its hung height, tilted about its own normal."""
    a = math.radians(deg)
    n = (math.cos(a), math.sin(a), 0.0)
    return DOOR.state(pos=(HX, HY, B + DOOR_H / 2 + rise), quat_xyzw=quat(deg, tilt, n), lin_vel=vel)


def racked(dx=0.0, dy=0.0, up=0.0, lean=0.0, yaw=90.0, flip=False, vel=None):
    """Standing in the rack slot: the panel's middle over (rack centre + dx, RACK_Y + dy), bottom edge up above the
    floor, leaning lean degrees sideways (about the slot's long axis). yaw 90: hinge end toward +x."""
    roll = 180.0 if flip else 0.0
    q = quat(yaw, roll, (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0))
    q = quat_mul(quat(0.0, lean, (1.0, 0.0, 0.0)), q)
    r = rot(q)
    corners = np.array([[sx * 0.005, MID_Y + sy * (DOOR_W1 - DOOR_W0) / 2, sz * DOOR_H / 2]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) @ r.T
    bottom = corners[np.argsort(corners[:, 2])[:4]]
    mid = bottom.mean(axis=0)  # the bottom edge's middle, relative to the door's origin
    mid[2] = corners[:, 2].min()
    target = np.array([sum(RACK_X) / 2 + dx, RACK_Y + dy, RACK_FLOOR + up])
    pos = target - mid
    return DOOR.state(pos=pos, quat_xyzw=q, lin_vel=vel)


def quat_mul(p, r):
    x1, y1, z1, w1 = p
    x2, y2, z2, w2 = r
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def flat(x, y, z_bottom=RACK_FLOOR):
    """Lying flat on its face on a surface, panel middle at (x, y)."""
    q = quat(90.0, 90.0, (0.0, 1.0, 0.0))
    r = rot(q)
    corners = np.array([[sx * 0.009, MID_Y + sy * 0.0755, sz * DOOR_H / 2]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    zlow = (corners @ r.T)[:, 2].min()
    pos = np.array([x, y, z_bottom - zlow]) - r @ np.array([0.0, MID_Y, 0.0]) * [1, 1, 0]
    return DOOR.state(pos=pos, quat_xyzw=q)


MOVING = (0.0, 0.0, -0.05)
HALF_PLAY = SLOT_W / 2 - 0.005  # 4 mm each side

CASES = [
    ("standing in the rack, centred", {"door": racked()}, 1.0, True),
    ("standing in the rack upside down (sleeves at the top)", {"door": racked(flip=True)}, 1.0, True),
    ("standing in the rack, hinge end toward the robot", {"door": racked(yaw=-90.0)}, 1.0, True),
    ("in the rack against one wall", {"door": racked(dy=HALF_PLAY)}, 1.0, True),
    ("in the rack, bottom edge 3 mm up (limit)", {"door": racked(up=0.003)}, 1.0, True),
    ("in the rack, bottom edge 4 mm up (resting on something)", {"door": racked(up=0.004)}, 0.0, False),
    ("in the rack, leaning 9.9 degrees", {"door": racked(lean=9.9)}, 1.0, True),
    ("leaning 10.5 degrees", {"door": racked(lean=10.5)}, 0.0, False),
    ("in the rack, panel middle at the slot's end (limit)", {"door": racked(dx=(RACK_X[1] - RACK_X[0]) / 2)}, 1.0, True),
    ("mostly past the slot's end", {"door": racked(dx=(RACK_X[1] - RACK_X[0]) / 2 + 0.01)}, 0.0, False),
    ("beside the rack, outside its walls", {"door": racked(dy=0.03)}, 0.0, False),
    ("in the rack but still moving", {"door": racked(vel=MOVING)}, 0.0, False),
    ("lying flat across the rack's walls", {"door": flat(sum(RACK_X) / 2, RACK_Y, RACK_FLOOR + 0.05)}, 0.0, False),
    ("dropped flat on the table", {"door": flat(0.25, -0.30)}, 0.0, False),
    ("habit: lifted straight away at the start angle, tab jammed under the keeper (6 mm up)", {"door": hung(30.0, 0.006)}, 0.0, False),
    ("swung to 90 degrees, still hung", {"door": hung(90.0)}, 0.0, False),
    ("lifted at 90 degrees, sleeves 2 mm below the lower pin's top", {"door": hung(90.0, PIN_H - 0.002)}, 0.0, False),
    ("lifted clear of both pins and held over them", {"door": hung(90.0, PIN_H + 0.012)}, 0.0, False),
    ("tilted 10 degrees at 40 degrees open, still on its pins", {"door": hung(40.0, 0.004, 10.0)}, 0.0, False),
]
