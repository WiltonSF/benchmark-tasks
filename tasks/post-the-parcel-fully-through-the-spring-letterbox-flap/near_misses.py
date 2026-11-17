"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success). Offline there is no robot, so before the cases this file writes into the task's per-step record
what the step callback would have measured with both hands 50 mm from the parcel. _release_checks() below then runs
the release measurement itself on synthetic hand surfaces (a broad flat face, a finger near the parcel, no
measurement) and raises if any of them scores wrongly.
"""

import math

import numpy as np

import task
from task import BOX_D, BOX_H, FLAP_H, FLAP_SHUT, LETTERBOX, PACKET_SIZE, AP_Z0, TOP, X_IN, X_OUT, YC

L, W, H = (float(v) for v in task.PACKET.size)  # the parcel's whole box, label and tape included


def qy(deg):
    """Turned about the world y axis (front end down for positive angles)."""
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def flat_at(low_x, bottom):
    """Lying flat along x with its trailing end at low_x and its underside at height `bottom` above the table."""
    return (low_x + L / 2, YC, TOP + bottom + H / 2)


def leaning(deg, low_x):
    """Tilted `deg` about y, with its bounding box's near side at low_x and its lowest point on the table."""
    a = math.radians(deg)
    hx = (L * math.cos(a) + H * math.sin(a)) / 2
    hz = (L * math.sin(a) + H * math.cos(a)) / 2
    return (low_x + hx, YC, TOP + hz)


def flap(q):
    return {"letterbox": LETTERBOX.state(joints={"flap": q})}


def away():
    """What the step callback records once both hands are 50 mm from the parcel."""
    task._rec.update(gap=0.05)


away()
MOVING = (0.0, 0.0, -0.05)
FLOOR = task.SHEET
IN_BOX = flat_at(X_IN + 0.03, FLOOR)

CASES = [
    ("inside on the box floor, flap shut", {"packet": (IN_BOX, None, None)}, 1.0, True),
    ("inside, trailing end exactly at the gate's inner face", {"packet": (flat_at(X_IN, FLOOR), None, None)}, 1.0, True),
    ("on the box floor, 1 mm still inside the throat", {"packet": (flat_at(X_IN - 0.001, FLOOR), None, None)},
     0.0, False),
    ("inside, leaning 60 deg against the gate's inner face",
     {"packet": (leaning(60, X_IN + 0.0005), None, None, qy(60))}, 1.0, True),
    ("inside against the back wall", {"packet": (flat_at(X_IN + BOX_D - L - 0.001, FLOOR), None, None)}, 1.0, True),
    ("inside, flap at the shut limit (5 mm)", {"packet": (IN_BOX, None, None), **flap(FLAP_SHUT)}, 1.0, True),
    ("inside, flap 6 mm open", {"packet": (IN_BOX, None, None), **flap(math.asin(0.006 / FLAP_H))}, 0.0, False),
    ("inside, flap held open 0.6 rad", {"packet": (IN_BOX, None, None), **flap(0.6)}, 0.0, False),
    ("inside but still sliding", {"packet": (IN_BOX, None, MOVING)}, 0.0, False),
    ("lying on the catch box roof", {"packet": (flat_at(X_IN + 0.04, BOX_H + 0.002), None, None)}, 0.0, False),
    ("lying on the gate's cap rail", {"packet": (flat_at(X_OUT - 0.05, task.GATE_H + task.CAP_T), None, None)},
     0.0, False),
    ("fell behind the catch box", {"packet": (flat_at(X_IN + BOX_D + 0.02, 0.0), None, None)}, 0.0, False),
    ("stuck flat in the slot, 70 mm in (no partial credit)",
     {"packet": (flat_at(X_OUT + 0.07 - L, AP_Z0), None, None)}, 0.0, False),
    ("stuck flat in the slot, flap resting on it",
     {"packet": (flat_at(X_OUT + 0.07 - L, AP_Z0), None, None), **flap(1.15)}, 0.0, False),
    ("in the slot, held 3 mm above its floor", {"packet": (flat_at(X_OUT + 0.07 - L, AP_Z0 + 0.003), None, None)},
     0.0, False),
    ("in the slot but still moving", {"packet": (flat_at(X_OUT + 0.07 - L, AP_Z0), None, MOVING)}, 0.0, False),
    ("only 9 mm into the slot", {"packet": (flat_at(X_OUT + 0.009 - L, AP_Z0), None, None)}, 0.0, False),
    ("hanging tilted 20 deg out of the slot", {"packet": ((X_OUT + 0.0, YC, TOP + AP_Z0), None, None, qy(-20))},
     0.0, False),
    ("hanging 48 deg in the throat, trailing end on the slot floor",
     {"packet": ((X_IN + 0.030, YC, TOP + AP_Z0 - 0.040), None, None, qy(48)), **flap(0.5)}, 0.0, False),
    ("on the table in front of the gate", {"packet": (flat_at(X_OUT - L - 0.01, 0.0), None, None)}, 0.0, False),
    ("standing on end against the plate", {"packet": ((X_OUT - H / 2 - 0.004, YC, TOP + L / 2), None, None, qy(90))},
     0.0, False),
]


# -- the release measurement itself ---------------------------------------------------------------------------------

def _square(centre, half):
    """Two triangles making a horizontal square face (a broad hull face, like the gripper's), half-width `half`."""
    cx, cy, cz = centre
    a, b = np.array([cx - half, cy - half, cz]), np.array([cx + half, cy - half, cz])
    c, d = np.array([cx + half, cy + half, cz]), np.array([cx - half, cy + half, cz])
    return np.array([[a, b, c], [a, c, d]])


def _release_checks():
    from yam_arena.analysis import box_signed_distance

    packet = task.PACKET.state(pos=IN_BOX)
    top = IN_BOX[2] + H / 2 + 0.0004  # the tape and barcode stand 0.4 mm proud of the lid
    bad = []

    # A 125 mm face lying 0.5 mm above the parcel's top: its corners are 24 mm out beyond the parcel's sides.
    tris = _square((IN_BOX[0], IN_BOX[1], top + 0.0005), 0.0625)
    corners = np.unique(tris.reshape(-1, 3), axis=0)
    boxes = task.PACKET.world_boxes({"base": (np.asarray(packet.pos), np.eye(3))})
    vertex_gap = min(box_signed_distance(corners, c, r, s).min() for _, c, r, s in boxes)
    face_gap = task.parcel_gap(task.sample_triangles(tris), packet)
    if not vertex_gap > 0.02:
        bad.append(f"face case: the corners should be over 20 mm away, got {vertex_gap * 1000:.1f} mm")
    if not face_gap < 0.001:
        bad.append(f"face case: the sampled face should read under 1 mm, got {face_gap * 1000:.2f} mm")
    task._rec.update(gap=face_gap)
    if task.score({"packet": packet, "letterbox": LETTERBOX.state()}) != 0.0:
        bad.append("a hand face 0.5 mm above the parcel (no vertex near) scored")

    # A finger 4 mm from the parcel's side, then 6 mm.
    for d, want in ((0.004, 0.0), (0.006, 1.0)):
        side = IN_BOX[1] + W / 2 + d
        tris = np.array([[[IN_BOX[0], side, IN_BOX[2]], [IN_BOX[0] + 0.01, side, IN_BOX[2]],
                          [IN_BOX[0], side + 0.01, IN_BOX[2]]]])
        task._rec.update(gap=task.parcel_gap(task.sample_triangles(tris), packet))
        got = task.score({"packet": packet, "letterbox": LETTERBOX.state()})
        if got != want:
            bad.append(f"a finger {d * 1000:.0f} mm from the parcel scored {got}, expected {want}")

    # No measurement (the arms could not be read): no credit.
    task._rec.update(gap=None)
    if task.score({"packet": packet, "letterbox": LETTERBOX.state()}) != 0.0:
        bad.append("no hand measurement scored")

    away()
    if bad:
        raise AssertionError("release check: " + "; ".join(bad))


_release_checks()
