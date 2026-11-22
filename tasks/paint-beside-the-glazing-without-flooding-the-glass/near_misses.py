"""Scorer controls for the declared paint rule, the protected glass line and the release gate.

Each control drives task.deposit() with brush poses built from the published geometry (no simulator results are
invented), then checks score(). Lot A (near-left light) unless stated.
"""
import math

import numpy as np

import lots
import task as t

lots.use("A")


def brush_at(tcp, c=t.C_PLAN, lean=t.LEAN, yaw_extra=0.0):
    """Brush gripped at GRIP_X with the reference grip, leaned `lean` degrees, bristles compressed by c (m)."""
    r = t.parts._rot((0, -1, 0), math.radians(lean)) @ t.parts._rot((0, 0, 1), math.pi + yaw_extra)
    pos = np.asarray(tcp, float) - r @ np.array([t.GRIP_X, 0, 0])
    q = t.mat_to_quat(r)
    assert np.allclose(t.parts._quat_to_matrix(q), r, atol=1e-9)
    return t.BRUSH.state(joints={"bristles": c}, pos=tuple(pos), quat_xyzw=tuple(q))


def sweep(u_heel, surface, c=t.C_PLAN, y_from=None, y_to=None, step=0.002, lean=t.LEAN, dx=0.0, plan_c=None):
    x0, x1, y0, y1 = t.LIGHTS[t.S["target"]]
    y_from = y1 - t.PAD_W / 2 if y_from is None else y_from
    y_to = y0 + t.PAD_W / 2 if y_to is None else y_to
    # The TCP x is planned for compression pc; the hand held lower presses the tips to c with the heel on the surface,
    # so the heel slides toward the glass with the extra compression.
    pc = c if plan_c is None else plan_c
    tcp_x = t.XG - u_heel - t.heel_offset(pc, lean)[0] + dx
    tcp_z = surface - t.heel_offset(c, lean)[2]
    for y in np.arange(y_from, y_to - 1e-9, -step):
        corners, axis_w, cc, tip = t.brush_geometry(brush_at((tcp_x, y, tcp_z), c, lean))
        t.deposit(corners, axis_w, cc, True, False, "left")


def reference(c=t.C_PLAN, dx=0.0):
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(t.U_PASS[0], t.Z_GLASS, c, dx=dx)
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(t.U_PASS[1], t.Z_WOOD, c, dx=dx)


def released(pos=None):
    t.S.update(known=True, hold=t.HOLD_STEPS)
    st = {o.name: o.state() for o in t.OBJECTS}
    if pos is not None:
        st["brush_left"] = t.BRUSH.state(pos=pos)
    return st


def controls():
    # Geometry: the heel line sits where the reference plans it.
    corners, axis_w, c, tip = t.brush_geometry(brush_at((0.30, 0.10, t.TOP + 0.13)))
    heel = corners[np.argmin(corners[:, 2])]
    want = np.array([0.30, 0.10, t.TOP + 0.13]) + t.heel_offset()
    assert np.allclose(heel[[0, 2]], want[[0, 2]], atol=1e-6), (heel, want)

    # The reference two passes: full band, putty line painted, at most 0.5 mm onto the glass.
    reference()
    m = t.paint_metrics()
    assert m["coverage"] >= 0.98 and m["line"] >= 0.98 and m["flood"] == 0 and m["lap_max_mm"] <= 0.5 + 1e-3, m
    assert m["stray"] <= t.STRAY_MAX, m
    assert t.score(released()) == 1.0 and t.success(released())
    # The same paint, but the brush still held, not yet still, or left off the rest.
    t.S["hold"] = t.HOLD_STEPS - 1
    st = released()
    t.S["hold"] = t.HOLD_STEPS - 1
    assert t.score(st) == 0.0, "1 s still hold"
    st = released(pos=(0.30, 0.35, t.TOP + 0.03))
    assert t.score(st) == 0.0, "brush on the bench, not on the rest"
    st = released()
    st["brush_left"] = t.BRUSH.state(pos=(t.BRUSH.pos[0], t.BRUSH.pos[1], t.BRUSH.pos[2] + 0.02))
    assert t.score(st) == 0.0, "brush held 20 mm above the rest"
    st = released()
    st["brush_right"] = t.BRUSHES["right"].state(pos=(0.30, -0.35, t.TOP + 0.03))
    assert t.score(st) == 0.0, "the other brush left off its rest"
    for flag in ("blob", "crushed", "dropped", "broken"):
        reference()
        t.S[flag] = True
        assert t.score(released()) == 0.0, flag
    reference()
    t.S["known"] = False
    assert t.score({o.name: o.state() for o in t.OBJECTS}) == 0.0, "unknown hand state"

    # Pressing too hard on the glass pass: the splay floods the glass for the whole trial.
    for c, flood in ((0.003, False), (0.0040, True), (0.006, True)):
        t.reset_state()
        t.S["load"]["left"] = t.LOAD_MAX
        sweep(-0.0005, t.Z_GLASS, c, plan_c=t.C_PLAN)
        assert (t.paint_metrics()["flood"] > 0) == flood, (c, t.paint_metrics())
    # A correct later pass cannot undo it.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(-0.0005, t.Z_GLASS, 0.006, plan_c=t.C_PLAN)
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(t.U_PASS[1], t.Z_WOOD)
    assert t.paint_metrics()["flood"] > 0 and t.score(released()) == 0.0, "overpressed glass pass"
    # The heel line placed 3 mm onto the glass at light pressure: flooding.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(-0.003, t.Z_GLASS)
    assert t.paint_metrics()["flood"] > 0 and t.score(released()) == 0.0
    # A heel line 1.9 mm onto the glass stays inside the 2 mm lap.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(-0.0019, t.Z_GLASS)
    assert t.paint_metrics()["flood"] == 0, t.paint_metrics()
    # Holding back 2 mm from the glass: putty line bare, capped at 0.5.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(0.002, t.Z_GLASS)
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(t.U_PASS[1], t.Z_WOOD)
    m = t.paint_metrics()
    assert m["line"] == 0.0 and m["coverage"] >= 0.9, m
    assert t.score(released()) == 0.5, t.score(released())

    # Compressed but not on the sash (tips 5 mm above it, e.g. pressed on something else): nothing.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    tcp_x = t.XG + 0.001 - t.heel_offset()[0]
    for y in np.arange(0.17, 0.03, -0.002):
        t.deposit(*t.brush_geometry(brush_at((tcp_x, y, t.Z_GLASS + 0.005 - t.heel_offset()[2])))[:3], True, False, "left")
    assert t.paint_metrics()["coverage"] == 0.0, "pressed off the sash"
    # Paint smeared across a far pane (the grid covers the whole sash): flooding.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    xf = t.LIGHTS["far_left"][0] + 0.08
    for y in np.arange(0.17, 0.05, -0.002):
        t.deposit(*t.brush_geometry(brush_at((xf - t.heel_offset()[0], y, t.Z_GLASS - t.heel_offset()[2])))[:3], True, False, "left")
    assert t.paint_metrics()["flood"] > 0, "far pane"
    reference()
    t.S["can_knocked"] = True
    assert t.score(released()) == 0.0, "can knocked"
    # Dry brush (never dipped): nothing.
    t.reset_state()
    sweep(-0.0005, t.Z_GLASS)
    assert t.paint_metrics()["coverage"] == 0.0
    # Hovering 1 mm off the work (no compression): nothing.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(-0.0005, t.Z_GLASS, c=0.0002)
    assert t.paint_metrics()["coverage"] == 0.0
    # One load runs out: the second pass without a reload leaves bare rail.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(t.U_PASS[0], t.Z_GLASS)
    sweep(t.U_PASS[1], t.Z_WOOD)
    m = t.paint_metrics()
    assert m["coverage"] < 0.8, m
    # Paint only the glass-side pass: about half the band.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(-0.0005, t.Z_GLASS)
    m = t.paint_metrics()
    assert 0.5 <= m["coverage"] < 0.8 and m["line"] >= 0.95, m
    assert t.score(released()) == 0.5
    # Only part of the rail: a quarter.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    sweep(-0.0005, t.Z_GLASS, y_from=t.LIGHTS["near_left"][3] - t.PAD_W / 2, y_to=t.LIGHTS["near_left"][3] - t.PAD_W / 2 - 0.040)
    m = t.paint_metrics()
    assert 0.25 <= m["coverage"] < 0.5, m
    assert t.score(released()) == 0.25
    # Repainting the strip that was already painted (the other pane) earns nothing.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    t.S["target"] = "near_right"
    sweep(-0.0005, t.Z_GLASS)
    t.S["target"] = "near_left"
    assert t.paint_metrics()["coverage"] == 0.0 and t.score(released()) == 0.0
    # Running the stroke on along the stile: more than 2 cm2 of stray paint.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    for y in np.arange(0.30, 0.10, -0.002):
        tcp = (t.XG + 0.010 - t.heel_offset()[0], y, t.Z_WOOD - t.heel_offset()[2])
        t.deposit(*t.brush_geometry(brush_at(tcp))[:3], True, False, "left")
    assert t.paint_metrics()["stray"] > t.STRAY_MAX and t.score(released()) == 0.0
    # Stationary loaded press: a blob the moment it has lasted 1 s (step logic mirrored from on_step).
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    for k in range(25):
        t.S["dwell"]["left"] += 0.05
        if t.S["dwell"]["left"] >= t.BLOB_S - 1e-6:
            t.S["blob"] = True
    assert t.S["blob"]
    # Ferrule driven onto the work.
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    t.deposit(*t.brush_geometry(brush_at((0.30, 0.10, t.TOP + 0.10), c=0.0095))[:3], True, False, "left")
    assert t.S["crushed"]
    # Hands: a gripper face 5 mm above the handle blocks release; 40 mm away passes.
    b = t.BRUSH.state()
    top = np.array(t.BRUSH.pos) + np.array([-0.05, 0, 0.0125])
    assert not t.hands_clear(top[None, :], b)
    assert t.hands_clear((top + [0, 0, 0.035])[None, :], b)
    # Lot B mirrors the ladder: its reference passes score 1.0 and lot A's band stays as painted before.
    lots.use("B")
    t.reset_state()
    t.S["load"]["left"] = t.LOAD_MAX
    for u, z in zip(t.U_PASS, (t.Z_GLASS, t.Z_WOOD)):
        t.S["load"]["right"] = t.LOAD_MAX
        x0, x1, y0, y1 = t.LIGHTS["near_right"]
        tcp_x = t.XG - u - t.heel_offset()[0]
        for y in np.arange(y0 + t.PAD_W / 2, y1 - t.PAD_W / 2 + 1e-9, 0.002):
            t.deposit(*t.brush_geometry(brush_at((tcp_x, y, z - t.heel_offset()[2])))[:3], True, False, "right")
    m = t.paint_metrics()
    assert m["coverage"] >= 0.98 and m["flood"] == 0, m
    lots.use("A")
    t.reset_state()


controls()
CASES = [
    ("do nothing", {}, 0.0, False),
    ("brush lifted off the rest", {"brush_left": ((t.BRUSH.pos[0], t.BRUSH.pos[1], t.BRUSH.pos[2] + 0.05), None, None)}, 0.0, False),
    ("brush left on the bench", {"brush_left": ((0.30, 0.35, t.TOP + 0.009), None, None)}, 0.0, False),
]
