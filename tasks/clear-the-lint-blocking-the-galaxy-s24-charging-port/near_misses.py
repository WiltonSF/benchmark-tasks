"""Near misses and unintended strategies for the charging-port lint task.

The lint rule is a history of tuft-tip positions and contact forces, so the behaviour cases drive the scorer's own
recorder (`task.record`, which the simulator callback calls every control step with the deflected tips of the 40
tufts and each tuft's contact force on the towel). These cases test the scoring logic only: the tips are placed by
hand here (one working row of four tufts in the port, the rest where they lie at the start), not by the physics.
A path gives the working row's tips (inner row t2 in the +u gap, t1 in the -u gap) in the port's coordinates (u through the phone's thickness, v along the
opening, d depth below the mouth, in mm; the four tips sit at v -3.3, -1.1, +1.1, +3.3 mm from the row's position).
Each case starts from a fresh history and scores with the toothbrush set down at its start, both hands clear and
everything still. CASES (run by the offline checker) then checks end states on top of a full, correct history."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import task  # noqa: E402

PHONE = (np.asarray(task.PHONE_POS, float), task.R_PHONE)
REST = task.brush_tips(task.BRUSH.state())  # every tip where the toothbrush lies at the start
# four tufts of each inner row: row t2 (x +1.25 mm) works the +u gap, row t1 (x -1.25 mm) the -u gap
ROWS = {su: [k for k, n in enumerate(task.TUFT_NAMES) if n.startswith(f"t{r}_")][3:7] for su, r in ((1, 2), (-1, 1))}
ROW = ROWS[1] + ROWS[-1]
OFFS = np.array([-3.3, -1.1, 1.1, 3.3])
NO_FORCE = np.zeros(task.N_TUFT)


def tips_at(u, v, d):
    """All tips: the working row's four at port coordinates (mm) around (u, v, d), the rest at rest. The row is the
    inner row that works the gap on u's side."""
    t = REST.copy()
    for k, dv in zip(ROWS[1 if u >= 0 else -1], OFFS):
        t[k] = task.from_port(np.array([u, v + dv, d]) / 1000.0, PHONE)
    return t


LINT_FORCE = np.full(task.N_TUFT, task.F_LINT)
APPLIED = LINT_FORCE.copy()

FORCE_SIGN = -1.0

def _applied(tips):
    """Mock force vectors for recorder tests; this does not test their physical delivery."""
    prev = task._S["prev_tips"]
    delta = np.zeros_like(tips) if prev is None else tips - task.from_port(prev, PHONE)
    return FORCE_SIGN * APPLIED[:, None] * delta / np.maximum(np.linalg.norm(delta, axis=1)[:, None], 1e-20)


def run(frames, force=NO_FORCE):
    for t in frames:
        task.record(PHONE, t, force, _applied(t))


def line(a, b, step=0.25):
    """Tip sets from port point a to b (mm), about ``step`` mm apart."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    return [tips_at(*(a + (b - a) * k / n)) for k in range(n + 1)]


def insert(su=1, d=4.5, sweep=1.5, v=0.0, end=None, step=0.25):
    """Down into gap ``su`` (+1 or -1) to depth d at ``sweep`` mm before v, a stroke to ``sweep`` past it and back,
    then to v and straight out: the scripted demo's motion. With ``end``: one stroke to ``end`` and out from there."""
    u = su * 0.815
    path = line((u, v - sweep, -5.0), (u, v - sweep, d), step)
    if end is not None:
        run(path + line((u, v - sweep, d), (u, end, d), step) + line((u, end, d), (u, end, -5.0), step))
        return
    path += line((u, v - sweep, d), (u, v + sweep, d), step) + line((u, v + sweep, d), (u, v - sweep, d), step)
    path += line((u, v - sweep, d), (u, v, d), step) + line((u, v, d), (u, v, -5.0), step)
    run(path)


def both(**kw):
    insert(1, **kw)
    insert(-1, **kw)


def towel(dx=0.0, n=26, force=0.05, height=0.0005):
    """The loaded row's tips 0.5 mm over the towel, pressed on it with ``force`` N each, wiped 1 mm a step."""
    f = np.zeros(task.N_TUFT)
    f[ROW] = force
    for k in range(n):
        t = REST.copy()
        for j, dv in zip(ROW, np.concatenate([OFFS, OFFS])):
            t[j] = [task.TOWEL_C[0] + 0.012 - 0.001 * k + dx, task.TOWEL_C[1] + dv / 1000.0, task.TOWEL_TOP + height]
        task.record(PHONE, t, f, _applied(t))


def touch_towel(steps=20):
    f = np.zeros(task.N_TUFT)
    f[ROW] = 0.05
    t = REST.copy()
    for j, dv in zip(ROW, np.concatenate([OFFS, OFFS])):
        t[j] = [task.TOWEL_C[0], task.TOWEL_C[1] + dv / 1000.0, task.TOWEL_TOP + 0.0005]
    for _ in range(steps):
        task.record(PHONE, t, f, _applied(t))


def set_down():
    for _ in range(3):
        task.record(PHONE, REST, NO_FORCE)


def released(hold=None):
    task._S.update(known=True, clear=True, supported=True, hold=task.HOLD_STEPS if hold is None else hold)


def start_states():
    return {o.name: o.state() for o in task.OBJECTS}


def full():
    both()
    towel()
    set_down()


def delayed_force_vibration():
    # Same one-control-step delay as _apply_drags: two 0.1mm steps each way. The force can assist reversals.
    tips=REST.copy();k=task.TUFT_NAMES.index('t2_4');uvd=task.LINT_UVD[3].copy()
    tips[k]=task.from_port(uvd,PHONE);applied=np.zeros((task.N_TUFT,3));prev=tips.copy();net_work=0.
    for step in range(120):
        if step:uvd[1]+=(.0001 if (step-1)%4<2 else -.0001)
        tips[k]=task.from_port(uvd,PHONE)
        net_work-=float(np.einsum('ij,ij->',applied,tips-prev))
        drags=task.record(PHONE,tips,NO_FORCE,applied);applied[:]=0
        for j,force in drags:applied[j]+=force
        prev=tips.copy()
    assert net_work<task.F_LINT*task.CATCH_TRAVEL/10, "negative must deliver almost no net resisting work"
    assert np.all(task._S['mode']==task.IN_PORT), "delayed assistance must not earn a capture"


RESULTS = []


def case(label, setup, want, objects=None):
    task.reset_state()
    setup()
    released()
    for k, v in getattr(setup, "after", {}).items():
        task._S[k] = v
    st = start_states()
    st.update(objects or {})
    got = task.score(st)
    RESULTS.append((label, got, want))
    mark = "ok  " if abs(got - want) < 1e-9 else "FAIL"
    print(f"{mark} {label}: {got} (want {want}; left {task.remaining(PHONE, REST)}, towel {task.on_towel()}, "
          f"modes {list(task._S['mode'])})")


def with_flags(fn, **flags):
    fn.after = flags
    return fn


case("delayed-force vibration with almost zero net work cannot capture lint", delayed_force_vibration, 0.0)

case("both gaps: tips 4.5 mm in, a 3 mm stroke each way along the opening, out; wiped 25 mm on the towel; laid down",
     full, 1.0)
case("nothing done", lambda: None, 0.0)
def low_force(value):
    global APPLIED
    APPLIED = np.full(task.N_TUFT, value)
    both()
    towel()
    set_down()
    APPLIED = LINT_FORCE.copy()

def assisting_force():
    global FORCE_SIGN
    FORCE_SIGN = 1.0
    both()
    towel()
    set_down()
    FORCE_SIGN = -1.0

case("0.03 N assisting the tip motion supplies no resisting work", assisting_force, 0.0)

case("no applied lint resistance: geometric sweep earns nothing", lambda: low_force(0.0), 0.0)
case("under-force lint stroke (0.01 N per tuft)", lambda: low_force(0.01), 0.0)

case("tips pushed 4.5 mm into both gaps and pulled straight out (no stroke), towel, laid down",
     lambda: (both(sweep=0.0), towel(), set_down()), 0.0)
case("strokes with the tips 3.0 mm deep (above the lint balls, 3.5-5.5 mm)", lambda: (both(d=3.0), towel(), set_down()),
     0.0)
case("strokes with the tips 3.6 mm deep (in the top of the balls)", lambda: (both(d=3.6), towel(), set_down()), 1.0)
case("strokes with the tips 5.5 mm deep (in the bottom of the balls)", lambda: (both(d=5.5), towel(), set_down()), 1.0)
case("jabbed 6.5 mm deep into both gaps and stroked: all four balls packed onto the floor",
     lambda: (both(d=6.5), towel(), set_down()), 0.0)
case("one gap jabbed 6.5 mm (two balls packed): plug cannot seat, despite other gap cleaned",
     lambda: (insert(1, d=6.5), insert(-1), towel(), set_down()), 0.0)
case("brushed across the outside of the mouth (tips 0.5 mm above it)", lambda: (both(d=-0.5), towel(), set_down()),
     0.0)
case("short strokes, 0.4 mm each way (under 2 mm through a ball)", lambda: (both(sweep=0.4), towel(), set_down()), 0.0)
case("teleport-sized jumps (4 mm a step) while in the port", lambda: (run(
    line((0.815, 0, -5), (0.815, 0, 4.5)) + [tips_at(0.815, 4.0 * (-1) ** k, 4.5) for k in range(30)]
    + line((0.815, 0, 4.5), (0.815, 0, -5))), towel(), set_down()), 0.0)
case("only the +u gap brushed (the two balls beside the tongue on the other side stay)",
     lambda: (insert(1), towel(), set_down()), 0.5)
case("both gaps brushed, then each stroke carried on to 3.5 mm along and out from there: the balls dragged to the "
     "slot's end are scraped off and stay", lambda: (insert(1, end=3.5), insert(-1, end=3.5), towel(), set_down()),
     0.5)
case("all lint out, the loaded bristles set on the towel and lifted without sliding (no wipe)",
     lambda: (both(), touch_towel(), set_down()), 0.75)
case("lint wiped only 8 mm across the towel (under 10 mm)", lambda: (both(), towel(n=9), set_down()), 0.75)
case("lint wiped 25 mm with the tips 0.5 mm over the towel but not pressing on it (no contact force)",
     lambda: (both(), towel(force=0.0), set_down()), 0.75)
case("carried tips 2 mm over towel despite force elsewhere on their shafts: no local transfer",
     lambda: (both(), towel(height=0.002), set_down()), 0.75)
case("lint wiped 25 mm with the tufts pressing under the 0.02 N wipe force (0.01 N)",
     lambda: (both(), towel(force=0.01), set_down()), 0.75)
case("all lint out on the bristles, toothbrush laid down without wiping it", lambda: (both(), set_down()), 0.75)
case("all lint out, then the loaded bristles pushed back into the port: the lint goes back in",
     lambda: (both(), run(line((0.815, 0, -5), (0.815, 0, 4.5)) + line((0.815, 0, 4.5), (0.815, 0, -5))), towel(),
              set_down()), 0.5)
case("all lint out and wiped off, then the clean bristles go back in and out", lambda: (
    both(), towel(), run(line((0.815, 0, -5), (0.815, 0, 4.5)) + line((0.815, 0, 4.5), (0.815, 0, -5))), set_down()),
     1.0)
case("all lint out and wiped off, then the bristles jabbed 7 mm in (nothing left to pack)", lambda: (
    both(), towel(), run(line((0.815, 0, -5), (0.815, 0, 7.0)) + line((0.815, 0, 7.0), (0.815, 0, -5))), set_down()),
     1.0)
case("lint wiped on the counter just off the towel's edge (not on it)", lambda: (both(), towel(dx=0.08), set_down()),
     0.75)
case("full clean but a hand still within 30 mm (holding the brush)", with_flags(full, clear=False), 0.0)
case("full clean, still for only 19 steps (under 1 s)", with_flags(full, hold=task.HOLD_STEPS - 1), 0.0)
case("full clean, still for exactly 20 steps (1 s)", with_flags(full, hold=task.HOLD_STEPS), 1.0)
case("full clean but a hand or the brush's head, neck or handle touched the port end",
     with_flags(full, hard_in_port=True), 0.0)
case("full clean but the phone was knocked in its stand", with_flags(full, knocked=True), 0.0)
case("full clean but the brush or phone fell off the counter", with_flags(full, dropped=True), 0.0)
case("full clean but a pose, contact or hand read failed (fail closed)", with_flags(full, broken=True), 0.0)
case("native shaft penetrates the colliding phone beyond solver tolerance", with_flags(full, broken=True), 0.0)
case("unreadable or obscured end quarter: evaluator cannot certify the outcome",
     with_flags(full, broken=True), 0.0)
def joint_fault_then_recover():
    from dataclasses import replace
    full()
    rest=task.BRUSH.state();j=task.TUFT_JOINTS[0]
    task.audit_brush_joints(replace(rest,joints=rest.joints|{j.name:j.upper+2*task.JOINT_SOLVER_TOL}))
    task.audit_brush_joints(rest)
case("invalid native joint coordinates then recovered geometry still fail closed",joint_fault_then_recover,0.0)
case("full clean but the scene was never read (fail closed)", with_flags(full, known=False), 0.0)

def packed_after_clean():
    full()
    task._S["mode"][0] = task.PACKED
case("one packed quarter prevents plug seating even with three clean quarters",packed_after_clean,0.0)

def force_free_withdrawal():
    global APPLIED
    u=.815
    run(line((u,-1.5,-5.),(u,-1.5,4.5)) + line((u,-1.5,4.5),(u,1.5,4.5))
        + line((u,1.5,4.5),(u,-1.5,4.5)) + line((u,-1.5,4.5),(u,0.,4.5)))
    assert (task._S["mode"] == task.CARRIED).any(), "negative case must first hook lint normally"
    APPLIED=NO_FORCE.copy()
    run(line((u,0.,4.5),(u,0.,-5.)))
    APPLIED=LINT_FORCE.copy()
    towel();set_down()
case("normal loaded hook followed by force-free withdrawal cannot certify extraction",force_free_withdrawal,0.0)

def delayed_force_extraction():
    tips=REST.copy();k=task.TUFT_NAMES.index('t2_4');uvd=task.LINT_UVD[3].copy()
    tips[k]=task.from_port(uvd,PHONE);applied=np.zeros((task.N_TUFT,3));previous=tips.copy();work=0.
    def step(delta):
        nonlocal applied,previous,work
        uvd[:]+=delta;tips[k]=task.from_port(uvd,PHONE)
        work-=float(np.einsum('ij,ij->',applied,tips-previous))
        drags=task.record(PHONE,tips,NO_FORCE,applied);applied[:]=0
        for j,force in drags:applied[j]+=force
        previous=tips.copy()
    step([0,0,0])
    for n in range(100):
        step([0,.0001 if n%16<8 else -.0001,0])
        if task._S['mode'][3]==task.CARRIED:break
    assert task._S['mode'][3]==task.CARRIED, 'must first capture with real resisted work'
    depth0=float(uvd[2]);work=0.
    for n in range(2000):
        step([0,0,-.0001 if n%4<2 else .00009])
        if task._S['out'][3] or task._S['broken']:break
    assert work<task.F_LINT*(depth0-uvd[2])/10, 'negative must extract almost without net real work'
    assert task._S['broken'] and not task._S['out'][3], 'force-lag pumping must not certify extraction'
case('delayed-force pumping cannot extract captured lint without net retention work',delayed_force_extraction,0.)


def uncertainty_history(extra):
    global APPLIED
    original=task.record;previous=APPLIED.copy();APPLIED=np.full(task.N_TUFT,task.F_LINT+extra)
    def receipt(pp,tips,ft,fl=None):return original(pp,tips,ft,fl,force_error=1e-6)
    task.record=receipt
    try:full()
    finally:task.record=original;APPLIED=previous
    if extra==0:assert np.all(task._S['mode']==task.IN_PORT), 'uncertain threshold force must not capture'
case('native force error: a nominal threshold force is not certified above its uncertainty',lambda:uncertainty_history(0.),0.)
case('native force error: independently resisted positive path above the margin remains possible',lambda:uncertainty_history(.0001),1.)

BAD = [r for r in RESULTS if abs(r[1] - r[2]) > 1e-9]
if BAD:
    raise SystemExit(f"near_misses: {len(BAD)} behaviour case(s) failed: {BAD}")

# End-state cases on top of a full, correct history (run in order by the offline checker).
task.reset_state()
full()
released()
B = task.BRUSH
P = task.PHONE
BP = np.asarray(B.pos, float)


def _turn_y(deg):
    m = task.R_PHONE @ task.parts._rot((0.0, 1.0, 0.0), np.radians(deg))
    return tuple(task.parts._matrix_to_quat(m))


from dataclasses import replace
_J=task.TUFT_JOINTS[0]
_INVALID_JOINT=replace(B.state(),joints=B.state().joints|{_J.name:_J.upper+2*task.JOINT_SOLVER_TOL})
_NAN_JOINT=replace(B.state(),joints=B.state().joints|{_J.name:float("nan")})
assert task.brush_joints_valid(B.state())
assert not task.brush_joints_valid(_INVALID_JOINT)
assert not task.brush_joints_valid(_NAN_JOINT)
assert not task.brush_joints_valid(replace(B.state(),joints={}))
# A missing native swing must fail even when the other swing and visible geometry remain valid.
_NATIVE_ZERO={j.name+":"+str(axis):0.0 for j in task.TUFT_JOINTS for axis in (1,2)}
assert task.brush_joints_valid(replace(B.state(),joints=_NATIVE_ZERO))
_MISSING_SWING=dict(_NATIVE_ZERO);_MISSING_SWING.pop(next(iter(_MISSING_SWING)))
assert not task.brush_joints_valid(replace(B.state(),joints=_MISSING_SWING))
CASES = [
    ("clean outcome with out-of-range native joint coordinate",{"toothbrush":_INVALID_JOINT},0.0,False),
    ("clean outcome with nonfinite native joint coordinate",{"toothbrush":_NAN_JOINT},0.0,False),
    ("cleaned, wiped, toothbrush back where it lay, all still", {}, 1.0, True),
    ("toothbrush still sliding (2 cm/s)", {"toothbrush": B.state(lin_vel=(0.0, 0.02, 0.0))}, 0.0, False),
    ("toothbrush lying on the paper towel", {"toothbrush": B.state(pos=(0.40, -0.21, task.TOP + task.REST_H + 0.0015))},
     1.0, True),
    ("toothbrush 1 cm above the counter (not set down)", {"toothbrush": B.state(pos=tuple(BP + [0, 0, 0.010]))}, 0.0,
     False),
    ("toothbrush left lying across the phone's top", {"toothbrush": B.state(pos=(0.33, -0.12, task.TOP + 0.16))}, 0.0,
     False),
    ("toothbrush left on the stand's base against the phone",
     {"toothbrush": B.state(pos=(0.29, -0.125, task.TOP + 0.0045 + task.REST_H))}, 0.0, False),
    ("phone moved 8 mm in its stand", {"phone": P.state(pos=tuple(np.asarray(P.pos) + [0.0, 0.008, 0.0]))}, 0.0,
     False),
    ("phone tipped 6 degrees further back", {"phone": P.state(quat_xyzw=_turn_y(6.0))}, 0.0, False),
    ("phone settled 0.3 mm onto its ledge (normal)", {"phone": P.state(pos=tuple(
        np.asarray(P.pos) - task.R_PHONE @ [0.0, 0.0, 0.0003]))}, 1.0, True),
    ("cleaned, wiped, toothbrush back where it lay, still (after the cases above)", {}, 1.0, True),
]

# Independent geometry guard: a resting brush is clear; a shaft deliberately placed in a solid frame is invalid.
from contact_audit.geometry import shaft_phone_penetrations
hits, _ = shaft_phone_penetrations(task.TUFT_JOINTS, task.PHONE.shapes, task.BRUSH.state(), PHONE)
assert hits.max() == 0.0
bad_target = task.from_port(np.array([0.003, 0.0, 0.003]), PHONE)
bad_brush = task.BRUSH.state(pos=np.asarray(task.BRUSH.pos) + bad_target - REST[15])
hits, _ = shaft_phone_penetrations(task.TUFT_JOINTS, task.PHONE.shapes, bad_brush, PHONE)
assert hits.max() > task.MAX_SHAFT_OVERLAP
print("ok   native shaft geometry guard: clear rest and deliberate solid penetration")

# Carried lint in air earns no deposit event. At gap u=0.815 mm a 2 mm extracted proxy contacts the wall;
# centred at u=0, only 0.5 mm in, it has not reached a wall or the tongue and must remain carried.
assert not task.carried_lint_contacts_phone(np.array([0., 0., 0.0005]), True)
assert task.carried_lint_contacts_phone(np.array([0.000815, 0., 0.0005]), True)
assert task.carried_lint_contacts_phone(np.array([0., 0., 0.0015]), True)
print("ok   carried-lint return requires contact with a phone solid, not an empty window")
