"""Near misses and unintended strategies for thickening the drink, in lot A (30 mL of thickener) unless stated.

The score rests on a history (the powder's tracked volumes, lumps, whether the hands were clear, the still hold),
so behaviour cases drive the scorer's own recorder (`task.record`, which the simulator calls every control step)
through a sequence of states. Most move the whisk and the scoop as if held rigidly by the demo's own targets
(their poses only), so the powder model itself decides what lands where, what is wetted and what lumps. CASES (run
by factory/check.py) then check end states on top of a full, valid history."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lots  # noqa: E402
import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

lots.use("A")
DT = task.DT
P = task.parts


def q_of(R):
    """xyzw from a rotation matrix (Shepperd's method: exact for half turns too)."""
    m = np.asarray(R, float)
    tr = np.trace(m)
    k = int(np.argmax([tr, m[0, 0], m[1, 1], m[2, 2]]))
    if k == 0:
        w = math.sqrt(1 + tr) / 2
        x, y, z = (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)
    elif k == 1:
        x = math.sqrt(1 + 2 * m[0, 0] - tr) / 2
        w, y, z = (m[2, 1] - m[1, 2]) / (4 * x), (m[0, 1] + m[1, 0]) / (4 * x), (m[0, 2] + m[2, 0]) / (4 * x)
    elif k == 2:
        y = math.sqrt(1 + 2 * m[1, 1] - tr) / 2
        w, x, z = (m[0, 2] - m[2, 0]) / (4 * y), (m[0, 1] + m[1, 0]) / (4 * y), (m[1, 2] + m[2, 1]) / (4 * y)
    else:
        z = math.sqrt(1 + 2 * m[2, 2] - tr) / 2
        w, x, y = (m[1, 0] - m[0, 1]) / (4 * z), (m[0, 2] + m[2, 0]) / (4 * z), (m[1, 2] + m[2, 1]) / (4 * z)
    return (float(x), float(y), float(z), float(w))


def start_pose(name):
    lot = lots.current()
    return np.array(lot.pos(name)), P._rot((0, 0, 1), lot.yaw_of(name))


SPEC = {"tumbler": task.TUMBLER, "whisk": task.WHISK, "scoop": task.SCOOP, "holder": task.HOLDER,
        "coaster": task.COASTER, "placemat": task.PLACEMAT}


def world(**poses):
    """States of every object: start poses unless given as name=(pos, R[, lin_vel])."""
    out = {}
    for name, spec in SPEC.items():
        if name in poses:
            p, R = poses[name][0], poses[name][1]
            v = poses[name][2] if len(poses[name]) > 2 else None
        else:
            p, R = start_pose(name) if spec.mass is not None else (np.array(spec.pos), np.eye(3))
            v = None
        out[name] = spec.state(pos=tuple(float(x) for x in p), quat_xyzw=q_of(R),
                               lin_vel=None if v is None else tuple(float(x) for x in v))
    return out


def T(p, R):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


def held_track(plan_wps, arm, obj, until=None, amp=1.0, centre=None):
    """The object's world pose at each control step while the arm's demo plan holds it rigidly (from the step the
    fingers have closed to the step they open), else None. `amp` scales the arm's motion about `centre` (to model
    the arm tracking a small stirring circle with less than its full amplitude)."""
    plan = ArmPlan(list(plan_wps), station.BASES[arm])
    n = int(round((until or plan.duration) / DT))
    p0, R0 = start_pose(obj)
    grip_T = None
    out = []
    for k in range(n):
        pos, rot, finger, label = plan.target((k + 1) * DT)
        if centre is not None and label == "stir":
            pos = centre + amp * (pos - centre)
        closed = finger > -0.010
        if closed and grip_T is None:
            grip_T = np.linalg.inv(T(pos, rot)) @ T(p0, R0)
        if not closed and grip_T is not None:
            out.append(None)
            continue
        out.append(None if grip_T is None else T(pos, rot) @ grip_T)
    return out


def play(no_stir=False, stir_amp=1.0, pour_speed=1.0, lot="A", stir_late=0.0, released=True,
         end=None, tumbler_shift=None):
    """Run the demo's object motion through the recorder. Options model unintended strategies."""
    lots.use(lot)
    task.reset_state()
    plans = task.demo()
    stir_arm, pour_arm = task._arms(lots.current())
    lt = lots.current()
    d = np.array([*lt.d, 0.0])
    sc_pts = np.array([w.pos for w in plans[stir_arm] if w.label == "stir"])
    sw = held_track(plans[stir_arm], stir_arm, "whisk", amp=stir_amp, centre=sc_pts.mean(axis=0))
    pw = plans[pour_arm]
    if pour_speed != 1.0:
        pw = [type(w)(w.duration / pour_speed if w.label.startswith("scoop") else w.duration, w.pos, w.finger,
                      w.label, yaw=float(w.yaw), tilt=getattr(w, "tilt", 0.0), heading=getattr(w, "heading", 0.0))
              for w in pw]
    sc = held_track(pw, pour_arm, "scoop")
    n = max(len(sw), len(sc)) + 40
    tum0, tumR = start_pose("tumbler")
    if tumbler_shift is not None:
        tum0 = tum0 + np.asarray(tumbler_shift)
    whisk_last, scoop_last = start_pose("whisk"), start_pose("scoop")
    w_rel_out = False
    for k in range(n):
        wm = sw[k] if k < len(sw) else None
        if no_stir and wm is not None:
            wm = None  # the whisk is never brought to the drink
        if stir_late and wm is not None and k * DT < stir_late:
            wm = None
        sm = sc[k] if k < len(sc) else None
        holding = wm is not None or sm is not None
        if wm is not None:
            whisk_last = (wm[:3, 3], wm[:3, :3])
        elif k > 0 and sw[min(k, len(sw) - 1)] is None and any(x is not None for x in sw[:k]) and not w_rel_out:
            # released into the holder: it drops to the floor of the cup
            hp, _ = start_pose("whisk")
            whisk_last = (hp, np.eye(3))
            w_rel_out = True
        if sm is not None:
            scoop_last = (sm[:3, 3], sm[:3, :3])
        elif any(x is not None for x in sc[:k]):
            p, R = scoop_last
            scoop_last = (np.array([p[0], p[1], task.MAT_Z]), R)
        st = world(tumbler=(tum0, tumR), whisk=whisk_last, scoop=scoop_last)
        task.record(st, (not holding) and released)
    return st


def scripted(seconds, whisk_tip, gamma, lot="A"):
    """Drive the whisk and the scoop directly: the whisk stands upright with its tip at whisk_tip(t) (tumbler frame,
    m, or None: lifted out of the drink), and the scoop is held over the tumbler tipped gamma(t) degrees. Every
    step is recorded with the hands not clear (still holding)."""
    lots.use(lot)
    task.reset_state()
    lt = lots.current()
    tum0, tumR = start_pose("tumbler")
    syaw = lt.yaw_of("scoop")
    st = None
    for k in range(int(round(seconds / DT))):
        t = (k + 1) * DT
        tip = whisk_tip(t)
        if tip is None:
            wp = (tum0 + [0.0, 0.0, 0.12], np.eye(3))
        else:
            wp = (tum0 + np.asarray(tip), np.eye(3))
        o, R = task.pour_pose(lt.d, lt.e, syaw, gamma(t))
        st = world(tumbler=(tum0, tumR), whisk=wp, scoop=(o, R))
        task.record(st, False)
    return st


TIP_Z = task.T_FLOOR + 0.006


def circle(r, hz, t0=0.0, cx=0.0):
    return lambda t: (cx + r * math.cos(2 * math.pi * hz * (t - t0)), r * math.sin(2 * math.pi * hz * (t - t0)), TIP_Z)


def ramp(t_start, t_end, g0, g1):
    return lambda t: g0 + (g1 - g0) * min(1.0, max(0.0, (t - t_start) / max(t_end - t_start, 1e-9)))


def finish(st=None, s=1.5, clear=True):
    st = st or world()
    for _ in range(round(s / DT)):
        task.record(st, clear)
    return st


RESULTS = []

def case(label, run, want):
    st = run()
    got = task.score(st if st is not None else world())
    RESULTS.append((label, got, want))
    print(f"{'ok  ' if abs(got-want)<1e-9 else 'FAIL'} {label}: {got} (want {want})", flush=True)


def made(share=1.0, clear=True):
    lots.use("A")
    task.reset_state()
    dose=task._L["dose"]
    task._L["hyd"][:]=share*dose/task.N_REG
    task._L["scoop"]=dose*(1-share)
    task._L["known"]=True
    task._L["clear"]=clear


def material(amount, destination):
    made()
    task._L["hyd"]-=amount/task.N_REG
    if destination=="spill": task._L["spill"]=amount
    else: task._L[destination][1]=amount
    return finish()


def putaway(whisk=True, scoop=True, seconds=1.5, clear=True):
    made()
    poses={}
    if not whisk:
        R=P._rot((1,0,0),math.pi/2);p=np.array([.20,.50,task.TOP]);temp=task.WHISK.state(pos=tuple(p),quat_xyzw=q_of(R));p[2]+=task.TOP-float(temp.low[2]);poses["whisk"]=(p,R)
    if not scoop:
        p,R=start_pose("scoop");poses["scoop"]=(np.array([p[0],-.40,task.TOP]),R)
    st=world(**poses)
    return finish(st,s=seconds,clear=clear)


case("demo lot A, revised delivery/defect contract",lambda:play(),1.0)
case("demo lot B",lambda:play(lot="B"),1.0)
case("demo lot C",lambda:play(lot="C"),1.0)
case("40 percent stirring amplitude",lambda:play(stir_amp=0.4),1.0)
case("30 percent stirring amplitude forms defects",lambda:play(stir_amp=0.3),0.0)
case("dump under a moving whisk",lambda:scripted(40,circle(.010,2),lambda t:0 if t<5 else 42),0.0)
case("one-second pour under a moving whisk",lambda:scripted(40,circle(.010,2),ramp(5,6,18,42)),0.0)
case("no whisking",lambda:play(no_stir=True),0.0)
case("late whisking",lambda:play(stir_late=45),0.0)
case("twice-speed controlled pour",lambda:play(pour_speed=2),1.0)
case("three-times-speed pour",lambda:play(pour_speed=3),0.0)
case("layout-A replay at C cup position",lambda:play(tumbler_shift=(-.03,.07,0)),0.0)
case("material done but hands never clear",lambda:play(released=False),0.0)
case("nothing done",lambda:(task.reset_state(),finish())[1],0.0)
case("delivery exactly 50 percent",lambda:(made(.50),finish())[1],.25)
case("delivery 49.9 percent",lambda:(made(.499),finish())[1],0.0)
case("delivery exactly 95 percent and both tools stowed",lambda:(made(.95),finish())[1],1.0)
case("delivery 94.9 percent",lambda:(made(.949),finish())[1],.25)
case("no tool stowed",lambda:putaway(False,False),.5)
case("only whisk stowed",lambda:putaway(True,False),.75)
case("only scoop stowed",lambda:putaway(False,True),.75)
case("both tools stowed",lambda:putaway(True,True),1.0)
case("only 0.9 seconds of release/stillness",lambda:putaway(seconds=.9),0.0)
case("held tools cannot earn any positive tier",lambda:putaway(clear=False),0.0)
case("dry marker just below observable boundary",lambda:material(task.DRY_VISIBLE*.999,"dry"),1.0)
case("dry marker at observable boundary",lambda:material(task.DRY_VISIBLE,"dry"),0.0)
case("any rendered agglomerate fails",lambda:material(task.LUMP_VISIBLE*2,"lumps"),0.0)
case("former 0.049 mL allowed lump now fails",lambda:material(.049,"lumps"),0.0)
case("4.966 percent powder spill",lambda:material(1.49,"spill"),1.0)
case("exactly 5 percent powder spill",lambda:material(1.5,"spill"),0.0)


def unknown():
    made();finish();task._L["known"]=False;return world()
case("unknown hands",unknown,0.0)

def created():
    made();finish();task._L["hyd"][0]+=.01;return world()
case("created powder inventory",created,0.0)

def negative():
    made();finish();task._L["hyd"][0]=-1.;return world()
case("negative powder inventory",negative,0.0)

def nonfinite():
    made();finish();task._L["hyd"][0]=float("nan");return world()
case("nonfinite powder inventory",nonfinite,0.0)

def tipped():
    made();p,R=start_pose("tumbler");st=world(tumbler=(p,P._rot((1,0,0),math.radians(26))))
    task.record(st,True);return finish()
case("juice spill cannot be erased",tipped,0.0)

def bad_pose():
    made();st=world();st["whisk"]=task.WHISK.state(pos=(float("nan"),0,1));task.record(st,True)
    return finish()
case("nonfinite physical interval stays invalid",bad_pose,0.0)

def off_mat():
    made();p,R=start_pose("scoop");st=world(scoop=(np.array([.31,.40,task.TOP]),R))
    assert not task._scoop_down(st["scoop"])
    return finish(st)
case("supported off-mat scoop is not fully stowed",off_mat,.75)

def airborne_holder():
    made();p,R=start_pose("whisk");h,H=start_pose("holder")
    st=world(whisk=(p+[0,0,.08],R),holder=(h+[0,0,.08],H))
    return finish(st)
case("airborne holder cannot support a credited whisk",airborne_holder,0.0)

def embedded_whisk():
    made();p,R=start_pose("whisk")
    return finish(world(whisk=(p+[0,0,-.002],R)))
case("whisk 2 mm below holder floor cannot earn credit",embedded_whisk,0.0)

def tilted_embedded_whisk():
    made();p,R=start_pose("whisk")
    return finish(world(whisk=(p,P._rot((1,0,0),math.radians(10)))))
case("tilted head penetrating holder floor cannot earn credit",tilted_embedded_whisk,0.0)

def tilted_supported_whisk():
    made();p,_=start_pose("whisk");R=P._rot((1,0,0),math.radians(10))
    vertices=np.concatenate([np.asarray(s.points) for s in task.WHISK.shapes if s.collide])
    p[2]-=float((vertices@R.T)[:,2].min())
    return finish(world(whisk=(p,R)))
case("tilted head resting exactly on holder floor earns credit",tilted_supported_whisk,1.0)

def embedded_scoop():
    made();p,R=start_pose("scoop");p[2]=task.TOP
    return finish(world(scoop=(p,R)))
case("scoop embedded in raised mat is not table-supported",embedded_scoop,0.0)

def numerical_contacts(depth):
    made();poses={}
    for name in ("tumbler","whisk","scoop"):
        p,R=start_pose(name);p[2]-=depth;poses[name]=(p,R)
    return finish(world(**poses))
case("0.2 mm numerical contact penetration is tolerated",lambda:numerical_contacts(.0002),1.0)
case("0.5 mm penetration exceeds numerical allowance",lambda:numerical_contacts(.0005),0.0)

def held_half():
    made(.5);return finish(clear=False)
case("held half-dose cannot earn partial credit",held_half,0.0)

def wet_scoop():
    made(.5);st=world();p=np.asarray(st["tumbler"].pos)+[0,0,task.T_FLOOR+.010]
    st["scoop"]=task.SCOOP.state(pos=tuple(p));task.record(st,True)
    assert task._L["measurement_invalid"]
    return finish()
case("wet scoop residue makes weighing ungradable",wet_scoop,0.0)

def unrecovered_spill():
    made();task._L["hyd"]-=.01/task.N_REG;task._note_spill(.01,np.array([1.,1.]))
    assert task._L["measurement_invalid"]
    return finish()
case("powder beyond recovery liner invalidates accounting",unrecovered_spill,0.0)

# The union of one arm's links cannot certify both arms.
from types import SimpleNamespace
pose=np.tile(np.array([10.,10.,10.,0.,0.,0.,1.]),(1,3,1))
def asset(names):return SimpleNamespace(body_names=names,data=SimpleNamespace(body_link_pose_w=pose.copy()))
full=list(task.HAND_LINKS)
class Scene(dict):
    env_origins=np.zeros((1,3))
scene=Scene(left_robot=asset(full),right_robot=asset(full))
base=SimpleNamespace(scene=scene)
scene["env_origins"]=np.zeros((1,3))
sim=SimpleNamespace(env=SimpleNamespace(unwrapped=base))
np_original=task.parts._np
try:
    task.parts._np=lambda x:np.asarray(x)
    assert task._hands_clear(sim,world()) is True,"complete hands should be measurable"
    scene["right_robot"]=asset(full[:-1])
    assert task._hands_clear(sim,world()) is None,"missing right link accepted"
    del scene["right_robot"]
    assert task._hands_clear(sim,world()) is None,"missing right arm accepted"
    scene["right_robot"]=asset(full)
    scene["right_robot"].data.body_link_pose_w[0,0,0]=np.nan
    assert task._hands_clear(sim,world()) is None,"nonfinite hand accepted"
finally:task.parts._np=np_original
assert not task.points_clear(np.array([[np.nan,0,0]]),world())
print("ok   complete/missing-arm/missing-link/nonfinite hand checks",flush=True)

_tw=world()["tumbler"]
_x=float(_tw.pos[0])-task.T_ROUT-.0005
_face=np.array([[_x,float(_tw.pos[1])+dy,float(_tw.pos[2])+.03+dz] for dy in np.linspace(-.006,.006,5) for dz in np.linspace(-.006,.006,5)])
assert not task.points_clear(_face,world())
assert task.points_clear(_face-np.array([.01,0,0]),world())
print("ok   broad-face release control",flush=True)

bad=[r for r in RESULTS if abs(r[1]-r[2])>1e-9]
if bad:raise AssertionError(bad)
made();finish()
TP,TR=start_pose("tumbler");HP,HR=start_pose("whisk");SP,SR=start_pose("scoop")
CASES=[
 ("both tools supported and released",{},1.0,True),
 ("whisk hovering 4 mm above holder floor",{"whisk":task.WHISK.state(pos=tuple(HP+[0,0,.004]))},0.0,False),
 ("scoop held 4 mm above mat",{"scoop":task.SCOOP.state(pos=tuple(SP+[0,0,.004]),quat_xyzw=q_of(SR))},0.0,False),
 ("tumbler 16 mm off its start",{"tumbler":task.TUMBLER.state(pos=tuple(TP+[.016,0,0]))},0.0,False),
 ("stale state after scoop moves outside mat",{"scoop":task.SCOOP.state(pos=(.31,.40,task.TOP),quat_xyzw=q_of(SR))},0.0,False),
 ("holder and whisk airborne together",{"holder":task.HOLDER.state(pos=tuple(np.asarray(task.HOLDER.pos)+[0,0,.08])),"whisk":task.WHISK.state(pos=tuple(HP+[0,0,.08]))},0.0,False),
]
print(f"{len(RESULTS)} behaviour cases and hand/release regressions passed",flush=True)
