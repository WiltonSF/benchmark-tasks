"""Near misses and unintended strategies, each played as a short timeline through the scorer's own step record.

The scorer keeps a per-trial record that only the simulator's step and reset hooks write: whether the tool's barbed
pad has snagged the clump (gone 10 mm into it, then drawn back 8 mm through it), whether the snagged clump was drawn
out of the drain's throat under the hair's load (and not jerked, pushed back or crept out), whether the tool ever
dropped off the vanity, whether both grippers are clear, and how long everything has been still. Each case resets that record, plays a timeline of states at 20 Hz
through every loaded copy of the task's step function, and leaves the timeline's last state for check.py to score.
Run alone: python near_misses.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task  # noqa: E402
import lots  # noqa: E402
import parts  # noqa: E402

DT = 0.05
MM = 1e-3
TOP, ZF, CT = task.TOP, task.ZF, task.CT
FAR = [np.array([[0.20, 0.30, TOP + 0.45]])]  # the hands parked well away
DOWN = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))  # tool frame x pointing straight down (a quarter turn about y)


def _q(R):
    return tuple(parts._matrix_to_quat(R))


def lying(lot="A", dx=0.0, dy=0.0, dz=0.0, joints=None):
    """The tool lying as the lot starts it, moved by (dx, dy, dz)."""
    p = np.asarray(task.LOTS[lots.index(lot)].pos(task.TOOL.name), float)
    p[2] = task.REST_Z
    p = p + [dx, dy, dz]
    return task.TOOL.state(pos=p, quat_xyzw=task.LYING, joints=joints)


def upright(pad_depth, r=0.011):
    """The tool held straight up and down over the drain, its pad's centre ``pad_depth`` below the flange's top and
    ``r`` from the drain's axis on the wand's side (only the pad's position matters to the scorer)."""
    side = -np.array([math.cos(task.PSI), math.sin(task.PSI), 0.0])
    pad = np.array([task.BX, task.BY, TOP + ZF - pad_depth]) + side * r
    origin = pad + np.array([0.0, 0.0, task.WAND_L - task.PAD_LINKS * task.LINK_L / 2])
    return task.TOOL.state(pos=origin, quat_xyzw=DOWN)


def hair_on(tool):
    """The clump where the declared rule carries it once caught: on the pad."""
    lp, lq = tool.links[task.LAST]
    p = np.asarray(lp, float) + task._R(lq) @ task.HAIR_ON_PAD
    return task.HAIR_PART.state(pos=p, quat_xyzw=lq)


def in_drain():
    return task.HAIR_PART.state()


def scene(tool, hair=None, vel=None):
    if vel is not None:
        tool = task.TOOL.state(pos=tool.pos, quat_xyzw=tool.quat_xyzw, lin_vel=vel)
    return {task.TOOL.name: tool, task.HAIR_PART.name: hair if hair is not None else in_drain()}


def _instances():
    """Every loaded copy of task.py (check.py loads its own), found through the step hooks they registered."""
    seen = {}
    for fn in parts._ON_STEP:
        g = fn.__globals__
        if "observe" in g and "REC" in g:
            seen[id(g["REC"])] = g
    return list(seen.values())


def play(*segments):
    """Reset every copy's trial record, then play (objects, hands, seconds) segments at 20 Hz."""
    t = 0.0
    for g in _instances():
        g["REC"].clear()
        g["REC"].update(g["_fresh"]())
    last = None
    for objects, hands, seconds in segments:
        n = max(1, round(seconds / DT))
        for _ in range(n):
            t += DT
            for g in _instances():
                g["observe"](objects, hands, t)
            last = objects
    return last if last is not None else scene(lying())


def near_tool(objects, gap):
    """One gripper point ``gap`` above the top of the tool's handle grip."""
    tool = objects[task.TOOL.name]
    p = np.asarray(tool.pos, float) + task._R(tool.quat_xyzw) @ np.array([task.GRIP_AT, 0.0, 0.0])
    return FAR + [np.array([[p[0], p[1], p[2] + task.GRIP_R + gap]])]


IN = upright(0.035)  # pad 35 mm down, 13 mm into the clump
HAND = [np.array([[0.30, 0.20, TOP + 0.30]])]  # the hand holding the tool, well away from it


def path(*legs, r=0.011):
    """Segments that move the pad, a control step at a time, through ``legs`` of (depth below the flange's top
    to reach, speed in m/s); the first leg's speed is ignored (the pad starts there and rests 0.3 s)."""
    depth = legs[0][0]
    out = [(scene(upright(depth, r)), HAND, 0.3)]
    for target, speed in legs[1:]:
        n = max(1, round(abs(target - depth) / (speed * DT)))
        for k in range(1, n + 1):
            out.append((scene(upright(depth + (target - depth) * k / n, r)), HAND, DT))
        depth = target
    return out


def drawn_out(final_tool, hold=1.2, depth=0.035, speed=0.020, hair=None, r=0.011, legs=None):
    """Pad ``depth`` below the flange's top, drawn steadily up the bore at ``speed`` to 12 mm above the flange's top
    (or along ``legs``), then the tool and the clump left as ``final_tool`` for ``hold`` s."""
    end = scene(final_tool, hair if hair is not None else hair_on(final_tool))
    legs = legs or [(depth, 0.0), (-0.012, speed)]
    return play(*path(*legs, r=r), (end, FAR, hold))


def _along(direction, origin, joints=None):
    """The tool with its axis along ``direction`` (wand end first), its hinges' axis level, wand junction at ``origin``."""
    x = np.asarray(direction, float) / np.linalg.norm(direction)
    y = np.cross([0.0, 0.0, 1.0], x)
    y /= np.linalg.norm(y)
    return task.TOOL.state(pos=origin, quat_xyzw=_q(np.column_stack([x, y, np.cross(x, y)])), joints=joints)


def in_basin():
    """The tool lying in the basin, its handle end up the bowl's left side, the wand's end curled up the right side."""
    return _along((0.0, -1.0, -0.25), (task.BX, task.BY + 0.090, TOP + ZF + 0.030),
                  joints={f"w{i}": -0.30 for i in range(task.N_LINKS)})


def across_rim():
    """The wand's end and the hair on the countertop's front strip, the handle down in the basin."""
    end = np.array([task.CTR_FRONT + 0.040, task.BY - 0.020, TOP + CT + 0.016])
    d = np.array([-0.30, -0.05, 0.40])  # from the handle down in the bowl up toward the front strip
    d /= np.linalg.norm(d)
    return _along(d, end - d * task.WAND_L)


class _Timelines:
    """check.py iterates CASES in order; each case's timeline is played just before check.py scores it."""

    def __init__(self, cases):
        self.cases = cases

    def __iter__(self):
        for label, build, want, ok in self.cases:
            lots.use("B" if label.startswith("lot B") else "A")
            yield label, build(), want, ok
        lots.use(None)


def hair_at(z_above_flange, r=0.010):
    """A caught clump left ``r`` from the drain's axis, its centre this far above the flange's top."""
    return task.HAIR_PART.state(pos=(task.BX - r, task.BY, TOP + ZF + z_above_flange))


def moved_after(objects, tool_by=(0.0, 0.0, 0.0), hair_by=(0.0, 0.0, 0.0)):
    """The state the step scores: the simulator moves the caught clump onto the pad after each observation (the
    declared rule), so the scored hair can sit a little off the observed one; the tool is never moved in between."""
    tool, hair = objects[task.TOOL.name], objects[task.HAIR_PART.name]
    tool = task.TOOL.state(pos=np.asarray(tool.pos) + tool_by, quat_xyzw=tool.quat_xyzw, joints=tool.joints)
    hair = task.HAIR_PART.state(pos=np.asarray(hair.pos) + hair_by, quat_xyzw=hair.quat_xyzw)
    return {task.TOOL.name: tool, task.HAIR_PART.name: hair}


def turned(deg, lot="A"):
    """The tool lying where the lot starts it, turned ``deg`` about the vertical through where the wand meets the handle
    (negative: the wand's end swings toward the top's front edge)."""
    t = lying(lot)
    return task.TOOL.state(pos=t.pos, quat_xyzw=_q(parts._rot((0.0, 0.0, 1.0), math.radians(deg)) @ task._R(task.LYING)))


HELD = (0.033, 0.0)  # the pad 11 mm into the clump
# snag (13 mm in, drawn up to 24 mm below the flange's top), then 12 strokes in place: down 4.8 mm in one step, back up
PUMP = [(0.035, 0.0), (0.024, 0.02)] + [(0.0288, 0.096), (0.024, 0.008)] * 12  # 40 mm of up-strokes under load


SPEC = [
    ("no history: nothing observed yet", lambda: play() or scene(lying()), 0.0, False),
    ("arms idle: tool on the counter, hair in the drain", lambda: play((scene(lying()), FAR, 2.0)), 0.0, False),
    ("full: pad 13 mm into the clump, drawn up the bore at 20 mm/s under the hair's load (snagged after 8 mm, free at"
     " the flange's top), tool laid back on the counter with the hair, hands clear, still 1.2 s",
     lambda: drawn_out(lying()), 1.0, True),
    ("full but still for only 0.85 s", lambda: drawn_out(lying(), hold=0.85), 0.0, False),
    ("pad only 9 mm into the clump (31 mm below the flange's top), then drawn out: the barbs never snag",
     lambda: drawn_out(lying(), depth=0.031, hair=in_drain()), 0.0, False),
    ("pad 10 mm into the clump (limit), then drawn out: snagged", lambda: drawn_out(lying(), depth=0.032), 1.0, True),
    ("pad 13 mm into the clump, then out of the drain in one jump (not drawn back through the clump): not snagged",
     lambda: drawn_out(lying(), legs=[(0.035, 0.0), (-0.012, 1.0)], hair=in_drain()), 0.0, False),
    ("pad beside the flange, outside the drain's bore: nothing snagged",
     lambda: drawn_out(lying(), r=0.020, hair=in_drain()), 0.0, False),
    ("jerked: drawn out at 0.11 m/s: the hair strips off the barbs and stays in the drain",
     lambda: drawn_out(lying(), speed=0.11, hair=in_drain()), 0.0, False),
    ("drawn out at 0.09 m/s (under the 0.10 m/s limit)", lambda: drawn_out(lying(), speed=0.09), 1.0, True),
    ("crept out at 1.5 mm/s, under half the hair's load: the hair slips off at the flange's top and stays",
     lambda: drawn_out(lying(), speed=0.0015, hair=in_drain()), 0.0, False),
    ("drawn out at 3 mm/s (three quarters of the load): free", lambda: drawn_out(lying(), speed=0.003), 1.0, True),
    ("snagged, then pushed back down 5.1 mm: the barbs let go, and the pad (now only 7 mm into the clump) cannot snag"
     " again on the way out",
     lambda: drawn_out(lying(), hair=in_drain(), legs=[HELD, (0.024, 0.02), (0.0291, 0.02), (-0.012, 0.02)]), 0.0,
     False),
    ("snagged, then pushed back down 4.9 mm (limit) and drawn out: still held",
     lambda: drawn_out(lying(), legs=[HELD, (0.024, 0.02), (0.0289, 0.02), (-0.012, 0.02)]), 1.0, True),
    ("pumping: snagged, then 12 up-and-down strokes of 4.8 mm in place (up at 8 mm/s under the load, down in one step),"
     " then crept out at 1 mm/s: only new height counts toward the 15 mm, so the hair slips off at the flange's top",
     lambda: drawn_out(lying(), legs=PUMP + [(-0.012, 0.001)]), 0.0, False),  # (scored 1.0 before the fix)
    ("snagged, pumped the same way, then drawn out steadily at 20 mm/s: free",
     lambda: drawn_out(lying(), legs=PUMP + [(-0.012, 0.02)]), 1.0, True),
    ("snagged, drawn up 20 mm and left there: the hair is still in the drain",
     lambda: drawn_out(lying(), hair=hair_at(-0.005), legs=[(0.035, 0.0), (0.005, 0.02)]), 0.0, False),
    ("naive: tool never taken to the drain, laid back where it was", lambda: play((scene(lying(dz=0.03)), FAR, 1.0),
                                                                                   (scene(lying()), FAR, 1.2)), 0.0, False),
    ("drawn out, tool and hair left lying in the basin", lambda: drawn_out(in_basin()), 0.25, False),
    ("drawn out, clump left over the drain's mouth 9 mm above the flange's top: still in the drain",
     lambda: drawn_out(in_basin(), hair=hair_at(0.009)), 0.0, False),
    ("drawn out, clump over the drain's mouth 10 mm above the flange's top (limit)",
     lambda: drawn_out(in_basin(), hair=hair_at(0.010)), 0.25, False),
    ("drawn out, clump shaken off onto the basin's bottom 28 mm from the drain's axis, beside the flange",
     lambda: drawn_out(in_basin(), hair=hair_at(-0.002, r=0.028)), 0.25, False),
    ("drawn out, hair on the counter but the tool's handle in the basin", lambda: drawn_out(across_rim()), 0.5, False),
    ("drawn out, tool laid 65 mm further back: its wand over the basin's front edge, the end and the hair hanging in",
     lambda: drawn_out(lying(dx=0.065, joints={f"w{i}": 1.0 for i in (5, 6, 7)})), 0.25, False),
    ("drawn out, tool laid with 75.5 mm of its handle past the top's end (start: 68 mm): not wholly on the counter",
     lambda: drawn_out(lying(dy=0.0075)), 0.5, False),
    ("drawn out, tool laid with 74.5 mm of its handle past the top's end (limit 75 mm)",
     lambda: drawn_out(lying(dy=0.0065)), 1.0, True),
    ("drawn out, tool laid turned 8 degrees: the wand's end and the hair 12 mm past the top's front edge",
     lambda: drawn_out(turned(-8.0)), 0.25, False),
    ("drawn out, tool laid turned 3 degrees: the wand's end 14 mm inside the top's front edge",
     lambda: drawn_out(turned(-3.0)), 1.0, True),
    ("drawn out and laid on the counter, still held: a fingertip 9 mm above the grip",
     lambda: play(*path((0.035, 0.0), (-0.012, 0.02)),
                  (scene(lying(), hair_on(lying())), near_tool(scene(lying()), 0.009), 2.0)), 0.0, False),
    ("drawn out and laid on the counter, a fingertip 14 mm above the grip",
     lambda: play(*path((0.035, 0.0), (-0.012, 0.02)),
                  (scene(lying(), hair_on(lying())), near_tool(scene(lying()), 0.014), 1.2)), 1.0, True),
    ("1.0 earned, then a fingertip comes back to the tool",
     lambda: play(*path((0.035, 0.0), (-0.012, 0.02)), (scene(lying(), hair_on(lying())), FAR, 1.5),
                  (scene(lying(), hair_on(lying())), near_tool(scene(lying()), 0.005), 0.5)), 0.0, False),
    ("hand state unknown at the end", lambda: play(*path((0.035, 0.0), (-0.012, 0.02)),
                                                  (scene(lying(), hair_on(lying())), FAR, 1.5),
                                                  (scene(lying(), hair_on(lying())), None, 0.1)), 0.0, False),
    ("tool still sliding at 1 cm/s", lambda: drawn_out(task.TOOL.state(pos=lying().pos, quat_xyzw=task.LYING,
                                                                      lin_vel=(0.01, 0.0, 0.0))), 0.0, False),
    ("tool dropped off the vanity onto the floor earlier, then a full end state",
     lambda: play((scene(lying(dz=-(CT - 0.020) - 0.002)), FAR, 0.2), *path((0.035, 0.0), (-0.012, 0.02)),
                  (scene(lying(), hair_on(lying())), FAR, 1.2)), 0.0, False),
    ("tool's handle dipped 19 mm below the countertop beside the vanity, then a full end state (limit)",
     lambda: play((scene(lying(dz=-0.019)), FAR, 0.2), *path((0.035, 0.0), (-0.012, 0.02)),
                  (scene(lying(), hair_on(lying())), FAR, 1.2)), 1.0, True),
    ("full, the clump moved 0.2 mm onto the pad after the last observation (the simulator's step order)",
     lambda: moved_after(drawn_out(lying()), hair_by=(0.0002, 0.0, 0.0)), 1.0, True),
    ("full, but the tool scored is not the state last observed (moved 0.1 mm): invalid",
     lambda: moved_after(drawn_out(lying()), tool_by=(0.0001, 0.0, 0.0)), 0.0, False),
    ("lot B: full in lot B's layout", lambda: drawn_out(lying("B")), 1.0, True),
    ("lot B: tool laid back in lot A's place", lambda: drawn_out(lying("A")), 1.0, True),
]

CASES = _Timelines(SPEC)


def run():
    bad = 0
    for label, objects, want, ok in CASES:
        got = task.score(objects)
        good = abs(got - want) < 1e-9 and task.success(objects) == ok
        bad += not good
        print("ok  " if good else "FAIL", f"{got:.2f} (want {want:.2f})", label)
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
