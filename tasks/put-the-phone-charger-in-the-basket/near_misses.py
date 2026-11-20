"""Scoring counterexamples, run offline: every level, both sides of each limit, the naive errand and other shortcuts."""
import copy
import math

import numpy as np

import lots
import parts
import task

CH, END, PLUG, BASKET = task.CHARGER, task.CORD_END, task.CORD_PLUG, task.BASKET
W1, W2 = task.WALL_OUTLETS
TOP = task.TOP


def quat(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    r = (parts._rot((0, 0, 1), math.radians(yaw_deg)) @ parts._rot((0, 1, 0), math.radians(pitch_deg))
         @ parts._rot((1, 0, 0), math.radians(roll_deg)))
    return parts._matrix_to_quat(r)


def in_wall(o, gap=0.0002):
    return (o.xf - gap, o.yc, o.zc)


PLUG_OUT = (W1.xf - 0.075, W1.yc, TOP + task.PLUG_BODY[1] / 2)  # pulled 75 mm back and set down on its side
PLUG_OUT_Q = quat(roll_deg=90.0)
ON_SIDE = quat(roll_deg=90.0)  # the charger lying on a side face, blades pointing along -y... (its +z now along -y)


def charger_lying(x, y, floor):
    """The charger lying on a side face on a surface at height ``floor``: its plug face's middle where it would be."""
    return (x, y + task.CHARGER_BODY[2] / 2, floor + task.CHARGER_BODY[0] / 2)


def charger_in(gap):
    """The charger in outlet A, its face ``gap`` above A's face."""
    return tuple(np.array(task.CHARGER_POS) + (0.0, 0.0, gap - task.START_GAP))


def states(charger=None, cq=None, plug=None, pq=None, basket=None, cover=0.0):
    lot = lots.current()
    basket = lot.pos(BASKET.name) if basket is None else basket
    charger = task.CHARGER_POS if charger is None else charger
    plug = in_wall(W1) if plug is None else plug
    return {CH.name: CH.state(pos=charger, quat_xyzw=cq), END.name: END.state(joints={"a_cover": cover}),
            PLUG.name: PLUG.state(pos=plug, quat_xyzw=pq),
            BASKET.name: BASKET.state(pos=basket, quat_xyzw=quat(yaw_deg=math.degrees(lot.yaw_of(BASKET.name))))}


def grade(objs, held=1.0, known=True, invalid=False, dropped=False, cord_held=False, yanked=False, stale=False):
    task.REC.update(known=known, invalid=invalid, dropped=dropped, cord_held=cord_held, yanked=yanked, held=held)
    task.REC["stamp"] = () if stale else task._stamp(objs)
    return task.score(objs)


# check.py runs these against the task's own history, which is unobserved before any simulator step: all score 0.
CASES = [
    ("charger in the basket, cord unplugged, but no observed history",
     states(charger_lying(0.140, 0.215, TOP + task.BASKET_WALL), ON_SIDE, PLUG_OUT, PLUG_OUT_Q), 0.0, False),
    ("everything at its start", {}, 0.0, False),
]


def run():
    old = copy.deepcopy(task.REC)
    lots.use("A")
    r = []
    try:
        bx, by, _ = lots.current().pos(BASKET.name)
        in_b = charger_lying(bx, by, TOP + task.BASKET_WALL)
        full = states(in_b, ON_SIDE, PLUG_OUT, PLUG_OUT_Q)
        r.append(("charger in the basket, cord unplugged at the wall: 1.0", grade(full) == 1.0))
        r.append(("same, only 0.95 s let go and still: 0", grade(full, held=0.95) == 0.0))
        r.append(("start (nothing done): 0", grade(states()) == 0.0))
        r.append(("start with the charger settled 1.3 mm proud of outlet A: 0", grade(states(charger_in(0.0013))) == 0.0))
        # the naive errand: the charger to the basket, the cord left live with its outlet open on the mat
        r.append(("NAIVE: charger in the basket, cord still in the wall: 0", grade(states(in_b, ON_SIDE)) == 0.0))
        r.append(("NAIVE, sloppier: charger dropped on the mat, cord still in the wall: 0",
                  grade(states(charger_lying(0.150, -0.020, TOP), ON_SIDE)) == 0.0))
        # partial levels
        r.append(("cord unplugged, charger left on the floor: 0.6",
                  grade(states(charger_lying(0.120, 0.090, TOP), ON_SIDE, PLUG_OUT, PLUG_OUT_Q)) == 0.6))
        r.append(("cord unplugged, charger standing on its blades' side on the basket's rim (outside the walls): 0.6",
                  grade(states(charger_lying(bx + task.BASKET_OUT[0] / 2 + 0.03, by, TOP), ON_SIDE, PLUG_OUT,
                               PLUG_OUT_Q)) == 0.6))
        r.append(("cord unplugged, charger still fully in outlet A: 0.3",
                  grade(states(plug=PLUG_OUT, pq=PLUG_OUT_Q)) == 0.3))
        r.append(("cord unplugged, charger half out of outlet A (8 mm of blade showing): 0.3",
                  grade(states(charger_in(0.008), plug=PLUG_OUT, pq=PLUG_OUT_Q)) == 0.3))
        # the wall plug: out means no blade tip in a slot mouth
        r.append(("charger in the basket, wall plug 5 mm out (blades showing, still live): 0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.005))) == 0.0))
        r.append(("charger in the basket, wall plug 15 mm out (1 mm of blade tip still in the slots): 0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.015))) == 0.0))
        r.append(("charger in the basket, wall plug 17 mm out (tips clear of the face): 1.0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.0175))) == 1.0))
        r.append(("charger in the basket, wall plug moved to the shuttered left outlet's mouths: 0",
                  grade(states(in_b, ON_SIDE, in_wall(W2, task.BLADE_L - 0.0015))) == 0.0))
        # the freed outlet's twist cover turned to CLOSE makes the cord end safe too
        shut = math.pi / 2
        r.append(("charger in the basket, cord still in the wall, freed outlet's cover at CLOSE: 1.0",
                  grade(states(in_b, ON_SIDE, cover=shut)) == 1.0))
        r.append(("same, cover 9 degrees short of CLOSE: 1.0",
                  grade(states(in_b, ON_SIDE, cover=shut - math.radians(9.0))) == 1.0))
        r.append(("same, cover 11 degrees short of CLOSE (slots partly open): 0",
                  grade(states(in_b, ON_SIDE, cover=shut - math.radians(11.0))) == 0.0))
        r.append(("charger on the floor, cord in the wall, cover at CLOSE: 0.6",
                  grade(states(charger_lying(0.120, 0.090, TOP), ON_SIDE, cover=shut)) == 0.6))
        r.append(("charger still in A, cord in the wall, cover reading CLOSE: 0 (nothing done)",
                  grade(states(cover=shut)) == 0.0))
        # a wall plug left part-way out (blade metal showing, still live) scores 0 even with A's cover at CLOSE
        r.append(("charger in the basket, cover at CLOSE, wall plug 8 mm out (blades showing): 0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.008), cover=shut)) == 0.0))
        r.append(("charger in the basket, cover at CLOSE, wall plug 2.5 mm out (blades showing): 0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.0025), cover=shut)) == 0.0))
        r.append(("charger in the basket, cover at CLOSE, wall plug 1.9 mm proud (fully in): 1.0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.0019), cover=shut)) == 1.0))
        r.append(("charger on the floor, cover at CLOSE, wall plug 8 mm out: 0, not 0.6",
                  grade(states(charger_lying(0.120, 0.090, TOP), ON_SIDE, in_wall(W1, 0.008), cover=shut)) == 0.0))
        r.append(("charger in the basket, cover at CLOSE, wall plug at the shuttered left outlet's mouths: 0",
                  grade(states(in_b, ON_SIDE, in_wall(W2, task.BLADE_L - 0.0015), cover=shut)) == 0.0))
        r.append(("charger out, wall plug pushed back fully into the wall: 0",
                  grade(states(in_b, ON_SIDE, in_wall(W1, 0.0005))) == 0.0))
        r.append(("wall plug 1.9 mm proud still counts as in the wall", task.wall_seated(
            states(plug=in_wall(W1, 0.0019))[PLUG.name], W1)))
        r.append(("charger 1.9 mm proud of A still counts as seated", task.a_seated(
            states(charger_in(0.0019))[CH.name], END.state())))
        r.append(("charger 2.1 mm proud of A is not seated but is partly in", not task.a_seated(
            states(charger_in(0.0021))[CH.name], END.state())
            and task.a_depth(states(charger_in(0.0021))[CH.name], END.state()) > task.IN_AT))
        r.append(("charger turned half a turn (wide blade over the narrow slot) is not seated", not task.a_seated(
            states(charger_in(0.0002), quat(yaw_deg=180.0))[CH.name], END.state())))
        r.append(("charger leaning 6 degrees in A is not seated", not task.a_seated(
            states(charger_in(0.0002), quat(roll_deg=6.0))[CH.name], END.state())))
        r.append(("charger 16.6 mm above A (tips clear) is out of it",
                  task.a_depth(states(charger_in(0.0166))[CH.name], END.state()) <= task.IN_AT))
        # basket edges: the charger's middle inside the inner walls and below the rim
        r.append(("charger middle 1 mm inside the inner wall: in", task.in_basket(
            states(charger_lying(bx + task.BASKET_IN[0] - 0.001, by, TOP + task.BASKET_WALL), ON_SIDE)[CH.name],
            BASKET.state(pos=(bx, by, TOP)))))
        r.append(("charger middle 1 mm past the inner wall: out", not task.in_basket(
            states(charger_lying(bx + task.BASKET_IN[0] + 0.001, by, TOP + task.BASKET_WALL), ON_SIDE)[CH.name],
            BASKET.state(pos=(bx, by, TOP)))))
        r.append(("charger held above the rim: out", not task.in_basket(
            states(charger_lying(bx, by, TOP + task.BASKET_OUT[2] + 0.010), ON_SIDE)[CH.name],
            BASKET.state(pos=(bx, by, TOP)))))
        # whole-trial rules and fail closed
        r.append(("a gripper held the cord at any time: 0", grade(full, cord_held=True) == 0.0))
        r.append(("the wall plug came out with no hand on its body (pulled by its cord or knocked out): 0",
                  grade(full, yanked=True) == 0.0))
        r.append(("a handled part left the floor area at any time: 0", grade(full, dropped=True) == 0.0))
        r.append(("hand state unknown: 0", grade(full, known=False) == 0.0))
        r.append(("a bad step earlier: 0", grade(full, invalid=True) == 0.0))
        r.append(("stale state: 0", grade(full, stale=True) == 0.0))
        bad = dict(full)
        bad[CH.name] = parts.PartState(**{**full[CH.name].__dict__, "pos": np.array([math.nan, 0.0, 0.0])})
        r.append(("NaN charger position: 0", grade(bad) == 0.0))
        # the cord-holding rule: both fingertips of one gripper within 2 mm of the same cord link
        links = [b for p in task.CORDS for b in task.boxes(p, p.state(), links=True)]
        c, rr, h = links[2]
        top_pt, bot = c + rr @ np.array((0.0, 0.0, h[2] + 0.0015)), c - rr @ np.array((0.0, 0.0, h[2] + 0.0015))
        far = np.array([[0.0, 0.0, 2.0]])
        pinch = [far, np.array([top_pt]), np.array([bot]), far, far, far]
        r.append(("both left fingertips 1.5 mm from one cord link: holding the cord", task.holds_cord(pinch, links)))
        one = [far, np.array([top_pt]), far + 0.1, far, far, far]
        r.append(("one fingertip on a cord link: not holding it", not task.holds_cord(one, links)))
        wide = [far, np.array([c + rr @ np.array((0.0, 0.0, h[2] + 0.0025))]),
                np.array([c - rr @ np.array((0.0, 0.0, h[2] + 0.0025))]), far, far, far]
        r.append(("fingertips 2.5 mm off a cord link: not holding it", not task.holds_cord(wide, links)))
        right = [far, far, far, far, np.array([top_pt]), np.array([bot])]
        r.append(("right gripper pinching a cord link: holding the cord", task.holds_cord(right, links)))
        # the plug-body rule: both fingertips of one gripper within 3 mm of the plug's body when it leaves the wall
        pb = task.boxes(PLUG, full[PLUG.name])
        c1, r1, h1 = pb[0]
        sides = [c1 + r1 @ np.array((0.0, s * (h1[1] + 0.0025), 0.0)) for s in (1, -1)]
        r.append(("both left fingertips 2.5 mm off the plug body's sides: holding it",
                  task.holds_body([far, np.array([sides[0]]), np.array([sides[1]]), far, far, far], pb)))
        r.append(("one fingertip on the plug body only (a push or a hook): not holding it",
                  not task.holds_body([far, np.array([sides[0]]), far + 0.1, far, far, far], pb)))
        # stillness and the let-go rule
        bxs = [b for p in task.WATCHED for b in task.boxes(p, full[p.name])]
        c0, r0, h0 = task.boxes(CH, full[CH.name])[0]
        side = c0 + r0 @ np.array([h0[0], 0.0, 0.0])
        r.append(("finger 4.9 mm off the charger: touching",
                  task.hand_gap([np.array([side + r0 @ np.array([0.0049, 0, 0])])], bxs) <= task.CLEAR))
        r.append(("finger 5.1 mm off the charger: clear",
                  task.hand_gap([np.array([side + r0 @ np.array([0.0051, 0, 0])])], bxs) > task.CLEAR))
        r.append(("NaN hand points fail closed", task.hand_gap([np.full((3, 3), np.nan)], bxs) == -math.inf))
        r.append(("no hand points fail closed", task.hand_gap([], bxs) == -math.inf))
        r.append(("still at 5 mm/s and 0.05 rad/s", task.still([(0.005, 0.05)] * 4)))
        r.append(("not still at 5.1 mm/s", not task.still([(0.0051, 0.0), (0.0, 0.0)])))
        r.append(("no previous pose: not still", task.motion(None, full[CH.name], 0.05) == (math.inf, math.inf)))
        # lot B: the basket stands at the back left
        lots.use("B")
        bx2, by2, _ = lots.current().pos(BASKET.name)
        in_b2 = charger_lying(bx2, by2, TOP + task.BASKET_WALL)
        r.append(("lot B: charger in the basket, cord unplugged: 1.0",
                  grade(states(in_b2, ON_SIDE, PLUG_OUT, PLUG_OUT_Q)) == 1.0))
        r.append(("lot B: charger dropped where lot A's basket stood, cord unplugged: 0.6",
                  grade(states(in_b, ON_SIDE, PLUG_OUT, PLUG_OUT_Q)) == 0.6))
        r.append(("lot B: NAIVE, charger in the basket, cord in the wall: 0", grade(states(in_b2, ON_SIDE)) == 0.0))
        r.append(("lot B: start: 0", grade(states()) == 0.0))
    finally:
        lots.use(None)
        task.REC.clear()
        task.REC.update(old)
    for name, ok in r:
        print("ok  " if ok else "FAIL", name)
    return sum(not ok for _, ok in r)


if __name__ == "__main__":
    raise SystemExit(run())
