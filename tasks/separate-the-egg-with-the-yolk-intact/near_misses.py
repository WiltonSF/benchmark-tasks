"""Near misses and unintended strategies. The material rules are tested on synthetic separator and yolk states (the
same functions the simulator hooks call); the end states are scored with synthetic sensor history. No simulator
results are made up here. Run: python near_misses.py (inside the task folder)."""
import math

import numpy as np

import task as t

L = t.lots
L.use("A")
TOP = t.TOP
YB_FLOOR = TOP + t.YB.floor_t
YOLK_TOP = t.YOLK_START[2] + t.YOLK_R[2]


def scene(mouth=None, q=0.0, yolk=None, sep_quat=None, yolk_quat=None, egg_bowl=None, yolk_bowl=None):
    """States as the simulator reports them. mouth: the separator's mouth (its part origin); q: both walls' travel."""
    return {
        "separator": t.SEPARATOR.state(joints={"wall_l": q, "wall_r": q}, pos=mouth, quat_xyzw=sep_quat),
        "yolk": t.YOLK.state(pos=t.YOLK_START if yolk is None else yolk, quat_xyzw=yolk_quat),
        "egg_bowl": t.EGG_BOWL.state(quat_xyzw=egg_bowl),
        "yolk_bowl": t.YOLK_BOWL.state(quat_xyzw=yolk_bowl),
        "saucer": t.SAUCER.state(),
    }


def run(path, yolk=None):
    """Play (mouth, wall travel) pairs through the bulb rule. Returns the releases."""
    out = []
    for mouth, q in path:
        st = scene(mouth, q, yolk if (yolk is not None or not t.S["inside"]) else t.PARK)
        r = t.bulb_step(st["separator"], st)
        if r is not None:
            out.append(r)
    return out


def ramp(mouth, q0, q1, n=40):
    return [(mouth, float(q)) for q in np.linspace(q0, q1, n)]


ON = (t.WB_XY[0], t.WB_XY[1], YOLK_TOP - 0.001)
OVER_YB = (t.YB_XY[0], t.YB_XY[1], YB_FLOOR + 0.0255)


def controls():
    # 1. Squeeze, mouth on the yolk, let go: the yolk is drawn up with 1 mL of thick white; no other white.
    t.reset_state()
    run(ramp(ON, 0.0095, 0.002))
    assert t.S["inside"] and abs(t.S["sep_white"] - t.CLING) < 1e-9 and abs(t.S["white_bowl"] - (t.WHITE0 - 1)) < 1e-9
    # Gentle squeeze over the small bowl: the yolk leaves after 8 mL, slowly, 1 mm above the floor.
    rel = run(ramp(OVER_YB, 0.002, 0.0095, 60))
    assert len(rel) == 1 and not t.S["inside"] and rel[0]["v_exit"] < 0.1
    assert 0.0 <= rel[0]["pos"][2] - t.YOLK_R[2] - YB_FLOOR <= 0.0015
    assert abs(t.S["yolk_white"] - 1.0) < 1e-9 and t.S["spill"] == 0 and not t.S["ruptured"]
    # 2. Letting go too little: 10 mL drawn while on the yolk is not enough; lifting resets it.
    t.reset_state()
    run(ramp(ON, 0.0095, 0.0062))
    assert not t.S["inside"] and t.S["intake"] > 9
    run([((ON[0], ON[1], ON[2] + 0.010), 0.0062)])
    assert t.S["intake"] == 0
    # 3. Mouth 1.1 mm above the yolk's top (no seal) or 6.1 mm to the side: nothing drawn (above the white: air).
    #    At exactly 1.0 mm above and 6.0 mm aside it still draws the yolk.
    for mouth in ((ON[0], ON[1], YOLK_TOP + 0.0011), (ON[0] + 0.0061, ON[1], ON[2])):
        t.reset_state()
        run(ramp(mouth, 0.0095, 0.002))
        assert not t.S["inside"] and t.S["sep_white"] == 0, mouth
    for mouth in ((ON[0], ON[1], YOLK_TOP + 0.001), (ON[0] + 0.006, ON[1], ON[2]), (ON[0], ON[1], YOLK_TOP - 0.0035)):
        t.reset_state()
        run(ramp(mouth, 0.0095, 0.002))
        assert t.S["inside"], mouth
    # 4. Tipped separator (30 degrees) on the yolk: no seal.
    t.reset_state()
    q30 = tuple(t.parts._matrix_to_quat(t.parts._rot((1, 0, 0), math.radians(30))))
    for q in np.linspace(0.0095, 0.002, 40):
        st = scene(ON, float(q), sep_quat=q30)
        t.bulb_step(st["separator"], st)
    assert not t.S["inside"]
    # 5. Mouth pushed down into the white beside the yolk and let go: white is drawn, not the yolk, until the white's
    #    surface falls below the mouth.
    t.reset_state()
    side = (ON[0] + 0.030, ON[1], TOP + t.WB.floor_t + 0.005)
    run(ramp(side, 0.0095, 0.002))
    assert not t.S["inside"] and 15.0 < t.S["sep_white"] < 22.5 and abs(t.S["white_bowl"] + t.S["sep_white"] - t.WHITE0) < 1e-9
    # 6. Yolk drawn, then the bulb opened further with the mouth down in the white: that white is carried over.
    t.reset_state()
    run(ramp(ON, 0.0095, 0.004))
    assert t.S["inside"]
    run([(side, 0.004)] + ramp(side, 0.004, 0.0))
    assert t.S["sep_white"] > 5.0
    rel = run(ramp(OVER_YB, 0.0, 0.0095, 60))
    assert len(rel) == 1 and t.S["yolk_white"] > 5.0, "extra white goes into the yolk bowl after the yolk"
    # 7. Squeezed out over the counter: the yolk drops from wherever the mouth is (speed rule in the simulator),
    #    and the white lands on the counter as spill.
    t.reset_state()
    run(ramp(ON, 0.0095, 0.002))
    counter = (0.20, 0.10, TOP + 0.20)
    rel = run(ramp(counter, 0.002, 0.0095, 60))
    assert len(rel) == 1 and abs(t.S["spill"] - 1.0) < 1e-9 and rel[0]["pos"][2] > TOP + 0.15
    # 8. Squeezed out with the mouth pressed to the bowl floor: no room, the yolk bursts.
    t.reset_state()
    run(ramp(ON, 0.0095, 0.002))
    run(ramp((OVER_YB[0], OVER_YB[1], YB_FLOOR + 0.010), 0.002, 0.0095, 60))
    assert t.S["ruptured"]
    # 9. Squeezed fast (one 20 Hz step): the exit speed alone is over 0.8 m/s.
    t.reset_state()
    run(ramp(ON, 0.0095, 0.002))
    rel = run([(OVER_YB, 0.002), (OVER_YB, 0.0095)])
    assert len(rel) == 1 and rel[0]["v_exit"] > t.SPEED_MAX
    # 10. Squeeze while sealed on the yolk before letting go: air out, no effect; squeezing the yolk back out over
    #     the egg bowl puts it back in the white bowl area (not the yolk bowl).
    t.reset_state()
    run(ramp(ON, 0.0095, 0.002))
    rel = run(ramp(ON, 0.002, 0.0095, 60))
    assert len(rel) == 1 and abs(t.S["white_bowl"] - t.WHITE0) < 1e-9 and t.S["yolk_white"] == 0
    # Tipped bowls and the hand rule.
    assert not t.bowl_upset(scene())
    tip = tuple(t.parts._matrix_to_quat(t.parts._rot((0, 1, 0), math.radians(15.5))))
    assert t.bowl_upset(scene(egg_bowl=tip)) and t.bowl_upset(scene(yolk_bowl=tip))
    ok = tuple(t.parts._matrix_to_quat(t.parts._rot((0, 1, 0), math.radians(14.5))))
    assert not t.bowl_upset(scene(egg_bowl=ok))
    st = scene()
    y = st["yolk"]
    assert t.yolk_gap(np.array([[y.pos[0], y.pos[1], YOLK_TOP + 0.0019]]), y) <= t.HAND_TOUCH
    assert t.yolk_gap(np.array([[y.pos[0], y.pos[1], YOLK_TOP + 0.0021]]), y) > t.HAND_TOUCH
    t.reset_state()


def endpoint(carry=1.0, spill=0.0, sep_white=0.0, down=True, clear=True, hold=20, yolk=None, mouth=None, q=0.0,
             last_out="yolk", hold_full=20):
    t.reset_state()
    t.S.update(known=True, clear=clear, hold=hold, yolk_white=carry, spill=spill, sep_white=sep_white, white_bowl=30.5,
               last_out=last_out, hold_full=hold_full)
    y = (t.YB_XY[0], t.YB_XY[1], YB_FLOOR + t.YOLK_R[2]) if yolk is None else yolk
    if mouth is None:
        mouth = t.SEP_START if down else (t.YB_XY[0], t.YB_XY[1], YB_FLOOR + 0.06)
    return scene(mouth, q, y)


def ladder():
    def sc(**kw):
        return t.score(endpoint(**kw))
    assert sc() == 1.0 and t.success(endpoint())
    assert sc(carry=2.0) == 1.0 and sc(carry=2.01) == 0.5 and sc(spill=1.0) == 1.0 and sc(spill=1.01) == 0.5
    assert sc(sep_white=1.01) == 0.5, "white left in the separator counts as spilled"
    assert sc(carry=5.0, spill=2.0) == 0.5 and sc(carry=5.01) == 0.25 and sc(spill=2.01) == 0.25
    assert sc(spill=5.0, carry=9) == 0.25 and sc(spill=5.01) == 0.0
    assert sc(carry=10.0) == 0.25 and sc(carry=10.01) == 0.0, "more than 10 mL carried over: nothing separated"
    # The yolk must reach the small bowl by being squeezed out over it: pushed, carried on the lip ring or squeezed out
    # elsewhere and then moved, it scores 0 even lying whole in the small bowl.
    for how in (None, "white", "spill"):
        assert sc(last_out=how) == 0.0, how
    assert sc(hold_full=19) == 0.75, "put down and hands clear for less than 1 s"
    assert sc(down=False) == 0.75, "separator still held over the bowl"
    assert sc(clear=False) == 0.75, "a hand within 30 mm"
    assert sc(hold=19) == 0.0, "not yet still for 1 s"
    for flag in ("ruptured", "upset", "broken", "inside"):
        st = endpoint()
        t.S[flag] = True
        assert t.score(st) == 0.0, flag
    st = endpoint()
    t.S["known"] = False
    assert t.score(st) == 0.0, "unknown hands or history fail closed"
    # Yolk 26 mm off the small bowl's axis, 4.5 mm above its floor, left in the egg bowl, on the counter: 0.
    for y in ((t.YB_XY[0] + 0.0255, t.YB_XY[1], YB_FLOOR + t.YOLK_R[2]),
              (t.YB_XY[0], t.YB_XY[1], YB_FLOOR + t.YOLK_R[2] + 0.0045), t.YOLK_START, (0.20, 0.10, TOP + 0.0125)):
        assert sc(yolk=y) == 0.0, y
    assert sc(yolk=(t.YB_XY[0] + 0.0249, t.YB_XY[1], YB_FLOOR + t.YOLK_R[2] + 0.0039)) == 1.0
    # The mouth 4.9 mm above the yolk in the small bowl: not a scored end state; at 5.0 mm it is (still held: 0.75).
    on = (t.YB_XY[0], t.YB_XY[1], YB_FLOOR + 2 * t.YOLK_R[2] + 0.0049)
    assert sc(down=False, mouth=on) == 0.0
    assert sc(down=False, mouth=(on[0], on[1], on[2] + 0.0001)) == 0.75
    # The separator laid on its side across the yolk (its weight is under the 0.5 N burst load): 0.
    side = tuple(t.parts._matrix_to_quat(t.parts._rot((1, 0, 0), math.radians(90))))
    st = endpoint(mouth=(t.YB_XY[0] - 0.050, t.YB_XY[1], YB_FLOOR + 2 * t.YOLK_R[2] + 0.021))
    st["separator"] = t.SEPARATOR.state(pos=(t.YB_XY[0] - 0.050, t.YB_XY[1], YB_FLOOR + 2 * t.YOLK_R[2] + 0.021),
                                        quat_xyzw=tuple(t.parts._matrix_to_quat(t.parts._rot((0, 1, 0), math.radians(90)))))
    assert t.score(st) == 0.0, "separator lying on the yolk"
    # Separator put down inside the yolk bowl's rim, or on the egg bowl's rim area: not "put down" (0.75).
    assert sc(mouth=(t.YB_XY[0] + 0.07, t.YB_XY[1], TOP + 0.0005)) == 0.75
    assert sc(mouth=(0.17, -0.12, TOP + 0.0005)) == 1.0, "on the counter, clear of both bowls"
    # Separator lying on its side on the counter, clear of both bowls: put down.
    side = tuple(t.parts._matrix_to_quat(t.parts._rot((1, 0, 0), math.radians(90))))
    st = endpoint(mouth=(0.17, -0.17, TOP + 0.020))
    st["separator"] = t.SEPARATOR.state(pos=(0.17, -0.17, TOP + 0.020), quat_xyzw=side)
    assert t.score(st) == 1.0
    t.reset_state()


controls()
ladder()

CASES = [
    ("do nothing", {}, 0.0, False),
    ("yolk carried to the small bowl (no history: fails closed)",
     {"yolk": ((t.YB_XY[0], t.YB_XY[1], YB_FLOOR + t.YOLK_R[2]), None, None)}, 0.0, False),
    ("separator stood in the yolk bowl", {"separator": t.SEPARATOR.state(pos=(t.YB_XY[0], t.YB_XY[1], YB_FLOOR))},
     0.0, False),
]

if __name__ == "__main__":
    print("near_misses: controls and ladder passed")
