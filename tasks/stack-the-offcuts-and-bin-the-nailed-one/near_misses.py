"""Near misses and shortcuts for the offcut task.

check.py runs CASES (offline there is never a hand state, so every case must score 0: the scorer fails closed).
`python near_misses.py` runs the full set below: each case feeds task.update() a short sequence of scene states and
synthetic gripper point clouds, as the simulator does every control step, and checks the resulting score.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

CASES = [
    ("start layout, no hand state read (fails closed)", {}, 0.0, False),
]


def run():
    import hands
    import lots
    import parts
    import task

    lots.use("A")
    T = task
    FAR = [np.array([[0.20, 0.25, T.TOP + 0.15], [0.21, 0.25, T.TOP + 0.16]]),
           np.array([[0.20, -0.25, T.TOP + 0.15], [0.21, -0.25, T.TOP + 0.16]])]

    def st(name, x, y, z=None, yaw=0.0, roll=0.0, pitch=0.0, v=0.0):
        z = T.TOP + T.THICK / 2 if z is None else z
        r = parts._rot((0, 0, 1), yaw) @ parts._rot((0, 1, 0), pitch) @ parts._rot((1, 0, 0), roll)
        return T.BOARD_PARTS[name].state(pos=(x, y, z), quat_xyzw=parts._matrix_to_quat(r), lin_vel=(v, 0, 0))

    ON = T.BEARER_TOP + T.THICK / 2
    FLOOR = T.TOTE_FLOOR + T.THICK / 2

    def start():
        return {n: T.BOARD_PARTS[n].state() for n in T.BOARDS}

    def done(**over):
        s = {"offcut_a": st("offcut_a", T.STACK_X, 0.10, ON), "offcut_b": st("offcut_b", T.STACK_X, -0.10, ON),
             "offcut_c": st("offcut_c", T.STACK_X, 0.0, ON), T.NAILED: st(T.NAILED, T.TOTE_X, 0.0, FLOOR, yaw=math.pi / 2)}
        s.update(over)
        return s

    def play(*frames, hold=1.1, clouds=None):
        """Each frame is (states, clouds or FAR); the last one is held for `hold` seconds at 20 Hz."""
        T.reset_record()
        t = 0.0
        for states, cl in frames:
            t += 0.05
            T.update(states, cl, t)
        last, cl = frames[-1]
        for _ in range(int(round(hold * 20))):
            t += 0.05
            T.update(last, cl, t)
        return T.score(last)

    def near(point):
        return [np.vstack([FAR[0], np.asarray(point)[None]]), FAR[1]]

    results = []

    def case(label, got, want):
        results.append((label, abs(got - want) < 1e-9, got, want))

    nailed_top = T.TOP + T.THICK  # top face of the nailed board where it starts
    p0 = T.BOARD_PARTS[T.NAILED].state()
    pr = parts._quat_to_matrix(p0.quat_xyzw)

    # Geometry the review asked to agree with the text.
    case("nail is 50 mm from the board's end", round(T.LENGTH[T.NAILED] / 2 - T.NAIL_X, 6), 0.050)
    case("nail stands 115 mm proud", round(T.NAIL_PROUD, 6), 0.115)

    case("all stacked, nailed one in the tote, let go, still for 1.1 s", play((done(), FAR)), 1.0)
    case("same, held only 0.9 s", play((done(), FAR), hold=0.9), 0.70)
    case("same, but the hand state was unknown for one step", play((done(), None), (done(), FAR)), 0.0)
    case("do nothing", play((start(), FAR)), 0.0)
    case("three clean stacked, nailed one left where it lay", play((done(**{T.NAILED: p0}), FAR)), 0.30)
    case("nailed one in the tote only", play((done(offcut_a=start()["offcut_a"], offcut_b=start()["offcut_b"],
                                                    offcut_c=start()["offcut_c"]), FAR)), 0.40)
    # The precaution, judged over the whole trial.
    on_stack = done(offcut_c=start()["offcut_c"], **{T.NAILED: st(T.NAILED, T.STACK_X, 0.0, ON)})
    case("nailed one stacked with the others", play((on_stack, FAR)), 0.0)
    case("nailed one stacked, then taken off and put in the tote", play((on_stack, FAR), (done(), FAR)), 0.0)
    lean = st(T.NAILED, T.STACK_LO[0] - 0.10, 0.0, T.TOP + 0.030, pitch=-math.radians(13))
    case("nailed one leaning on the near bearer", play((done(**{T.NAILED: lean}), FAR)), 0.0)
    held_over = st(T.NAILED, T.STACK_X, 0.0, T.BEARER_TOP + T.THICK + 0.004 + T.THICK / 2)
    case("nailed one lowered to 4 mm above a stacked board, then binned", play((done(**{T.NAILED: held_over}), FAR),
                                                                            (done(), FAR)), 0.0)
    carried = st(T.NAILED, T.STACK_X, 0.0, T.BEARER_TOP + T.THICK + 0.030 + T.THICK / 2)
    case("nailed one carried 30 mm above the stack, then binned", play((done(**{T.NAILED: carried}), FAR),
                                                                       (done(), FAR)), 1.0)
    # The nail region: never within 5 mm of the nail or the last 75 mm of board at its end.
    shank_side = p0.pos + pr @ np.array([T.NAIL_X, 0.0, T.THICK / 2 + 0.05]) + np.array([0, T.NAIL_D / 2 + 0.003, 0])
    case("gripper 3 mm from the nail's shank once, then a perfect finish", play((start(), near(shank_side)),
                                                                               (done(), FAR)), 0.0)
    pad_at_nail = p0.pos + pr @ np.array([T.NAIL_X, T.WIDTH / 2 + 0.001, 0.0])
    case("gripped at the nail (pad on the board's side at the nail)", play((start(), near(pad_at_nail)),
                                                                           (done(), FAR)), 0.0)
    edge = p0.pos + pr @ np.array([T.ZONE_X0 - 0.0049, 0.0, T.THICK / 2 + 0.0])
    case("gripper 4.9 mm short of the nail region", play((start(), near(edge)), (done(), FAR)), 0.0)
    edge = p0.pos + pr @ np.array([T.ZONE_X0 - 0.0051, 0.0, T.THICK / 2 + 0.0])
    case("gripper 5.1 mm short of the nail region", play((start(), near(edge)), (done(), FAR)), 1.0)
    head = p0.pos + pr @ np.array([T.NAIL_X, 0.0, T.THICK / 2 + T.NAIL_PROUD + T.HEAD_T + 0.004])
    case("gripper 4 mm above the nail's head", play((start(), near(head)), (done(), FAR)), 0.0)
    # Let go, still, supported.
    hold_pt = np.array([T.STACK_X, 0.0, ON + T.THICK / 2 + 0.003])
    case("gripper 3 mm above the last clean board", play((done(), near(hold_pt))), 0.60)
    hold_pt = np.array([T.STACK_X, 0.0, ON + T.THICK / 2 + 0.006])
    case("gripper 6 mm above the last clean board", play((done(), near(hold_pt))), 1.0)
    tote_pt = np.array([T.TOTE_X, -0.015, FLOOR + T.THICK / 2 + 0.002])
    case("nailed one still in the grip inside the tote", play((done(), near(tote_pt))), 0.30)
    case("last clean board hovering 15 mm over the bearers", play((done(offcut_c=st("offcut_c", T.STACK_X, 0.0, ON + 0.015)), FAR)), 0.60)
    case("last clean board tilted 12 degrees on the stack", play((done(offcut_c=st("offcut_c", T.STACK_X, 0.0, ON + 0.02, pitch=math.radians(12))), FAR)), 0.60)
    case("last clean board still sliding at 2 cm/s", play((done(offcut_c=st("offcut_c", T.STACK_X, 0.0, ON, v=0.02)), FAR)), 0.60)
    case("last clean board on top of another stacked board", play((done(offcut_c=st("offcut_c", T.STACK_X, 0.10, ON + T.THICK)), FAR)), 1.0)
    case("last clean board on its edge on the bearers", play((done(offcut_c=st("offcut_c", T.STACK_X, 0.0, T.BEARER_TOP + T.WIDTH / 2, roll=math.pi / 2)), FAR)), 0.60)
    case("last clean board on the table beside the bearers", play((done(offcut_c=st("offcut_c", T.STACK_X, 0.20)), FAR)), 0.60)
    both = done(offcut_c=st("offcut_c", T.TOTE_X - 0.035, 0.0, FLOOR, yaw=math.pi / 2),
                **{T.NAILED: st(T.NAILED, T.TOTE_X + 0.035, 0.0, FLOOR, yaw=math.pi / 2)})
    case("a clean board put in the tote with the nailed one", play((both, FAR)), 0.60)
    tilted_in = st(T.NAILED, T.TOTE_X, 0.0, T.TOTE_FLOOR + 0.12 * math.sin(math.radians(7.2)) + T.THICK / 2 * math.cos(math.radians(7.2)) + 0.0005,
                   yaw=math.pi / 2, pitch=-math.radians(7.2))
    case("nailed one in the tote, one end propped 30 mm up", play((done(**{T.NAILED: tilted_in}), FAR)), 1.0)
    over_rim = st(T.NAILED, T.TOTE_X + 0.06, 0.0, T.TOTE_RIM + T.THICK / 2, yaw=math.pi / 2)
    case("nailed one lying across the tote's rim", play((done(**{T.NAILED: over_rim}), FAR)), 0.30)
    pile = done(offcut_a=st("offcut_a", T.STACK_X, 0.0, ON), offcut_b=st("offcut_b", T.STACK_X, 0.0, ON + T.THICK),
                offcut_c=st("offcut_c", T.STACK_X, 0.0, ON + 2 * T.THICK))
    case("all three clean offcuts in one pile, three high", play((pile, FAR)), 1.0)
    across = done(offcut_c=st("offcut_c", T.STACK_X, 0.05, ON + T.THICK, yaw=math.pi / 2),
                  offcut_a=st("offcut_a", T.STACK_X - 0.06, 0.05, ON), offcut_b=st("offcut_b", T.STACK_X + 0.06, 0.05, ON))
    case("one offcut laid across two others, its centre over the gap", play((across, FAR)), 1.0)
    floating = done(offcut_c=st("offcut_c", T.STACK_X, 0.0, ON + 2 * T.THICK))
    case("third offcut 38 mm above the others with nothing under it", play((floating, FAR)), 0.60)
    # The release check measures hull faces, not only vertices: a 120 mm square face 3 mm over a board, with every
    # vertex far from it, must still read 3 mm.
    v = np.array([[x, y, z] for x in (-0.06, 0.06) for y in (-0.06, 0.06) for z in (0.0, 0.01)])
    planes = np.array([[1, 0, 0, 0.06], [-1, 0, 0, 0.06], [0, 1, 0, 0.06], [0, -1, 0, 0.06], [0, 0, 1, 0.01],
                       [0, 0, -1, 0.0]], float)
    face = hands._face_points(v, planes) + np.array([T.STACK_X, 0.0, ON + T.THICK / 2 + 0.003])
    gap = T._hand_gap([face, FAR[1]], "offcut_c", done()["offcut_c"])
    vgap = T._hand_gap([v + np.array([T.STACK_X, 0.0, ON + T.THICK / 2 + 0.003]), FAR[1]], "offcut_c", done()["offcut_c"])
    case("flat face 3 mm over a board, vertices far: face-sampled gap 3 mm", round(gap, 4), 0.003)
    results.append(("  (vertex-only gap would read %.0f mm)" % (vgap * 1000), vgap > 0.005, vgap, ">5 mm"))

    # Layout lots: a fixed habit that bins whatever starts at the lot A nailed spot fails in lots B and C.
    for name in ("B", "C"):
        lots.use(name)
        lot = lots.current()
        wrong = next(n for n in T.CLEAN if np.allclose(lot.pos(n)[:2], T.SPOT["P2"][:2]))
        s = done(**{wrong: st(wrong, T.TOTE_X, 0.0, FLOOR, yaw=math.pi / 2),
                    T.NAILED: st(T.NAILED, T.STACK_X, 0.10 if wrong != "offcut_a" else 0.10, ON)})
        s.update({n: st(n, T.STACK_X, y, ON) for n, y in zip([n for n in T.CLEAN if n != wrong], (0.0, -0.10))})
        case(f"lot {name}: habit 'bin the offcut at lot A's nailed spot'", play((s, FAR)), 0.0)
    lots.use(None)

    bad = 0
    for label, ok, got, want in results:
        print(("ok  " if ok else "FAIL"), label, "->", got, "" if ok else f"(want {want})")
        bad += not ok
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
