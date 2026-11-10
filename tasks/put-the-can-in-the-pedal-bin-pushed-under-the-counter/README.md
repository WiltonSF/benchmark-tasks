# Put a can in a pedal bin whose lid is blocked by the counter it is pushed under

A small pedal bin stands pushed in under a low counter, its front 30 mm out from the counter's edge and the rest
under it. Its lid is hinged along the back, so under the counter the lid can rise only until it meets the
counter's underside, about 20 mm of headroom over the lid: far too little for the 50 mm can. A red drinks can
stands on the table. The robot is asked to put the can in the bin. The instruction does not mention the counter.
The way to do it is the way cleaners do: pull the bin out by its rim (here a pull fin) until the lid's hinge is
clear of the counter, work the pedal, drop the can in, and let the lid shut.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with hinges) and `lots.py` (layout
lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

Both the bin and the can move between lots. Lot A (the screenshot): the bin 5 cm left of the midline, its front
face 30 mm out from the counter's edge; the can 20 cm in front, 15 cm right. Lot B: the bin 5 cm right of the
midline, same depth; the can 20 cm in front, 15 cm left (the arms swap jobs). Lot C: the bin 5 cm left, pushed 15 mm
further in (front face 15 mm out); the can 28 cm in front, 21 cm right. A fixed routine (grip the fin where lot A
has it and pull back a set distance) misses the fin in lot B and leaves the hinge under the counter in lot C.
Trial k uses lot k mod 3, lot A first, so a run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C` pins
every trial to one lot. The lots stop a single fixed routine, not a memorised replay of all three layouts.

`lots.py` here differs from the stock copy in one place: a loose hinged part (this bin) reached only about 83 % of a
requested move in one pose write (traced: 83 of 100 mm, 71 of 85 mm), so after each settle the goal pose is written
again, up to four times, until every moved object is within 0.5 mm of its spot. Each trial logs the worst placement
error (`[lots] lot B: worst placement error 0.3 mm`); in the final traces lots B and C settled within 0.3 mm.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the empty drinks can in the pedal bin."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 3 --store-frames --log-dir ~/yam-eval/bin-dream

# Scripted demo, all three lots (each reaches 1.0 in about 25 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 650 --epochs 3 --store-frames --log-dir ~/yam-eval/bin-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 80 s of simulated time (1,600 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## The pedal

The pedal and lid are two hinges of the bin (`parts.Hinge`). A `parts.on_step` callback is the linkage: once the
pedal is pressed past 0.12 rad it sets the lid's drive target to 1.5 rad; once the pedal is let up below 0.06 rad it
sets it back to 0, and the lid falls shut. The lid's drive is a spring of 0.3 N m/rad, so under the counter it pushes
the lid against the counter's underside and stops there. The pedal has its own return spring (0.25 N m/rad; it
rests 0.03 rad down under its own weight). The lid can also be lifted by hand.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`), for the bin and the can;
  success must hold 1 s. Upright: the bin tilted at most 10 degrees.
- **Can in the bin:** the can's centre, in the bin's own frame, within 45 mm of the bin's centre along both
  horizontal axes (the inside walls) and between the floor's top (5 mm) and the rim (80 mm) in height.
- **Lid shut:** the lid hinge at most 5 degrees open.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the bin, its lid and pedal,
  and the can (as its box); it must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this counts as
  clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if the bin is ever tilted more
  than 30 degrees. It is cleared only by a scene reset: the can back within 0.5 mm of its start and at rest, after a
  jump of over 20 mm in one control step. A robot cannot move it back that way.

## Scripted demo

`demo()` uses both arms (lot B, with the bin right of the midline, swaps them). The left arm grips the pull fin from above (fingers 24 mm apart, closing 3 mm past the
fin's 12 mm faces), pulls the bin toward the robot to 10 mm past a fixed end spot 21.5 cm in front of the arm bases (it ends
there, the hinge 45 mm clear of the counter: 105 mm out in lots A and B, 120 mm in lot C), lets go and rises; then, fingers shut, it presses the pedal 110 mm in front of where the bin ends up
and holds it for 8 s. Meanwhile the right arm waits over the can until the bin is out, picks the can round its body,
carries it 12 mm over the shut lid to wait beside the bin, moves over the opened bin and lets it drop in. Then the
left arm lets the pedal up and the lid falls shut. Positions come from the trial's lot, which the cameras show; the
demo reads nothing from the simulator.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm for either job, lifting the lid by hand instead of the pedal, laying the can in
  rather than standing it, pulling the bin out further, pushing it back under the counter afterwards.
- **Score 0:** pressing the pedal or lifting the lid under the counter (it stops at the counter); the can on the lid,
  on the floor or held; the lid left open or propped on the can; the bin pulled out too little, or pulled out with the can left outside; the bin tipped over
  30 degrees at any time.

The cases in `near_misses.py` cover the 5-degree lid limit on both sides, the bin pulled out with no can (0),
the can in a bin pushed back under the counter (1.0), a can standing,
lying and in a corner, the lid held open, the can on the lid or the floor, a moving can, a tilted bin (8 degrees,
fine) and a tipped one (35 degrees, which then stays 0 after being set upright), and scene resets.

## Measured in simulation

- Final traces (`--epochs 3`, trace-20261109T154953): 1.0 in each lot. The bin ended on its end spot (hinge 45 mm clear of the counter), the
  pedal went to 0.25 rad, the lid to 1.5 rad, the can dropped in and stood on the floor, and the lid shut after the
  pedal was let up; success held only after both arms had let go.
- Earlier traces: with both arms working at once the can-carrying arm and the pulling arm interfered and the bin
  slid 15 to 50 mm sideways, so the demo now waits with the can until the bin is out; with a weaker pedal spring the
  pedal rested past the shut threshold and the lid never closed.

## Recorded runs

- No-pull habit (the demo without its pull, lot A, earlier layout, not for submission): the pedal went to 0.25 rad
  but the lid stopped at 0.27 rad (16 degrees) against the counter; the can never went in. Score 0.
- Scripted demo, `--epochs 3`: 1.0 in each of lots A, B and C (task_score 1.0).
- Dream-1, 1,600 steps, `--epochs 3`: 0.0 in all three lots.
