# Open a patient hoist's legs round a wheelchair before bringing it in

A 1:6 model of a mobile patient hoist stands in front of a wheelchair, its two legs closed and pointing at the
chair. The robot is asked to bring the hoist up to the wheelchair so it is ready to lift from the seat. The
instruction does not mention the legs: closed, they are too narrow to pass the chair's front legs and strike it;
swung out to their stops, they pass either side and the hoist's spreader block comes over the seat.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts and the leg hinges) and `lots.py` (layout
lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

Lot A (the screenshot): the hoist and the chair on the midline. Lot B: both 70 mm to the robot's right. Lot C: as lot
A, but the chair 40 mm farther out, so the hoist must be pushed 40 mm farther. Trial k uses lot k mod 3, lot A first,
so a run needs `--epochs 3` (or a multiple); `TASK_LOT=A|B|C` pins every trial to one lot. The chair is a loose 3 kg
part only so the lots can move it.

`lots.py` moves a part by writing its root body's pose; for this articulated hoist the legs then pull the frame back
by their share of the mass (8.3 mm of a 70 mm move, measured). Lot B therefore asks for the hoist's move scaled by
the hoist's total mass over its frame's mass (`LOT_MOVE_GAIN` in `task.py`), which lands it 70 mm over.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Bring the hoist up to the wheelchair so it is ready to lift from the seat."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/hoist-dream

# Scripted demo, all three lots (each reaches 1.0 in about 23 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --epochs 3 --store-frames --log-dir ~/yam-eval/hoist-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means the hoist's frame moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`) and
  neither leg hinge turning more than 0.005 rad in the last control step. Success must hold 1 s.
- **Legs open:** each leg's hinge angle at least 0.45 rad (the stop is at 0.55 rad).
- **0.5 tier:** both legs open, the spreader more than 30 mm from the seat centre, the mast at most 2 mm farther
  (horizontally) from the chair's centre than at the trial's start, and the hoist turned at most 15 degrees from its
  start heading. Opening the legs and then dragging the hoist away, or spinning it, earns nothing.
- **In position:** the spreader block's centre (from the hoist's pose) within 30 mm of the chair's centre, which
  is the seat's centre, measured horizontally.
- **Touching:** every shape of the hoist (frame, battery box, mast, boom, spreader, both legs) and every shape of
  the chair is taken as an oriented box; any pair within 1 mm (separating-axis test) is a touch. Cylinders (the
  wheels) are taken as their boxes.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the hoist; it must exceed
  5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the hoist
  touches the chair, the chair is more than 5 mm (horizontally) from its lot's spot or turned more than 3 degrees,
  the hoist is tilted more than 15 degrees, or its lowest point (bounding box) is more than 10 mm above the table. It is cleared
  only by a scene reset: the hoist back within 2 mm of its lot's start and at rest, after a jump of over 20 mm in
  one control step. A robot cannot move it back that way.

## Scripted demo

`demo()` drives both arms with the fingers pointing straight down. The right arm comes down between the closed legs
(tips 7 mm above the table, 150 mm along the legs) and swings the right leg out to its stop, commanded 4 mm past
it (more turns the whole hoist). It parks, and the left arm does the same for the left leg. Then the right arm, with
its fingers part open so that two finger tips about 70 mm apart meet the frame's back edge, comes down 20 mm behind
the frame and pushes the hoist toward the chair (190 mm in lots A and B, 230 mm in lot C), commanded 20 mm past the
mark that puts the spreader over the seat; it backs off and parks. The hoist's place is the one the cameras show for the trial's lot; the demo reads
nothing from the simulator.

## Measured in simulation

- **All three lots, current version** (trace-20261109T222843): 1.0 in each lot. At the end the legs stood at 0.51 to
  0.54 rad and the spreader was 10.1 mm (lot A), 11.4 mm (lot B) and 10.0 mm (lot C) from the seat centre.
- **Earlier version, two lots** (trace-20261109T200541, the push commanded 10 mm past its mark): each leg reached
  its 0.55 rad stop and the score went to 0.5; the push brought the mast to 295 mm out (the spreader 20 mm short of
  the seat centre; the soft arm lags the command), and both lots scored 1.0. That is why the push is now commanded
  20 mm past.
- **Earlier versions:** with the legs resting on the table, pushing a leg turned the whole hoist instead of opening
  the leg (trace-20261109T175439); with a 200 mm mast the arms struck the boom while opening the legs; pushing a leg
  18 mm past its stop turned the hoist 6 degrees, so the push now stops 4 mm past it.
- **Fixed replay** (a copy of the task whose demo always works at lot A's spots, trace-20261110T021032, three lots):
  1.0 in lot A; 0 in lot B (one leg missed, at 0.047 rad, and the hoist was pushed sideways onto the midline); 0.5 in
  lot C (legs open, pushed to lot A's mark, the spreader 50 mm short of the seat); task_score 0.5. An earlier
  two-lot version of this probe (trace-20261109T200105) shoved the chair 15 mm in lot B.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any order, opening the legs by pulling from outside, pushing or pulling the hoist
  by any part, stopping anywhere the spreader is within 30 mm of the seat centre.
- **0.5:** both legs open and the hoist let go short of the seat, not farther from the chair than it started and
  still facing it, without having touched the chair.
- **Score 0:** pushing the hoist in with its legs closed or one leg closed, which strikes the chair's legs or wheels;
  pushing it on until the frame meets the chair; shoving or turning the chair; lifting the hoist over the chair or
  tipping it; and, for the rest of the trial, any of these even if the hoist is then put right.

The cases in `near_misses.py` cover the 0.45 rad and 30 mm limits on both sides, legs open with the hoist dragged
10 mm back (0) or 2 mm back (0.5), spun to 90 degrees (0), turned 16 degrees (0) and 14 degrees (0.5), one leg open, a closed hoist
stopped 2 mm short of the chair (no fault) and pushed into it, the frame meeting the chair, a hoist turned 3 degrees
(still clear) and 10 degrees (a leg meets the chair), the chair shoved 6 mm, the hoist lifted 11 mm and tipped 16
degrees, each fault followed by a hoist in place that still scores 0, and scene resets.

## Recorded runs

- Scripted demo, `--epochs 3` (demo-20261110T021900): 1.0 in lots A, B and C (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 3` (dream-20261110T022349): 0.0 in all three lots; every trial hit the step limit.
