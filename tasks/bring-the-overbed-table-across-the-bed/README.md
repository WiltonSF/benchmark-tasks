# Bring the overbed table across the bed

A 1:6 model of a hospital bed with a patient lying in it, and a loose overbed table: a C-shaped base (a bar under
the column and two feet), a column at one end and a top cantilevered from the column over the feet. The feet are
18 mm tall and pass under the bed frame (40 mm clear); the top, 142 mm up, passes over the mattress and the
patient. The bar and column cannot go under the frame or over the mattress, so the table only fits across the bed
when it is brought in with its feet toward the bed and the column outside it. The robot is asked to put the table
across the bed so the patient can use it; the instruction does not say how.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in
this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

Lot A (the screenshot): the table to the right of the hands' line and turned a quarter turn, feet pointing left
along the bed's near side. Lot B: the table in front of the bed, in line with the patient's hands, feet pointing at
the bed. Trial k uses lot k mod 2, lot A first, so a run needs `--epochs 2` (or a multiple); `TASK_LOT=A` or
`TASK_LOT=B` pins every trial to one lot. Outcomes in lot A, each a near-miss case:

- pushed straight in as it stands: the column stops at the bed with the top's centre short of the bed's near side
  and one foot outside the bed, 0 ("turned 90 degrees and pushed to the bed");
- turned about its grip block and pushed straight in: the right foot ends outside the bed's right end, 0;
- turned about its column and pushed straight in: across the bed, but the top's centre 75 mm from the hands, 0 (the
  task is pass/fail; "turn in place and push straight" averages at most 0.5 over the two lots).

The lots stop a single fixed push, not a memorised replay of both layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the overbed table across the bed so the patient can use it."

# Dream-1, both lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/overbed-dream

# Scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 2 --store-frames --log-dir ~/yam-eval/overbed-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is two trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`). The `on_step` callback
  times how long the table has been still with both grippers clear; a scored state needs at least 1 s of that
  (offline it counts as long enough), and success must then hold 1 s more.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the table; it must exceed
  5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **On the floor and level:** the table's lowest point (bounding box) within 1 mm of the table top, tilt at most
  5 degrees.
- **Across the bed:** each foot's centre point (the middle of its 150 mm length, in the table's frame, moved into
  the world) at least 40 mm inside the frame's outline (x 0.36 to 0.52 m, y within 0.165 m), and the top's centre
  inside that outline seen from above. **Within reach:** the top's centre within 60 mm of the hands' centre in y.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the table's
  lowest point is more than 20 mm above the table top, it tilts more than 10 degrees, or any of its shapes comes
  within 5 mm of the patient's body, head or hands (oriented-box separating-axis test). It is cleared only by a
  scene reset: the table back within 2 mm of its start and at rest after a jump of over 20 mm in one control step,
  which a robot cannot do.
- Comparisons use a 1e-6 margin, so stated limits pass exactly at the limit.

## Demo

`demo()` uses the right arm only, with fixed waypoints for each lot's starting layout (the poses the cameras show):
it comes down behind the grip block, closes 3 mm past its faces, (lot A) draws the table back toward the hands'
line while turning the wrist a quarter turn so the feet come round to point at the bed, then pushes the table in
until the column is about 10 mm short of the bed, lets go, backs off and lifts clear. All grips are straight down.

## Recorded runs

Earlier runs (lots in the other order, grey base, 0.5 tier): trace-20261109T203342, demo-20261109T203750 (1.0 in
each lot) and dream-20261109T204017 (0.0) on the same physics and demo.
- Scripted demo, `--epochs 2` (demo-20261109T205506): 1.0 in lot A (the turn) and in lot B (task_score 1.0).
- Dream-1, `--epochs 2`, 1,200 steps (dream-20261109T205738): 0.0 in both lots (task_score 0). In lot A the right
  arm pushed the table about near its start and tipped it; it was never brought across the bed.
