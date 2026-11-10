# Split a tablet in a splitter: seat it in the V-cradle, then close the blade on it

A tablet splitter stands on the table: a base plate with a V-shaped cradle and a hinged lid carrying a blade bar
along the V's centre line. A large round tablet lies on the plate in front of the V. The robot is
asked to cut the tablet in half with the splitter. The instruction does not mention the V: a round tablet pushed in
until it touches both walls sits centred under the blade, and a blade closed on a tablet anywhere else would cut it
unequally.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts and the lid hinge) and `lots.py` (layout
lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

Three layouts. Lot A (the screenshot): the splitter on the midline, the tablet left of the blade line. Lot B: the
splitter 80 mm to the right and turned 20 degrees, the tablet right of the blade line. Lot C: the splitter 80 mm to
the left and turned 20 degrees the other way, the tablet on the blade line just short of the V, under the blade, so
closing the blade straight away is the fault. Trial k uses lot k mod 3, lot A first, so a run needs `--epochs 3` (or
a multiple); `TASK_LOT=A|B|C` pins every trial to one lot. The splitter and its two walls are heavy loose parts only
so the lots can move them. `lots.py` moves a jointed part by writing its root body; the lid then pulls the plate
back by its share of the mass, so the plate's move is asked for scaled by total over plate mass (`LOT_MOVE_GAIN`).

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Cut the tablet in half with the tablet splitter."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/splitter-dream

# Scripted demo, all three lots (each reaches 1.0 in about 19 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/splitter-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means the tablet moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`) and the lid's
  hinge turning less than 0.005 rad in the last control step (0.1 rad/s). Success must hold 1 s.
- **Seated:** the tablet tilted at most 10 degrees, its lowest point (bounding box) within 1 mm of the plate, and the
  distance from its centre to each wall's inner face, minus its 15 mm radius, at most 1 mm. The walls' faces are
  read from where the walls are.
- **Blade down:** the bar's underside, taken at the point of the bar nearest the tablet's centre seen from above,
  at most 1 mm above the tablet's top face. Only counted when the bar passes over the tablet (its centre within
  17 mm of the bar's centre line).
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the tablet and of the lid; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the blade is
  down on the tablet while, seen from above, the tablet's gap to either wall is over 1.5 mm (a bounce of the tablet
  under the resting bar does not count), or the splitter is more than 3 mm from its lot's place or turned more than
  2 degrees from its lot's turn, or a wall is more than 2 mm from its place. It is cleared only by a scene reset: the
  tablet back within 2 mm of its lot's start and at rest, after a jump of over 20 mm in one control step. A robot
  cannot move it back that way.

## Scripted demo

`demo()` works in the splitter's frame for the trial's lot. In lot A the right arm pushes and the left arm closes the
lid; in lots B and C the arm on the splitter's side does both. The pushing arm, fingers closed and turned along the
push, comes down behind the tablet (tips 3 mm above the plate) and pushes it along the straight line to its seated
place, commanded 15 mm past it; the walls take up the rest and the arm's compliance absorbs the overshoot. It holds
1 s, backs off and parks. The closing arm comes down behind the top of the open lid and pulls it 46 mm toward the
robot, past upright; the lid falls on its damped hinge onto the tablet in about 2 s. Both arms then wait 7 s. Where
the splitter and tablet are comes from the trial's lot, which the cameras show; the demo reads nothing from the
simulator. The finger yaw along a push is plain `Waypoint.yaw`; no tilted grips are used.

## Measured in simulation

- **All three lots** (trace-20261109T202924): each push left the tablet within 0.2 mm of its seated place, each lid
  fell to 1.571 rad on the tablet, and each trial scored 1.0. The splitter and walls stayed where the lots put them.
- **Earlier version** (trace-20261109T202404): the two walls overlapped at the apex and pushed each other 1.8 mm
  apart at the start; they now meet without overlapping.
- **Replay probe** (a copy of the task whose demo, in every lot, replays lot A's push, then lot B's push, then lot
  A's lid tip; trace-20261109T203308): 1.0 in lot A; 0.5 in lot B (the tablet seated, but lot A's lid motion misses
  lot B's lid); 0 in lot C (nothing touched the tablet; the lid stayed up); task_score 0.5. A replay that also played
  lot B's and lot C's lid motions in turn was not run.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any push path, closing the lid by hand instead of letting it fall, closing it
  empty first and opening it again before seating the tablet.
- **0.5:** the tablet seated in the V, the lid left open or held.
- **Score 0:** closing the lid without seating the tablet (in lot C, on a tablet under the blade: a fault);
  shoving the splitter or a wall; a tablet against one wall only or short of the walls; a
  tablet tipped or riding up a wall; a tablet still moving or still touched; and, for the rest of the trial, bringing
  the blade down on a tablet that is not seated, even if it is then pushed home.

The cases in `near_misses.py` cover the 1 mm wall gap and 1 mm blade gap on both sides, a tablet bouncing 1 mm and
1.5 mm up or 1.4 mm off a wall under the resting bar (no fault), the blade closed on a tablet on the blade line short
of the V, a wall shoved 3 mm and the splitter shoved 4 mm, a tilted tablet, a tablet on
the blade line but short of the walls, a tablet against one wall only, a lid shut empty, faults followed by a seated
tablet that still scores 0, and scene resets.

## Recorded runs

- Scripted demo, `--epochs 3` (demo-20261109T204213): 1.0 in lots A, B and C (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 3` (dream-20261109T204705): 0.0 in all three lots; every trial hit the step limit.
