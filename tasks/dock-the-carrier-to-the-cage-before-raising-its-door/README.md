# Dock the transfer box to the cage before raising its sliding door

A cat sits at the back of a cage with a barred roof whose small doorway is closed by a vertical sliding (guillotine) door.
A transfer box, open at one end, lies on the table. The robot is asked to open the cage door so the cat can go into
the box. Staff who move a frightened cat this way first butt the box's open end flush against the doorway, and only
then raise the door; raising it with no box docked lets the cat out. The rule is not stated.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts and the sliding door) and `lots.py`
(layout lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

The box always starts on the same spot; only the way its open end points changes. Lot A (the screenshot): left. Lot
B: right. Lot C: toward the robot. So lot A needs the box turned a quarter turn one way, lot B the other way, and lot
C a half turn. Trial k uses lot k mod 3, lot A first, so a scored run needs `--epochs 3` (or a multiple);
`TASK_LOT=A`, `B` or `C` pins every trial to one lot. A fixed turn (always a quarter turn clockwise, say), or carrying
the box over unturned, docks its closed back or a side in at least two of the three lots, and raising the door then
is a fault. The lots stop a single fixed habit, not a memorised replay of all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Open the cage door so the cat can go into the transfer box."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir ~/yam-eval/dock-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir ~/yam-eval/dock-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- **Docked:** the box's tilt at most 5 degrees and its lowest point within 2 mm of the table; the direction of its
  open end within 5 degrees of straight at the cage; the centre of its open end between 1 mm through and 6 mm short
  of the cage's near face, and within 11 mm of the doorway's centre left to right (the 72 mm opening then covers the
  50 mm doorway).
- **Door up on its catch:** the door's slide at 46 mm or more. A `parts.on_step` callback acts as the catch: once the
  door reaches 46 mm it moves the door spring's rest point above the top stop, so the spring holds the door up
  against gravity; until then the spring and gravity pull it shut. The record is cleared by `parts.on_reset` at each
  scene reset, which the robot cannot trigger.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the door is more
  than 20 mm up while the box is not docked, or the cat's position is more than 20 mm from its start.
- **Let go:** a `parts.on_step` callback reads the poses of every moving link of both arms (links 1 to 5, gripper and fingers) every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the box and of the door; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Still:** the box moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold
  1 s; the 0.5 tier is the state at the last step.

## Demo

`demo()` reads only the lot's layout (which the cameras show). The left arm takes the box by its roof handle, with
the gripper turned to match the box, turns it so the open end faces the cage (in lot C it first turns it a quarter turn on the spot and takes it again,
since a half turn is past the wrist's range), sets it down against the cage face and
lets go. The right arm then takes the door's handle, lifts it until the catch holds it, and lets go. The demo aims
the box 4 mm into the face (7 mm in lot C, where it is taken a second time), lets go, then pushes the box's
closed back with closed fingers so the open end sits flush; the face stops the box. The demo does not
use `tilt.py`.
