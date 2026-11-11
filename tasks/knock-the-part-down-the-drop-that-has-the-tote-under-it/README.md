# Knock a part out of a sorting nest down the drop-through that has the tote under it

A steel sorting nest has a roofed cross-shaped channel with a small aluminium part standing at its centre, too low
for a gripper to reach in. Three drop-throughs open in the channel floor (left, right and far), each above a bore.
A yellow tote stands in one bore; the other two are bare down to a black floor. The drop-throughs are 42 mm deep
from the roof top to the channel floor, so the top camera, which looks down at about 61 degrees from behind the
robot, shows only a strip of the tote's yellow at the far side of its drop-through; from straight above it shows
fully.
Three slide knock-outs (50 g moving, no spring, damped at 2 N per m/s, about 0.2 N at a 10 cm/s push, with end stops) with red knobs above the roof can
push the part along the channel: the left one sends it right, the right one sends it left, the near one sends it
forward. The robot is asked to knock the part into the yellow tote. It has to look down the drop-throughs to find the one
with the tote under it, then use the knock-out on the opposite side. A part sent down a bare bore cannot come back.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with sliders) and `lots.py` (layout lots),
both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

There are three layouts. Lot A (the screenshot): the tote in the right bore. Lot B: the left bore. Lot C: the far
bore. Trial k uses lot k mod 3, lot A first, so a run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C`
pins every trial to one lot. Always using one knob succeeds in one lot of three and averages 0.33; pushing the knob
on the tote's own side never succeeds. The lots stop a fixed habit, not a memorised replay of all three layouts.
Offline, with `lots.use("B")` or `lots.use("C")`, the lot-A answer (the part in the right bore) scores 0.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Knock the part out of the nest into the yellow tote."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/nest-dream

# Scripted demo, all three lots (each reaches 1.0 after about 17 to 22 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/nest-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`; with jitter the tote could be pushed into its bore wall.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold 3 s.
- **In the tote:** the part's centre, in the tote's frame, within 17 mm of the tote's centre in both directions
  (inside its inner walls), above the top of the tote's floor and at most at the tote's rim height; the part's
  bounding-box top at or below the channel floor. The centre is used rather than the box's bottom because a tilted cylinder's box reaches below the
  cylinder itself.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (`yam_arena.analysis`) to the part's box; it must exceed 5 mm.
  Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the part's
  lowest point is more than 5 mm below the channel floor while its centre is not over the tote's 38 mm square
  footprint, or the tote is more than 5 mm from its lot's position or tilted more than 10 degrees. The record is
  cleared only by a scene reset: the part back within 0.5 mm of its start and at rest after a jump of more than
  20 mm in one control step, which a robot cannot do.
- **Knobs:** moving a knob is never a fault in itself. A knob started the wrong way can be pulled back (by hooking
  the knob from its other side) as long as the part has not dropped down a bare bore.

## Scripted demo

`demo()` first looks: the left arm holds its wrist camera over the left drop-through and then the far one, parks to
its side, and the right arm looks over the right drop-through and parks. It then pushes the knob opposite the
drop-through that showed the tote, which in the script is the trial's lot: the left arm for the left or near knob,
the right arm for the right knob. It lowers its closed fingers beside the knob's outer face, pushes the knob along
its slot to its end stop (commanded 8 mm past it, as the arm gives way under load), lifts off, goes back and parks.
The part slides along the channel and drops into the tote; lying on its side it can roll to and fro in the tote for
a few seconds before it is still. Success must hold 3 s, so the videos show the arm lifting off. The demo uses no
tilted grips.

With `--epochs 3` the demo video, the Dream-1 video and the screenshot show the first trial, lot A (tote in the
right bore, the left knob pushed); `score.txt` and `scores` are the means over lots A, B and C. To watch another
lot, run once with `TASK_LOT=B` or `TASK_LOT=C`.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score and exits non-zero on a mismatch
```

The cases include: the part upright, lying and leaning in the tote (1.0), at the tote's inner wall (1.0), still
moving (0), pushed only to the drop edge (0), pushed the wrong way (0), caught on the tote's rim (0), the "always
the right knob" and "always the near knob" habits in lot A (part down a bare bore: 0, and still 0 if it is later
found in the tote), a part lodged 6 mm down a bare drop-through (0 for the trial) against 4 mm (no fault), the tote
lifted 6 mm (0 for the trial), the tote shifted 4 mm (1.0), and the tote moved to another bore with the part in it
(0).
