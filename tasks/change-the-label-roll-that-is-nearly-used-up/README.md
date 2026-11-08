# Change the label roll that is nearly used up for the full spare

Three label printers stand in a row, each with a label roll over an upright spindle on its deck. Two rolls are full
(86 mm across); one is nearly used up (50 mm across, a thin band of labels on its core, the top of it red: the
end-of-roll stripe). A full spare roll and a grey bin stand in front of the printers. The robot is told that one
printer is about to run out and must fit the spare to that printer and bin the old roll. Nothing names the low roll;
the robot has to see how much label is left. The spare's 36 mm core hole must go down over the 30 mm spindle.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in
this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

There are three layouts (columns are the three printers: left, centre, right):

| Lot | Nearly used-up roll | Spare in front of | Bin in front of |
|---|---|---|---|
| A (screenshot) | left | left | centre |
| B | centre | left | right |
| C | right | centre | right |

Trial k uses lot k mod 3, lot A first, so a run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C` pins
every trial to one lot. The lots are chosen so that each of these habits picks the low roll in exactly one lot and
averages 1/3: always the left, centre or right printer; the printer behind the spare; the printer behind the bin; the
printer with nothing in front of it. A replay of one lot's motions changes a full roll in the other two lots.
`python near_misses.py` scores each habit's end state in every lot and prints these averages (all 0.33; the
intended choice 1.00).

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="One of the label printers is about to run out of labels. Fit the spare roll to it and put the old roll in the grey bin."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/roll-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/roll-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`), for every roll and the
  bin; the trial ends at 1.0 once the 1.0 state has held for 1 s.
- **Rolls with a hole:** the kit has only solid shapes, so each roll is two rings of overlapping upright cylinders
  (core and labels). The hole's surface is 18 to 19.6 mm from the axis. A roll's lowest point is computed from its
  true round outline (the lower rim of whichever end is down), not its bounding box.
- **On a spindle:** lowest point within 1 mm of the deck (40 mm up), the axis within 4 mm of the spindle's (the hole
  gives 3 mm of play) and within 5 degrees of vertical. A roll resting on the spindle's top or caught on it fails the
  height test.
- **In the bin:** the roll's centre (halfway along its axis) inside the bin's 100 x 100 mm, measured in the bin's own
  frame wherever the bin now stands (upright within 10 degrees), lowest point within 20 mm of the bin floor, any
  orientation. **On the table:** lowest point within 2 mm of the table top, anywhere, any orientation.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the spare and the old roll; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, once either full roll on a
  printer is more than 10 mm from its lot's start. A roll on a spindle can shift at most 3 mm sideways; lifting it
  off the 28 mm spindle moves it further, so pulling a full roll and putting it back still scores 0. The record is
  cleared only by a scene reset: every roll and the bin back within 0.5 mm of their starts and at rest, after one of
  them jumped more than 20 mm in one control step. A robot cannot move them back that way.

## Scripted demo

`demo()` follows the trial's lot, as the cameras show it (which printer holds the thin red-striped roll, where the
spare and the bin stand); it reads nothing from the simulator. Each arm works only its own side and the centre. The
arm on the bin's side opens to 70 mm over the old roll, closes 3 mm past its faces 10 mm down its side, lifts it
clear of the spindle, carries it over the bin and drops it from 15 mm above the rim. The arm on the spare's side
(the same arm in lots A and C; in lot B the right arm bins the centre roll and the left arm fits the spare) opens
fully (94 mm) over the spare, closes 3 mm past its 86 mm faces, lifts it 17 mm clear of the spindle top, carries it
over the spindle, lowers it until its underside is 6 mm above the deck, opens, rises and backs off.

## Measured in simulation

- Trace with `--epochs 3` (trace-20261108T045418): in each lot the old roll landed upright on the bin floor, the
  spare dropped over the spindle with no offset correction and settled within 2.4 mm of its axis, the full rolls did
  not move, and the trial scored 1.0 about 19 to 20 s in, after the fingers had opened.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any grip, the old roll in the bin on its side, the bin nudged with the old roll
  still in it, or fitting the spare after setting the old roll down somewhere first.
- **0.5:** everything right but the old roll left anywhere on the table instead of in the bin.
- **Score 0:** the spare fitted to a printer with a full roll; more than one roll changed; a full roll lifted off and
  put back; the spare left on the table, in the bin, on a spindle top or on the old roll; the old roll still on its
  spindle, on a printer, on the bin's rim or off the table; anything still held or moving.

The cases in `near_misses.py` cover the 1 mm seat and 4 mm axis limits on both sides, the spare beside the spindle,
on its top and caught leaning 8 degrees, a moving spare, the old roll on its side in the bin and on the table, held
3 mm over the table, on the rim, on a printer and off the table, a moved bin, a full roll nudged 3 mm and lifted 9 mm
(no fault) and 12 mm (fault, then a correct change still 0), the wrong printer, all three changed, and scene resets.

## Recorded runs

- Scripted demo, `--epochs 3`: 1.0 in lots A, B and C (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 3`: 0.0 in all three lots (step limit reached).
