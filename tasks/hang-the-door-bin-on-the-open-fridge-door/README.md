# Hang the door bin on the open fridge door, whichever way the door stands

An open fridge door stands on the table with its inner liner toward the robot. On the liner are two channels that
are closed at the front and open only at the top. A door bin with two T-tabs on its back stands on the table. The
robot is asked to fit the bin to the door; the instruction does not say how it fixes on. The bin must be turned
until its back is parallel to the door, lifted above the channels, brought onto their line and lowered so both
tabs drop in and rest on the stops. The door's angle changes from trial to trial. A bin pushed at the door at hang
height meets the closed channel fronts and does not go on.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in
this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

Lot C (the first trial, so the screenshot and both videos show it): the door turned 40 degrees with its right end
toward the robot, the bin 12 cm left of the midline, square; the left arm works it. Lot A: the door square to the
robot, the bin 12 cm right of the midline, turned 20 degrees. Lot B: the door turned 30 degrees with its left end
toward the robot, the bin turned -30 degrees. The door is a heavy loose part (4 kg) only so the lots can turn it.
Trial k uses lot k mod 3 in the order C, A, B, so a run needs `--epochs 3` (or a multiple); `TASK_LOT=C`, `A` or
`B` pins every trial to one lot. The lots stop a fixed habit (always yaw 0, always one place), not a memorised
replay of all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the door bin to the fridge door."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/bin-dream

# Scripted demo, all three lots (each reaches 1.0 in about 15 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 400 --epochs 3 --store-frames --log-dir ~/yam-eval/bin-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold 1 s.
- **Door frame:** each tab head's centre is transformed into the door's own frame (x' into the door from the liner
  face, y' along it, z up), so the same test holds at any door angle.
- **Pass/fail.** 1.0 when both tabs are in their channels and on the stops, the bin upright and parallel, let go
  and still; 0 otherwise.
- **Tab in its channel:** the head lies between the lips' back face (12 mm in front of the liner) and the liner,
  its centre within 3.5 mm of the channel's centre line along the door (the pocket is 25 mm wide, the head 18 mm),
  and its underside between 1 mm below and 3 mm above the stop's top (35 mm up).
- **Upright and parallel:** the bin's vertical axis within 10 degrees of vertical; the angle between the bin's and
  the door's long axes, both projected onto the table, at most 3 degrees. Hung, the bin leans forward on its tabs
  by a few degrees (the tabs have 6 mm of play front to back).
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the bin; it must exceed 5 mm.
  Offline (`near_misses.py`) there is no robot, so this check counts as clear.

## Scripted demo

`demo()` drives the right arm in lots A and B and the left arm in lot C (the right arm cannot turn its wrist far
enough to the left of the midline). It grips the bin's front wall across its thickness (pads opened 24 mm, closed
1.5 mm past each face), lifts it until the tabs are 12 mm above the channel tops, turns it square to the door while
carrying it in front of the door, moves it onto the channels' line (its back aimed 2.5 mm behind the lips' front
face, because the loaded arm lags about 5 mm toward the robot), lowers it at about 2 cm/s until the tabs are 6 mm
above the stops, opens and backs off. The door's angle and the bin's come from the trial's lot, which the cameras
show; the demo reads nothing from the simulator.

The bin's back wall is full height only over its middle 12.2 cm; its end sections, around the tabs, start 2 cm up.
That lets the heads drop 2 cm into the pockets, which then guide the bin front to back, before the wall comes down
level with the lips' tops.

## Measured in simulation

- Demo, three lots (trace-20261109T170723): 1.0 in each. The heads entered the channels about 7 mm behind the lips'
  front face and settled with their fronts against the lips; the bin's yaw matched the door within 0.1 degree.
- Shortcuts traced (trace-20261109T171108, a habit copy of the task): in lot A, carrying the bin straight at the
  door at hang height left the tabs against the channels' closed fronts, 22 mm in front of the liner, and the bin
  dropped to the table (0). In lots B and C, replaying lot A's demo gripped at lot A's spot and yaw and missed the
  bin (0 in each).
- Earlier traces set the tab play: with 2.5 mm of play along the door the lot A bin, carried 3 mm off the line,
  landed on the channel tops (trace-20261109T170312); with the aim 0.5 mm in front of the lips the heads stalled on
  the lips' tops (trace-20261109T165850).

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any grip, sliding the tabs down from any height, a bin that settles anywhere in
  the tabs' play; turning the door first (it is a loose 4 kg part so the lots can turn it, and the score is measured
  in the door's own frame) and then hanging the bin.
- **Score 0:**
  - pushing the bin at the door at hang height: the tabs meet the channels' closed fronts;
  - lowering it at the wrong yaw or off the channels' line: the heads land on the channel tops or the lips;
  - leaving it on the channel tops, on top of the door, on the table, jammed part way down the channels, upside
    down, or still held; a bin still swinging.

The cases in `near_misses.py` cover the 3 mm stop limit on both sides, heads 9, 10 and 20 mm into the channels
(all 0), an upside-down bin, the
pocket's play along the door (3.5 mm in, 4 mm out), the tabs' front-to-back play, a 6-degree and an 11-degree lean,
a 1.5-degree turn, a swinging bin, a bin held in front of the channels, on the table, on top of the door, the bin
hung on a door turned 30 and -40 degrees, and a bin at yaw 0 in front of a turned door.

## Recorded runs

- Scripted demo, `--epochs 3` (demo-20261109T173717): 1.0 in lots C, A and B (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 3` (dream-20261109T174053): 0.0 in all three lots.
- The screenshot and both videos are the first trial's: lot C.
