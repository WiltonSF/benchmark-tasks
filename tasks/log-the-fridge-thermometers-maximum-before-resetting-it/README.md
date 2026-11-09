# Log the fridge thermometer's maximum before resetting it

A max/min thermometer unit lies on top of a vaccine fridge (the table). Beside its five colour bands, a black marker
stands where the temperature reached since the last reset. A RESET button returns the marker to the present
reading. A log board to the right has the same five bands and a sliding pointer. The robot is asked to reset the
thermometer and log this morning's maximum. The instruction names the reset first; the log has to come first,
because the reset erases the reading.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with sliders) and `lots.py` (layout lots),
both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes
with the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

The marker starts in the yellow band (lot A, the screenshot), red (B) or orange (C); the present reading (the index)
is always in green, so the maximum never shares a band with it. Trial k uses lot k mod 3, lot A first, so a run needs
`--epochs 3` (or a multiple); `TASK_LOT=A` (or `B`, `C`) pins every trial to one lot. The fixed thermometer cannot be moved by a lot, so an
`on_reset` hook sets the marker's spring target for the lot, and every lot moves the vial carton (a distractor) by zero so the
scene settles 0.5 s and the first frame shows the marker in place. Expected averages of fixed habits (offline):
reset first, 0; always log green (the present reading), 0; always log the top (red), 0.33; replaying lot A, 0.33;
logging correctly but never resetting, 0.5. The lots stop a single fixed habit, not a memorised replay of all three
layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Reset the vaccine fridge's thermometer for the day and log this morning's maximum on the board."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/thermo-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/thermo-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- **Bands:** the pointer is in a band when its tab's centre is within the band's 28 mm (edges included).
- **Reset:** the marker's slide is within 3 mm of its reset place against the index. The marker is a slider held
  by a 50 N/m spring; an `on_step` hook moves the spring's target to the index once the RESET cap is 5 mm or more
  down, and keeps it there for the rest of the trial. Pushed by hand, the marker springs back when let go.
- **Order fault (whole trial):** any step with the cap 5 mm or more down while the pointer is not in the lot's band
  sets a fault, and the score is 0 for the rest of the trial. It is cleared only by a scene reset (the `on_reset`
  hook; offline, the pointer back within 0.5 mm of its start with the cap up, after a jump of over 20 mm in one
  control step, which a robot cannot do).
- **Let go and still:** an `on_step` hook reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to the pointer tab, marker and button cap; it
  must exceed 5 mm. Pointer, marker and button must each move under 0.3 mm per control step. Success must hold 1 s.
  Offline (`near_misses.py`) there is no robot, so these count as met.

## Scripted demo

`demo()` moves the right arm's closed fingers down just left of the pointer tab, pushes the tab right to the middle
of the band the marker stands in for the trial's lot (which the cameras show), and lifts away. Then the left arm
presses RESET with closed fingers commanding 16 mm past first contact (the cap stops at 8 mm) and lifts away. The demo reads nothing from the
simulator.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, pushing or gripping the tab; any path, as long as the pointer is in the right band
  before RESET is pressed.
- **Score 0:** pressing RESET first, or with the pointer passing through another band; logging the present (green)
  reading or any band other than the morning's maximum; leaving the pointer in the blank zone; holding a part at the
  end.
- **0.5:** the right band logged, RESET never pressed.

The cases in `near_misses.py` cover the band edges on both sides, the 3 mm reset limit on both sides, the 5 mm press
limit on both sides, reset before the log, reset with the pointer in the wrong band or passing through one, a later
move to a wrong band, and scene resets.

## Recorded runs

- Calibration trace (with a former lot D, since dropped): 1.0 in each lot; the pointer ended 1 to 2 mm short of each
  band's middle.
- Scripted demo, `--epochs 3`: 1.0 in each of lots A, B and C (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 3`: 0.0 in lots A and B (the pointer was never logged); in lot C the Dream-1 service refused the inference request (DA-4000 BAD_REQUEST) and the trial errored without a score.
