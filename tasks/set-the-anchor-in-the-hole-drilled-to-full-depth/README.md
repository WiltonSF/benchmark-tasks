# Set the anchor bolt in the drilled hole that is deep enough

A concrete slab has three drilled holes that look the same from above. One is drilled to 50 mm; the other two stop at
25 mm, where the drill hit rebar (a hidden plug at the hole floor). The robot has an anchor bolt with a 45 mm shaft
and is asked to set it flush in one of the holes. In a short hole the anchor's washer stands 20 mm proud, so only the
full-depth hole can pass. Which hole is full depth changes between trials (layout lots), and nothing visible shows
it: the robot has to try the anchor and see how far it goes down.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in
this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

Lot A (the screenshot): plugs in holes 1 and 3, hole 2 (middle) full depth. Lot B: plugs in holes 1 and 2, hole 3
(right) full depth. Lot C: plugs in holes 2 and 3, hole 1 (left) full depth. Trial k uses lot k mod 3, lot A first,
so a run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C` pins every trial to one lot. A single-hole habit
(always the same hole, the nearest or the middle one) is right in one lot of three and averages 0.33. Trying the
holes in a fixed order and checking each fit is a valid test fit and scores 1.0; leaving the anchor in the last hole
of a fixed order without checking scores 0.33. The plugs are loose (150 g) only so the lots can move them; they sit inside the holes and
cannot be reached.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Set the anchor bolt flush in one of the three drilled holes."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/anchor-dream

# Scripted demo, all three lots (each reaches 1.0 in 15 to 32 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --epochs 3 --store-frames --log-dir ~/yam-eval/anchor-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- The anchor's frame has its origin at the centre of the washer's underside. Seated: that point within 3 mm of the
  hole's centre in x and in y (the shaft has 2 mm of play each side), within 1 mm of the slab top in height, and the
  anchor's axis tilted at most 3 degrees. The hole it is seated in must be the lot's full-depth hole.
- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`), for the anchor and both
  plugs; success must hold 1 s.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the anchor; it must exceed
  5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- Pressing the anchor into a short hole cannot lower it: the plug stands on the hole floor, which is fixed.

## Scripted demo

`demo()` drives the left arm only. It pinches the anchor's nut from above (pads closed to 6 mm on the 12 mm nut),
lifts it out of the holder and tries the holes from left to right. Each try: lower the shaft tip into the hole until
the washer is 20 mm above the slab, open the pads to 18 mm, then to 30 mm, rising 35 mm. In a short hole the anchor drops 5 mm onto
the plug and stands 20 mm proud; the arm pinches it again and lifts it out. In the full-depth hole it drops to flush
and the arm rises and backs away. The number of tries follows the trial's lot; it stands for seeing in the cameras
whether the anchor stopped proud or went down flush. The aim points are offset by up to 4 mm per hole for where the
loaded arm was traced to settle against its commanded target.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any grip, any order of tries, probing a hole with something else first.
- **Score 0:** the anchor in a 25 mm hole, still held, tilted over 3 degrees, lying on the slab, on the table or in
  its holder; the anchor flush in the right hole but still moving.

The cases in `near_misses.py` cover the 3 mm centre, 1 mm height and 3-degree tilt limits on both sides, the anchor
proud in each short hole, an impossible flush anchor in each short hole, a moving anchor, a settling plug, and the
fixed habits.

## Recorded runs

- Release check, lot C: a probe demo that carried the anchor to the full-depth hole and held it there, flush,
  still and upright, for 10 s scored 0 for the whole trial.

- Scripted demo, `--epochs 3`: 1.0 in lots A, B and C (task_score 1.0). Lot A tries hole 1 then sets it in hole 2;
  lot B tries holes 1 and 2, then hole 3; lot C sets it in hole 1 at the first try.
- Dream-1, 1,200 steps, `--epochs 3`: 0.0 in all three lots (each hit the step limit).
