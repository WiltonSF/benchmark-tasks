# Flip the short-part stop up before loading a long bar

A cut-off fixture: a fence with a fixed end stop, and a hinged flip stop set 12 cm in from the end stop for short
parts. The robot is asked to set an 18 cm bar against the fence and the end stop. The flip stop is in the way and
the instruction does not mention it: the robot has to flip it up and back over the fence on its hinge (past its
balance point, where its weight holds it up), then lay the bar in and push it home against the fence and the end
stop.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena), on the `station` scene. The hinged flip stop is built with `parts.py` (in this folder), which adds
multi-shape parts and hinges to the task-file API at import time; it uses only NumPy and the `yam_arena` package.
`near_misses.py` lists near-successes and wrong outcomes with the score each must get.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Set the long bar flat against the fence and the end stop, ready for the long cut."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flip-stop-dream

# Scripted demo (reaches 1.0 in about 45 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flip-stop-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0` (as submitted): every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift only the loose bar.

## How it is scored (simulator details)

- Gaps are computed from the bar's true pose (centre and orientation), not its world-aligned bounding box, so a
  turned or skewed bar is measured at its real corners. Each tolerance is compared with a 1e-6 m margin so a gap of
  exactly 1 mm (or 1.5 mm, 5 mm) passes.
- "At rest": the bar's speed under 1 cm/s and spin under 0.5 rad/s.
- **Released:** after every control step an `on_step` callback reads both arms' `gripper`, `tip_left` and
  `tip_right` link poses and measures their collision hulls against points every 5 mm or less over the bar's six
  faces. The bar counts as released only when every hull is at least 5 mm away. Offline (`near_misses.py`, no
  simulator) the check is skipped and counts as released.
- Both scoring tiers need the flip stop up (125 degrees or more), so a stop lowered back onto a loaded bar scores 0.
  There is no credit for the flip stop alone. The 1.0 condition must hold for 3 s (`SUCCESS_HOLD_S = 3.0`).
- The flip stop's angle is its hinge joint position: 0 hanging down on its lower stop, 135 degrees on its upper
  stop. Its balance point is about 120 degrees, so any angle of 125 degrees or more stays up by itself.
- With the flip stop down the bar cannot be loaded: only 12 cm is left between the flip stop and the end stop, the
  stop reaches 45 mm out from the fence face and down to 4 mm above the table, and pushing the bar endwise against
  it gives no turning force about the hinge.

## Scripted demo

`demo()` uses both arms. The left arm grips the flip stop's free end with its fingers closing along the hinge axis,
so the stop turns between the pads, and carries it along its arc. The stop lags its grip, so the arc is commanded
to 175 degrees; the stop reaches its 135-degree stop, and the left arm lets go and parks. The right arm picks the
bar meanwhile, waits, lays it in 25 mm off the fence and 4 mm short of the end stop, releases it from 6 mm up with
the fingers opened only 7 mm a side (so the far finger clears the fence), then with closed fingers pushes it
sideways onto the fence (three pushes) and endwise into the end stop, then parks well away so the bar is
released for the 3 s hold. Pushes are commanded several mm past contact
because the arm gives way under load. No step reads hidden simulator state; every waypoint is fixed.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: loaded (1.0), gaps at exactly 1 mm and 1.5 mm (1.0), the bar turned end for end or upside down
(1.0), end gap 1.1 mm and 5 mm (0.5), 5.1 mm (0), side gap 1.6 mm (0), a skewed bar, a moving bar, a bar 1.5 mm up,
tilted 3 degrees or on its edge (0), the stop at exactly 125 degrees with the bar loaded (1.0) and at 124.9 degrees
(0), the stop lowered back onto a loaded bar (0), the stop up with the bar elsewhere (0), the bar fallen off the
table (0), and the fixed habits: doing nothing, sliding the bar along the fence into the flip stop, the bar against
the flip stop in the short-part position, laid over it, or loaded past its reach 46 mm off the fence (all 0).
