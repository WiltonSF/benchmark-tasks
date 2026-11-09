# Lock a multipoint back door: lift the handle before the key will turn

A section of a uPVC back door stands on the table facing the robot, with a lever handle and, below it and to one
side on its own plate, a key already in the lock. The lever has a white end and the key's red bow a white stripe,
so both angles show on video. Like most multipoint locks in UK and European homes, the key will not turn until the handle has been
lifted to its upper notch, which throws the hook bolt into the keep. The robot is asked only to lock the door with
the key; the instruction does not mention the handle.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with hinges and sliders), in this folder;
it imports only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with the score each
must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Lock the back door with the key."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/door-dream

# Scripted demo (reaches 1.0 in about 21 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/door-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. The door is fixed and has no loose parts; run with `-E object_jitter=0`.
- **Layout:** one fixed layout. There is no positional choice for a habit to win, so there are no layout lots; a
  memorised replay of the demo is not ruled out.

## How the lock is modelled

The door is one fixed part with three moving links: the handle (a hinge about the spindle, -0.6 to 0.8 rad), the key
(a hinge about the cylinder, 0 to 1.6 rad) and the hook bolt (a slider, 30 mm up). A `parts.on_step` callback runs
the lock's gear every control step:

- the handle's spring pulls it to the nearer notch: level (0) below 0.35 rad, lifted (0.7 rad) above;
- the hook bolt is driven up in proportion to the handle's lift (30 mm at 0.7 rad);
- the key's upper limit is 1.6 rad while the handle is lifted 0.5 rad or more, and 0.15 rad otherwise;
- once the key is past 0.3 rad, the handle's lower limit is 0.5 rad (a locked door's handle cannot be pressed down).

## Scoring details the simulator applies

- **Let go:** the callback measures the shortest distance from both arms' finger and gripper collision hulls
  (yam_arena.analysis) to the handle lever and the key's shank and bow; it must exceed 5 mm. Offline
  (`near_misses.py`) there is no robot, so this counts as clear.
- **Still:** neither the handle nor the key moved more than 0.002 rad in the last control step (0.04 rad/s);
  success must hold 1 s.
- 1.0 needs handle >= 0.5 rad and key >= 1.4 rad; 0.25 needs the handle in its notch, >= 0.65 rad. All limits
  pass exactly at the limit (1e-6 margin). Partial credit needs the notch, not just a lift: a lever brushed up past
  0.35 rad springs into the notch, so the tier is kept small (0.25).

## Scripted demo

`demo()` drives the right arm only. It turns the wrist 90 degrees so the fingers close across the lever's 12 mm
thickness, pinches the lever 70 mm from the spindle, raises it along its arc in six steps to 0.85 rad (the stop is
at 0.8; the notch holds it at 0.7) and lets go. Then, fingers closed, it puts the fingertips beside the key's bow and
pushes the bow round its arc in eight steps to 1.8 rad (the stop holds it at 1.6), rises and parks. It reads nothing
from the simulator.

## Measured in simulation

- Demo trace (trace-20261109T010314, final layout): the handle rose to 0.68 rad and stayed in its notch, the hook bolt rose 26 mm,
  the key turned to its 1.6 rad stop, and the run succeeded at 20 s with score 1.0.
- Key-first probe (trace-20261108T234153, a throwaway copy of the demo that skips the handle and pushes the key
  round the same arc): the key stopped at its 0.15 rad interlock, the handle stayed level, score 0.
- An earlier layout, with the key directly below the spindle, failed: the fingers turning the key pressed on the
  lifted lever. The key now sits 60 mm toward the door's edge and turns away from the lever.

## Other ways to succeed, and what scores 0

- **Also full score:** any arm, any grip, any path, as long as the handle ends lifted, the key at its stop and the
  robot has let go.
- **0.25:** the handle in its upper notch and the key not home.
- **0:** turning the key without lifting the handle (it stops at 9 degrees); pressing the handle down (the opening
  move); leaving it short of its notch; still holding the handle or key at the end.

The cases in `near_misses.py` cover the 0.5, 0.65 and 1.4 rad limits on both sides, the key stopped at its interlock,
the handle pressed down and a handle lifted only part way.

## Recorded runs

- Scripted demo: 1.0 (success at about 20 s).
- Dream-1, 1,200 steps: 0.0. The left arm went close to the key and the right arm toward the handle, but the handle
  was not lifted into its notch and the lock was not locked; the run hit the step limit.
