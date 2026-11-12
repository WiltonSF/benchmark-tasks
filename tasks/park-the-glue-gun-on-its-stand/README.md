# Park a tipped-over mini glue gun so it stands with its hot nozzle off the mat

A corded mini hot-glue gun (the cord is not modelled) lies tipped forward on a silicone craft mat, resting on its handle's front edge and its
nozzle. Its centre of mass is ahead of the handle, so it cannot stand on the handle alone. It has a fold-out wire stand
under the front of the housing, folded. The robot is asked to leave the gun standing on its handle with its nozzle at
least 4 cm above the mat; the instruction does not mention the stand.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with joints) and `tilt.py` (leaned grips for
the scripted demo), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with the score each must get;
they are plain-Python checks of `score()` and `success()` and need no simulator. No third-party assets are used.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Leave the glue gun standing on its handle on the silicone mat, with its nozzle at least 4 cm above the mat."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/gluegun-dream

# Scripted demo (reaches 1.0 in about 16 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/gluegun-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz), about 1 to 2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## The start pose

`task.py` declares the gun standing upright on its handle (the kit can only turn a part about the vertical). It is
nose-heavy, so during the embodiment's 0.5 s settle after every reset it tips forward onto its nozzle and comes to rest
leaning 38.7 degrees, before the first frame. That resting pose is the published start (`task.START` computes it; the
GPU traces show the gun there, within 2 mm, from the first logged step). The same fall shows what happens to a gun
stood up and let go without its stand.

## The stand and its detent

The stand is a hinge from 0 (folded back under the housing) to a stop at 135 degrees (swung down and forward). A
`parts.on_step` callback acts as the click detent: while the stand is short of 20 degrees its spring (0.08 N m/rad)
pulls it back folded; past 20 degrees the spring's rest point jumps beyond the open stop, so it springs open and
stays there. With the gun on the open stand its weight presses the stand into its stop.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold 1 s.
  `score()` returns 1.0 only once the state has held 1 s.
- **Standing:** every box and cylinder of the gun and its stand (cylinders taken as their boxes) is placed in the world;
  the lowest corner must be within 3 mm of the mat's top, and every corner within 2 mm of that lowest one counts as
  touching. All touching corners must lie inside the mat's outline, and the centre of mass (the body's 0.23 kg spread
  evenly over its shapes plus the stand's 20 g) must lie inside their convex hull seen from above. The handle's axis
  must be within 30 degrees of vertical and the centre of the nozzle tip's end at least 40 mm above the mat.
- **Let go, fail closed:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and
  measures the shortest distance from their collision hulls (yam_arena.analysis) to every shape of the gun and its
  stand; it must exceed 5 mm. Until the callback has measured, nothing is credited. Offline (`near_misses.py`, no
  simulator) each case stands for a state already held for 1 s with the grippers clear.
- Pass/fail: a released gun either stands clear of its nozzle or rests on it (or on its side); there is no safe
  halfway state worth credit. Opening the stand while the gun still lies on its nozzle scores 0.

## The demo

Open loop from the published start pose, using no simulator state. The demo imports `tilt.py` (in this folder; a
runtime wrapper that lets a scripted waypoint lean the gripper; it affects only the demo, never Dream-1 or scoring).
The right arm, its fingertips leaning 30 degrees toward the gun so its fingers stay clear of the housing, presses the
stand's near wire down past its click (it springs out until its foot meets the mat) and moves away. The left arm grips the housing across its
sides over the centre of mass, lifts the gun 75 mm so the stand swings fully open to its stop, lowers it until the
stand's foot is on the mat, and lets go; the gun rocks back onto its handle and rests leaning about 13 degrees with
the nozzle tip about 68 mm up.

## Offline checks

`near_misses.py` sets the environment variable `GLUE_GUN_TASK_OFFLINE_CHECK=1` before its cases run; only then does
`task.py` treat a state as already held 1 s with the grippers clear. Without it (in the simulator) the release and
hold checks above apply, failing closed.
