# Stand a bike on its side stand, clear of the wall it leans on

A model bicycle leans back against a wall, its side stand folded. The robot has to swing the stand down, then tip
the bike off the wall, through upright and over onto the stand, so that it stands by itself leaning toward the
robot, clear of the wall. Upright on its narrow tyres alone, or left against the wall with the stand down, it does
not count.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. `parts.py` (a small parts kit) builds the bike as one loose part with a hinged stand;
`tilt.py` provides the waypoint type the demo uses (this demo uses only straight-down grips). Both files must sit
next to `task.py`. `near_misses.py` checks `score()` and `success()` without a simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Stand the bike up on its kickstand, clear of the wall."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/bike-dream

# Scripted demo (reaches 1.0 in about 15 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/bike-demo
```

- **Runtime per trial:** at most 40 s simulated (800 steps at 20 Hz), under a minute of wall time on an L40S once
  Isaac Sim has started.
- **Seeds:** none. Run with `-E object_jitter=0`: the bike starts against the wall, and jitter would push it in.

## How the scene is built

- **The start lean.** The task-file format places loose objects upright, so a reset hook (`parts.on_reset`) sets
  the bike's pose after every reset: leaning 11.5 degrees back on its tyres' wall-side edge, the handlebar's end
  0.5 mm from the wall. The same hook folds the stand and clears the trial record.
- **The stand's detent.** An on-step hook moves the stand spring's rest point: past 45 degrees to beyond its down
  stop (it snaps down), short of 45 degrees to beyond its folded stop.
- **Measures in the scorer.** Lean from the bike's orientation; tyre heights from points around both rims; the
  foot's height from the stand link's corners; wall clearance from the bike's bounding box. The "lay down" record
  (more than 35 degrees at any step) is cleared only by the reset hook.
- **Released.** The scorer is given only object states, so an on-step hook reads both arms' joint angles and computes
  each gripper's pose (from the arm's own kinematics) and opening. Each finger is a box 10 mm thick, 20 mm wide and
  45 mm long back from its tip; success needs every finger point more than 5 mm from every shape of the bike.

## Scripted demo

One arm (the right). Fingers closed, it presses the folded stand's leg down from above past the detent; in the
trace this also rocked the bike off the wall to upright, where it stood on its tyres. It then comes down with its
fingers open across the saddle (one finger each side) and draws the saddle 6 cm toward the robot, so the bike tips
over onto its stand (10.5 degrees in the trace), then at once backs the far finger off the saddle and rises clear.
In the GPU trace the fingers were last within 5 mm of the bike about 0.4 s before success first held, and the 1 s
hold ran with every finger 10 to 24 mm or more away.

## Habits that score 0

There is one bike, so there is no "nearest" or "leftmost" choice. The habits that fail: kicking the stand down and
leaving the bike on the wall (0 cm clearance); pulling the bike upright and letting go (it stands on its tyres, foot
1 cm clear: lean under 4 degrees); pulling it over before the stand is down (it lies down). `near_misses.py` has a
case for each, plus a 30-degree lean that is not lying down, a bike held 1 cm up, a moving bike, one on its stand
but only 2 cm from the wall, and a bike stood on its stand after lying down (0). Two more cases put a gripper astride
the saddle of the bike on its stand: a finger 4 mm from it scores 0, every finger 6 mm clear scores 1.0 (these need
`python near_misses.py`, which places the grippers itself). Holding it upright with one hand
while kicking the stand down with the other is also a correct method.
