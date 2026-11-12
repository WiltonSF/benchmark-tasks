# Singulate the middle box from a tight cluster

Nine 5 cm cardboard cubes stand packed side to side in a 3 x 3 square on a grey mat. The blue one in the middle has
no free side face, and the gripper (about 94 mm open) cannot take two cubes at once, so no cube can be grasped as it
stands. The robot has to open finger room around the blue cube without grasping (for example by pushing a row of
cubes along), then pick the blue cube and set it on the green pad, keeping the eight brown cubes flat on the mat
the whole time.

This differs from sliding a single item to an overhang before lifting it: the blue cube cannot be gripped until room
is made beside it by moving its neighbours (one push of its row along its length is enough, as the demo shows), and
those neighbours must stay flat on the mat throughout. The task is for finger grippers; a
suction cup could lift a packed box from above.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment
for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the blue box from the middle of the cluster on the green pad. Keep the other eight boxes on the grey mat the whole time, each sitting flat on the mat, not on another box."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/singulate-dream

# Scripted demo (reaches 1.0 in about 20 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/singulate-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

Score tiers: 1.0 for the blue cube flat on the pad, inside its edges and at rest; 0.5 for the blue cube set down flat
off the mat (on the table, or on the pad over its edge) and at rest; 0.25 for the blue cube flat on the mat and at
rest with a 15 mm strip clear of other cubes along each of two opposite side faces (finger room); 0 otherwise. Every tier needs the eight brown cubes intact: flat on the mat at every step of the trial (never
lifted, stacked, tipped past flat or past the mat's edge, even for a moment) and at rest at the end.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds and jitter:** run with `-E object_jitter=0`, as submissions are. Every object then starts exactly where
  `task.py` puts it and no seed is needed. Jitter would shift each cube independently and break the packed square
  (and the scorer's new-trial detection, which looks for every cube at its starting place), so the task needs
  `object_jitter=0`.
- **Whole-trial rule:** at every control step `task.py` records any brown cube that is not on the mat: its centre
  past the mat's edge, its bottom more than 3 mm above the mat's top (lifted or stacked), or its highest point more
  than 53 mm above its lowest (tipped). One such step zeroes the trial, even if the cube is later put back. The
  record clears only on an episode reset: every cube within 0.5 mm of its start and at rest, reached by a jump of more
  than 50 mm since the previous step, so a robot putting a cube back in its place cannot clear it. Run one trial per process if
  you reuse the module outside the embodiment.

## Scripted demo

`demo()` drives the left arm only. With its fingers closed and pointing down, tips 4 mm above the mat, it pushes
the middle row of the square (left brown cube, blue cube, right brown cube) 115 mm along its length toward the
robot's right, so the blue cube slides out of the square with nothing in front of or behind it (the 0.25 state,
which holds after the arm lifts away). It then turns the fingers to close front to back, opens them to 72 mm,
grips the blue cube, slides it 60 mm toward the robot along the mat (out from between its two pushed neighbours,
which it still touches), lifts it, carries it to the pad, lowers it to 7 mm above the pad and lets go. Lifting it
straight up from between those neighbours tipped one of them past flat in simulation, which the whole-trial rule
fails, so the demo slides it clear first.

Two offsets in the demo were measured in simulation (Isaac Sim on an L40S) and cancel the arm's tracking error: the push is commanded 15 mm
further than the row should move, and with the fingers turned the grip and place targets aim 18 mm further from the
robot than the cube and pad centres.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the blue cube on the pad centred, 14 mm off centre and turned 45 degrees (1.0); over the pad's
edge or on the bare table (0.5); held 5-6 mm up, still moving or leaning 10 degrees (0); the middle row pushed
115 mm (0.25) or 60 mm (0); an intact cluster with simulator-noise rotations (0); finger room but held 2 mm above the mat (0); neighbours moved back 16 mm (0.25) or 10 mm (0); a brown cube slid to the mat's edge or
elsewhere on the mat (still 1.0); a brown cube stacked, leaning or sliding (0); a brown cube lifted 20 mm, stacked on
another or tipped 45 degrees part way through a roll, then put back flat on the mat before the blue cube reaches the
pad (0 for the whole trial); a brown cube lifted exactly 3 mm and put back (1.0) or 3.1 mm and put back (0); a brown cube pushed off the mat and put back, even exactly at its start, or carried to the pad (0 for the whole
trial); and exact-limit cases on both sides of 1 mm, 3 mm, 53 mm, the pad edge and the 15 mm strip.
