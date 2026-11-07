# Fit an inside-bolt-circle gasket: far bolts first as stops, then the near bolts

Two flanges are held 20 mm apart. A ring gasket that just fits inside their bolt circle waits on a block in front of
the gap, and four bolts stand on the table. The gasket centres itself only against bolts. With the two near bolts in,
it can't pass between them. So the robot puts the far bolts in first, slides the gasket in by its tab until it rests
against them, lets go, and then adds the near bolts.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. `parts.py` next to it builds the flanges (one fixed part), the gasket and the
bolts (loose multi-shape parts). It patches `yam_arena`'s scene spawning and step and reset at import time, in the
task's own process only, and changes nothing on disk. `near_misses.py` holds near-successes, fixed habits and wrong
outcomes with the score each must get. It runs as plain Python without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the gasket between the flanges, centred on the pipe, put all four bolts in and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/gasket-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/gasket-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, and `task.json` states it in words.

- **Runtime per trial:** at most 70 s of simulated time (1,400 steps at 20 Hz). The demo reaches 1.0 at about 54 s.
- **Seeds:** none. Run with `-E object_jitter=0`, so the layout is the same every trial.

## Scoring in the simulator

- **Positions:** each part's `pos` is its origin, the centre of the gasket's underside or the end of a bolt's shank.
  A seated bolt's origin is 3 mm above the table, with its head resting on the upper flange.
- **Release check:** every step an `on_step` hook places the hulls of both arms' gripper and finger links at their
  simulated poses (fingers as they are). It measures their distance to every shape of the gasket and of each bolt.
  A part counts only while no gripper is within 5 mm of it. Offline there is no robot, so this check passes.
- **Holes and bolts:** the bolts are identical, so any bolt may fill any hole.

## Geometry

- **Gasket fit:** the gasket's outer radius (59.5 mm) is 0.5 mm inside the bolts' inner edges (65 mm bolt circle
  less the 5 mm shank radius).
- **Near bolts block the gasket:** they are 92 mm apart centre to centre, 82 mm clear, so a 119 mm gasket cannot
  pass between them.
- **Far bolts centre the gasket:** pushed against the two far bolts, it stops 0.7 mm past the axis when they sit in
  the middle of their 13 mm holes. It stops at most about 3.7 mm past when the push shoves them to the far side of
  their holes. Both are inside the 4 mm tolerance, and the near bolts then clear its edge.
- **Lower flange holes:** these are 16 mm clearance holes, so a bolt is located by the upper flange alone.

## Habits that must not pass

The layout is fixed, so each ordinary habit was checked against it:
- **Nearest holes first** (the near holes are nearer the arms): the gasket then stops against the near bolts
  40 mm short, and the trial scores 0.
- **All four bolts first:** the gasket can't get in, and the trial scores 0.
- **Gasket first with no bolts,** then all four bolts: fair, if it is centred by eye to about 2 mm in each diagonal direction.
- **Gasket centred with only the near bolts in:** 0.
- **Gasket pushed in hard with no stops** ends 10 mm past the axis: 0.

## Scripted demo

`demo()` uses fixed waypoints and reads no simulator state. The arms take turns, so their grippers are never near
each other:
1. The left arm puts a bolt in the far-left hole.
2. The right arm puts a bolt in the far-right hole.
3. The right arm grips the gasket's tab and slides the gasket in to 3 mm past touching the far bolts, then lets go.
4. The left arm puts a bolt in the near-left hole.
5. The right arm puts a bolt in the near-right hole.

For each bolt the arm grips the head (fingers 5 mm past its faces) and carries it over the hole. It lowers the bolt
until the shank end is 2 mm above the upper flange and settles there. Then it lowers the bolt in until the head is
3 mm above its seat, and lets go.

In trace-20261107T160429 (GPU trace), every bolt went in without aim offsets. The demo reached 0.5 when
the gasket was in against the far bolts, and 1.0 at about 54 s. The gasket ended 2.7 mm from the axis. Its push
moved one far bolt 2 mm toward the far side of its hole.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases cover:
- the gasket tolerance boundaries: centre at 4.0 and 4.1 mm, height at 1.5 and 2 mm, tilt at 2.9 and 3.1 degrees;
- the 0.5 tier: far bolts only, and far bolts plus one near bolt;
- the habits above;
- the bolt limits: a 3 degree lean (in) and 9 degrees (out), head height at 2 and 2.5 mm, head centre at 3 and 3.5 mm from the hole centre;
- a moving gasket, and doing nothing.
