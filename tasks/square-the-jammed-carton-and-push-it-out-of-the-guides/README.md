# Clear a carton jammed askew where the conveyor guides narrow

A carton has turned askew in a conveyor lane and caught one leading corner on the square shoulder where the guides
narrow. The robot has to get it through the narrow part and out of the far end. Pushing it forward, anywhere on
its rear face, turns it harder into the guides and it stays caught; it has to be backed off, then fed in square.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. `parts.py` (in this folder) is imported only for its step and reset hooks, used
by the release check; every object is a plain box. `near_misses.py` scores near-successes and wrong outcomes without
a simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Clear the jammed carton: push it through the guides and out of their far end."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/carton-dream

# Scripted demo (reaches 1.0 in about 20 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/carton-demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about a minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none. Run with `-E object_jitter=0`: the carton starts touching a guide and a shoulder, and jitter
  would push it into them.

## Scoring in the simulator

`score()` and `success()` in `task.py`. The carton's position and skew come from its pose and its world-aligned
bounding box (so a turned carton's box is a little larger, which only makes "beyond the far end" stricter). "Still"
is under 1 cm/s and 0.5 rad/s. "Let go" is measured every control step by a callback that takes the convex hulls of
both arms' finger and gripper links and their distance to the carton's box; the 1.0 and 0.5 states need it at 5 mm or
more, so a carton still being pushed does not count. Success must hold for 1 s.

The carton must go through the lane, never around the guides. Every step `score()` and `success()` record a fault,
which zeroes the rest of the trial, if the carton's bounding box goes back past the guides' near end (x below 0.16)
or reaches past either guide's outer face anywhere between x 0.16 and 0.40. The record clears in the embodiment's
reset hook, and offline only when the carton jumps more than 20 mm in one step back to within 0.5 mm of its start,
at rest (a scene reset, which the robot cannot do). At the end its centre must be within 6 cm of the lane's centre
line. The demo backs the carton off to about 11 mm short of the near-end limit.

## Scripted demo

`demo()` uses fixed waypoints only. The right arm, fingers closed, comes down in the narrow part just ahead of the
carton's front face, 15 mm to the left of its centre line, and pushes the face back 6 cm: the carton slides back off
the shoulder, still askew. It then comes down behind the carton on the lane's centre line and pushes straight
through; a single finger on the askew rear face turns it square as it goes, and it leaves the far end. The left arm
parks.

## What was traced on the GPU (L40S, trace mode)

Probe runs used the same guides and carton (some with a 6 cm carton and 5 cm guides) in two lanes at once:
- Forward pushes on the caught carton all stayed caught (score 0): a finger on the middle of the lane, a finger at
  the caught side's rear corner, and an open gripper used as a 6 cm wide pusher. The carton did not move or turned
  back to 9 degrees against the shoulder.
- Backing off only 2 cm and then pushing up the middle: the carton turned back into the shoulder (0). It needs
  about 5 cm of run before the shoulder to come square.
- Turning it square in place by pushing back at its free corner: it pivots on the caught corner and ends up against
  the left guide, 7 to 8 mm off centre, and catches again (0). The narrow part allows 3 mm per side.
- The mirror-image layout (turned the other way) behaved the same way with the mirrored demo (1.0).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

Cases include: out at exactly the far end (1.0), 1 mm short and square in the narrow part (0.5), exactly 30 mm past
the shoulder (0.5) and 29 mm (0), the start pose and a harder-jammed pose (0), backed off and square but not fed in
(0), pushed out of the near end (0), out but moving, held 5 mm up, tipped on its end or tilted 8 degrees (0), out
but 7 cm to the side (0), and the route around the guides: backed out of the near end or pushed sideways past a
guide's outer face, then brought back in line past the far end (0 for the rest of the trial, until a scene reset).
