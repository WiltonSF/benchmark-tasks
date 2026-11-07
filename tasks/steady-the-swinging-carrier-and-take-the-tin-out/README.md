# Steady a swinging conveyor carrier before lifting a tin out of it

A C-hook carrier, like one on an overhead conveyor, hangs from a high pivot and is swinging toward and away from the
robot when the trial starts. An open tin stands loose in its basket. The robot has to stop the swing, then lift the
tin out and stand it on a green pad on the table. Lifting the tin out while the carrier still swings scores 0, and
so does tipping the tin or dropping it.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. `parts.py` (next to it) builds the hinged carrier. It patches `yam_arena`'s
scene spawning and step and reset at import time, in the task's own process only, and changes nothing on disk.
`near_misses.py` holds near-successes and wrong outcomes with the score each must get. It runs as plain Python
without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="The hanging carrier is swinging. Steady it, then lift the open tin out, stand it on the green pad and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/carrier-dream

# Scripted demo (reaches 1.0 at about 19 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/carrier-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, and `task.json` states it in words.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz). That is about a minute of wall time on
  an L40S once Isaac Sim is running.
- **Seeds:** none. Run with `-E object_jitter=0`. The start is the same every trial: the carrier hangs at rest
  through the embodiment's settle, then a reset hook gives the swinging part 0.90 rad/s, away from the robot, and
  gives the tin the matching velocity, so the tin starts moving with the basket. The setup screenshot is taken
  before that hook runs, so it shows the carrier hanging straight.

## How the scoring works in the simulator

- **Swing:** an `on_step` hook records the carrier's hinge angle every control step. "Steady" means half the
  peak-to-peak angle over the last 1.5 s (about one swing period) is under 2 degrees. Offline, with no simulator
  steps, the hinge angle in the case stands in for it.
- **Whole-trial records:** tipped (more than 20 degrees from the basket's vertical while in the basket, from vertical
  once out of it), dropped (the tin's lowest point within 2 mm of the table top), and lifted out while swinging (the
  tin's centre outside the basket walls or its base above the rim, its lowest point more than 2 mm above the pad's
  top, while the carrier swings 2 degrees or more). Any of these sets the score to 0 for the rest of the trial. They
  are cleared only by the scene reset hook, which the robot cannot trigger.
- **On the pad and let go:** the tin stands within 10 degrees of upright and at rest (under 1 cm/s and 0.5
  rad/s), its lowest point within 2 mm of the pad's top, and its centre within 25 mm of the pad's centre. No
  gripper may be within 5 mm of it: every step an `on_step` hook places the hulls of both arms' gripper and finger
  links at their simulated poses (fingers as they are) and measures their distance to the tin's box. The demo lets
  go 6 mm up, so the tin drops onto the pad. Offline there is no robot, so this check passes.
- A gripper holding the carrier still counts as steady. Holding the carrier with one hand while the other unloads it
  is a fair way to do the job.

## Scripted demo

`demo()` uses fixed waypoints timed from the start and reads no simulator state. The timing comes from an idle GPU
trace: the period is 1.429 s, and the first turning point toward the robot is at 1.07 s. The left arm waits above
the point where the grab tab stops at the near end of its swing, with its fingers part-closed (they close slowly).
At the third turning point (3.93 s) it drops to tab height. The tab slides in between the fingers as it comes back,
and the fingers pinch it as it comes to a stop. The arm then eases the carrier down its arc to hanging over 3.2 s,
holds for 1.5 s, lets go and backs off. The leftover swing is under 0.2 degrees. Last, it grips the tin 30 mm below its top
from above, lifts it straight up until its base is 10 mm above the rim, carries it to the pad and lets go 6 mm up.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases cover:
- the steady boundary on the pad: 1.9 degrees scores 1.0, and 2.0 and 8 degrees score 0.5;
- the pad centre (25 and 26 mm off), upright (10 and 11 degrees), height (2 and 3 mm above) and a tin still moving;
- the tin still in the basket, lifted clear of the rim over a steady carrier, or leaning 19 degrees against the wall,
  all of which score 0 without tripping a record;
- whole-trial cases: the tin plucked out over the rim while the carrier swings 5 degrees (still 0 after it is then
  set on the pad), the tin tipped 25 degrees in the basket, and the tin dropped on the table.

## Habits that must not pass

The layout is fixed, so each ordinary habit was checked against it:
- **Taking the tin straight out** of the swinging basket scores 0 for the whole trial, even if the carrier is
  steadied afterwards (near-miss cases "plucked out ... while the carrier swings 5 deg" and "then set on the pad").
- **Lifting the tin clear and leaving it there,** or **holding the carrier still and stopping,** scores 0. Only the
  tin standing on the pad earns credit.
- **Setting the tin down on the nearest free table space** scores 0 for the whole trial: it touches the table.
- **Doing nothing** scores 0 (see below).

## Strategies checked on the GPU

- **Doing nothing** scores 0. The swing decays only from 11.7 to 10.3 degrees in 45 s.
- **Blocking the swing with the hand** means closing the fingers on the tab where it hangs, so the basket runs into
  the hand at speed. In trace-20261107T112911 (cheat copy) this tipped the tin to 50 degrees against the basket
  wall, and the trial scores 0. The tin is tall and narrow (30 mm across, 90 mm tall) so that it tips over at about
  a third of g sideways. The demo's catch at the turning point peaked at 11.6 degrees, which is the basket's own
  tilt.
- The start is fixed, so a policy that memorises one timing could catch the carrier at the same moment every trial.
