# Put a dishwasher rack's dropped wheels back on their rail, then roll it home

A bench-scale dishwasher with no top stands open on the table, its door folded down toward the robot. The lower rack
has been shoved against one side wall, so the other side's two wheels have dropped off the inner edge of their rail
onto the tub floor and the rack leans. A heater bar lies across the floor near the back. The robot is asked to roll
the rack back in; the instruction does not say that a side is down. Pushed in as it is, the low side runs into the
heater. The right way is to see which side is down, lift that side back onto its rail, then roll the rack home.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with joints: the rack's wheels turn on
free hinges, with stops at 40 rad either way, far more than a push needs) and `lots.py` (layout lots), both in this folder; they import only NumPy and `yam_arena`.
`near_misses.py` lists near-successes and wrong outcomes with the score each must get; they are plain-Python checks
of `score()` and `success()` and need no simulator.

## Lots

Lot B (the first trial, so the screenshot and both videos show it): the rack, centre 25 cm out, shoved against the
left wall; its right wheels are down. Lot A: centre 28 cm out, shoved against the right wall; its left wheels are
down. In both, the rack is declared level and centred on the rails, then moved sideways by the lot until its outer
wheels are 0.5 mm from the wall, and the far side falls onto the floor while the scene settles. Trial k uses lot k
mod 2 in the order B, A, so a run needs `--epochs 2` (or a multiple); `TASK_LOT=A` or `TASK_LOT=B` pins every trial
to one lot. The lots stop a fixed habit (always lift the left side), not a memorised replay of both layouts.

## Retaining lips

A steel lip 5 mm thick runs along the top of each side wall, reaching 15 mm in over the rail, its underside 3 mm
above the top of a wheel standing on the rail. A wheel shoved against its wall sits under the lip (its rim spans 14
to 15 cm from the midline, the lip 13.5 to 15 cm), so the side still on its rail cannot be lifted more than 3 mm.
That side can only come out from under the lip by moving the rack toward the low side, which the low side's wheels,
against the rail's inner face, prevent until that side is lifted. So lifting the whole rack straight up, or lifting
the side that is still on its rail, does not work; the low side has to be found and lifted first. A centred wheel
(rim 12 to 13 cm) is clear of the lip.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Roll the lower rack back into the dishwasher."

# Dream-1, both lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/rack-dream

# Scripted demo, both lots (each reaches 1.0 in about 19 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 2 --store-frames --log-dir ~/yam-eval/rack-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is two trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold 1 s.
- **Wheel on its rail:** each wheel's centre is computed from the rack's pose (the wheel's axle point in the rack's
  frame). It must lie over its own rail's top (11.5 to 15 cm from the midline on that side, between the tub's
  front at 14 cm and back wall at 47 cm), and its lowest point (centre minus 15 mm) must be between 1 mm below and
  3 mm above the rail top.
- **Level:** the rack's vertical axis within 2 degrees of vertical. **Home:** the middle of the back bar's back face
  at most 15 mm in front of the back wall's inner face (the rack's centre at least 36.5 cm out).
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the rack, wheels included; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Heater record (whole trial; the scoring text says "touches"):** set, and the score is 0 for the rest of the trial, if any of the rack's shapes
  (frame, tines, axles, wheels as their boxes) comes within 2 mm of the heater bar (an oriented-box separating-axis
  test). It is cleared only by a scene reset (`parts.on_reset`; offline, the rack back within 0.5 mm of its declared
  start and at rest after a jump of over 20 mm in one control step). A robot cannot clear it.

## Scripted demo

`demo()` drives the arm on the side that is down (left in lot A, right in lot B). It grips that side's round bar
from above at the rack's middle (pads opened 24 mm, closed 3 mm past each face), lifts it 42 mm so the dropped
wheels clear the rail by about 15 mm, moves it 2 cm outward so the wheels are over the rail, lowers it until the
wheels are 6 mm above the rail and lets go. It then closes its fingers, sets them against the middle of the front
bar, pushes the rack 11 to 14 cm until it stops at the back wall, backs off and rises. Which side is down and where
the rack stands come from the trial's lot, which the cameras show (the low side, the tilt and the rack's offset
toward one wall); the demo reads nothing from the simulator.

## Measured in simulation

- Demo with the retaining lips, both lots (trace-20261109T173220): 1.0 in lot B and lot A, as before the lips.
- Demo, both lots, before the lips were added (trace-20261109T163601): after the lots' settle the rack leaned 6.9 degrees with its low side's
  wheels on the floor. In lot A the lift and set-down put all four wheels on the rails (score 0.5 at about 9 s), the
  push brought the rack's centre to 37.9 cm and success held from about 18 s; lot B the same, success from about
  18 s of its trial. The wheels turned about 7 rad during the push.
- Shortcuts traced: pushing the rack straight in at the leaning front bar's middle stopped it with its centre at
  30.3 cm, against the heater, in both lots (score 0; trace-20261109T164742 of a habit copy of the task). Replaying
  lot A's lift of the left side in lot B (where the right side is down) left the right wheels on the floor (score 0;
  trace-20261109T164148). A two-arm lift of the front bar near both ends (trace-20261109T173540) did not get a
  grip on the leaning bar, so it does not test the lips; it scored 0 in both lots.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm; lifting the low side first and then lifting the whole rack with one or both
  arms and setting it on the rails anywhere along them, then pushing it home; pushing on any part of the rack.
- **Score 0:**
  - pushing the rack in with a side still down: the low side runs into the heater and the trial scores 0 even if the
    rack is then re-seated and pushed home;
  - lifting the side that is on its rail (it is held under its lip), or lifting the whole rack straight up;
  - pushing the rack sideways: the dropped wheels meet the 3 cm side of the rail and cannot climb it;
  - lifting the rack out and setting it on the door or the table, holding it on the rails, or a rack still rolling.
- **0.5:** all four wheels on their rails, level, let go and still, but not pushed home.

The cases in `near_misses.py` cover the 15 mm home limit on both sides, the rails' inner edges (a wheel centre at the
edge, and 0.5 mm past it, on each side), 3 mm and 4 mm above the rails, a rolling rack, a 3-degree yaw, a pitched
rack, the dropped start, a low side pushed to 3 mm short of the heater (no fault) and into it (a fault that a later
re-seat does not clear), the rack on the door and beside the dishwasher, and scene resets.

## Recorded runs

- Scripted demo, `--epochs 2` (demo-20261109T174207): 1.0 in lot B and in lot A (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 2` (dream-20261109T174629): 0.0 in both lots.
- The screenshot and both videos are the first trial's: lot B.
