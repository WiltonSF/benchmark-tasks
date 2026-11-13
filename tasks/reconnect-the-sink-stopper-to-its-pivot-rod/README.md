# Reconnect a bathroom-sink pop-up stopper to its pivot rod

A bathroom-sink pop-up drain body stands upright in a bench stand with its pivot port pointing at the robot. The
pop-up stopper sits closed in it. At the bottom of the stopper's stem is an eye, a ring with a 9 mm hole, and the pivot
rod has to pass through that eye. The eye is inside the drain, 26 mm below the port and turned a quarter turn away from
it, hidden by the drain body. The rod lies on two rests on the bench. The robot is asked to reconnect the stopper
to the rod.

When the stopper rests closed, no rod pushed through the port can reach the eye. The stopper has to be lifted about
26 mm by its head and turned a quarter turn, and held there while the rod is pushed in through the port and the eye
until its pivot ball reaches the seat washer. That takes both arms. A rod pushed straight in hits the stem. A rod pushed
in under a stopper lifted too far seats its ball and looks finished, but it has not caught the stopper.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts), which sits in this folder and imports
only NumPy and `yam_arena`. At import it patches `yam_arena`'s scene spawning, step and reset in the task's own
process only, and changes nothing on disk. `near_misses.py` holds near-successes and wrong outcomes, each with the
score it must get. They are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Reconnect the drain stopper to its pivot rod."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stopper-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/stopper-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, and `task.json` describes it in words.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. The layout is fixed. Run with `-E object_jitter=0`.

## Geometry

The holes are round. The kit has only solid boxes and cylinders, so each round hole is a ring of overlapping
cylinders ("lobes") whose inner faces form the bore: 14 lobes for the eye and the seat washer, 16 for the spout. The
drain tube is 24 flat staves with a 32 mm bore. The opening through its wall facing the port is 12 mm wide and 12 mm
tall. The pivot ball is a round barrel, 14 mm across and 10 mm long. In the 16 mm spout it lets the rod tilt or swing
about 14 degrees. The stopper's guide fins are 30.5 mm across in the 32 mm bore. They keep the stopper upright and
centred while letting it turn.

## Scoring details the simulator applies

- **Through the eye:** the rod's centreline is intersected with the plane through the middle of the eye's ring, in
  the stopper's own frame. It must cross that plane less than 4.5 mm from the hole's centre (the hole's radius), and
  run within 35 degrees of the hole's direction. The rod's inner end must be at least 2 mm past the ring's far face,
  and the ball must be on the port side of its near face. A rod beside, under or over the ring fails this, wherever
  its ball is.
- **Ball in the port:** the ball's centre is within 2 mm of the port's axis. Its nearest point, taking the barrel's
  tilt into account, is at most 3 mm from the seat washer's outer face.
- **Stopper in the drain:** the head's centre is within 4 mm of the drain's axis, the stopper is tilted at most
  6 degrees, and the head's underside is between 1 mm below and 40 mm above the flange top. At 40 mm, 10 mm of the
  guide fins are still in the bore.
- **Let go:** a `parts.on_step` hook reads both arms' finger and gripper link poses every step. It measures the
  shortest distance from their collision hulls (`yam_arena.analysis`) to every shape of the rod and of the stopper.
  Each distance must be at least 5 mm. Before the first measurement after a reset, nothing is credited, so the check
  fails closed. Offline, `near_misses.py` sets a flag that counts its cases as let go, and it checks the rule
  separately with a gripper 4.9 mm and 5.0 mm away and with no measurement.
- **Still, for 1 s:** both parts move under 1 cm/s and turn under 0.5 rad/s. The `on_step` hook counts consecutive
  control steps on which the rod is through the eye, the stopper is in its guide and both parts are let go and
  still; 1.0 and 0.5 need 20 of them (1 s at 20 Hz), and before the first count nothing is credited. For the rod, spin
  about its own centreline is left out. In the simulator the round rod can keep rolling slowly in place in the
  port, at about 1 rad/s, which moves nothing; a swing of the rod is still counted.
- **Score:** 1.0 when all of the above hold. 0.5 when everything holds except that the ball is more than 3 mm from
  the washer, which is reachable while the rod is still through the eye (ball up to 9.5 mm out). 0 otherwise.

## Scripted demo

The demo runs open loop from the published start layout and reads nothing from the simulator. It uses only the stock
straight-down grips.

- **Left arm:** opens to 54 mm, pinches the stopper's head just above the flange, lifts it 26 mm and turns it a
  quarter turn. It holds the stopper there while the rod goes in, then opens, slides off sideways with its pads clear
  of the head, and moves away.
- **Right arm:** grips the rod from above 100 mm out from the ball, which is close to the rod's balance point, so the
  rod does not droop in the pads. It lifts the rod to the port's height, lines it up with the port 6 mm short of the
  spout, and pushes it straight in to 2 mm past the seat. Then it opens, rises straight up and moves away.
- **Tuning:** the waypoint heights carry offsets measured in traces for how far each arm sags under its load: 2.5 mm
  for the left arm, 2.3 mm up and 0.8 mm sideways for the right.

## Measured in simulation

Traces on the GPU ran with the final geometry and demo.

- The held stopper's eye sat within 0.3 mm of the port's height, and the rod's inner end entered it 1.3 mm off centre
  and went home.
- After both arms let go, the rod's long outer end dropped. Its inner end then held the stopper about 28 mm open,
  which is what a pop-up rod with no pivot nut does. The rod settled tilted about 9 degrees and swung 6 degrees
  sideways, with the ball 2 mm from the washer and the stopper tilted 3.6 degrees and 2.6 mm off centre. Everything
  was still from about 2 s after the release.
- An earlier layout, with the port pointing to the robot's right, made the right arm's forearm hit the left gripper
  during the push. That is why the port points at the robot.

## Other ways to succeed, and what scores 0

- **Also full score:**
  - turning the stopper either way, since the eye's ring is symmetric;
  - either arm doing either job;
  - inserting the rod at a tilt, if the eye is lined up with it;
  - lifting and turning the stopper by another grip.
- **Score 0:**
  - a rod pushed in while the stopper rests closed (it meets the stem);
  - a rod pushed under or beside the eye, even with its ball seated;
  - a rod short of the eye;
  - a stopper not turned, or turned only 60 degrees;
  - a stopper lifted more than 40 mm, out of its guide, or tilted more than 6 degrees;
  - a part still held or still moving.

`near_misses.py` covers these limits:

- the 3.0 mm and 3.5 mm seat limits;
- the 2 mm past-the-ring limit;
- a rod 4.4 mm and 4.6 mm below the eye's centre;
- the rod under the eye, beside it, and with the stopper unturned;
- the stopper turned 30 and 60 degrees;
- 3.9 and 4.1 mm off centre, 5.5 and 6.5 degrees of tilt, and lifted 41 mm;
- the tilted resting pose with the rod's tip up or down;
- a rod rolling about its own axis (still) and one swinging (not still);
- the release and fail-closed cases, and a hold of 19 steps (no credit) against 20 (credited).

## Recorded runs

- Scripted demo: 1.0 (success held 1 s after both arms let go, about 18 s into the trial).
- Dream-1, 1,200 steps: 0.0.
