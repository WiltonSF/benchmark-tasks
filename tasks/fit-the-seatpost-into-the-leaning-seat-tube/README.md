# Fit a seatpost into a seat tube that leans back 17 degrees

A bike-frame stand, side-on to the robot, holds a seat tube that leans 17 degrees (a 73-degree seat angle), its open
top toward the robot's left, so the lean shows from the top camera. A seatpost stands upright in a socket beside it. The post has to go into the tube until its minimum-insertion
band is inside the tube, and be let go there. The bore leaves 2 mm per side, so the post can only go in along the
tube's own axis: held upright and pushed straight down, it catches on the mouth and does not enter (GPU trace below).
The robot has to see the tube's lean and turn the post to match it before pushing. The instruction states only the
goal.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. It uses `parts.py` (in this folder) for the frame,
the stand and the post, and the scripted demo uses `tilt.py` (in this folder) for its tilted grip. `tilt.py` is
imported inside `demo()` only, so it affects only the scripted demo, never Dream-1 or the scoring. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; it runs without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the seatpost into the bike frame seat tube, past its minimum-insertion line, and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames

# Scripted demo (reaches 1.0 after about 15 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 500 --store-frames
```

The run prints `task_score`: 1.0 on success, otherwise 0.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), a few minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it. The layout is fixed, so a policy that memorised this one motion would pass; the lean direction cannot be
  varied per trial with layout lots, because the frame is a fixed part and lots move only loose objects.

## Scoring details

`success()` (pass/fail; it must hold for 1 s) reads the post's pose and the tube's pose as the simulator has them:

- **Inserted:** the post's bottom end is at least 68 mm below the centre of the tube's mouth, measured along the
  tube's axis (the band's top edge at or below the mouth), and within 3.8 mm of the tube's axis sideways (inside
  the bore).
- **Lined up:** the post's axis within 3 degrees of the tube's.
- **Released:** no finger or gripper collision hull within 3 cm of the post, measured in the simulator after every
  control step against points on the post's surface. Offline it is taken as true.
- **Still:** under 1 cm/s and 0.5 rad/s.
- **Tube in place, over the whole trial:** the tube's hinge within 2 degrees of its 17-degree stop at every control
  step. The lowest angle reached is recorded after every step and reset only when the episode resets.

## How the frame is built

The parts kit cannot turn a fixed shape, so the seat tube is a hinged link on the fixed frame (1.5 kg, no spring). It
is written upright; its own weight and a lug on the side it leans to tip it onto its upper joint stop at 17 degrees
during the embodiment's 0.5 s settle at reset, so it already leans in the first camera frame (the setup screenshot).
Nothing else holds it, so a hard sideways shove could lift it off the stop; scoring fails any trial in which the tube
leaves its stop by more than 2 degrees at any point.

## Demo

Fixed waypoints, no feedback from object poses. The left arm grips the post just under the saddle with a
straight-down grip turned a quarter turn (the fingers close along x) and lifts it out of its socket. It leans the grip
17 degrees with `tilt.Waypoint`: the fingertips, and the post's lower end, swing toward the robot's right, so the post
turns about the grip axis and matches the tube. It lines the post up just above the mouth, pushes it in along the
tube's axis until the band is 15 mm inside, lets go, and backs out along the axis. The post slides down to the stop.
The aim is 3.5 mm further from the robot and 5 mm high, because in the first traces the arm sagged about 4 mm and the
post sat about 3.5 mm off centre in the fingers, so its end caught on the mouth's rim.

`POST_DEMO=vertical` plays the shortcut for checking on the GPU: the post gripped lower down (upright, the arm cannot
reach high enough holding it under the saddle), held upright 35 mm over the mouth, lowered straight down and pushed
30 mm past the mouth, then let go. `POST_DEBUG=<file>` logs the insertion depth, offset, angle and tube angle every
step. Both are for debugging only.

## Checked

- Offline: `near_misses.py` (16 cases: the band's top edge at the mouth and 1 mm short, the band's centre at the mouth, 3 and 4 degrees off the axis, a post
  jammed upright at the mouth and at the deepest an upright post could go, a moving post, a post beside the tube,
  the tube pushed off its stop, the post lying down or upside down in its socket) and the arm's reach and clearance
  along the demo (the tube's moving link checked separately: the fingers stay at least 25 mm clear).
- GPU (yam-isaac-lab-arena on an L40S): the demo scores 1.0; the post went in lined up within about 1 degree, slid
  to the stop (90 mm in), and the tube stayed at 17.0 degrees from the first frame. The upright shortcut
  (`POST_DEMO=vertical`) scores 0: the post caught on the mouth's rim and never entered, then fell over onto the table
  when let go; the tube did not move. Dream-1 (1,200 steps) scores 0.

## Licence

All geometry is built from primitives in `task.py`; no third-party assets. Released under the repository's licence.
