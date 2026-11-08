# Take a drawer right out of its chest past the anti-pull-out stop

A small chest of drawers stands on the table with its front facing away from the robot. Its drawer has a stop tab on
its back that meets the rail above the opening when the drawer is nearly out. Pulling harder, or lifting the drawer
level, does not get it past. Lifting the front tilts the drawer on its back edge, the tab drops under the rail, and
the drawer comes free. The robot has to take it out and set it down on the table.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. `parts.py` (in this folder) builds the drawer as one loose rigid part from
several boxes and runs the release check. `tilt.py` (in this folder) lets the scripted demo lean the gripper; it
affects only the demo, never Dream-1 or scoring. `near_misses.py` scores near-successes and wrong outcomes without
a simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the drawer out of the chest and set it down on the table."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drawer-dream

# Scripted demo (reaches 1.0 in about 20 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/drawer-demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about a minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none. Run with `-E object_jitter=0`: the drawer starts on its runners with 5 mm of side play, and
  jitter would push it into the chest.

## Scoring in the simulator

`score()` and `success()` in `task.py` (pass/fail). "Out" is the drawer's world-aligned bounding box not
overlapping the chest's footprint seen from above (x 0.078 to 0.25, y -0.217 to -0.023), so it may be set down
anywhere on the table clear of the chest. "Upright" is its bottom within 5 degrees of level; "on the table" is its
lowest point within 1 mm of the table; "still" is under 1 cm/s and 0.5 rad/s (the stock `resting()` test). "Let go" is measured every control
step by a callback that takes the convex hulls of both arms' finger and gripper links and their distance to every
box of the drawer; it must be 5 mm or more. Success must hold for 1 s.

## What the cameras see

The chest faces away from the robot so that the drawer pulls out toward the middle of the table, within the right
arm's reach. The setup screenshot (top camera, from the robot's side) therefore shows the chest from behind; the
drawer's front, handle, red stop tab and the front rail face away. `tab_frame.png` (in this folder) is a top-camera
frame from the demo, 14 s in: the drawer is leaned front-up and the red tab, at the drawer's back, is passing under
the front rail.

## Scripted demo

`demo()` uses fixed waypoints only. The right arm pinches the bar handle across its 12 mm thickness (fingers turned
90 degrees so they close along x; the lean is about y, across the pads, so the bar cannot turn in them). It pulls
the drawer out until the tab meets the rail (commanded 6 mm past, as the arm is soft), leans it front-up 20 degrees
about its back edge (with `tilt.py`, fingertips leaning away from the arm's base), draws it out 12 mm with the tab
under the rail, leans on to 30 degrees and draws it clear, levels it in front of the chest, lowers it to 7 mm above
the table and lets go.

Geometry that makes the window: the tab stands 16 mm above the drawer and the rail's underside 12 mm above it, so
the tab passes under only with a lean of about 18 degrees or more; leaning over about 22 degrees while the tab is
still behind the rail brings the drawer's sides up against the rail's front edge. The demo's 20 degrees sits in
that window.

## What was traced on the GPU (L40S, trace mode)

- A hard straight pull 40 mm past the stop, held, then a level lift of 15 mm and another 40 mm pull, held: the
  drawer stayed in the chest, tab against the rail (0).
- Earlier geometry (5 or 8 mm gap over the drawer, 15 mm rail): the lean stalled at 9 to 14 degrees against the
  rail's front edge and the bar slipped out of the fingers (0); the final geometry was set from those traces.
- The final demo, twice: 1.0, the drawer set down 16 mm clear of the chest.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

Cases: out with its back exactly at the chest's front (1.0) and 1 mm inside (0), shut (0), pulled to the stop (0),
pulled to the stop and lifted level (0), out and turned 30 degrees about the vertical (1.0), out but held 5 mm up,
moving, tipped 8 degrees, on its side or upside down (0), set down beside the chest with its side exactly at the
chest's side (1.0) or 1 mm over the chest's footprint (0).
