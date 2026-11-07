# Set up a two-section folding ramp from the floor to the top of a step

A loose aluminium ramp lies folded in front of a fixed 35 mm step. It is two 100 mm panels with side rails, hinged
across the rail tops, the second panel lying face down on the first. The robot is asked to set the ramp up onto the step
for a wheelchair. Laid folded it is half length and too steep; it has to be opened flat
first (the hinge then catches), and then laid with one end on the floor and its underside on the step's edge. The
instruction does not say to unfold it.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (in this folder) for the hinged ramp, and `tilt.py` (in
this folder) for the scripted demo only. Both import only NumPy and `yam_arena`. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Set up the folding ramp onto the step for a wheelchair."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ramp-dream

# Scripted demo (reaches 1.0 in about 29 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/ramp-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`: every object starts exactly where `task.py` puts it. Jitter
  has not been tested with this task.

## Scoring details the simulator applies

- "At rest" is `ObjectState.resting()` on the ramp's base panel: under 1 cm/s and 0.5 rad/s.
- **Unfolded:** the hinge angle is at least 172 deg (flat is 180 deg; the hinge cannot open past flat), and the base
  panel's up axis is within 30 deg of vertical (rails up).
- **Ends:** the middle of each free end's bottom edge, from the base panel's pose and the hinge link's pose.
- **Set at the step (1.0):** the lower end is 2 mm below to 1.5 mm above the table top and in front of the step's
  front face (x 0.37 m); the higher end is at least 15 mm past that face; the straight line between the two ends'
  bottom edges passes within 2 mm of the step's top at the front face; the line's heading is within 10 deg of the
  x axis; and the four corners of the ramp's 70 mm width at the two ends lie within the step's 200 mm width.
- **0.5:** unfolded, released, at rest, not set at the step, and lying unaided: both ends' bottom edges within
  1.5 mm of the table top (or of the step's top, over the step); or one end on the floor and the underside within
  2 mm of the step's top at its front face; or level (ends within 3 mm of each other in height) on the step's top
  with its middle over the step. Held off every surface, one end held up, or knocked off the table (lowest point more
  than 5 mm below the table top) scores 0.
- **Released:** a `parts.on_step` callback reads both arms' joint angles each control step and computes each
  gripper's tip point (between the finger tips, from the arm's own kinematics) and half opening. The ramp counts as
  released when every tip point is more than 5 mm above the ramp's top, or farther from its bounding box sideways
  than its half opening plus 10 mm (a finger's thickness) plus 5 mm. Offline (near misses) there are no grippers.
- The trial ends with `success` once the 1.0 condition has held for 1 s; otherwise `task_score` is `score()` at the
  last step.

## The hinge catch

`task.py` registers a `parts.on_step` callback. Until the hinge has been opened to within 2 deg of flat, it sets the
hinge drive's target a little ahead of the hinge's own angle each step, so the panel turns freely (light damping,
0.04 N m s/rad). Once the hinge reaches 178 deg, it sets the target to flat for the rest of the trial; the
drive's 4 N m/rad spring then pulls it back to flat whenever it bends (it is a spring, not a rigid lock), so the ramp stays within 2 deg of straight when it spans the step (it would otherwise sag shut at
the hinge, which sits at the rail tops). A scene reset releases the catch (`parts.on_reset`).

## The scripted demo

`demo()` uses `tilt.py`, a small runtime patch that lets a scripted waypoint lean the gripper off straight down. It
affects only the scripted demo, never Dream-1 or scoring. The left arm grips the folded panel's rails 20 mm from its
free end and swings it up about the hinge to 60 deg, leaning the grip with it. The right arm's closed fingers come
against the panel's free end; the left arm lets go and moves clear; the right arm pushes the panel past upright to
112 deg and lifts away, and the panel settles flat, where the catch holds it. The left arm then grips the ramp
across its rails at the hinge, lifts it, carries it so its far end is 35 mm past the step's front edge, and lets go
3 mm above the step, so the near end drops to the floor. Traced on the GPU: the ramp rests at about 12 deg, its low
end on the table, and success holds after the fingers open.

## Licences

No third-party assets: every object is built from boxes and cylinders in `task.py`.
