# Stow the clamp lever on an articulated light arm without moving the light

An articulated bench light arm holds a 150 g LED panel light. One clamp screw at the elbow locks the arm's joints,
and its red clamp lever is indexable: a square socket on the lever sits over a square drive on the screw, pressed
down by a spring. Seated, the lever turns the screw; lifted more than 7 mm it turns freely on the screw's round
neck. The robot must leave the lever pointing back along the arm toward its base. Turning the lever the loosening
way (counter-clockwise from above) frees the end joint once the screw is about 10 degrees loose, and the light drops.

Instruction: "Leave the clamp lever on the light arm pointing back along the arm toward its base, without moving
the light."

## Run

On the program's Brev launchable (yam-isaac-lab-arena, the program's YAM embodiment for Isaac Lab-Arena), from
this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \
      --instruction "Leave the clamp lever on the light arm pointing back along the arm toward its base, without moving the light." \
      --policy yam_scripted --scorer task_score --max-steps 900 --epochs 2 --store-frames

Use `--policy dream` for Dream-1. `--epochs 2` is required: trial k uses lot k mod 2 (lot A first), and the score is
the mean. `TASK_LOT=A` or `TASK_LOT=B` in the environment pins every trial to one lot. Runs use
`object_jitter=0`. A trial is at most 900 control steps (45 s at 20 Hz); about 1 to 2 minutes of wall time each.

Files: `task.py` (scene, scoring, demo), `parts.py` (multi-shape parts and joints), `lots.py` (per-trial layouts),
`near_misses.py` (offline scoring cases). `parts.py` and `lots.py` patch the yam_arena embodiment class in the
task's own process at import; nothing is changed on disk. No third-party assets are used.

## Lots

- Lot A (screenshot): the handle points straight away from the robot. Stowing needs a quarter turn
  counter-clockwise, the loosening way.
- Lot B: the handle points right, along the segment toward the light. Stowing needs a half turn counter-clockwise
  (or a quarter turn clockwise, which a seated lever cannot make because the screw is at its tight stop).

A replay of lot A's motion in lot B leaves the lever pointing away from the robot (0 in lot B). The demo with its
lift removed (the lever turned while seated) was run on the GPU on an earlier build with the same lever and screw
(the elbow 3 cm farther out, a stiffer spring, the end joint released at a fixed 10 degrees of screw turn): in lot A the screw turned 75 degrees and the light fell to about
49 degrees, and both lots scored 0.

## Scoring (exact)

Pass/fail, per trial; the run's score is the mean over trials.

- **Seated:** the lever's underside within 1.5 mm of the flange top (83 mm above the table) and its tilt at most
  5 degrees. A lever resting on top of the drive (7 mm up) or held up is not seated.
- **Stowed:** the handle's direction within 15 degrees of pointing back along the arm toward the base. The square
  drive leaves about 11 degrees of play.
- **Released:** in the simulator an `on_step` callback measures, every control step, the distance from every
  finger and gripper hull of both arms to every shape of the lever and the light; all must be over 5 mm. Until it
  has measured, nothing is credited (fail closed).
- **Still:** the lever under 1 cm/s and 0.5 rad/s.
- **Held:** the four conditions above hold continuously for 1 s (timed by the same callback; the simulator also
  ends the trial once `success()` has held for 1 s).
- **Light kept, over the whole trial:** the light's end joint never turned more than 2 degrees from its start, and
  the light's centre never moved more than 3 mm. The record is cleared only by the simulator's scene reset (an
  `on_reset` callback), never by anything the robot does, so a light that drooped and was lifted back still fails.

Offline (`near_misses.py`, no simulator) each case is taken as already held 1 s with the grippers away.

## Mechanism as modelled

- Screw: a hinge about the vertical axis, lower limit 0 (its tight stop), friction modelled as a drive capped at
  0.2 N m whose rest point follows the screw once it has been turned more than 1 degree, so it stays where turned.
- End joint: a hinge about a front-to-back axis held by a stiff drive (30 N m/rad, capped at 0.4 N m). Its
  friction is modelled every control step: it can hold 0.4 N m with the screw tight, falling linearly to zero at
  12 degrees loose. While that exceeds the light's weight torque (0.15 kg, centre 50 mm out and 45 mm up from the
  axis, about 0.07 N m level) the drive holds the light where it is; once the weight wins (about 10 degrees loose)
  the drive's rest point is moved to 0.85 rad and the light drops to the joint's stop. Tightened again, it holds
  wherever the light then is. This is a slip/hold switch on a friction limit, not a continuous friction model.
- Lever spring: a downward force on the lever along the screw axis of 1.0 N plus 20 N/m times its lift, set every
  control step.

## Demo

Scripted. At the first control step an `on_step` callback reads the lever's handle direction from its pose (the
direction the top camera shows), rounds it to the nearest quarter turn and writes the needed quarter turns into the
plan's waypoints; nothing else is read, and the plan then runs open loop. Both arms first wait aside for 1.5 s. For each quarter turn one arm pinches the handle 40 mm from the axis, lifts it about 10 mm against the
spring, swings it a quarter turn counter-clockwise in six steps, lowers it until the spring seats it and lets go.
The right arm makes turns whose wrist starts at 0 degrees and the left arm turns that start at -90 degrees, so each
wrist turns within its reach; the other arm waits aside. The commanded lift is 20 mm for the right arm and 30 mm for
the left, which sags more where it reaches across. Apart from that one look at the lever's direction, the demo reads no
simulator state.

## Seeds

Scenes are deterministic with `object_jitter=0`; no other randomness.
