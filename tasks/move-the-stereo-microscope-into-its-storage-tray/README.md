# Move a stereo microscope into its storage tray

A student stereo microscope stands on the table, side-on to the robot, next to an empty storage tray. Its two
eyepieces are separate parts that sit loosely in their tubes on top of the head, held there only by gravity, as on a
real stereo microscope. They are the highest and most graspable parts. The robot is asked to put the microscope in
the tray; the instruction does not say how to hold it. Lifting by an eyepiece pulls it out of its tube.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in this
folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with the
score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## True size

The microscope is sized and weighted like light plastic school stereo microscopes (retail listings give about
135 x 100 x 220 mm and about 0.6 kg). Here it is 135 x 100 x 207 mm and 0.64 kg: a 0.56 kg body (metal pillar, plastic
base, focus block and head) and two 40 g plastic eyepieces. The eyepiece barrels are 23.2 mm across, the usual
eyepiece size.

## Lots

Lot A (the screenshot): the microscope 23 cm left of the midline, pillar toward the robot's left. Lot B mirrors it
23 cm to the right, pillar toward the robot's right. The tray is on the midline in both. Trial k uses lot k mod 2, lot
A first, so a run needs `--epochs 2` (or a multiple); `TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot. A
replay of one lot's motions grasps empty air in the other and averages at most 0.5. The lots stop a fixed position
habit, not a memorised replay of both layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the microscope in the storage tray."

# Dream-1, both lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/scope-dream

# Scripted demo, both lots (each reaches 1.0 in about 13 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 2 --store-frames --log-dir ~/yam-eval/scope-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started; a scored run is two trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Geometry (microscope frame: origin under the base's centre, +x toward its front, z up)

| Part | Shape and size | Position |
|---|---|---|
| Base | box 135 (x) x 100 (y) x 25 mm | centred on the origin |
| Stage plate | disc 60 mm across, 2 mm | x +20 mm, on the base |
| Pillar | round, 25 mm across | x -48 mm, 25 to 190 mm high |
| Focus block | box 38 x 48 mm | x -66 to -28 mm, 100 to 160 mm high |
| Focus knobs | discs 32 mm across, 12 mm thick, axis along y | x -45 mm, y +-30 mm, 130 mm high |
| Head | box 78 x 90 mm | x -28 to +50 mm, 115 to 160 mm high |
| Objective | round, 32 mm across | x +20 mm, 90 to 115 mm high |
| Eyepiece tubes | bore 26.2 mm across, walls 3.5 mm | axes at x +20 mm, y +-30 mm, 160 to 177 mm high |
| Eyepieces (loose, 40 g each) | barrel 23.2 x 16 mm; eyecup 30 mm across, 30 mm tall | barrel 161 to 177 mm, eyecup 177 to 207 mm |

Each tube's bore is built from four flat walls and four corner blocks, so it meets the round barrel at eight points on
a 26.2 mm circle (1.5 mm play each side). The eyecup's underside rests on the tube's rim. The storage tray is fixed:
inside 150 x 175 mm, floor 4 mm, rim 8 mm above the floor (12 mm above the table), walls 8 mm. Friction is 1.2 static
and 1.0 dynamic on every part (parts.py default). Masses are spread over each body's shapes by volume.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`), for the microscope and
  both eyepieces.
- **Eyepiece position:** each eyepiece's frame origin (the bottom of its barrel's axis) is taken into the microscope's
  frame. "Rise" is its height above the seated height (161 mm); "sideways" is its distance from the tube's axis.
  Seated: rise at most 1 mm and sideways at most 3 mm.
- **In the tray:** all four bottom corners of the base inside the tray's inner outline, the lowest between 2 mm
  below and 1 mm above the floor (contact depth), and the microscope's tilt at most 3 degrees.
- **0.5 tier:** the whole-trial record also notes whether the base's lowest corner was ever more than 15 mm above the
  table (above the 12 mm rim, so pushing it onto the rim does not count) while both eyepieces were seated and no fault had been set. 0.5 then needs, at the end: no fault, let go, the
  base's lowest corner between 2 mm below and 1 mm above the table top, tilt at most 3 degrees, microscope and
  eyepieces still and seated. Pushing it along the table never earns it, and a fault zeroes it.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the microscope and both
  eyepieces; it must exceed 5 mm. The same callback times how long the success state has held; `score()` gives 1.0
  only after 1 s. It fails closed: until the grippers have been measured, nothing is credited. Offline
  (`near_misses.py` sets an environment flag) each case stands for a state held 1 s with the grippers clear. The flag
  is ignored once any simulator callback has run in the process, so it can never relax live scoring.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step an eyepiece's rise
  is over 5 mm or its sideways offset over 3 mm, the microscope tilts over 10 degrees, or it moves down faster than
  0.6 m/s. It is cleared only by a scene reset (the `on_reset` hook at each trial's reset; offline, the microscope
  back within 0.5 mm of its start and at rest after a jump of over 20 mm in one control step, which a robot cannot do).

## Scripted demo

`demo()` is not told the lot. At the trial's first control step a `parts.on_step` callback reads where the
microscope stands (a coarse, camera-visible pose) and picks the nearest of the two published start spots (lot A or B
in the setup text, with their headings). The plan's waypoints are worked out from that spot when the plan reaches
them; for the first control step, before the reading, the screenshot's spot is assumed. The arm on the microscope's
side does the job and the other arm moves to a waiting point on its own side. The working arm comes straight down beside the pillar, above the focus block, with the fingers open 42 mm and a
finger on the pillar's front and back (tips 166 mm up). It pinches the pillar firmly, lifts it (commanded 30 mm; the
loaded arm sags, and the base clears the table by about 22 mm), carries it sideways over the tray, lowers it to 7 mm
over the floor (commanded), opens, rises along the pillar and backs off. The carry aims 12 mm past the tray's centre
because the loaded arm stops short of a sideways move's end. It uses no other simulator state (no lot id, masses,
friction, joint angles or pose feedback).

## Measured in simulation

- **Demo, both lots:** the microscope stayed within 3 degrees of upright while carried, both eyepieces stayed within
  0.2 mm of seated, and it came to rest in the tray within 4 mm of the tray's centre. Score 1.0 in each lot.
- **Counter-test, lifting by an eyepiece (lot A, a temporary demo variant, not in this folder):** gripped 10 mm below
  its top and lifted 40 mm, the eyepiece slid 41 mm up out of its tube and the microscope did
  not move; score 0.
- **Earlier version, low carry:** with the base carried only 15 mm up, it caught on a 16 mm rim and tipped past
  30 degrees; score 0. The rim is now 12 mm and the carry higher.

## Other ways to succeed, and what scores 0

- **0.5:** lifting it correctly (eyepieces in) and setting it down upright on the table instead of in the tray.
- **Also full score:** either arm, any hold that keeps the eyepieces in (pillar, focus block, head, base, or both
  arms), the microscope set in the tray at any heading that fits.
- **Score 0:**
  - lifting or pulling an eyepiece more than 5 mm, even if it is pushed back and the microscope then put in the tray;
  - tipping the microscope past 10 degrees, or dropping it;
  - pushing it along the table without lifting it; leaving it on the rim or partly over it; holding it in the tray or
    just above the table; leaving an eyepiece out or loose in the tray.

The cases in `near_misses.py` cover the 1 mm floor limit, the rim's inside along both axes (on and 1 mm over), the
3 degree and 10 degree tilt limits, the 1 mm and 5 mm eyepiece limits, a 4 mm sideways eyepiece, a rattling eyepiece,
a fall at 0.7 m/s, a microscope turned 90 and 180 degrees in the tray, the eyepieces lifted out and laid in the tray,
faults followed by a correct placement (still 0), the 0.5 tier (lifted 16 mm then set down; exactly 15 mm, pushed onto the rim and back, held 3 mm
up, tilted 4 degrees, an eyepiece 2 mm up, a lift with an eyepiece raised, a fault after the lift: all 0), and scene
resets.

## Recorded runs

- Scripted demo, `--epochs 2`: 1.0 in lot A and in lot B (task_score 1.0), with the demo reading the start spot at
  the first control step.
- Dream-1, 1,200 steps, `--epochs 2`: 0.0 (mean of lots A and B). It ran on the final scene before the 0.5 tier was
  added, and is re-scored under the final scorer, judged from its video (the run logs actions, not object states):
  - Lot A: the left arm prodded the microscope for the whole trial; it stayed on its start spot with no visible lift
    and the gripper was still on it at the end. 0.0 (not let go; no lift seen).
  - Lot B: the right arm pushed the microscope onto the tray, where it ended turned, with a corner of its base over
    the rim. 0.0: not wholly inside the rim, so not 1.0; and a base partly over the 12 mm rim cannot stand on the
    table within 3 degrees of level, so not 0.5.
