# Fit a brake caliper with new, thicker pads back over its disc

A brake disc stands on edge on its hub, with the two abutments of the caliper bracket beside its top edge. The caliper
lies on the bench with new pads fitted. Its piston is still out where the worn pads left it, so the gap between the
new pads (20 mm) is narrower than the disc (28 mm) and the caliper will not pass over it. Lowered straight on, it
perches on the disc's rim. To score, the robot has to push the piston back into its bore first (about 1 N; it stays
where it is pushed), then lift the caliper over the disc and hang it on the bracket. Scoring is pass/fail. The instruction does not mention the piston.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. The disc, bracket and caliper are built with
`parts.py`, a small parts kit included in this folder (multi-shape parts with hinges and sliders, built as Isaac Lab
articulations; it needs only NumPy and `yam_arena`). `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the caliper over the brake disc so it sits on its bracket, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream

# Scripted demo (reaches 1.0 in about 19 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir logs/demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1-2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as for all submissions; every object then starts exactly
  where `task.py` puts it. The layout is fixed, so a memorised replay of this one layout cannot be ruled out.

## How it is modelled and scored

- **Piston:** a kit `Slider` (16 mm travel toward the robot) carrying the near pad and a bright steel cylinder (18 mm
  across, 6 mm long) behind it, with a spring of 1000 N/m whose rest point an `on_step` hook moves to 0.5 mm behind the furthest point
  reached. Pushed back, the piston stays there; moving it takes about 1 N, less than the caliper's 4.2 N weight. A
  stiffer ratchet (5 N and 10 N) was tried: the arm's push through the pad gap then moved the piston 2 mm and 0 mm.
- **Grippers:** the scorer is given only object states, so an `on_step` hook reads both arms' joint angles each step
  and computes each gripper's pose (from the arm's own kinematics) and opening. Each finger is a box 10 mm thick,
  20 mm wide and 45 mm long back from its tip. The caliper is *released* when every finger point is more than 5 mm
  from every shape of the caliper (body, pads and piston).
- **Fitted:** the caliper's origin (the ears' undersides are 5 mm below it) from 1 mm below to 2 mm above the disc's
  top, within 4 mm of the disc's mid-plane in x and 10 mm of its centre in y, tilted at most 5 degrees and turned at
  most 10 degrees. Its legs straddle the abutments with 2 mm of play, so only a caliper over the disc can sit there.
  Still means under 1 cm/s and 0.5 rad/s. 1.0 once fitted, released and still for 1 s without a break; 0 otherwise.
- **No partial credit.** An earlier 0.5 tier for "piston pushed back, caliper set down" was dropped: pinching the near
  leg with one finger in the pad gap pushes the piston back, so setting the caliper down anywhere would have scored.
- Fixed habits and what defeats them: setting the caliper straight on the disc (perches on the rim: 0); turning it
  round (piston away from the robot: 0); pushing the piston back and setting the caliper down anywhere but the
  bracket (0).
- **Pushing the piston with the disc** (GPU traces at 1 N, piston not pushed first): hanging the far pad past the
  disc's far face with the near pad on the rim, letting go and pushing the caliper away from the robot pushed the
  piston back 6.7 mm and left the caliper jammed on the rim, tilted 32 degrees (0); letting go of it level on the rim
  tipped it and pushed the piston back 4.2 mm (0). Fitting needs 8 mm, so wedging it on did not score in these traces,
  but a harder or repeated wedge might; the fidelity note says so. With a 10 N ratchet both pushed it under 0.5 mm.

## Scripted demo

`demo()` drives the right arm. With its fingers closing along x and open, it lowers the far finger through the open
top into the pad gap beside the piston's pad (the near finger stays outside the caliper), and moves toward the robot,
pushing the pad back its full 16 mm while the stop block holds the caliper. Then it pinches the grip boss, lifts the
caliper until the pads are 1 cm above the disc's top, carries it over the disc, lowers it until the ears are 4 mm
above the abutments and lets go. The targets are offset to cancel the arm's tracking error measured in GPU traces.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: fitted with the piston fully back, or 8 mm back (1.0); ears 2 mm up (1.0) and 3 mm up (0); 4 mm off
centre (1.0); tipped 6 degrees or still moving (0); set straight on with new pads, perched on the rim (0), and with
the piston only 4 mm back (0); fitted turned round (0); piston fully back with the caliper set down on the bench, where
it started or 10 cm away, or level on the abutments 2 cm off the disc (0); untouched (0); held 5 mm up (0); on its
side (0); off the table (0). Two more place a gripper over the boss of the fitted caliper: a finger 4 mm from it (0),
every finger 6 mm clear (1.0).
