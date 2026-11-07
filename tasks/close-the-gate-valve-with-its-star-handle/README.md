# Close a gate valve fully by its star handle, one spoke at a time under a pipe rack

A bench gate valve with a four-spoke star handle needs 1.5 turns clockwise to reach its closed stop. A pipe rack just
above the far part of the handle stops a hand reaching the spokes there, as under a pipe rack in a plant room, so the
robot turns the handle in strokes, letting go of one spoke and taking the next, until it stops. A position indicator
on the valve's front shows how far it has closed.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-colour parts with hinges and sliders) must sit next to `task.py`; `task.py` imports it.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Close the gate valve fully by its star handle."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Everything is fixed (the handle and indicator are jointed parts of the fixed valve), so
  `object_jitter` has no effect; submitted runs use `-E object_jitter=0`.

## Scoring as implemented

`score()` and `success()` in `task.py` read the handle's hinge angle: radians turned clockwise (seen from above) from
fully open, with the closed stop at 3 pi (1.5 turns). 1.0 within 5 degrees of the stop; 0.5 at least one full turn;
else 0. The score is the angle at the end of the trial (a handle turned back open loses its credit).
`SUCCESS_HOLD_S = 3`, so the video runs on while the arm lifts away.

The indicator is a slider on the valve body whose spring rest point an `on_step` callback sets to 45 mm times the
handle's share of its 1.5 turns (less the pointer's 2 mm sag on its spring), so the pointer stands for a stem
position indicator. It is for a person reading a photo: half a turn and 1.5 turns both leave the red knob over the
white tab, but the pointer is then 15 mm and 45 mm down. The scorer reads the hinge, not the pointer.

## Reach under the pipe rack

Checked offline with the arm's collision hulls (`yam_arena.analysis`), with the pipes taken as a box shaved 6 mm on
the near side: with a straight-down grip at yaws 0, 90 and -45 degrees and finger radii 30 to 75 mm from the axis,
the hand meets the pipes wherever the fingers are within about 75 to 90 degrees of straight ahead (0 degrees). The
free part of the circle spans at most about 210 degrees, so one stroke turns the handle less than a full turn and
even 0.5 takes at least two.

## Scripted demo

`demo()` drives the right arm with closed fingers. Each stroke: lower the fingers 45 mm from the axis at 255 degrees
(in the gap behind a spoke), sweep clockwise round the axis to 110 degrees in 14 short moves (pushing the spoke ahead
of them), lift clear of the knobs and go back. Seven strokes are planned; fixed waypoints only.

Measured in a trace (scripted, object_jitter=0): strokes turned the handle about 1.55 to 1.6 rad (about 90 degrees) each, the handle slipped
back up to 0.05 rad as the fingers lifted, it reached its stop at 3 pi on the sixth stroke and the trial scored 1.0;
the pointer ended at 45.0 mm.

## Checks without a simulator

`python near_misses.py` (needs `yam_arena` importable) scores each case: at the stop, 4 and exactly 5 degrees short
(1.0); 6 degrees short and exactly one turn (0.5); just under one turn, three quarters, half a turn, a quarter and
untouched (0).

Fixed habits, each a near-miss scoring 0: turn it anticlockwise (the open stop holds it), stop when the red knob
first reaches the white tab (half a turn), and one long sweep (under about 210 degrees). The layout is fixed, so a
memorised replay of the demo's waypoints cannot be ruled out.
