# Fit the reducing sleeve before racking a conical tube in an oversize heat-block well

A dry heat block has a 30 mm well. The sample tube is 16 mm across with a conical end, so it cannot stand on its own:
in the bare well it leans over against the rim. A reducing sleeve fits the well, and the tube stands straight in it.
The instruction only says to put the tube in the well. In lot A the well is bare and the robot must fit a sleeve
first; in lot B a sleeve is already seated in the well and the other sleeve must stay on the bench. A well plug, which
also fits the well, is the nearest loose object to the left arm.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-shape loose parts) and `lots.py` (layout lots) must sit next to `task.py`; `task.py` imports both.

## Lots

Two layouts. Lot A (the screenshot): the near well bare, both sleeves on the bench. Lot B: the second sleeve already
seated in the near well (placed where a sleeve fitted by the arm was traced to settle), the first on the bench.
Trial k uses lot k mod 2, lot A first, so a run needs `--epochs 2` (or a multiple); `TASK_LOT=A` or `TASK_LOT=B`
pins every trial to one lot. The scoring rule is the same in both lots. A fixed habit scores at most 0.5: always
fitting a bench sleeve fails lot B (the second sleeve lands on the seated one), and never fitting one fails lot A
(the tube leans in the bare well). The lots stop a single fixed habit, not a memorised replay of both layouts.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the sample tube in the empty well of the heat block."

# Dream-1, both lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/dream

# Scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started, so about twice that for the two lots.
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`: the demo's aim corrections (below) were measured
  in that fixed layout, and jitter would move the tube in its rack. With `--epochs 2` the videos and screenshot show
  lot A and the score is the mean of the two trials.

## Scoring as implemented

`score()` and `success()` in `task.py`, pass/fail, the same in both lots. 1.0 when, for one of the two sleeves:

- **Sleeve seated:** its centre within 5 mm of the near well's centre in x and in y (the well's 4 mm play a side,
  plus 1 mm), tilted at most 2 degrees, and its collar's underside within 2 mm of the block's top.
- **Tube standing in it:** the tube's tip (the tube part's origin) within 5 mm of that sleeve's axis (inside the
  10 mm pocket) and within 1 mm of the sleeve floor, and its axis within 8 degrees of vertical (`MAX_LEAN`). In the
  sleeve's 20 mm bore the 16 mm tube can lean about 6.3 degrees into a corner; in the bare well it leans 16. The
  1 mm test fails a tube held above the floor. It does not prove release on its own (a tube held with its tip on
  the floor would pass); the scorer has no gripper check, and the 3 s hold runs while the demo's arm lifts away.

and also:

- **Other sleeve on the table:** its lowest point within 1 mm of the table top.
- **Plug clear:** the plug's centre is not within 29 mm of the well's centre with its bottom less than 20 mm above
  the block's top.
- All four loose objects still (under 1 cm/s and 0.5 rad/s); the trial ends once this has held 3 s
  (`SUCCESS_HOLD_S`), so the video runs on while the arm lifts away.

## Measured in traces (scripted, object_jitter=0)

| Run | Result | Score |
|---|---|---|
| The tube lowered centred into the bare well, tip 2 mm above the floor, fingers opened over 2 s, lifted straight up | Leaned to 16.2 degrees and stayed there | 0 |
| Lot A, before the settle pauses (trace-20261107T162637) | Sleeve seated 3.8 mm beyond and 3.9 mm left of the well's centre, tube 3.4 degrees from vertical, tip on the sleeve floor | 1.0 |
| Lot B, first two traces: tube lowered straight from the carry into the pre-seated sleeve | Still swinging about 5 mm when it reached the rim; its body caught on the sleeve's top, tip 36 mm above the floor | 0 |
| Lot B, with settle pauses | Tube in the bore, but its tip 2.5 mm off the sleeve's axis landed on the pocket's frame, 4.8 mm up | 0 |
| Both lots (trace-20261107T164736), settle pauses and the aim moved 2 mm nearer and 1 mm right | Lot A: sleeve seated 2.9 mm beyond and 3.9 mm left of the well's centre, tube 0.4 degrees from vertical, tip on the floor. Lot B: tube 5.4 degrees from vertical in the pre-seated sleeve, tip on the floor, bench sleeve untouched | 1.0, 1.0 (mean 1.0) |

Earlier traces showed three things now fixed in the demo: gripping the sleeve's thin walls let it slip, so the demo
grips the collar; the sleeve settles about 3.5 mm beyond and 4 mm left of the well's centre, and the carried tube's
tip lands about 6 mm nearer the robot and 6 mm right of its aim, so the tube is aimed with both corrections (refined by 2 mm and 1 mm once the arm pauses before lowering); and with
an 8 mm pocket the tip caught on the pocket's frame, so the pocket is 10 mm.

## Scripted demo

`demo()` reads the trial's lot (the layout the cameras show). Lot A: the left arm grips a bench sleeve's collar,
carries it over the well, lowers it to 4 mm above seated and lets go, then parks. Lot B: the left arm only moves to
the same parking spot. In both lots the right arm waits 11 s, grips the tube 44 mm below its top (so the fingers stay
above the sleeve), lifts it out of the rack, carries it over the sleeve's bore, pauses 1 s, lowers until the tip is
10 mm above the sleeve's top, pauses 1 s, lowers to 5 mm above the sleeve floor and lets go. Fixed waypoints only.

## Checks without a simulator

`python near_misses.py` (needs `yam_arena` importable) scores each case, including: done (1.0); tube leaning 4
degrees, 6.3 degrees into a corner, and exactly 8 degrees (1.0), and 8.1 degrees (0); tip 1 mm off the floor (1.0)
and held 3 mm up (0); tip outside the pocket, still moving, collar 3 mm proud, sleeve tilted 3 degrees (0); lot B done
(tube in the pre-seated sleeve, bench sleeve left: 1.0); and the fixed habits, each 0: the tube straight into the
bare well (leaning, as traced), the plug into the well first (nearest object), every loose object used (plug left on
the block over the well), and in lot B the bench sleeve set on the seated one or on the block's top.
