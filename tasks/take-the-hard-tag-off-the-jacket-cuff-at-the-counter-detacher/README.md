# Take an anti-theft hard tag off a jacket cuff at a counter-mounted magnetic detacher

A jacket cuff lies on a checkout counter with a retail hard tag pinned through its hem. A magnetic detacher is set
flush in the counter beside it. The robot has to get the tag off the cuff and leave both on the counter. The
instruction gives only the goal:

`Take the anti-theft tag off the jacket cuff and leave both on the counter.`

The way it is done at a real till: the tag goes dome-down into the detacher's well, where the magnet pulls the tag's
clutch open, and the pin is drawn out from the front by lifting the cloth it is pinned through. Pulled any other way,
the clutch holds the pin and the tag comes up with the cuff. Once the dome leaves the well, the clutch grips the pin
again.

**Credit.** This task builds on the idea of PR 1888, "Unpin an anti-theft hard tag from a jacket sleeve cuff with a
magnetic detacher" (TK-VMJ24A, a revision of TK-J86EJ7 and TK-XQERT7), whose 48-hour revision hold lapsed. All of the
code here is new; nothing from PR 1888's files was used. Its three reviews raised these points:

| Review point | What this version does |
|---|---|
| The simulated clutch stays released after the detacher moves away (TK-J86EJ7, TK-XQERT7). | No clutch state is stored anywhere. The clutch is a spring-loaded steel plate inside the tag (a slider joint) whose narrow slot sits in the pin's groove. The magnet stand-in only moves the plate spring's rest point, and only on control steps where the dome is in the well (`_magnet()` in `task.py`). The step the dome leaves, the 150 N/m spring pushes the plate back into the groove. |
| Make it re-engage when the detacher leaves before the pin is extracted, and show it (TK-XQERT7, TK-VMJ24A). | Trace `late_pull` (below): the dome is seated and the plate drawn to 6 mm, then the tag is lifted out of the well; the plate returns to 0 and the cuff is then lifted. The pin does not move in the tag, and the tag is lifted with the cuff. |
| The scorer permanently records the clutch as released, so a later pull gets full credit (TK-XQERT7). | The scorer keeps no record of a release. It counts pin travel step by step, and only on steps where the dome is in the well. |
| The scorer records absolute pin position whenever seated, so a pin pulled after the detacher leaves earns full credit if it is reseated (TK-VMJ24A). | Credited travel is the sum of each step's change in pin depth, added only on seated steps. Travel outward on an unseated step adds nothing; travel back in subtracts on any step. Reseating with the pin already out adds nothing, because the pin does not move on those steps. |
| Tests for a late pull followed by reseating, and for a partly seated pull finished later (TK-VMJ24A). | `near_misses.py`: "late pull, then the tag put back in the well with the pin already out" scores 0. "drawn 8 mm in the well, the tag taken out, the rest drawn outside" scores 0.25 and never succeeds. "drawn 8 mm in the well, out, reseated and finished in the well" scores 1.0, because all 16 mm were drawn while seated. |
| Videos from the wrong revision; no full-score run or Dream-1 trial of the corrected scene (TK-VMJ24A). | The screenshot, demo video and Dream-1 video all come from runs of the `task.py` in this folder. Scores are below. |

The reviews raised no importance or realism points. This version also changes the setup: PR 1888 used a
hand-held detacher pushed against the tag's side and a 14 mm knob on the pin. Here the detacher is a fixed counter
unit with a round well, and the pin has a real 15 mm domed head that sits flat on the hem. The YAM cannot pinch that
head off a flat surface, so the pin is drawn by lifting the cuff, as a cashier lifts the garment.

## Files

- `task.py`: the scene, the per-step record and scoring, the magnet rule and the scripted demo, for the `yam_arena`
  embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), `station` scene.
- `parts.py`: builds the counter and detacher as one fixed part, the tag as a loose articulation (body plus the
  clutch plate on a spring slider) and the cuff as a loose articulation (cuff plus the hem and pin on a hinge), and runs
  the per-step and reset callbacks. It patches the stock `yam_arena` scene and embodiment in the task's own process
  only; nothing on disk changes.
- `near_misses.py`: 24 scoring cases run offline through the same recorder the simulator calls.

No third-party assets: every body is built from boxes and cylinders in `task.py`.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the anti-theft tag off the jacket cuff and leave both on the counter."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/hardtag-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/hardtag-demo

# The late-pull check (not a submission video): seat the tag, take it back out, then lift the cuff
DEMO_VARIANT=late_pull inspect-robots run ... (as the demo)

# Offline scoring cases, no simulator
python near_misses.py
```

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz), about a minute of wall time on an L40S
  once Isaac Sim is up.
- **Seeds:** one fixed layout. Run with `-E object_jitter=0`. The two epochs are two trials of the same layout; the
  score is their mean.

## The tag, the pin and the detacher

- **Tag:** 74 x 42 x 20 mm, 30 g, with a 20 mm lock dome 6 mm proud on the underside, centred 14 mm from the pin end.
  A 5 mm square channel runs down from the top over the dome to a floor 3.5 mm thick.
- **Pin:** 3 mm shaft, 19 mm below a 15 mm domed head (2.5 mm rim plus a 1.5 mm dome). A groove 2.4 mm long with a
  1.2 mm core runs from 2.5 mm to 4.9 mm above the tip. Pushed home through the 3 mm hem, the tip is 4 mm above the
  channel floor, so 16 mm of shaft is inside the tag.
- **Clutch:** a 1.2 mm steel plate (6 g) inside the tag, 7.1 to 8.3 mm above its underside, on a slider along the
  tag's length. At rest a 150 N/m spring, preloaded 0.45 N, holds it against its stop. There, a slot 1.7 mm wide sits
  in the groove, so the shaft cannot pass up or down. Drawn back 6 mm, the plate brings a 6.4 mm wide hole under the
  shaft and the pin is free.
- **Magnet (the stand-in rule):** on every control step, if the dome sits in the well (its centre within 4.5 mm of
  the well's axis, its bottom within 2.5 mm of the well floor, the tag within 8 degrees of level), the plate spring's
  rest point is set 12 mm back, which draws the plate to its 6 mm stop. Otherwise the rest point is 3 mm forward of
  the stop, the spring's normal preload. No body is frozen, freed or switched. The plate moves only as the spring and
  its contacts with the pin allow.
- **Detacher:** a flush counter unit with a 74 mm top and a 28 mm well, 7 mm deep, its bore a 16-sided ring. With the
  tag flat on the counter and its dome in the well, the dome's bottom is 1 mm above the well floor.
- **Cuff:** 80 x 60 x 23 mm, 80 g. The 3 mm hem, 24 mm deep (8 g), carries the pin and is hinged to the cuff on an
  axis through the pin, so lifting the cuff pulls the pin along its own axis wherever the cuff lies.
- Contact offsets on the tag and the cuff are 0.1 mm, below the 0.25 mm clearance between the plate's slot and the
  groove's core.

## Scoring in the simulator

`score()` and `success()` read a record that an `on_step` callback updates every control step.

- **Pin depth:** the pin tip's height below the tag's top face, measured in the tag's frame, while the tip is within
  3 mm of the channel's axis; 0 otherwise. Drawn = 16 mm minus the depth.
- **Credited travel:** each step's change in drawn length is added if the dome is seated on that step, and only
  negative changes (the pin pushed back in) are added on other steps; the total is kept between 0 and 16 mm.
- **0.25:** credited travel at least 5 mm. **0.5:** the pin out (its tip at least 0.5 mm above the tag's top face, or
  more than 3 mm off the channel's axis) and credited travel within 0.5 mm of 16 mm. **0.75:** as well, no hand point
  within 5 mm of any box of the cuff, hem or pin, and the tip 10 mm or more from the pin hole's centre on the tag's top face.
  **1.0:** as well, no hand point within 5 mm of the tag, and over the last 1 s no box corner of the tag or the cuff
  moved more than 2 mm.
- **Hand points:** every vertex of the collision meshes of both grippers and all four fingertips, plus each triangle
  face sampled about every 3 mm.
- **Zero for the whole trial:** a box corner of the tag or the cuff more than 3 cm below the counter top (off the
  counter), or the hand or object state unavailable at any step (fail closed).
- Limits are compared with a 1e-6 margin, so a value exactly at a stated limit passes. The record resets on scene
  reset (`on_reset`), never during a trial.

## Real-world trial

Any flush or counter-top "golf" style magnetic detacher (a round unit about 70 to 75 mm across with a central well) and
a rectangular hard tag about 74 x 42 mm with a domed grooved pin of about 15 mm head and 19 mm shaft fit the layout.
For the counter, a 25 mm board with a cut-out for a flush unit, or the unit itself screwed to the table (then its
top stands proud, unlike here). The cuff can be the cuff of any lined or quilted jacket. Score from the video and a
final photo with a ruler. On a real tag the pin stays in the cuff; the 16 mm credited in the simulator is the full
length drawn out of the tag. Real magnets have strong fields: keep phones and cards off the counter, and keep the
grippers' steel parts clear of the well.

## The scripted demo

`demo()` is fixed waypoints from the published layout; it reads no simulator state.

1. Left hand, fingers closing across the tag's width: takes the tag 40 mm from the pin on its free end, lifts it 30
   mm with the cuff trailing, carries it 120 mm to the detacher and sets it down at its start height, so the dome
   is over the well, then lets go. The tag settles level with the dome in the well (about 10 s), and the magnet
   rule draws the plate back. The target is 3 mm to the right of the well's centre line, the left arm's measured
   drift under this load in the first trace.
2. Right hand, fingers closing across the cuff's 80 mm width: takes the cuff 52 mm behind the pin, lifts it 35 mm
   straight up (the pin comes out), carries it 70 mm aside, sets it down and lets go.

## Traces (L40S, Isaac Lab-Arena, `-E object_jitter=0`)

1. The dome seated and the plate drew back, but the cuff's lift pushed the pin 0.9 mm sideways in its channel, the
   plate's release hole was too narrow for that play, and the tag came up with the cuff (0). The hole was widened from
   4.4 to 6.4 mm, and the placement was moved 3 mm.
2. Seated at 10.2 s, pin out at 18 s, cuff set aside, 1.0 from 24.3 s.
3. `late_pull` (`DEMO_VARIANT=late_pull`): the dome was seated from 10.2 s with the plate drawn to 6.0 mm. The left
   hand took the tag again and lifted the dome out of the well; at 14.2 s the plate was back at 0 (re-engaged), with
   the pin unmoved. The left hand then held the tag 40 mm up while the right hand took the cuff and lifted it. The pin
   moved at most 0.6 mm in the tag (the groove's play) and the tag went up with the cuff. Credited travel 0, score 0.

Final runs (2 trials each): demo 1.0 and 1.0; Dream-1 0 and 0 (both hit the 900-step limit).

## What is simplified

- Rigid bodies. The cuff is one stiff quilted piece; only its hem bends, at the pin. Real cloth drapes and gathers.
- The magnet is not modelled as a field: the rule above moves the clutch spring's rest point while the dome is in the
  well. The clutch is a sliding plate in a grooved pin. Many real tags use balls or jaws that grip a smooth shaft at
  any depth; this plate catches only at the groove. So a pin that was already partly drawn is held only by friction
  after the dome leaves, and the scoring handles that case by counting only travel made with the dome in the well.
- The counter is a 25 mm board on the table so that the detacher can sit flush; the unit's magnet below it is not
  modelled.
- One fixed layout. The task has no choice for a fixed habit to win, but a replay of this layout cannot be ruled out.
