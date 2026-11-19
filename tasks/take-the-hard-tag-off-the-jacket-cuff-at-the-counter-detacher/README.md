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
magnetic detacher" (TK-VMJ24A), whose 48-hour revision hold lapsed. All of the
code here is new; nothing from PR 1888's files was used. Its three reviews raised these points:

| Review point | What this version does |
|---|---|
| The simulated clutch stays released after the detacher moves away (earlier reviews). | No clutch state is stored anywhere. The clutch is a spring-loaded steel plate inside the tag (a slider joint) whose narrow slot sits in the pin's groove. The magnet stand-in only moves the plate spring's rest point, and only on control steps where the dome is in the well (`_magnet()` in `task.py`). The step the dome leaves, the 150 N/m spring pushes the plate back into the groove. |
| Make it re-engage when the detacher leaves before the pin is extracted, and show it (later reviews). | Trace `late_pull` (below): the dome is seated and the plate drawn to 6 mm, then the tag is lifted out of the well; the plate returns to 0 and the cuff is then lifted. The pin does not move in the tag, and the tag is lifted with the cuff. |
| The scorer permanently records the clutch as released, so a later pull gets full credit (an earlier review). | The scorer keeps no record of a release. It counts exposed-head withdrawal step by step while engaged and qualifies the first whole-pin removal event only while engaged. |
| The scorer records absolute pin position whenever seated, so a pin pulled after the detacher leaves earns full credit if it is reseated (TK-VMJ24A). | Credited withdrawal follows the exposed head. Positive movement counts while seated, and actual reinsertion debits it. A visible removal event outside the well cannot be rescued by reseating or a free near approach. |
| Tests for a late pull followed by reseating, and for a partly seated pull finished later (TK-VMJ24A). | `near_misses.py`: "late pull, then the tag put back in the well with the pin already out" scores 0. "drawn 8 mm in the well, the tag taken out, the rest drawn outside" scores 0.25 and never succeeds. "drawn 8 mm in the well, out, reseated and finished in the well" scores 1.0, because all 16 mm were drawn while seated. |
| Videos from the wrong revision; no full-score run or Dream-1 trial of the corrected scene (TK-VMJ24A). | The screenshot, demo and Dream-1 videos all come from final runs of this version's code and scorer (see Traces). |

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
- `near_misses.py`: scoring-history regressions run offline through the same recorder the simulator calls.

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
  body underside (0.5 mm above the 3.5 mm channel floor), so 16 mm of shaft is inside the tag.
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

`score()` and `success()` read a whole-trial record updated every control step. Withdrawal is judged from the
exposed silver head, not from the concealed pin tip's depth.

- **Visible withdrawal:** the exposed head's top is measured along the tag's axis relative to its fully inserted
  height. Positive movement counts only while the dome is seated; reinsertion subtracts anywhere. Sideways movement
  of a buried tip creates no withdrawal. Until the shaft leaves its actual channel, all exposed-head rise is accounted for; it is not capped at 16 mm. Real judges
  do not measure concealed travel or reconstruct it to 0.5 mm.
- **Whole-pin removal:** the entire shaft/head envelope, including orientation, must be at least 5 mm clear of the
  tag's outside envelope and visible rim. A lone tip outside the channel is insufficient. At actual channel exit, at least the 5 mm progress tier and all recorded withdrawal must be credited.
  No fixed 16 mm exposed-head rise is imposed on an angled extraction. The first 5 mm-clear removal event must
  also occur while engaged.
  Provenance is captured when the shaft actually leaves the channel, before the larger 5 mm display threshold.
  It stays fixed while the shaft is outside the actual pin channel: lowering, approaching or touching
  the exterior is not reinsertion. Actual shaft re-entry debits extraction and revokes that event. An already-removed
  initial pin, outside finish or later reseating/retreat cannot manufacture valid removal.
- **0.25:** at least 5 mm of credited exposed-head withdrawal. **0.5:** complete valid seated-withdrawal history and
  the whole pin at least 5 mm clear. **0.75:** additionally the cuff is released on the raised counter and the whole
  pin is at least 10 mm clear of the tag. **1.0:** tag also released on that counter, and both still for 1 s with no
  box corner moving more than 2 mm.
- **Returned/support check:** a lower corner of the tag's body floor or cuff's quilted body is within 2 mm of the
  raised counter plane over its actual footprint. The dome may extend into the well. Suspended parts and parts on
  the lower table cannot satisfy returned-object stages.
- **Release:** every vertex of both grippers' and all four fingertips' collision meshes, with triangle faces sampled
  about every 3 mm, must be more than 5 mm from the relevant whole object.
- **Whole-trial zero:** any tag/cuff box corner below `TOP + 5 mm` (20 mm below the raised counter), or unavailable
  hand/object state at any step. The lower table is 25 mm below the counter, so either part resting there fails;
  lifting it back never clears failure. Reset is only through the existing scene-reset hook.
- Positional numerical epsilon is 1 micrometre. The fall comparison uses that numerical allowance: a corner exactly at its cutoff does
  not itself latch a fall. This epsilon is not a concealed real-trial pin measurement.

## Real-world trial

Use a flush-mounted golf-style magnetic detacher, about 70–75 mm across with a central well, and a matching
74 x 42 mm rectangular hard tag with a 15 mm domed head and 19 mm shaft. The counter is the specified 25 mm board
with a cut-out for the flush unit. The cuff is from a lined/quilted jacket matching the declared flattened size.
Record side and overhead video, an inserted-start reference photo, and a final photo with a millimetre ruler in the
same plane as the observed feature. The pin stays in the cuff.

Judge engagement from the tag's external outline aligned over the detacher, the dome entering its matching well,
and the body resting level at the seated height. Keep the well/outline visible; hovering, resting nearby, or a
remembered earlier engagement is insufficient. Observe the exposed head/hem throughout withdrawal.

| Tier | Visible observation |
| --- | --- |
| 0.25 | Exposed silver head rises at least 5 mm from its inserted reference while the tag remains engaged. Reinsertion removes that progress. Seating/touching alone earns zero. |
| 0.5 | The whole pin (head and shaft) visibly outside the tag with at least 5 mm gap from its outline; video establishes withdrawal while engaged. No concealed tip coordinate or 0.5 mm travel measurement is required. |
| 0.75 | Cuff released on the raised counter, grippers/fingertips more than 5 mm clear, and entire exposed pin at least 10 mm clear of the tag. |
| 1.0 | Tag also released on the raised counter, grippers/fingertips more than 5 mm clear, and both remain within 2 mm of their final positions for 1 s. |

If a required feature is occluded, use another camera or the final ruler photo; do not infer hidden extraction from
timing or magnet contact. A pin drawn after leaving the well cannot be rescued by reseating the separated tag. A
partial withdrawal may continue after reseating only when video shows no further withdrawal outside the well.
Either part falling onto the lower table or off the counter zeros the whole trial, even after recovery.

Real magnets have strong fields. Keep **gripper pads and steel hardware clear of the exposed pin and strong
detacher magnet/well**. Use the existing remote grasp areas on the tag's free end and cuff body; keep phones and
cards off the counter. These are handling requirements, not measured magnetic-field safety results.

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

Traces 1 to 3 were run during the build, with the earlier scorer. Item 4 is the final run of this version.

1. The dome seated and the plate drew back, but the cuff's lift pushed the pin 0.9 mm sideways in its channel, the
   plate's release hole was too narrow for that play, and the tag came up with the cuff (0). The hole was widened from
   4.4 to 6.4 mm, and the placement was moved 3 mm.
2. Seated at 10.2 s, pin out at 18 s, cuff set aside, 1.0 from 24.3 s.
3. `late_pull` (`DEMO_VARIANT=late_pull`): the dome was seated from 10.2 s with the plate drawn to 6.0 mm. The left
   hand took the tag again and lifted the dome out of the well; at 14.2 s the plate was back at 0 (re-engaged), with
   the pin unmoved. The left hand then held the tag 40 mm up while the right hand took the cuff and lifted it. The pin
   moved at most 0.6 mm in the tag (the groove's play) and the tag went up with the cuff. Credited travel 0, score 0.
4. Final runs of this version's code and scorer, 2 trials each. Scripted demo: 1.0 and 1.0 (dome seated, pin drawn
   out in the well, cuff set aside, both parts still; full score from 23.5 s in each trial). Dream-1: 0.0 and 0.0. Both
   trials hit the 900-step limit; the tag was never seated in the detacher and no pin withdrawal was recorded.

## What is simplified

- Rigid bodies. The cuff is one stiff quilted piece; only its hem bends, at the pin. Real cloth drapes and gathers.
- The magnet is not modelled as a field: the rule above moves the clutch spring's rest point while the dome is in the
  well. The clutch is a sliding plate in a grooved pin. Many real tags use balls or jaws that grip a smooth shaft at
  any depth; this plate catches only at the groove. So a pin that was already partly drawn is held only by friction
  after the dome leaves, and the scoring handles that case by counting only travel made with the dome in the well.
- The counter is a 25 mm board on the table so that the detacher can sit flush; the unit's magnet below it is not
  modelled.
- One fixed layout. The task has no choice for a fixed habit to win, but a replay of this layout cannot be ruled out.
