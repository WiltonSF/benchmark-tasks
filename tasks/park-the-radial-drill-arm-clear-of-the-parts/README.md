# Park a radial drill's arm without its spindle striking the parts on the table

A bench radial drill stands on the table: a green arm swings on a column, and a red drill head slides along a slot
in the arm. The head's black spindle hangs lower than the tops of two aluminium parts laid out on the drill's base
plate, and the head cannot be lifted off the arm. The robot is asked to park the drill (arm against its stop, head
run in to the column) without the head hitting the parts. Where the parts stand decides the order: a part close to
the column must be passed with the head run out, a part far from it with the head run in, and when one of each
stands in the swing the head has to change over between them.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts and joints) and `lots.py` (layout lots),
both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes
with the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

Part 1 stands on the arm's line after 25 degrees of swing, part 2 after 75 degrees. Each is in the inner band (ends
62.5 and 162.5 mm from the column's axis) or the outer band (137 and 237 mm). The spindle's start radius, 150 mm, is
inside both bands, so swinging without moving the head strikes part 1 in every lot.

| Lot | Part 1 | Part 2 | What clears both |
|---|---|---|---|
| A (screenshot) | inner | outer | run out, swing past part 1, run in, swing on |
| B | inner | inner | run out, swing past part 2, run in near the stop |
| C | outer | outer | run in, swing to the stop |

Trial k uses lot k mod 3, lot A first, so a scored run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C`
pins every trial to one lot. Checked offline against the spindle's 20 mm footprint along each fixed path (the
spindle swept at a fixed radius, from the task's own geometry): swinging as it is strikes in all three lots; "always
run in first" clears only lot C; "always run out first" clears only lot B; lot A's own path (out, then in at 53
degrees) clears only lot A. Each fixed habit averages at most 0.33. The lots stop a single fixed habit, not a
memorised replay of all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Park the drill: swing its arm round to the yellow line and slide the drill head in to the column. Do not let the drill head hit the parts on the table."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir ~/yam-eval/drill-dream

# Scripted demo, all three lots (each plan lasts 24 to 28 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir ~/yam-eval/drill-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- **Arm angle** is the hinge's joint position (0 pointing at the robot, 110 degrees at the stop, which is the
  joint's upper limit). Parked: at least 108 degrees.
- **Run in:** the horizontal distance from the head's origin (on the spindle's axis) to the column's axis is at most
  85 mm; the slot's inner end stops it at 80 mm.
- **Still:** the head moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold
  1.5 s. The head rides on the arm, so a moving arm moves it.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the head and of the arm; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Strike record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step either part's
  centre is more than 3 mm from its lot's start or the part is tipped more than 3 degrees from upright. Putting a
  part back does not clear it. It is cleared only by a scene reset: the head jumps over 20 mm in one control step and
  lands within 0.5 mm of its start, at rest, with the arm within 0.5 degrees of its start and both parts within
  0.5 mm of theirs. A robot cannot move the head that fast.
- **Partial credit:** 0.5 for the arm parked, let go and still with the head not run in and no strike. This is the
  intended method short of its last step; a habit that strikes a part scores 0.

## Scripted demo

`demo()` drives the left arm only. It does not look at the camera images: it takes its plan from the trial's lot
(`lots.current().bands`, the band each part stands in), the same layout the top camera shows. It reads no simulator
state. It grips the white knob from above (pads 3 mm inside its
faces, tips 6 mm up the knob). For each part in swing order it sets the head's radius clear of the part before the
swing reaches it (run out past an inner part, run in past an outer one), changing over between the parts if needed
at 56% of the way from part 1's line to part 2's (53 degrees). Radial slides turn the grip so the pads close along the arm (the
round head turns with it), except at the start, where the wrist cannot turn that far. If the head is still out after
the last part, it runs it in at 105 degrees. The swing goes in 5-degree steps to 4 degrees past the stop (the arm is
soft), and the head is held in, commanded 25 mm past the slot's end because the loaded arm lags. Then it opens, lifts and moves away.

## Measured in simulation

Traces on the GPU (`trace` mode, `--epochs 3`):

- First version, pads always closing along world y: the swing itself worked in every lot, but radial pulls at an
  angle slipped in the pads; in lot A the head stayed out and struck part 2 (moved 4.5 mm, scored 0).
- With the arm's ways at friction 1.2 the pulled head tilted and bound in its slot. At 0.3 (oiled ways) lot A ran in
  from 218 to 80 mm in one move and scored 1.0, and lot C scored 1.0; lot B's run-in at the stop stopped at 92 mm
  (0.5).
- Pressing the head in at the stop itself kept stopping at 92 mm in lot B, so part 2 now stands on the 75-degree
  line and the demo runs the head in at 105 degrees, 30 degrees past it, before the last 5 degrees of swing. With that
  all three lots scored 1.0 (trace-20261111T054346); neither part moved in any lot.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any grip or pushing, any path that never moves a part over 3 mm or tips it over
  3 degrees, ending parked and let go with the head run in.
- **Score 0:**
  - striking a part with the spindle, the head or a gripper (moved over 3 mm or tipped over 3 degrees), even if it
    is put back afterwards;
  - lifting a part out of the way and back;
  - the arm short of its stop by more than 2 degrees, or still held, or still moving.
- **0.5:** parked and let go with the head left out from the column.

The cases in `near_misses.py` cover the 2-degree and 5 mm limits on both sides, a moving head, a part nudged 3 mm
(no strike) and pushed 4 and 5 mm, tipped 2.9 (no strike) and 4 degrees, a part lifted and put back, and scene
resets.

## Recorded runs

- Scripted demo, `--epochs 3`: 1.0 in lots A, B and C (task_score 1.0).
- Dream-1, 900 steps, `--epochs 3`: 0.0 in all three lots (each hit the step limit). An earlier wording of the
  instruction ("swing its arm round against its stop") was refused by the Dream API as forcing a joint against a hard
  stop, so the instruction names the yellow line instead.
