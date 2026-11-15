# Put each empty tote on the tote behind it so it stacks on the full one and nests in the empty one

Four totes of one stack-nest model: open-topped containers 75 mm tall, with a floor, four walls that lean out
6 degrees (150 x 200 mm at the top of the walls) and a solid rim lip, 20 mm tall and 12 mm thick, round the top
(166 x 216 mm over the lip). Each long wall has two yellow lugs on its inner face 10 mm below
the rim and, at the same two places, a recess in its lower 33 mm (dark from inside). The lugs are off centre (60 mm
toward one end, 25 mm toward the other), so turning a tote end for end moves its recesses away from the lugs of the
tote below.

- **Aligned** with the tote below, a tote's recesses pass over the lugs and its tapered walls slide inside the lower
  tote's walls: it **nests**, its underside 38 mm above the table (rim 113 mm), until the walls meet.
- **Turned 180 degrees**, its underside lands on the four lugs: it **stacks**, underside 65 mm up (rim 140 mm),
  clear of anything below the lugs.

One blue tote is full (a carton of parts, top 54.5 mm up, just under the lugs); the other is empty. Both grey totes
start aligned with the blue totes. So the grey tote in front of the full tote must be turned to stack (nested, it
would come down onto the carton), and the other must be left as it is to nest in the empty tote (turned, it would
stack on the empty tote's lugs and waste the space nesting saves).

**Layout lots.** In lot A (the screenshot) the left blue tote is full; in lot B the carton is in the right one.
Trial k uses lot A when k is even and lot B when k is odd (`lots.py` in this folder), so run with `--epochs 2`;
`TASK_LOT=A` or `TASK_LOT=B` in the environment pins every trial to one lot. Two lots stop a fixed habit, not a
memorised replay of both layouts.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. It uses `parts.py`, a small helper in this folder that builds multi-shape parts (each tote
is 43 boxes, some turned to follow the taper); it imports only NumPy and `yam_arena`. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get. It checks `score()` and `success()` in plain Python
and needs no simulator.

## How stacking and nesting are both demonstrated

- The scripted demo does both in every trial: the arm in front of the full tote turns its grey tote end for end and
  stacks it on the lugs; the other arm sets its grey tote, unturned, into the empty tote, where it nests. Lot B
  swaps the roles of the two arms.
- Nesting is also the failure on the full tote. A GPU trace of a variant demo that does not turn the left tote shows
  it nesting down onto the carton (see "Traces" below); that blue tote then cannot qualify for the rest of the trial.
- Both outcomes come from contact between the totes' boxes (lugs, recesses, tapered walls). Nothing in `task.py`
  moves a tote or decides where it stops.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put each empty grey tote on the blue tote behind it so it stacks on the full one and nests in the empty one."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/totes-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --epochs 2 --store-frames --log-dir ~/yam-eval/totes-demo
```

The run prints `task_score`, the mean over the two trials (one per lot). Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial (two trials per run):** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** run with `-E object_jitter=0`, so every object starts exactly where `task.py` puts it and no seed is
  needed. With jitter on, seeded resets would shift the grey totes and the carton; use `object_jitter=0`.

## How the simulator scores it

- Rim heights are computed from each grey tote's pose and its true dimensions (the four outer rim corners).
- **Stacked on the full tote:** upright (axis within 3 degrees of vertical), every rim corner within 1.5 mm of
  140 mm above the table, centre within 5 mm of the full tote's centre in x and y, moving under 1 cm/s and turning
  under 0.5 rad/s. The mouth leaves 3.9 mm of play per side at the lugs.
- **Nested in the empty tote:** the same, with every rim corner from 4 mm below to 8 mm above 113.27 mm (where the walls meet), so 109.27 to 121.27 mm. A
  tote let go a little off level can wedge in the taper a few mm high (a lot A trace stopped 4 mm high, cocked
  1.3 degrees); it is still nested, well below a tote pressed onto the carton (rim about 129.5 mm).
- **Let go:** an `on_step` hook measures, every control step, the distance from both arms' fingers and grippers to
  each grey tote's boxes. A tote counts only while every one is more than 5 mm away. Before the first measurement
  (and offline) the state is unknown, which gives no credit: the check fails closed.
- **Full tote, judged over the whole trial:** once a grey tote's underside (the middle of it, so a tote tilted on the
  lugs does not count) has been below 63 mm above the table while its centre was over the full tote's inside, or
  the carton has been more than 5 mm from its start, the full tote cannot qualify again in that trial. A tote turned
  to stack is held at 65 mm by the lugs and cannot reach 63 mm.
- Both records are cleared only at a new trial: when every loose object is back within 0.5 mm of its start, at
  rest, having got there by a jump of more than 20 mm in one control step (a scene reset, which the robot cannot
  produce).
- A trial succeeds once both blue totes have qualified for 3 s (`SUCCESS_HOLD_S`), long enough for the demo's
  arms to park clear before the episode ends.
- Either grey tote may go to either blue tote. 0.5 each; `success()` is both.
- **Checked at every control step.** Let go: every finger and gripper more than 5 mm from that grey tote, unknown
  counts as not let go. Still: under 1 cm/s and 0.5 rad/s.
- Fixed habits, mean over lots A and B: never turning 0.5 (0.5, 0.5); turning both 0.5 (0.5, 0.5); always turning
  the left tote 0.5 (1.0, 0.0); always turning the right tote 0.5 (0.0, 1.0). Only reading which tote is full gives
  1.0.

## Scripted demo

The demo first takes where the carton starts, which the top camera shows in the first frame, and treats the blue tote
it is in as the full one. It never reads the lot's name. Each arm then pinches the rim lip of one long wall of its
grey tote, fingers closing across the wall, 15 mm from the wall's middle (between the lugs).

- The arm in front of the full tote grips the far wall and lifts the tote until its underside is 12 mm above the
  blue rims. It turns the tote end for end in four eighth turns about the tote's centre while carrying it over the
  full tote, lowers it into the mouth, and lets go with the underside 5 mm above the lugs.
- The other arm grips the near wall, carries its tote straight across without turning it, and lets go 6 mm above
  where it nests.
- Both arms then park out to the sides, clear of the totes, so the end state is visible from the top camera.

Both arms aim off by the tracking error measured in GPU traces, and turn the wrist back by the 2 to 4 degrees the
tote turns in the fingers as they close (fixed numbers in `task.py` for each arm and move, calibrated once). The
demo is open loop from the published start layout; it reads no simulator state.

## Traces

GPU traces of this scene (stock yam-isaac-lab-arena, `object_jitter=0`):

- **Demo, lot A:** the left grey tote ends turned end for end on the full tote's lugs, underside 65.2 mm above the
  table (rim 140.2 mm); the right grey tote nests in the empty tote, wedged 4 mm high and cocked 1.3 degrees (rim
  corners 114.2 to 120.2 mm). The carton does not move. Score 1.0.
- **Demo, lot B:** the right grey tote ends turned end for end on the full tote's lugs (rim 140.2 mm); the left grey
  tote nests fully, underside 38.4 mm up (rim 113.4 mm). The carton does not move. Score 1.0.
- **Never turning (a variant demo on the earlier one-lot layout, not part of this folder):** the left grey tote is carried to the full tote
  unturned and let go at the same height as the turned one. Its recesses pass the lugs and it comes down past them,
  ending with its underside middle 58 mm up, tilted 4.7 degrees, resting over the carton (top 54.5 mm up), where a
  stacked tote sits at 65 mm. The full tote is marked pressed for the rest of the trial; the right tote nests as in
  the demo. Score 0.5.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The cases include: both done (1.0); each alone (0.5); never turning, so the left nests onto the carton (0.5, and
still 0.5 after it is lifted out, turned and stacked); turning both, so the right stacks on the empty tote (0.5);
the two grey totes swapped (1.0); lowered nest-wise over the full tote to an underside of 63.0 mm (no penalty)
versus 62.9 mm (penalty); 1.5 mm above the lugs (1.0) versus 2 mm (0.5); tilted 2 degrees on the lugs (0.5, and no
penalty); still moving (0.5); 5 mm (1.0) versus 6 mm (0.5) off centre; wedged 8 mm high (1.0) versus 9 mm (0.5), and 4 mm low (1.0) versus 5 mm (0.5);
nested but tilted 4 degrees (0.5); upside down (0.5); carton moved 5 mm (1.0) versus 6 mm (0.5, and still 0.5
after it is put back); one grey tote nested in the other in the empty tote (0.5); carton lifted out and a grey tote
nested to the full tote's floor (0, then 0.5); a finger still within 5 mm of either tote (0.5); grip state never
measured (0).
