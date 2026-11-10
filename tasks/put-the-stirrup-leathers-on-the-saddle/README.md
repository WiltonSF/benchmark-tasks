# Hang the stirrup leathers on the saddle's bars with the safety catches left down

Each stirrup bar on a saddle ends in a hinged safety catch. For riding the catch must be down (open). Then, if a
rider falls with a foot caught in the iron, the leather slides off the bar instead of dragging them. The robot is
asked to put two stirrup leathers on the saddle. It has to slide each leather's loop onto its bar from the open end,
and every catch must be down at the end. A catch that is up blocks the loop, and every lot starts with at least one
catch up, so it must be turned down first. Turning a catch up afterwards, which looks like securing the leather, scores 0. The instruction does not
mention the catches.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts, joints and step hooks) and `lots.py`
(layout lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes
and wrong outcomes with the score each must get. They are plain-Python checks of `score()` and `success()` and need
no simulator.

## Lots

Lot A (the screenshot and the first trial of each video): the right catch up. Lot B: the left catch up. Lot C: both
catches up. Trial k uses lot k mod 3,
lot A first, so a scored run needs `--epochs 3` (or a multiple). `TASK_LOT=A`, `B` or `C` in the environment pins
every trial to one lot. Lots can only move loose objects, so a reset hook (`parts.on_reset`) sets the catches. Then
`lots.py` settles the scene for 0.5 s before the first frame. A scene reset is the only way this happens; the robot
cannot trigger it. A script that ignores the catches scores 0 in every lot: the loop jams on a raised catch, and a
raised catch left at the end scores 0 anyway. A fixed habit of turning one side's catch fails the other lots. The
lots do not stop a memorised replay of all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the stirrup leathers on the saddle."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 2400 --epochs 3 --store-frames --log-dir ~/yam-eval/stirrup-dream

# Scripted demo, all three lots (each reaches 1.0 in 15 to 21 s; the run stops at success)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1000 --epochs 3 --store-frames --log-dir ~/yam-eval/stirrup-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 120 s of simulated time (2,400 steps at 20 Hz), about 2 to 4 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## How the saddle works in the simulator

The saddle is one fixed `parts.Part` (stand, seat block, two bars and their posts) with two hinges, the catches
(about left-right axes, -0.35 rad swung toward the saddle, 0 hanging, to 2.9 rad). A loose leather collides with the catches and the bars, so a catch
that is up stops the loop. A `parts.on_step` hook gives each catch a stiff hinge: its drive (0.05 N m/rad) follows
it with a 0.05 rad lag, so it stays where it is left. The peg rack is a separate fixed part. Each leather is one loose
rigid part made of six boxes.

## Scoring details the simulator applies

- **Catch down:** its hinge within 30 degrees of hanging (1e-6 rad margin; pinned on both sides in `near_misses.py`).
  Either catch up gives 0, whether or not a leather hangs on that bar.
- **Hung:** measured from the leather's pose. The loop's centre is within 7 mm sideways of the bar's centre line (the
  opening's slack round the 10 mm bar). The loop's inner top is at or above the bar's top and its inner bottom at or
  below the bar's bottom. The loop's front and back faces lie between the bar's open end and its post. The leather is
  within 20 degrees of upright, and its lowest point is at least 10 mm above the table. Either leather may hang on
  either bar; each bar counts once.
- **Still:** both leathers under 1 cm/s and 0.5 rad/s (`ObjectState.resting()`), and both catches turning under
  0.005 rad per control step.
- **Let go:** the hook measures the shortest distance from both arms' finger and gripper collision hulls
  (`yam_arena.analysis`) to both catches and every box of both leathers. All must exceed 5 mm. Offline
  (`near_misses.py`) there is no robot, so this counts as clear and still.

## Scripted demo

`demo()` reads only the trial's lot (which catches the cameras show up). Each arm works its own side. Where the catch
is up, the closed fingertip (wrist turned 90 degrees) comes down 10 mm in front of the catch's hinge, onto the
catch's tip (which leans 25 degrees toward the robot), and presses it over toward the robot and down. Then, 6 mm
below the hanging catch, it sweeps in to 4 mm short of the hinge so the catch hangs, backs off and rises. The fingers then take the leather's loop by its two sides (pads 3 mm into the 36 mm width), lift it 8 mm off
the peg, and slide it forward off the peg's end. They carry it in line with the bar, 9 mm above hanging height (the
loop's opening then clears the hanging catch's hinge by 3 mm and its bottom end by 3 mm), slide it onto the bar from
the open end until its centre is 45 mm in, lower it 13 mm so the loop rests on the bar, open, rise and park.
The fingers pass either side of the bar inside the loop's sides. `check.py` treats the fingers as closed,
so the saddle and the rack are listed in `DEMO_TOUCHES`; the catch-pressing moves were checked separately against
the saddle with no touch allowed (no link enters it). With `TASK_GAPS=1` in the environment the step hook prints
each new closest approach of the fingers to a bar or post during a run (debugging only; it changes nothing).

## Other ways to succeed, and what scores 0

- **Also full score:** either leather on either bar; any grip; the loop anywhere along the bar between its end and
  its post.
- **Score 0:** a catch left up or turned up at the end (with the leathers hung or not), which includes ignoring the
  catches in any lot; both leathers left on the pegs;
  a leather set on top of a bar instead of round it, held, swinging or on the table.
- **0.5:** one leather hung with both catches down.

The cases in `near_misses.py` cover the start, each tier, the 30-degree catch limit on both sides, a catch up with
one or both leathers hung, the loop at the bar's open end and at its post (in and 1 mm out), the sideways slack limit,
a loop set on top of the bar, a leather held 16 mm up, swung 15 and 25 degrees, swinging, standing on the table, and on the
floor.

## Recorded runs

Before the review fixes (both catches down in lot A, a 14 mm catch hinged level with the bar's top):
- Traces, `--epochs 3`: with the first catch the pressing finger pushed the catches further up, the loops jammed on
  them and lots B and C scored 0 (trace-20261109T082532), as a policy that ignores the catches would. With the
  catch at 135 degrees and a sweep at the bottom, 1.0 in each lot (trace-20261109T084644).
- Scripted demo (demo-20261109T085352): 1.0 in each lot. Dream-1 (dream-20261109T085901): 0.0 in each lot; the arms
  hovered near the saddle and moved nothing.

After the review fixes (lot A right catch up, 22 mm catch at 155 degrees):
- Trace, `--epochs 3`, with `TASK_GAPS=1` (trace-20261109T154339): 1.0 in each lot; catches ended within 5 degrees
  of hanging; the closest any finger came to a bar or post was 8.3 mm (while pressing a catch), so the open fingers
  stayed clear of the bars and posts throughout. An earlier trace with the hinge 3 mm above the bar and a 5 mm
  carry clearance (trace-20261109T153312) jammed the right loop on its lowered catch in lots A and C (0.5 each).
- Scripted demo, `--epochs 3` (demo-20261109T155048): 1.0 in lots A, B and C (task_score 1.0); the video's first
  trial is lot A, with the right catch up.
- Dream-1, 2,400 steps, `--epochs 3` (dream-20261109T155641): 0.0 in every lot; the arms hovered near the saddle and
  moved neither leather nor any catch.
