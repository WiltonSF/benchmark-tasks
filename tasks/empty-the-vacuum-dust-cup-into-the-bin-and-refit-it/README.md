# Empty a vacuum's dust cup through its bottom flap into the bin and click it back in

A bagless vacuum's dust cup is docked in a tray on the vacuum body, held by a sprung catch, with three balls of
"dirt" in it. The robot is asked to empty the cup into the bin and put it back. Doing it properly means holding the
catch back while lifting the cup out, opening the cup's own bottom flap (by its red button) only over the bin, and
pressing the cup back into its tray until the catch springs over its rim. The instruction names only the goal.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with hinges and sliders), in this folder;
it imports only NumPy and `yam_arena`. This folder's `parts.py` differs from the kit's in one line: a 4th colour component sets a
surface's opacity, used for the cup's see-through walls (appearance only). `near_misses.py` lists near-successes and wrong outcomes with the score each
must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Empty the vacuum cleaner's dust cup into the bin and put the cup back on the vacuum."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/cup-dream

# Scripted demo (reaches 1.0 in about 30 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/cup-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 90 s of simulated time (1,800 steps at 20 Hz), about 2 to 3 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`.
- **Layout:** one fixed layout. There is one bin and one tray, so no positional choice for a habit to win and no
  layout lots; a memorised replay of the demo is not ruled out.

## How the mechanisms are modelled

The vacuum body is a fixed part with one slider, the catch (tongue and post, 22 mm travel, spring 40 N/m). The cup is
a loose part with two links: the bottom flap (a hinge at its far edge, 0 to 1.2 rad, spring 1.0 N m/rad) and the
release button (a slider, 6 mm down, spring 200 N/m). A `parts.on_step` callback runs every control step:

- while the button is pressed 4 mm or more the flap's spring target is 1.0 rad (it swings open); otherwise 0 (shut);
- once the flap is within 0.035 rad of shut with the button let go, its upper limit drops to 0.035 rad (latched)
  until the button is pressed again, so the balls' weight cannot open it;
- the catch's tongue is pulled back (spring target 22 mm) while a cup that came from above is within 30 mm over the
  tongue and within 10 mm of the tray's centre: this stands in for the tongue's sloped face. Once the cup's rim is
  below the tongue the target goes back to 0 and the tongue springs over the rim. A cup that is docked and caught
  is held by the tongue itself; only pushing the post frees it.
- The tray's 15 mm walls keep the cup from being slid out sideways under the tongue.

## Scoring details the simulator applies

- **Let go:** the callback measures the shortest distance from both arms' finger and gripper collision hulls
  (yam_arena.analysis) to every shape of the cup (body, flap, button), the catch link and the three balls; it must
  exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this counts as clear.
- **Still:** cup and balls under 1 cm/s and 0.5 rad/s (`ObjectState.resting()`); success must hold 1 s.
- **Spill record (whole trial):** set if any ball, while outside both the cup's walls and the bin's walls, has its
  underside within 3 mm of the table top or the tray floor, is below the table top, or is still anywhere. It is
  cleared only by a scene reset: every ball back within 0.5 mm of its start and at rest, after a jump of over 20 mm
  in one control step (a robot cannot move them back that way).
- The cup frame is the middle of its underside; "in the cup" is tested in that frame, so a tipped cup still counts.
- A ball counts as in the bin only if it is also out of the cup. A cup dropped upside down into the bin, its dirt
  still on the grille, scores 0 (a near-miss case).

## Scripted demo

`demo()` uses both arms. The left arm puts its closed fingers beside the catch's post and pushes it 22 mm to the
left, holding it there while the right arm, fingers closed on the handle bar, lifts the cup 35 mm out of the tray;
then the left arm rises (the catch springs back) and parks. The right arm carries the cup over the bin, sets its
flange on the bin's rim and lets go, presses the red button for 2 s with closed fingers (the flap swings open and
the balls drop), lets go of it (the flap springs shut and latches), grips the handle again and carries the cup back.
It lowers the cup into the tray, aiming 6 mm farther and 3 mm to the left of the tray's centre to make up for the
loaded arm's measured lag, and lets go 6 mm up. The demo reads nothing from the simulator.

## Measured in simulation

- Demo trace (trace-20261109T002905, before the see-through walls, wider tongue and orange balls; the final demo
  demo-20261109T011047 succeeded the same way): the cup lifted out with the catch held back, the three balls fell into the bin
  when the button was pressed (score 0.5 from 20 s), and the cup docked 2 mm from the tray's centre with the catch
  back over its rim and the flap shut; success at 29.5 s.
- Earlier traces found: the button sagging under its own weight with a weak spring; the two arms' forearms touching
  when the catch post was 13 cm from the handle (the post is now 17 cm away); and the cup perching on the tray's
  front wall when it landed 10 mm short (the tray now has 8 mm of play front and back and the demo aims for it).

## Other ways to succeed, and what scores 0

- **Also full score:** holding the cup over the bin instead of resting it on the rim; either arm doing either job.
  Tipping the cup upside down does not empty it: the grille's gaps are 14 mm and the balls 16 mm, so the flap has
  to be used.
- **0.5:** emptying correctly and leaving the cup on the bin, on the table, or in the tray without the catch over its
  rim.
- **0.25:** lifting the cup out and setting it down upright with the dirt still in it.
- **0:** dropping the whole cup into the bin, upright or upside down; pressing the button with the cup in the tray or over the table (the balls land outside the bin: spilled for
  the rest of the trial, even if they are picked up later); dropping a ball; leaving the cup on the floor or tilted
  more than 10 degrees with the dirt in it; still holding the cup.

The cases in `near_misses.py` cover the catch (1 mm and 2 mm out), the flap (3 and 4 degrees), the cup's height
(1 and 2 mm), sideways and front-to-back offsets at and past their limits, a tilted or moving cup, a rolling ball,
every partial tier, spills onto the table, tray, body, bin rim and floor (each followed by a clean finish that still
scores 0), a ball in flight (no spill) and scene resets.

## Recorded runs

- Scripted demo: 1.0 (success at about 30 s).
- Dream-1, 1,800 steps: 0.0. Both arms came down near the vacuum and stayed there; the cup never left its tray
  and the run hit the step limit.
