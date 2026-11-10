# Push the container into bay 2 of the roller deck and lock it between the raised locks

A model ULD container sits on a roller deck with three bays. The deck slopes 3 degrees down toward the robot's left
(bay 1), so the container rolls that way unless a raised lock holds it. Between the bays stand two yellow pop-up
locks, each worked by a pedal off the deck's near edge with a push-push catch (press fully to drop, press fully
again to raise). The robot is asked to stow the container in bay 2 so it cannot roll back. From bay 1 it has to drop
the 1|2 lock, push the container (too wide to grip) up the slope into bay 2, hold it there and raise the lock behind
it with the other hand; let go before the lock is up and the container rolls back into bay 1. From bay 3 it lets the
container roll down into bay 2 and raises the lock behind it. Any lock that starts down must be raised as well.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts, the sliding locks) and `lots.py` (layout
lots), both in this folder; they import only NumPy and `yam_arena`. This copy of `parts.py` adds one field to the
kit's `Part`, `roll` (a tilt about the x axis), used for the sloping deck. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

- Lot A (the screenshot): container in bay 1 (the low end, robot's left) against the end stop, both locks up.
- Lot B: container in bay 3 (the high end) resting on the 2|3 lock, the 1|2 lock down.
- Lot C: container in bay 1, the 2|3 lock down.

Trial k uses lot k mod 3, lot A first, so a scored run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C`
pins every trial to one lot. Each lot starts a different set of locks down. Every full press toggles a lock, so a
fixed routine of pedal presses that ignores the scene leaves both locks up in at most one lot. `near_misses.py`
checks this offline with `lots.use()`: for every fixed count of presses per lock, it scores the best possible end
(the container stowed in bay 2) in each lot, and asserts at most one 1.0. The blind two-sweep routine (drop and raise
the 1|2 lock, then drop and raise the 2|3 lock) can score in lot A only, a mean of at most 0.33. The lots stop a fixed
routine, not a memorised replay of all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Stow the container in bay 2 of the roller deck so it cannot roll back."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/uld-dream

# Scripted demo, all three lots (1.0 in about 21 s in lots A and B, 28 s in lot C)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir ~/yam-eval/uld-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- Positions are judged in the deck's own frame (sloping 3 degrees), from the container's pose.
- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold 1 s.
- **In bay 2:** the container's centre within 15 mm of the bay's centre along the deck and within the guide rails'
  10 mm play across it, its underside centre within 2 mm of the roller tops, tilted at most 5 degrees from square to
  the deck.
- **Locks up:** each lock's slide position (0 at the top, 30 mm at the bottom of the pedal's travel) at most 3 mm.
- **Released:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the container and of both
  locks; it must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this counts as clear.
- **Lift fault (whole trial):** set, and the score is 0 for the rest of the trial, if the container's underside
  centre is ever more than 10 mm off the roller tops. A trace pushing the container hard into a raised lock tipped
  it onto its side over the lock: the fault caught it. The record is cleared only by a scene reset: the container
  back within 0.5 mm of its start and at rest, after a jump of over 20 mm in one control step.
- **The locks:** each is a sprung slide (40 N/m, 12 g). An `on_step` callback runs the push-push catch: pressed past
  27 mm, a lock toggles between "held up" (spring target above the top stop) and "held down" (target 20 mm; it rests
  about 23 mm down, its top 3 mm below the deck); it must come back above 25 mm before it can toggle again. While a
  lock is set up and within 2 mm of its top, the callback limits its slide to 1 mm, and lifts that limit only while
  the pedal is being pressed: a finger or gripper surface within 2 mm of the pedal and the pedal pushed down at least
  0.7 mm against the hold. Without the hold, an early trace showed a shoved container camming a raised lock down and
  sliding over it. Each lot's down locks are set at reset by the same catch.

## The demo

`demo()` uses only what the top camera shows: which bay the container is in and which locks are down. Lot A: the
left arm presses the 1|2 pedal to drop the lock, pushes the container's rear face up the rollers with its closed
fingertips (25 mm above the rollers, over the low end stop and the locks) until it rests on the 2|3 lock, and holds
it there; the right arm comes in low and presses the 1|2 pedal to raise the lock behind it; the left arm eases off
and the container rolls back 12 mm onto the raised lock. Lot C first raises the 2|3 lock with the right arm. Lot B:
the left arm raises the 1|2 lock, the right arm drops the 2|3 lock, the container rolls down into bay 2 against the
1|2 lock, and the right arm raises the 2|3 lock behind it. No gripping. A trace that skipped raising the 1|2 lock in
lot A ended with the container rolled back into bay 1 (score 0).

## Checks

`near_misses.py` covers: stowed against either lock; a lock behind or ahead down, both down, a lock held 4 mm down,
3 mm down (passes), half up; still rolling; 15 and 16 mm off the bay's centre; turned 4 degrees; left in bay 1, or
rolled back past a dropped lock; pushed on into bay 3 and locked there; straddling a dropped lock; tilted 6 degrees;
pushed off the deck; resting 3 mm high; carried 9 mm over a lock (no fault) and 11 mm (a fault that a later stow does
not erase); tipped onto its side; a scene reset; and the blind-routine check above.
