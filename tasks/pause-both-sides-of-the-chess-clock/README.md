# Pause a mechanical chess clock without starting the other player's clock

A mechanical chess clock stands between the arms with one player's button raised, so that player's clock runs. The
robot must pause the game: both clocks stopped, which on a seesaw-rocker clock means both buttons level. Pressing
the raised button fully, the obvious move, stops that clock but starts the other player's.

Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. The task is
`task.py` plus two helper files that must stay next to it: `parts.py` (multi-colour parts with hinges: the rocker
and the two minute hands), `lots.py` (per-trial start layouts) and `tilt.py` (leaned grips for the scripted demo). They patch the stock embodiment in the task's own
process only; nothing on disk changes. No third-party assets are used.

## Run and score

    inspect-robots run --embodiment yam_arena -E task=tasks/pause-both-sides-of-the-chess-clock/task.py \
        -E object_jitter=0 --instruction "Pause the chess clock so neither player's time runs, without starting the other player's clock." \
        --policy <policy> --scorer task_score --max-steps 1200 --epochs 2 --store-frames

- `--epochs 2` is required: trial k uses lot k mod 2 (A: left button up; B: right button up) and `task_score` is
  their mean. With one epoch only lot A runs. `TASK_LOT=B` pins every trial to one lot.
- `--policy yam_scripted` plays the demo. Runtime about 1 minute per trial on an L40S. `object_jitter=0`.

## Scoring, exactly

Every control step, from the simulator's state (cap height = 50 mm x sin(rocker angle) above or below level):

- Paused: both caps within 1.5 mm of level (rocker within asin(1.5/50) = 1.72 degrees), plus 1e-6 rad. Both
  clocks are then stopped; above the band the raised side's clock runs, below it the other side's.
- Credit needs: both arms' fingertips and gripper bodies more than 5 mm from every shape of the clock (case,
  rocker, caps, hands), from the arms' collision hulls; the rocker turned less than 0.1 degree in the last control
  step; the clock at rest. Until the first step is measured these count as not met (no credit).
- Whole-trial faults, any one zeroes the trial until a scene reset (the simulator's reset hook): the other side's
  clock ever runs (rocker past the band toward the side that started down); the clock ever more than 10 mm from its
  start or tipped more than 5 degrees.
- Pass/fail: `score()` is 1.0 only once the pause has held unbroken for 2 s (the step hook counts the held time;
  any break resets it), otherwise 0. The episode's success hold is 2.1 s, so it ends just after `score()` credits it.
- Release check fails closed: all six arm bodies (two fingertips and the gripper on each arm) must be found.

Mechanism: the rocker is a hinge with stops at +-6.9 degrees (caps 6 mm above or below level). Its holding drive
(0.5 N m/rad) keeps a rest point that follows the rocker only once it is pushed more than 0.2 degrees past it, so
the rocker stays where it is left, like the friction in a real lever train. Each step the running side's minute
hand is turned at one turn an hour.

`near_misses.py` has 22 offline cases: band limits on both sides, half pressed, still sliding, the naive full
press, a press briefly past level then brought back, the clock shoved or tipped. In lot B the mirrored cases score
the same (checked offline).

## Demo

`demo()`, the same plan in every trial: both arms at once bring their closed fingertips down onto their own caps,
leaned 30 degrees toward the midline (with `tilt.py`, which only affects the scripted demo) so the elbows stay clear,
and stop at the height a level cap's top has. The raised cap is pushed down and the low one rises against the other
hand until both are level; both hold, then lift away together. It never reads which side is up.
