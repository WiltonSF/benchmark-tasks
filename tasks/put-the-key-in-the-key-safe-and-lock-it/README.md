# Put the key in a combination key safe and lock it

A combination key safe stands open on its code. The robot is asked to put the key in and lock it. Shutting the lid
does not lock it: a shut safe on its code lifts open. The robot has to put the key in, shut the lid, and then turn
at least one wheel off the code. Turning a wheel first throws the bolt, and the lid then stops 9 degrees short of
shut. The instruction says nothing about the code.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts, joints and step hooks) and `lots.py`
(layout lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes
and wrong outcomes with the score each must get. They are plain-Python checks of `score()` and `success()` and need
no simulator.

## Lots

Lot A (the screenshot): lid open 109 degrees, key 235 mm in front of the arm bases and 170 mm left of the midline.
Lot B: lid open 90 degrees, key 335 mm in front and 150 mm left. Lot C: lid open 100 degrees, key 160 mm in front
and 90 mm left. Trial k uses lot k mod 3, lot A first, so a scored run needs `--epochs 3` (or a multiple).
`TASK_LOT=A`, `B` or `C` in the environment pins every trial to one lot. Lots can only move loose objects, so a reset
hook (`parts.on_reset`) sets the lid's angle and puts every wheel on the code. Then `lots.py` settles the scene for
0.5 s before the first frame. A scene reset is the only way this happens; the robot cannot trigger it. The lots stop
one fixed script from passing every lot (it grips air where lot A's key was). They do not stop a memorised replay of
all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the key in the key safe and lock it."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 2400 --epochs 3 --store-frames --log-dir ~/yam-eval/keysafe-dream

# Scripted demo, all three lots (each reaches 1.0 in about 27 s; the run stops at success)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1000 --epochs 3 --store-frames --log-dir ~/yam-eval/keysafe-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 120 s of simulated time (2,400 steps at 20 Hz), about 2 to 4 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## How the safe works in the simulator

The safe is one fixed `parts.Part` with four hinges: the lid (about the left-right axis along the back top edge,
0 shut to 1.92 rad) and three wheels (about left-right axes, -3.1 to 3.1 rad). The kit's moving links do not collide
with their own body, so a joint limit is the lid's shut stop. A `parts.on_step` hook runs every control step:

- **Wheel clicks:** each wheel's drive (0.15 N m/rad) pulls it to the nearest multiple of 36 degrees, so a wheel
  pushed past 18 degrees springs on to the next number and one pushed less springs back.
- **Stiff lid hinge:** the lid's drive (3 N m/rad) follows it with a 0.02 rad lag, so it stays where it is left;
  within 2 degrees of shut it is pulled onto its seat.
- **Bolt:** a wheel is off the code when its tooth is 18 degrees or more from straight up. With every wheel on the
  code the lid's range is 0 to 1.92 rad. With a wheel off the code and the lid shut (2 degrees or less), the range
  becomes 0 to the lid's angle at that moment (at least 0.001 rad): locked. With a wheel off the code and the lid
  open, the bolt stands out: the lower limit becomes 0.15 rad (9 degrees), or the lid's angle if it is already below
  that, but never less than 2 degrees + 0.02 rad (3.1 degrees). That limit is fixed when the bolt first meets the lid
  and is not lowered again until every wheel is back on the code, so a lid open when a wheel was turned can never be
  pressed into the locked range. `near_misses.py` replays this rule step by step at import (lid 2.5 degrees open,
  wheel turned, lid pressed to 0: the limit holds at 3.1 degrees; wheels back: free; shut, then turned: locked).

## Scoring details the simulator applies

- **Inside:** all 16 corners of the key's two boxes (fob and blade, from its pose) inside the compartment's inner
  outline (72 x 80 mm) and no higher than the box's rim. A key on the lid, on the wheel strip or poking out fails.
- **Shut:** the lid's hinge at 2 degrees or less (1e-6 rad margin; the limits are pinned in `near_misses.py`).
- **Off the code:** some wheel's angle, wrapped to plus or minus 180 degrees, at 18 degrees or more.
- **Still:** the key under 1 cm/s and 0.5 rad/s (`ObjectState.resting()`), and the lid and every wheel turning under
  0.005 rad per control step.
- **Let go:** the hook measures the shortest distance from both arms' finger and gripper collision hulls
  (`yam_arena.analysis`) to the lid, the three wheels (with their teeth) and the key. All must exceed 5 mm. Offline
  (`near_misses.py`) there is no robot, so this counts as clear and still.
- There is no partial credit. A shut safe left on its code is the failure the task is about, so it scores 0, and so
  does the key alone in an open safe.

## Scripted demo

`demo()` uses both arms and reads only the trial's lot (where the cameras show the key). The left arm grips the fob
from above (fingers 2 mm into its 20 mm sides), lifts it, carries it in front of the lid and over the compartment,
lowers it and lets go with the fob 8 mm above the floor, blade toward the back, then parks. The right arm then lowers
a closed fingertip behind the open lid and pushes it forward to about 40 degrees, lifts, presses the lid down shut
from above, and pushes the middle wheel's tooth away from the robot with the fingertip 3 mm above the rim. The
wheel clicks on to the next number (36 degrees) and the arm parks.

## Other ways to succeed, and what scores 0

- **Also full score:** any arm, any wheel or several wheels turned either way, the key in any pose inside.
- **Score 0:** shutting the lid and stopping; turning a wheel before shutting (the lid stops at 9 degrees); leaving
  the key on the table or on the lid; holding the lid or a wheel at the end; anything still moving.

The cases in `near_misses.py` cover the start, the key inside with the lid open, shut on the code, each wheel off in
either direction, the half-number limit and the 2-degree shut limit on both sides, the bolt-held lid, the key outside
(on the lid, on the wheel strip, 1 mm past the back wall), the key turned or on its side inside, and a moving key.

## Recorded runs

- Traces, `--epochs 3`: an early layout scored 1.0 in one lot only (trace-20261109T081118: the lid at 60 degrees
  hung over the opening and the key was knocked out; a lid lip stopped the wheel between numbers). After the fixes:
  1.0 in each lot (trace-20261109T082041), and lot B again after a path change (TASK_LOT=B).
- Wrong order, not for submission (a copy whose right arm turns the middle wheel before shutting the lid, lot A):
  the wheel clicked to 36 degrees, the bolt stopped the lid at 0.15 to 0.16 rad (9 degrees) and the trial scored 0
  (trace-20261109T083159 of that copy).
- Bolt near shut, not for submission (a copy that starts the lid 4 degrees open, turns the middle wheel, then presses
  the lid hard): the lid stayed at 0.057 rad (3.3 degrees) under the press and the trial scored 0
  (trace-20261109T152951 of that copy).
- Before the review fixes: scripted demo, `--epochs 3` (demo-20261109T083846): 1.0 in lots A, B and C (task_score 1.0).
- Before the review fixes: Dream-1, 2,400 steps, `--epochs 3` (dream-20261109T084527): 0.0 in every lot. In lot A the left arm picked the key
  up and carried it toward the safe but did not put it in; the lid stayed open and no wheel turned.
- After the review fixes (bolt held where it first meets the lid; key corners tested): scripted demo, `--epochs 3`
  (demo-20261109T153133): 1.0 in lots A, B and C (task_score 1.0). Dream-1, 2,400 steps, `--epochs 3`
  (dream-20261109T153917): 0.0 in every lot; in lot A the left arm reached the key and pushed it a little on the
  table but never lifted it, the lid stayed open and no wheel turned.
