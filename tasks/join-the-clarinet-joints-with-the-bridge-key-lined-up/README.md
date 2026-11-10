# Join a clarinet's upper and lower joints with the bridge-key levers lined up

The two middle joints of a clarinet: the lower joint stands socket up on a stand, with a blue bridge-key lever pad
on its side between two key posts; the upper joint stands tenon down in a holder, with its own lever hanging below
its shoulder. The robot is asked to join them. The instruction does not say that the levers must line up. The
tenon is round and unkeyed, so the joint goes in at any angle, but only one angle works:

- levers within about 12 degrees: the upper pad passes between the key posts and stops 2 mm above the lower pad,
  and the joint seats;
- about 13 to 75 degrees off: the upper pad lands on a key post and the joint stands proud of the rim (12 mm if it
  stays upright; in a trace it leaned over and stood 5 mm proud);
- further off: the joint seats, but the levers miss each other, so the keys would not work.

A seated joint cannot be twisted into line: the hanging pad runs into a post. It has to be lifted and set in again.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in
this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

Three layouts turn the two levers. Lot A (the screenshot): the lower lever points away from the robot, the upper
lever to the robot's left. Lot B: lower lever left, upper lever away. Lot C: lower lever right, upper lever back
toward the robot, 30 degrees to the right. The turn needed is -90, +90 and +60 degrees, and the lever's final
direction is different in each lot, so these habits each pass at most one lot: no turn, a fixed turn, a fixed final
direction, or replaying one lot's demo. Trial k uses lot k mod 3, lot A first, so a run needs `--epochs 3` (or a
multiple); `TASK_LOT=A`, `B` or `C` pins every trial to one lot. The lots stop a single fixed habit, not a memorised
replay of all three layouts. The lower joint is a loose 3 kg part only so the lots can turn it.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Join the upper joint of the clarinet to the lower joint."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/clarinet-dream

# Scripted demo, all three lots (each reaches 1.0 in about 14 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 3 --store-frames --log-dir ~/yam-eval/clarinet-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`), for both joints;
  success must hold 2 s (`SUCCESS_HOLD_S`), so the arm has backed off when a trial ends.
- **Home:** the upper joint's shoulder centre (its part origin) within 2 mm above or below the rim's centre
  (the lower joint's frame, so a slightly tilted stand is followed), within 5 mm of it horizontally, and the upper
  joint tilted at most 5 degrees.
- **Lined up:** each lever's direction is the part's own x axis seen from above; the angle between the two is at
  most 12 degrees.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures
  the shortest distance from their collision hulls (yam_arena.analysis) to every shape of the upper joint; it must
  exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the lower
  joint is more than 10 mm from its start or tilted more than 5 degrees. It is cleared only by a scene reset: the
  upper joint back within 0.5 mm of its start and at rest, after a jump of over 20 mm in one control step. A robot
  cannot move it back that way.

## Scripted demo

`demo()` drives the left arm only. It grips the upper joint round its body (fingers open 70 mm, closing 3 mm past
the 50 mm body's faces, tips 11 mm down from its top), lifts it out of the holder until the tenon's end is 9 mm
over the rim, carries it over the socket while turning the grip by the angle between the two levers (half before
and half after, to stay within the wrist's reach), lowers it over 3 s to 4 mm above seated, opens, rises 30 mm and backs
off 16 cm toward the robot and 18 cm to its left, so the top camera sees the lined-up levers at the end. The levers' directions come from the trial's lot, which the cameras show; the demo reads nothing from the
simulator. The carry target is 2.5 mm farther out than the socket, because the held joint lagged about 3 mm toward
the robot while lowering.

## Measured in simulation

- Final traces (`--epochs 3`): 1.0 in each lot; the joint seated with the shoulder on the rim, axes 1 to 2 mm
  apart, levers 0.3 to 1.4 degrees apart, and success held only after the fingers had opened.
- An earlier version with 3 mm of clearance and no lag allowance jammed the tenon's end on the rim in two lots.
- No-turn habit (the same demo with the turn removed): see "Recorded runs".

## Other ways to succeed, and what scores 0

- **Also full score:** either arm, any grip, turning before, during or after the carry, lifting and re-seating.
- **Score 0:** seating with the levers apart (the joint goes in, the keys don't connect); landing the pad on a key
  post (standing proud); leaving the joint in the holder, on the table, upside down on the rim or held in the socket;
  shoving the lower joint more than 10 mm or tipping it more than 5 degrees at any time.

The cases in `near_misses.py` cover the 12-degree, 2 mm, 5 mm and 5-degree limits on both sides, levers 90, 120
and 180 degrees apart, a joint proud on a post, still moving, lying on the table, upside down, beside the lower
joint, the lower joint shoved 11 mm (and put back) or tipped 6 degrees, a 9 mm nudge (no fault), and scene resets.

## Recorded runs

- Scripted demo, `--epochs 3`: 1.0 in each of lots A, B and C (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 3`: 0.0 in all three lots.
- No-turn habit (the demo with its turn removed, `--epochs 3`, not for submission): 0.0 in every lot. In lots A and
  B the joint seated with the levers 90 degrees apart; in lot C (55 degrees off) the pad landed on a key post and
  the joint stood 5 mm proud, leaning 17 degrees.
