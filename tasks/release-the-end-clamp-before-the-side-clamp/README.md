# Release the end clamp before the side clamp so the blank reaches both locators

A light acetal plastic blank lies loose on a heavy fixture plate, 15 mm short of the end locator and 8 mm short of the side
locator. Two identical yellow spring pushers are held back by latches: one behind the blank's short end, one beside
its long side. Each is released by the red button on top of its own housing. The robot is asked to clamp the blank so
it sits against both locators. Released first, the end pusher slides the blank onto the end locator; the side pusher
then slides it sideways onto the side locator. Released first, the side pusher presses the blank onto the side
locator, friction pins it, and the weaker end pusher can no longer move it: the blank stays 15 mm short of the end
locator. The instruction does not give the order. Two layout lots turn the fixture end for end, so the button to
press first is not always the one nearest the robot, the midline or either arm.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (the sliding pushers and buttons) and `lots.py` (the layout
lots), both in this folder; it imports only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` (in lot A) and need
no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable. Always pass `--epochs 2` (or a multiple
of 2): trial k uses lot k mod 2 (lot A first), and the task's score is the mean over the trials. With the default
`--epochs 1` only lot A runs.

```bash
I="Clamp the blank in the fixture so it sits against both locators."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/clamp-dream

# Scripted demo (reaches 1.0 in about 10 s in lot A and 10 s in lot B)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --epochs 2 --store-frames --log-dir ~/yam-eval/clamp-demo
```

- To run one lot only, put `TASK_LOT=A` or `TASK_LOT=B` in front of the command.
- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is two trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`; jitter could start the blank against a locator.
- The videos and screenshot of a multi-trial run are the first trial's, so they show lot A.

## Lots

- **A** (the screenshot): fixture plate centred at (0.295, 0.165) m; end locator face at x 0.340, side locator face at
  y 0.190; end pusher's button at (0.200, 0.162), side pusher's at (0.295, 0.097).
- **B:** the fixture turned 180 degrees about its centre, the centre moved to (0.289, 0.165), and the blank moved with
  it to (0.299, 0.168). The end locator then faces away from the robot and the side locator faces left; the end
  pusher's button is at (0.384, 0.168) and the side pusher's at (0.289, 0.233). B's 6 mm shift keeps the blank's new
  start clear of lot A's end locator while `lots.py` moves the parts. In the simulator the moved fixture settles
  about 1.6 mm nearer the robot and 1.2 mm right of that centre (measured in traces), so the fixture-moved rule
  measures from where the fixture stands at the trial's first step.

Habits and their mean score over A and B (each picks the first button by position): nearest the robot 0.5 (right in
A only), nearest the midline 0.5 (B only), nearest the left arm 0.5 (A only), nearest the right arm 0.5 (B only),
leftmost 0.5 (A only). Both pushers and buttons look the same, so no colour habit exists. Replaying lot A's press
positions in lot B presses nothing (both points fall beyond the turned plate's locators): 0. Checked offline with
`lots.use("B")`: clamped in B scores 1.0; pinned short in B, a blank at lot A's clamped spot, and a fixture shoved
6 mm all score 0.

## How the clamps work in the simulator

- The fixture is one loose `parts.Part` of 5 kg (plate, locators and housings), so that the lots can turn it. Each
  pusher is a `parts.Slider` on it with a spring (end 80 N/m, side 280 N/m, 40 mm of travel). Latched, its spring is
  aimed at its retracted stop. A `parts.on_step` callback watches each button (a 10 g slider pressed down against a
  50 N/m spring, 8 mm of travel); once a button is pressed more than 5 mm, it aims that pusher's spring at its far
  end with `set_target`, and the pusher stays released until the scene resets (`on_reset` re-latches both).
- Where each meets the clamped blank, the end spring pushes about 1.6 N and the side spring about 7.6 N. The blank
  (0.10 kg, acetal plastic; a solid acetal blank this size is about 0.11 kg) needs about 1 to 1.2 N to slide on
  the plate. Released first, the side pusher's 7.6 N on the blank's side
  gives far more friction (about 18 N at the side locator and head) than the end pusher can overcome; released after
  it, the side pusher overcomes the friction of the end pusher's 1.6 N (about 5 N in all).
- All scoring is done in the fixture's own frame (from its measured pose), so it is the same in both lots.

## Scoring details the simulator applies

- **Still:** the blank moved less than 0.5 mm over the last 1 s (20 control steps). Pressed against a locator by the
  side pusher, the simulated blank reports a steady contact velocity of about 1.3 cm/s while not moving at all, so
  the usual velocity test (`ObjectState.resting()`) would never pass. Offline (`near_misses.py`) the velocity test is
  used.
- **Against a locator:** the four corners of the blank's far end (true corners, from its pose) lie between 1 mm
  short of and 1 mm past the end locator's face; the same for its locator-side long face and the side locator.
- **Flat:** its lowest corner within 1 mm of the plate's top.
- **Pushers:** each released by its button (the latch record; offline, a pusher counts as released when it is more
  than 2 mm off its stop), with its face within 1 mm of the blank. A pusher pushed forward by hand while latched
  does not count.
- **Let go:** the callback measures the shortest distance from both arms' finger and gripper collision hulls to the
  blank and both pusher heads; success needs more than 5 mm. Offline this check counts as clear.
- **Fixture never moved:** a record kept over the whole trial: once the fixture's body has moved more than 5 mm, or
  turned or tipped more than 3 degrees, from where it stood at the trial's first step, the trial scores 0 even if it
  is put back. Only a scene reset clears it.

## Scripted demo

`demo()` drives the left arm only, fingers closed. It finds the two buttons from the fixture's pose in the trial's
lot (what the cameras show), comes down on the end pusher's button, presses it (commanded 4 mm past full travel;
the arm is soft), lifts, waits 1 s while the blank slides onto the end locator, then presses the side pusher's
button and moves clear. With `CLAMP_SIDE_FIRST=1` in the environment it presses the side button first (counter-test
only).

## Measured in simulation

- **Demo traces** (trace-20261109T000121, both lots): 1.0 in lot A (from step 200) and lot B (from step 195 of its
  trial). After the end button the end pusher moved 20 mm and the blank 15 mm onto the end locator; after the side
  button the side pusher moved 13 mm and the blank 8 mm onto the side locator. The fixture did not move during either
  trial.
- **Wrong order** (trace-20261109T000124, `CLAMP_SIDE_FIRST=1`, both lots): 0 in each. The blank went onto the side
  locator, the end pusher moved 3 to 5 mm to the blank and stopped, and the blank stayed 15 mm short of the end
  locator.

## Other ways to succeed, and what scores 0

- **Fallback that also scores 1.0:** holding the blank against the end locator by hand while releasing the pushers,
  as long as the robot then lets go. The clamps alone seat the blank only in the right order.
- **Score 0:** the wrong order (blank pinned short of the end locator); both buttons at once (the side pusher wins);
  a pusher still latched; a pusher pushed by hand instead of released; the blank lifted off the plate or turned
  against a locator; the robot still holding it; the fixture shoved more than 5 mm or turned more than 3 degrees at
  any time.

## Recorded runs

- Scripted demo (`--epochs 2`): 1.0 in lot A and 1.0 in lot B; task score 1.0.
- Dream-1 (`--epochs 2`, 1,200 steps per trial): 0.0 in both lots. Both arms moved toward the fixture without
  pressing either button (judged from the five-frame contact sheet; the run logs no object states).
