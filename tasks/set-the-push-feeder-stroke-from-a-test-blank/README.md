# Scale a test shot to the press lane to set a spring kicker's stroke

A blank under a press guard must be fed by a spring pusher so it coasts into the die nest: too little stroke and
it stops short under the guard, too much and it slides off the end into the scrap bin. How far it coasts depends
on the batch's friction, which cannot be seen. An open test slide beside it, with the same pusher and marks, holds
a test blank from the same batch, but the press lane drags 1.5 times as much as the test slide (stated in the
instruction and on a plate reading 1.5). A stroke that lands on the test slide falls short on the press lane, so
the robot has to scale its test result rather than copy it.

Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene.

## Files

- `task.py`: scene, scoring and scripted demo.
- `parts.py`: multi-colour parts and spring sliders (the lanes and pushers). It patches yam_arena's scene and
  embodiment at import, in this process only; nothing is changed on disk.
- `lots.py`: layout lots, trial k uses lot k mod 3 (it gives the embodiment a `prepare_trial` that counts trials).
- `near_misses.py`: offline score checks.

## Run

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/set-the-push-feeder-stroke-from-a-test-blank/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Use the spring pusher to feed the blue blank under the guard so it stops in the green nest. A test blank from the same batch is on the left slide; the press lane drags 1.5 times as much." \
      --scorer task_score --max-steps 1200 --store-frames --epochs 3

Use `--policy yam_scripted` (640 x 360 cameras) for the demo. `--epochs 3` runs lots A, B and C and reports the
mean `task_score`; without it only lot A runs, and one fixed stroke can pass it. `TASK_LOT=A|B|C` in the
environment pins every trial to one lot. Submissions use `-E object_jitter=0`. A trial is at most 60 s of
simulated time (1,200 steps); the demo finishes in about 17 to 22 s per lot. Seeds: the default seed; with
jitter off nothing in the start is random.

Offline: `python near_misses.py` (from this folder) scores every near miss.

## Lots

The lots change only the batch. An `on_reset` hook in `task.py` sets each blank's friction for the lot through
the PhysX material view (Isaac Lab's `root_view.set_material_properties`, as its own
`randomize_rigid_body_material` event does). PhysX combines it with the lanes' 0.10 as the mean, and the press
blank gets a value that makes its sliding friction exactly 1.5 times the test blank's: test slide / press lane,
lot A 0.18 / 0.27 (dry), lot B 0.08 / 0.12 (oiled), lot C 0.12 / 0.18 (lightly oiled). The blanks look alike.

Measured on the GPU: the blank coasts about 28.6 * stroke^2 / friction metres (stroke in m). The nest needs 20.8
to 28.8 cm of travel (24.8 cm centred). Each mark is 1.225 times the last, so it sends a blank 1.5 times as far,
and a stroke anywhere on the right 3 mm tick still lands (within 12 % of the centred distance; the nest allows
16 %). On the press lane each lot has one right mark: A purple (4), B orange (2), C red (3). Coast on the press
lane, in cm (bin = slides off the lane end):

| stroke | lot A (0.27) | lot B (0.12) | lot C (0.18) |
|---|---|---|---|
| mark 1, 26.3 mm | 7 | 16, short | 11 |
| mark 2, 32.3 mm | 11 | 24.8, in (measured) | 17, short |
| mark 3, 39.5 mm | 17, short | bin | 24.9, in (measured) |
| mark 4, 48.4 mm | 24.2, in (measured) | bin | bin |

So a fixed mark passes at most one lot in three (task_score 0.33 or 0). Copying the mark that lands on the test
slide always falls short on the press lane (16.5 cm in lot A of the demo's test, the same law). The test slide's
landing mark is one lower than the press lane's: A 3, B 1, C 2.

## Demo

`demo()`: the left arm grips the test slide's knob, pulls it to the orange mark (2), lets go and lifts clear; the
test blank stops short (lot A, 16.5 cm), in the nest (lot C) or goes into the bin (lot B). The right arm then pulls
the press lane's knob to one mark above the test slide's landing mark: from the test, the landing mark is one
higher if it stopped short, the same if it landed, one lower if it went past; the press lane's 1.5 then adds one
more. The demo's choice is precomputed per lot (from `lots.current()`), matching what its test shot shows in that
lot; it does not read friction or joint states at run time. The soft arm gives way under the spring, so the demo
commands more pull than the mark (calibrated on the GPU: 38.5, 47.1, 57.2 and 69.1 mm for the four marks; the
knob reached 32.2 to 32.4, 39.4 and 48.2 mm).

## Scoring in the simulator

- Barrier: the slotted roof's 10 mm slot is narrower than a finger, so the nest cannot be reached by hand.
- Light curtain (backup): an `on_step` callback checks every control step whether any vertex of either arm's link hulls
  (links 1 to 5, gripper, finger tips) is beyond x = 0.48 m with y between -0.19 and -0.05 m. Once crossed, the
  trial scores 0; the record is cleared only by the simulator's reset for a new trial.
- Released: the same callback measures the distance from both grippers and their finger tips to the press blank;
  success needs more than 5 mm.
- The blank's true footprint (its box turned by its yaw) must lie within the nest, underside within 1.5 mm of
  the floor top, tilt at most 10 degrees, at rest (`resting()`), held for 1 s (`SUCCESS_HOLD_S`).
