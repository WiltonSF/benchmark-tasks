# Discharge an extinguisher at a burning bin: pull its pin, aim the horn, squeeze

A small extinguisher stands in a floor stand. Its squeeze lever is blocked by a safety pin through two lugs; its
horn swivels and starts pointing straight ahead, at nothing. A burning red waste bin stands 40 degrees to one side
of the horn's line and a blue recycling bin 40 degrees to the other. The instruction does not mention the pin.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-colour parts with hinges) and `lots.py` (layout lots) must sit next to `task.py`; `task.py`
imports both.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable. Always pass `--epochs 2` (or a multiple
of 2): trial k uses lot k mod 2 (A, then B), and with the default single epoch only lot A runs.

```bash
I="Put out the fire in the waste bin with the extinguisher."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir logs/demo
```

- One lot only: put `TASK_LOT=A` or `TASK_LOT=B` in front of the command.
- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time per
  trial on an L40S once Isaac Sim has started. The score is the mean over the trials.
- **Seeds:** submitted runs use `-E object_jitter=0`. The videos and screenshot are the first trial's (lot A).

## Lots

Lot A: red waste bin on the left mark (46.2 cm in front, 10.1 cm left), blue bin on the right mark (46.2 cm in
front, 22.1 cm right). Lot B swaps the bins. `lots.py` moves the bins after the stock reset at the start of each
trial; nothing else changes. A fixed swing direction scores 1.0 in one lot and 0 in the other (mean 0.5). Lots do not
rule out a memorised replay of both layouts.

## Scoring as implemented

An `on_step` callback updates a whole-trial record every control step (`_update` in `task.py`), cleared only by the
simulator's scene reset (`on_reset`):

- **discharge**: lever hinge at least 0.28 rad (its stop is 0.33), the pin's origin at least 60 mm from its start (its
  rod is then clear of both lugs), and the horn's line within 40 mm of the waste bin's centre: the perpendicular
  distance, seen from above, from the bin's centre to the line from the horn's mouth along its swivel direction
  (the straight distance from the mouth if the bin is behind it). Held 2.0 s in a row: discharged.
- **fault**: the lever at 0.12 rad or more (a part squeeze, which would start a discharge) while the horn's line
  misses the waste bin by more than 40 mm, or either bin more than 20 mm from its lot's mark. With the pin left in,
  a hard press moved the lever at most 0.09 rad on the GPU, so a blocked squeeze is not a fault.

`success()` (held 1.5 s) and `score()` (1.0 or 0): discharged, no fault, lever back at 0.05 rad or less, both arms'
finger and gripper hulls (`yam_arena.analysis`) at least 5 mm from every shape of the lever, horn and pin, and the
pin, bins and extinguisher at rest. The lever is a hinge sprung to 0 (0.3 N m/rad, 15 g, so it sags under 2 degrees
once the pin is out); squeezing it to its stop takes about 0.9 N at its end, well under a real lever's tens of N. The pin blocks it by contact: the lever's tab is 1.5 mm above the pin.

## Scripted demo

`demo()`, right arm only, fixed waypoints per lot: grip the pin's ring from above (fingers closing front to back)
and pull it 90 mm to the right, open (it drops to the table); grip the horn 11.5 cm from the swivel and carry it
round in 8 steps to 40 degrees toward the burning bin, turning the wrist with it, let go; press the lever's end with
closed fingers to its stop and hold 3 s; lift away. The side is the lot's (the cameras show which bin is burning).

## Measured on the GPU

Traces (scripted, object_jitter=0, `--epochs 2`): in both lots the pin came out (moved about 90 mm, then dropped to
the table), the horn reached 39.7 degrees (lot A) and -39.5 degrees (lot B), the lever reached its 0.33 rad stop and
was held there 3 s, and each trial scored 1.0 once the lever was let go and the hand was clear; task_score 1.0. The
final demo run (taller flame tongues, 0.12 rad fault threshold) also scored 1.0 in each lot.

Habit trace (lot A, pin left in, horn swung to the fire, lever pressed with the same 10 mm overshoot past its stop):
the lever moved at most 0.09 rad (5 degrees, contact give between the tab and the pin) against the 0.28 rad needed;
score 0. An earlier build with a 7 mm pin and 20 g let the tab sink to 0.24 rad, so the pin was made 10 mm thick,
60 g, in holes 1 mm wider than it, and the tab moved out to 7 cm from the hinge.

## Checks without a simulator

The cases in `near_misses.py` are meant to run in order on one copy of the task module (offline each `score()` call counts as one 50 ms step, and a state with
every object at its start clears the record). Cases, lot A: discharge held 2.0 s then let go (1.0); held 1.45 s,
lever only to 0.27 rad, horn 25 degrees off the bin, lever still held at the end (0); horn 9 degrees off (about
18 mm at the bin: 1.0). Partial squeezes: 0.11 rad at the blue bin and then a proper discharge (1.0); 0.12 rad at the blue bin or at nothing,
then a proper discharge (0); a 0.09 rad push with the pin in, then the pin out and a proper discharge (1.0). Habits,
each 0: squeeze where the horn points (straight ahead); squeeze at the blue bin first and then at the fire; aim at the fire with the pin in; pull the pin and aim without squeezing; pull the pin
only. Offline, in lot B (`lots.use("B")`), the lot A answer (horn swung left) scores 0 and the swing to the right
scores 1.0.
