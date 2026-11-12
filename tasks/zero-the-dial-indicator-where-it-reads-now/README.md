# Zero a dial indicator at its current reading without disturbing the setup

A 0.01 mm dial indicator lies face up on a granite surface plate, its plunger resting on a loose steel block that
sits against a fixed parallel. The bezel was left turned by an earlier job. The robot must bring the bezel's zero
line under the pointer where it reads now, without moving the plunger or the block, and let go.

Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. The task is
`task.py` plus two helper files that must stay next to it: `parts.py` (multi-colour parts with hinges and sliders:
the bezel, pointer and plunger) and `lots.py` (per-trial start layouts). Both patch the stock embodiment in the
task's own process only; nothing on disk changes. No third-party assets are used.

## Run and score

    inspect-robots run --embodiment yam_arena -E task=tasks/zero-the-dial-indicator-where-it-reads-now/task.py \
        -E object_jitter=0 --instruction "Zero the dial indicator at its current reading without disturbing the setup." \
        --policy <policy> --scorer task_score --max-steps 1200 --epochs 3 --store-frames

- `--epochs 3` is required: trial k uses lot k mod 3 (A, B, C) and `task_score` is their mean. With one epoch only
  lot A runs. `TASK_LOT=B` in the environment pins every trial to one lot.
- `--policy yam_scripted` plays the demo. Runtime about 1 to 2 minutes per trial on an L40S.
- Seeds: every trial uses `object_jitter=0`; the layout is fully set by the lot.

## Lots

The only difference between lots is where the bezel's red zero line starts, measured clockwise from straight ahead
(+x) seen from above: A 28.2 degrees, B 208.2, C 93.2. The pointer is at about 133 degrees in every lot, so the
bezel must turn 105 degrees clockwise (A), 75 counter-clockwise (B) or 40 clockwise (C). Offline, the best single
fixed turn for all lots (-112 degrees) averages 0.33, setting the zero line to 12 o'clock (opposite the stem)
scores 0 in every lot, and doing nothing scores 0.

## Scoring, exactly

Every control step, from the simulator's state:

- Zero error: the angle between the bezel's zero line and the pointer seen from above, wrapped to -180..180.
  1.0 if within 2 divisions (7.2 degrees), 0.5 if within 5 (18 degrees), else 0; limits pass with a 1e-6 margin.
- Credit needs: both arms' fingertips and gripper bodies more than 5 mm from every shape of the indicator (case,
  stem, bezel, pointer, plunger) and the block, measured from the arm's collision hulls; the bezel turned less than
  0.5 degrees in the last control step; the block at rest (under 10 mm/s, 0.5 rad/s). Before the first step is
  measured these count as not met (no credit).
- Whole-trial faults, any one zeroes the trial until a scene reset: the plunger more than 0.02 mm from the position
  it settled at on the first step; the block more than 0.5 mm from, or turned more than 1 degree from, its pose on
  the first step. A scene reset is the simulator's reset hook, which a robot cannot trigger.
- Success: score 1.0 held for 2 s. `score()` returns 1.0 whenever success holds.

Mechanism: the plunger is a spring slider (400 N/m toward its free end, about 0.95 N on the block); each step the
pointer's drive target is set from the plunger position at one turn per mm (2.37 mm in reads 37 divisions on the
third turn). The bezel is a hinge whose drive target follows its own angle each step, so it stays where it is left
and can be turned by hand, like a friction bezel. Steel and granite parts use friction 0.25 static, 0.20 sliding.

`near_misses.py` lists 29 offline cases (tier boundaries both sides, a full extra turn, the naive "push the plunger
to zero", the block pulled back and zero set on the new reading, block slid or turned, put back afterwards) and
`factory`-style checks pass on all lots.

## Demo

`demo()` (left arm): over the bezel with the fingers wide, down beside the rim, pinch the 75 mm bezel across a
diameter (pads 3 mm inside its faces), turn the wrist, open and lift away. The turn comes from reading the dial as a
person would: the pointer's position on the bezel's own scale in whole divisions (`dial_reading`), looked up in the
module-level table `TURN_FOR_READING` (back by r divisions, or on by 100 - r, whichever is shorter). In the
simulator the dial is read once, on the trial's first control step, before the arm reaches the bezel; there is no
feedback during the turn and the demo never reads the lot. The bezel has no hidden stop (its hinge allows three
turns either way).
