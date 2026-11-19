# Unlock a pair of locked pruning shears in a blade guard so their handles spring open

Instruction: "Unlock the guarded pruners and leave their handles open."

A pair of bypass pruning shears lies flat in a weighted stand, locked shut by its stopper. A rigid guard on the stand
covers the blades over their whole sweep, so no edge is exposed and no cutting is involved. The two handles lie on
the stand's 30 mm rest, toward the robot. The robot has to unlock the shears and leave them open in the stand, with
nothing touching them.

The shears are modelled on the ARS VS-8Z, whose stopper is released by gripping the handles. The maker's own wording
is that the stopper releases "just by gripping" the handles. Pulling the locked handles apart, the obvious move,
only pulls the stopper's pin harder into its notch. A short extra squeeze lifts the pin out of the notch, and a
light spring then turns the stopper off the pin. After that the return spring opens the handles once the squeeze is
let off.

## What is hard, physically

- **The first move goes the wrong way for a person who has not used one.** Opening needs a closing squeeze first:
  the moving handle has to go about 3 degrees past its locked position (about 5 mm at the grips) toward the fixed one.
  A squeeze that stops short leaves the pin in the notch, and the shears stay locked when it is let off.
- **Pulling apart does not work and must not be forced.** The notch holds the pin against an opening pull. Forcing
  the handles more than 3 degrees open while the stopper is still on its pin scores 0 for the whole trial.
- **The hand must let the handles go.** After the squeeze, the spring pushes the moving handle out against the
  finger. A gripper that just opens leaves the handle pressed on its finger, short of its stop, and a gripper that
  lifts while pressed drags the tool. The handle has to reach its stop with every gripper part more than 5 mm away.
- **The tool must stay in its stand.** Its pivot must stay within 8 mm and 5 degrees of where it lay. Taking it out
  of the guard (pivot over 20 mm away, or lifted over 15 mm) scores 0 for the whole trial.

## How this differs from nearby tasks

No pending or accepted task found on the board unlocks pruning shears. The nearest ones test other abilities.
A rose-pruning task cuts a stem. A kitchen-shears task takes a pivot apart. A nail-clipper task reverses a lever.
A locking-pliers task locks a toggle under a squeeze. Here nothing is cut, taken apart or locked. The robot has to
make a short closing squeeze that frees a real catch, then let a spring open the tool without trapping it.

## Scene (true size)

Positions in cm: x ahead of the line through the arm bases, y to the robot's left of the midline; heights above the
table. All parts are rigid boxes and cylinders generated in `task.py`.

- **Shears (loose, 220 g, after the ARS VS-8Z).** 200 mm long: a 54 mm blade ahead of the pivot bolt and two grips
  running 60 to 146 mm behind it. The tool is 20 mm thick, lying flat. Each grip is 16 mm wide, coated red, and lies
  10 degrees off the tool's axis when locked. The grip ends' centres are then 51 mm apart. The fixed half is 120 g:
  its grip, a steel neck and the hook blade, in the lower 9 mm. The moving half is 85 g: its grip, neck and the
  cutting blade, in the upper 9 mm, turning about the pivot. It is stopped 4 degrees past locked when squeezed
  (grip ends 41 mm apart) and 20 degrees open at its natural stop (grip ends 100 mm apart). The pivot nut stands
  4 mm proud. The return spring gives 0.6 N m when locked and 0.4 N m at the opening stop, about 7 N at the middle
  of the grips.
- **Stopper.** A 15 g steel latch plate on top of the fixed grip, pivoted 125 mm behind the pivot bolt. Its notch,
  3 mm deep with 0.3 mm play a side, sits over a brass pin (7 mm across, 4.5 mm tall) on top of the moving grip.
  The return spring pulls the pin into the notch's floor, so the plate cannot turn. A light spring (about
  0.009 N m) turns it 25 degrees off the pin once the pin is out of the notch. With the plate off, the handle opens
  freely.
- **Stand (loose, 1.5 kg, dark grey).** A rest 250 x 150 mm, 30 mm high, under the whole tool. Over the blades is a
  rigid orange guard 115 mm long and 90 mm wide: a 4 mm roof 56 mm up, both sides and the far end, open only toward
  the handles. It covers the blades through their whole sweep, from squeezed to fully open. Inside the guard the
  tool is located with about 1 mm play: two blocks hang from the roof either side of the pivot nut, and two 8 mm
  high blocks flank the hook blade, below the cutting blade's sweep.
- **Lot A (the screenshot):** the pivot over (36.0, -3.0), the tool's blades pointing 20 degrees to the robot's right
  of straight ahead, handles toward the robot. **Lot B:** pivot over (40.0, 2.0), blades 25 degrees to the robot's
  left. In both, the tool starts locked: the moving handle on the stopper, the pin against the notch's floor.
- Friction (static/dynamic): shears 0.6/0.5, stand 1.2/1.0. Both arms start at their ready pose, grippers open.
  Trials are run on lots A and B and averaged.

## Scoring

Exactly as `task.py` computes it, at each control step:

- **1.0, unlocked and open:** the stopper turned 15 degrees or more off its pin; the moving handle within 5 degrees
  of its opening stop; the tool in its rest (pivot within 8 mm and 5 degrees of its start, its underside within
  2 mm of the rest's top), and the stand within 8 mm and 3 degrees of its start.
- **0.5, unlocked, partly open or shifted:** the stopper turned 15 degrees or more, and the grip ends at least 10 mm
  further apart than when locked, but not 1.0.
- **Both tiers also need,** held for 2 s:
  - Release: no point on any face of either gripper or its finger tips within 5 mm of any shape of the shears or
    the stand. Gripper faces are sampled about every 3 mm, corners included.
  - Stillness: every part of the shears and stand moving under 5 mm/s and turning under 0.05 rad/s.
- **0** for anything else: still locked, squeezed and held, the stopper off but the handles shut, or still held.
- **0 for the whole trial** once any of these happens:
  - forced past the stopper: the handle more than 3 degrees open while the stopper is turned less than 10 degrees;
  - out of its guard: the pivot more than 20 mm from its start, the tool's underside more than 15 mm above or below
    the rest, or the stand moved more than 20 mm;
  - no credit at a step where either hand cannot be read; the 2 s hold then starts again (fail closed).

The trial's score is the score at its last step. The whole-trial record is cleared only by the harness's scene reset
between trials (an `on_reset` hook), never by objects returning to their start. Trials run on lots A and B in turn
(trial k uses lot k mod 2) and the task score is their mean.

A real trial can be scored from a photo and a ruler: the stopper's position, the grip-end spacing, and where the tool
lies in its stand. The never rules can be seen in the video.

## Run

From the task folder, with yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Unlock the guarded pruners and leave their handles open." --scorer task_score --max-steps 600 --epochs 2
```

Use `--policy dream` with the same arguments for Dream-1. `--epochs 2` runs lots A and B. Setting `TASK_LOT=B` in the
environment pins every trial to one lot. Submissions run with `object_jitter=0`. Each trial is at most 600 control
steps (30 s); the scripted demo finishes in about 15 s.

Two helper files sit next to `task.py` and patch the embodiment in the task's own process only; nothing is changed
on disk: `parts.py` (multi-shape parts, the two hinges and their springs) and `lots.py` (the per-trial stand pose).
After the lot move, `task.py` lays the stand and shears exactly at the lot's pose, and lays them again at the first
control step if either is more than 1 mm off.

Offline, `python near_misses.py` scores every near-miss case without a simulator and checks the step-side rules.

## Demo

The demo runs open loop. It reads only the stand's place and turn for the trial (`lots.current()`, visible in the
top camera) and fixed offsets on the tool. It reads no joint, contact or exact pose at run time. The right arm:
1. comes down over the two grips, 88 mm behind the pivot, fingers wide, finger tips 3 mm above the rest;
2. closes across both grips, squeezing the handles to their stop, and holds the squeeze 1 s;
3. opens its fingers slowly over 3 s, so the spring opens the moving handle against the finger;
4. slides 15 mm toward the pivot and 12 mm to the moving handle's side, so the handle reaches its stop clear of the
   finger;
5. lifts away and parks. The left arm stays parked.

## Fidelity

- **Product data.** Length, blade length, mass and pack size are the maker's. The grip outline, the 20-degree opening,
  the spring torques, the squeeze travel and the stopper's shape are estimates: none is published. The real
  stopper's exact cam shape is not given; this one is a notch over a pin that needs the same extra closing squeeze.
- **Simplifications.** Grips are straight, built from short boxes; no cushion bumpers, sap groove or wire cutter.
  The handles stop at joint limits, not by touching. The real tool is relocked by pushing the stopper back with the
  thumb; that is not needed here.
- **Guard and stand** are a fixture for safe trials. They are not part of the product.
- **Lots.** The two stand poses stop one fixed motion from scoring in both, not a policy that has memorised both.

## Results

All runs are simulations (yam-isaac-lab-arena on Isaac Lab-Arena, GPU physics, `object_jitter=0`, 20 Hz); none is a
hardware result.

The final demo and Dream-1 runs used the same task.py (SHA-256
`56c1763d0fc85864df875c82c9871a216dc51793e722a46a6feee9ab73a32c5e`) and instruction, with `--epochs 2`.

- **Scripted demo: 1.0 in each of lots A and B** (task score 1.0). The demo's plan is 15.2 s, well inside the 30 s
  limit per trial.
- **Dream-1:** lot A (the first trial) was refused by the Dream API before any step: "unsafe instruction in this
  scene, refused: Exposing sharp pruners with open blades creates a hazard." Lot B ran and scored 1.0, so the
  reported task score (1.0) rests on lot B alone. The Dream-1 video is the refused lot A trial.

Earlier runs while building, kept for the record:
- **Trace 1:** in both lots the squeeze took the stopper off its pin (it turned to its 25-degree stop while the
  handle was at its squeezed stop) and the handles then sprang to their 20-degree stop. But the guard had no
  locating blocks, and the tool slid 8 mm (lot A) and 13 mm (lot B) in its stand as the handle opened. Both lots
  scored 0.5. The keeper blocks inside the guard were added after this.
- **Trace 2:** with the keepers the tool moved under 3.5 mm. Both lots scored 1.0, with no never rule tripped: the
  handle was never open while the stopper was on its pin.

Offline checks, all in MuJoCo with default contacts and the base held still, before any GPU run:
- the locked catch held with no input, and against an extra opening pull of up to 2 N m on the handle (about 22 N
  pulling the grips apart at their middle);
- a 4 N m pull (about 44 N) forced the pin through the notch's soft contact; the forced-through rule scores that 0;
- a squeeze to 1 or 2 degrees past locked did not free the stopper, and the shears stayed locked when let off;
  2.5 degrees or more freed it, and the handles sprang to their stop.

`near_misses.py` checks each tier's limits on both sides, the forced-through and out-of-guard rules, the 2 s hold,
the stillness limits, missing hand data, and the release check with a flat gripper face over the tool and no corner
near (no credit). Replaying lot A's grip in lot B closes the fingers more than 40 mm from lot B's grips (not run in
the simulator).

## Sources

- ARS VS-8Z product page (Japanese): overall length 200 mm, blade 54 mm, mass 220 g, aluminium die-cast handles with
  vinyl-coated grips, individual pack 250 x 75 x 22 mm; lists the one-touch stopper.
  https://ars-edge.co.jp/product/vs-8z/
- ARS, one-touch stopper (Japanese): a mechanism that frees the stopper of a saw or pruner easily; on pruners the
  stopper is released just by gripping the handle, with one hand.
  https://www.ars-shop.net/contents/technology/one-touch-stopper/
- A retailer's description of the ARS 8-inch pruner's lock: squeeze the handles together to unlock, push the
  mechanism back with the thumb to lock. https://onegreenworld.com/product/ars-8-heavy-duty-hand-pruners/

ARS is a trademark of its owner, named for identification only. This task is not made, endorsed or sponsored by ARS.

## Licence

Code MIT; documentation CC BY 4.0. All geometry is generated in task.py; no third-party assets.
