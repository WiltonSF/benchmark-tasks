# Drive a bushing to its depth below the housing face, choosing the stepped driver over the plain drift

A bronze bushing sits in a 10 mm bore with its top 1 mm below the housing face and must end 5 mm below it. The closed
fingertips cannot enter the bore, so fingers cannot push it at all. Two steel tools stand nearby: a stepped driver, whose
collar lands on the face when its pilot has pushed the bushing to depth, and a plain drift whose end is
mushroomed wider than the bore, so it cannot reach the bushing. The instruction does not name a tool.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-colour parts with hinges and sliders) and `lots.py` (layout lots) must sit next to `task.py`;
`task.py` imports both.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the bronze bushing 5 mm below the housing face, then take the tool out and let go of it."

# Dream-1, both lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream --epochs 2

# Scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/demo --epochs 2
```

- **Layout lots:** trial k uses lot k mod 2 (lot A first), so `--epochs 2` runs lot A and lot B and `task_score` is
  their mean. Lot A has the driver in the right-hand stand; lot B swaps the two tools. `TASK_LOT=A` or `TASK_LOT=B`
  in the environment pins every trial to one lot. Without `--epochs 2` only lot A runs, and a fixed "take the
  right-hand tool" habit would score full marks there. The videos and screenshot show the first trial (lot A).
- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** submitted runs use `-E object_jitter=0`; each lot's layout is fixed. With jitter the tools would start
  off their sockets.

## Scoring as implemented

`score()` and `success()` in `task.py` (pass/fail):

- **Depth** (`depth`): the bushing slider's position plus its 1 mm start: how far the bushing's top is below the
  face. Fitted when 4.25 to 5.75 mm (limits inclusive, 1e-6 m margin).
- **Tools out** (`tool_in_bore`): no shape of either tool (cylinders as their boxes, in the tool's real pose)
  overlaps the bore below the face (a 10 x 10 mm column from 0.5 mm below the face to the bore's floor).
- **Released and still** (`_settled`): an `on_step` callback measures every control step the distance from both arms'
  finger and gripper hulls (`yam_arena.analysis`) to every shape of the bushing and of both tools; all must be over
  5 mm. Both tools at rest (`resting()`: under 1 cm/s and 0.5 rad/s) with their lowest point above 5 mm below the
  table top, and the bushing moved under 0.5 mm in the last control step. Offline there is no robot, so near misses
  count as released and still.
- 1.0 when all hold (success, held 1 s); 0 otherwise.

The press fit: the bushing rides a straight-down slider (13 mm travel) on a 3,000 N/m spring. An `on_step` ratchet
moves the spring's rest point to 0.5 mm short of the deepest point reached, so the bushing moves down only under more
than 1.5 N (less its own 0.2 N weight), springs back 0.5 mm when the push stops, and never comes back up. Either
tool's weight alone (0.8 N) does not move it. The bushing is a link of the housing and does not collide with the bore.

The driver's pilot reaches 5.4 mm below its collar. Pressed until the collar lands on the face, the bushing goes to
5.4 mm and rests at about 5.0 mm after the spring-back. The drift's 9 mm rod presses the whole sleeve, but its
mushroomed end (12 mm across, 3 mm long) is wider than the 10 mm bore, so it stops on the face without reaching the
bushing. The bushing slides straight down its bore on the slider and cannot cock, so no tilt check is needed.

Why fingers cannot do it: from the URDF collision hulls, the closed fingertips measure 12.4 x 5.8 mm at their very
end and widen above it, and one fingertip is 12.4 mm wide, so neither fits the 10 mm bore in any yaw. Fingers cannot
reach the bushing.

## Scripted demo

`demo()` reads only the lot's layout, which the cameras show (the tool with the wide collar): the arm on the driver's
side grips its square handle 24 mm below the top (pads 3 mm past each flat, pinching along x), lifts it out, carries
it over the bore with a measured offset for the loaded arm, lowers the pilot into the bore onto the bushing and
presses 10 mm past the collar landing, holds, lifts it out, stands it back in its socket and lets go. The other arm
stays parked.

Measured in traces (scripted, object_jitter=0, final code, `--epochs 4`, lots A, B, A, B): every trial scored 1.0;
the bushing went to 5.5 mm under the press and rested at 5.0 to 5.1 mm once the driver was lifted out, success
holding from about 19 s. The driver leans about 2 degrees in the pads under the press. Earlier versions failed on
that lean: with the bushing starting proud, the unguided pilot slid off it and caught the face (4 of 12 trials), and
with a 48 mm collar the leaning collar landed early (2 of 4 trials short). Starting the bushing 1 mm below the face,
so the pilot is in the bore before it pushes, a 7 mm pilot (1.5 mm clearance) and a 36 mm collar fixed both.

## Checks without a simulator

`python near_misses.py` cases (scored by `factory/check.py`): the start (1 mm below the face), flush, 1 mm, 4.2 /
4.25 mm, 5.0 mm, 5.75 / 5.8 mm, overdriven to 14 mm; the driver left in the bore; the drift pushed home (its end on
the face, the bushing untouched); the drift stood on the face over a fitted bushing (allowed); the driver laid on the
face beside the bore (allowed); a tool still falling; a tool off the table; the driver dropped into the bore onto the
bushing.

Fixed habits: fingers cannot reach the bushing (0). Two habit traces were run on the GPU on the final code, each
from a modified copy of the demo that is not part of the task:

- The drift used carefully (lot A, left arm, drift pinched along y, lifted clear of its socket, carried slowly):
  centred within 0.7 mm of the bore's axis and commanded 15 mm past the face, its mushroomed end stopped on the face
  and never touched the bushing, which stayed at its 1.1 mm start. Score 0.
- Lot A's demo replayed in lot B (`TASK_LOT=B`: the right arm takes the right-hand tool, now the drift, with the
  driver's waypoints): the driver's 30 mm lift does not clear the drift's 60 mm rod from its socket, the drift never
  came within 96 mm of the bore and ended on its side on the table, and the bushing never moved. Score 0, so a fixed
  side habit averages 0.5 over the two lots.

## Files

- `task.py`: scene, press-fit ratchet, scoring and scripted demo.
- `near_misses.py`: offline cases.
- `parts.py`, `lots.py`: the parts kit and layout lots (unchanged copies).
- `DESIGN.md`: the design note.
