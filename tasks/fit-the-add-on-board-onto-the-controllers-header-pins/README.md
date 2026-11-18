# Fit an add-on board onto a robot controller's 2 x 20 header with every pin in its own socket

Instruction: "Fit the blue add-on board fully onto the controller's header, with every pin in its own socket and none
bent, then let go."

## Credit

This task takes up the idea of pull request #1891, "Seat a robot controller's add-on board on its two rows of header
pins without bending any pin" (TK-V6HHU5, revised as TK-X7VF9Y). It was not accepted, with a request for revision, and
its 48-hour hold lapsed with no revision. The scene, scoring and demo here are a new implementation, written from that
task's description and its two reviews. No files or code were copied.

## How the reviews of TK-V6HHU5 and TK-X7VF9Y are addressed

First review (TK-V6HHU5):

- **"The board cannot rest on 6 mm pin tips when its level underside is 9.4 mm above the header."** Nothing here rests
  where it cannot. The board starts on a foam holder off the header. On the header, its pin rows physically stand on
  the socket entry face (a collision surface), so its underside is exactly 6.0 mm (the pin length) above the header top.
  There is no other support.
- **"Correct the header width so both pin rows fit."** The header is a standard 2 x 20 socket at 2.54 mm pitch in both
  directions: rows 2.54 mm apart, body 5.08 mm wide, 1.27 mm of plastic outside each row's centre line. The board's
  pins match it.
- **"Specify the actual support or pin engagement."** At the start, the board lies flat on two foam blocks with its pins
  in the slot between them. On the header, the pin tips stand on the socket entries. They enter only when every tip is
  within 0.7 mm of a socket centre and the board is within 3.4 degrees of level, and only under a push of at least 24 N.
  The `setup` and `fidelity` fields give the numbers.
- **"Releasing the board does not lock its tilt or suspend it."** The add-on board is a free rigid body. There are no
  brakes, latches or locked joints on it. Let go above the header, it falls onto the socket entries. Pressed part-way
  in and let go, it stays where it was pushed, because the entry face it stands on stays there (contact friction).
- **New full-score demo; setup, reset and fidelity updated.** See Results, and `task.json`.

Second review (TK-X7VF9Y):

- **"The 7 mm seat-latch threshold can engage before the board reaches the 6 mm support height."** There is no seat
  latch or height threshold. The press fit is the entry face itself. It only moves when the board's pins are standing
  on it (within 0.5 mm of it), so a board held higher cannot be held or credited. A lifted board that is let go falls.
- **"Full-score demo of the corrected current revision within 1,200 steps, including its score and step count."** See
  Results. Every run listed there used the submitted `task.py` (SHA-256 prefix given there).
- **"Run Dream-1 on that revision with its video and score."** See Results. Both videos come from this revision.

## What is tested, and why it is not "press a snap-fit until it clicks"

The hard part comes before any force: lining up 40 pins in two rows over 40 sockets, to within 0.7 mm across and
along, with the board turned no more than 1.5 degrees (at 1.6 degrees the end pins are 0.71 mm off) and the right way
round. The robot has to do this before it presses. A snap-fit locates itself by its own shape as it is pushed. A pin
header does not. Nor is it like seating a memory module or a processor, which a slot, keys or a socket frame guide
into place: nothing guides the board until its pins are already in their sockets. At half a pitch off, the tips stand on the plastic between sockets, and pressing then bends pins. A
whole pitch off (one row across, or one pin along), the pins drop into the wrong sockets or hang past the header, and
the board goes fully down and looks fitted. Rotated 180 degrees, it also goes down. Those three all score 0. Only after
alignment is there a push, about 24 N, as level as the 3.4-degree rule allows.

## Scene (true size)

- Controller (fixed): aluminium carrier plate 100 x 70 x 6 mm, green FR-4 PCB 85 x 56 x 1.6 mm, and a black 2 x 20
  socket header 50.8 x 5.08 x 8.5 mm (top face 16.1 mm above the table) at the PCB's centre. An 8 mm white square marks
  pin 1.
- Add-on board (loose, 35 g): blue FR-4 PCB 65 x 56 x 1.6 mm, aluminium heat sink 25 x 25 x 12 mm at its centre, two
  rows of 20 gold pins (0.64 mm square, 6.0 mm long) under it, and an 8 mm white pin-1 square.
- Foam holder (fixed): two black antistatic foam blocks 20 x 60 x 20 mm with a 10 mm slot between them.

All shapes are boxes built by `parts.py` (a kit for multi-shape and jointed parts, next to the task file). No
third-party assets are used.

The centred stacking header follows the original task's layout. Many boards put their header along one edge and add
corner standoffs instead. That layout is not modelled here.

## The press fit (simulation)

The header's top 6 mm (the socket entries) is a straight-down slider on a 40,000 N/m spring. The pin rows stand on its
top face. Each control step, an `on_step` rule sets where the spring rests:

- While every pin tip over the header is within 0.7 mm of a socket centre, the board is within 3.4 degrees of level,
  and the push is at least 24 N (0.6 N per contact, an estimate within the usual range for 2.54 mm sockets), the rest
  point follows the face down. The face stays wherever it is pushed: it never springs back while the pins are in.
- Within 0.6 mm of full depth, the face goes to its 6 mm stop and stays there.
- Otherwise the rest point stays put, so the face does not yield at any force. This covers pins on the plastic, a
  tilted board and a light push.
- When the pins are lifted more than 1 mm off the face, it returns to the top (the pins are out).

The sockets do not hold a fitted board sideways or resist pulling it off. The scorer stands in for the sideways hold:
pins moved more than 0.7 mm off their sockets while in them count as bent. The arms are soft, about 500 N/m at the
fingertips (measured: closed fingertips pressed on a sprung platen gave 27 N with the arm 52 mm short of its target),
so the demo commands the press 60 mm past the seated position.

## Scoring (exact definitions)

The board's pose is measured in the header's frame: x across the rows, y along them.

- **Over sockets:** the pin field's centre offset (dx, dy) is split into whole pitches (n_x, n_y) and a remainder
  (r_x, r_y). The board is over sockets when |r_x| + 25.4 mm x |sin yaw| <= 0.7 mm and |r_y| + 2.54 mm x |sin yaw| <=
  0.7 mm, with yaw taken modulo 180 degrees.
- **Correct:** over sockets, with n_x = n_y = 0 and |yaw| <= 90 degrees (pin 1 over pin 1).
- **Seated:** correct; the board's underside at most 0.3 mm above the header top; the entry face within 0.3 mm of full
  depth; tilt at most 3.4 degrees. **Started:** correct, pins at least 2 mm in, tilt at most 3.4 degrees.
- **Let go:** no point of either arm (link3, link4, link5, gripper, both fingertips; hull faces sampled every 3 mm)
  within 5 mm of the board. This is measured in the simulator every control step. Until it is measured, or if any link
  cannot be read, the board counts as held (fail closed).
- **Still:** the board's pose within 0.2 mm and 0.2 degrees of the previous step's, for 20 steps in a row (1 s).
- **Bent (whole trial):** pins at least 0.5 mm into the sockets with the board tilted over 3.4 degrees, or not over
  sockets. Or pins standing on the entry face but not over sockets, pushed with 12 N or more (an estimate: the tips
  on the lead-in chamfers are forced sideways). From then on the trial scores 0. The record is cleared only by a scene
  reset: the board back within 0.5 mm of its start, at rest, after a jump of over 20 mm in one control step.
- **Score:** 0 if bent. Otherwise, let go and still: 1.0 if seated, 0.5 if started, else 0. Success is the 1.0 state;
  the episode ends once it has held 1 s.

`near_misses.py` checks every tier and limit offline: lifted and let go over the header; resting on the entries;
1 mm and 2 mm in; 0.3 and 0.4 mm high; held, unmeasured and an arm face 0.5 mm away; off by one row or one pin either
way; turned 180 degrees; 0.7 mm off (passes) and 0.71 mm off (fails); turned 1.5 degrees (passes) and 1.6 degrees
(fails); pressed on the plastic at 11 N (fine) and 12 N (bent); tilted 3.3 degrees (fine) and 3.5 degrees (bent);
dragged sideways once seated; and dropped beside the header.

## Run and score a trial

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "Fit the blue add-on board fully onto the controller's header, with every pin in its own socket and none bent, then let go." \
        --scorer task_score --max-steps 1200

For the scripted demo, add `--policy yam_scripted` (the demo results below used `--epochs 2`); for Dream-1, add
`--policy dream`. One trial takes about 2 minutes on an L40S GPU. Use `object_jitter=0`. `parts.py` must sit next to
`task.py`.

The demo is open loop from the published start layout: fixed waypoints, with no reading of object poses, joint states
or the press-fit state. The right arm grips the heat sink with its pads 6 mm past each face, lifts the board out of
the foam, carries it over the header, lowers the pin tips onto the socket entries, presses (commanded 60 mm past
seated), eases back to 3 mm past seated, opens and moves away. The left arm stays parked. `DEMO_VARIANT` (traces
only) runs the same plan with the board carried half a pitch across (`half_pitch_press`), one row across
(`one_row_over`) or one pin along (`one_pin_along`), or let go above the header (`drop`).

## Real trial

Use a 2 x 20 socket header controller and an add-on board with matching 0.64 mm pins on a carrier plate, with the
board in an antistatic foam holder. Check seating with a 0.3 mm feeler gauge at the four corners. Check alignment
with a photo along each row against a 1 mm grid: every pin in a socket, none past the ends, pin 1 marks together.
Check for bent pins with a 10x loupe and a straight edge after removing the board with a puller.

## Results

Runs in Isaac Sim (Isaac Lab-Arena, yam_arena, one L40S). The demo and Dream-1 runs used this `task.py` (SHA-256
prefix 3d7b6101638a, equal to `eval.environment_revision` in both logs). The traces used the same scene and scoring;
the first one predates the `DEMO_VARIANT` switch.

| Run | Policy | Result | task_score |
|---|---|---|---|
| Demo (`--policy yam_scripted --epochs 2`, 1,200-step limit) | scripted | Both trials ended on success, 709 steps for the two (about 355 each). Pins lined up within 0.3 mm across and along, turned 1.15 degrees (end pins 0.51 mm off), tilt at most 2.2 degrees while going in, fully seated, let go. | 1.0, 1.0 (mean 1.0) |
| Dream-1 (`--policy dream`, 1,200 steps, 320 x 176) | Dream-1 | The right gripper took the board off the foam and held it beside the controller. The pins never reached the header. Hit the step limit. | 0.0 |
| Trace, `half_pitch_press` | scripted | Pin tips landed 1.2 to 1.5 mm off across, on the plastic. The arm pushed about 32 N, the entry face did not give way, the board stayed 6 mm up, and the pins counted as bent. | 0.0 |
| Trace, `one_row_over` | scripted | Shifted 2.6 mm across (one row), the board went fully down. Let go, it tipped 18 degrees with one row off the header's edge. | 0.0 |

The `one_row_over` tip after release comes from the simulation: real pins in their sockets would keep the board level.
It scores 0 either way, because the pins are in the wrong sockets.
