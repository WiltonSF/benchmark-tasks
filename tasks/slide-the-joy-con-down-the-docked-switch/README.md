# Attach a loose Joy-Con to a docked Nintendo Switch by sliding it down the console's side rail until its catch clicks

Instruction: "Attach the loose Joy-Con to the console."

A Nintendo Switch stands in its TV dock with its left Joy-Con attached. The right Joy-Con lies loose on the table,
on its rail edge. Nintendo's own instructions are to slide the Joy-Con onto the console from top to bottom until it
clicks. Here the robot has to pick the Joy-Con up and turn it upright with its rail facing the console and its stick
toward the screen side. It then lines the slider up over the open top end of the console's rail channel, pushes it
100 mm down to the stop until the sprung catch springs into its notch, and lets go.

## What is hard, physically

- **The rail only takes the slider at its top end.** The channel's lips hold the slider's head captive, so the only
  way on is to start the slider in the open top end and slide it down the whole rail. Pressing the Joy-Con sideways
  against the console's bare end, the obvious move, leaves it resting on the lips and not attached. It scores 0.
- **The Joy-Con must be turned before it can go on.** It starts lying on its rail edge. In lot A its top end points
  at the robot; in lot B it points to the robot's left. Either way it has to be turned upright, with the rail facing
  the console and the stick facing the screen side, while held by a 13.9 mm thick edge. Upside down or backwards it
  cannot be seated.
- **Alignment at height.** The slider (4 mm stem, 8 mm head, 0.35 mm play a side) has to enter a mouth whose edges
  are rounded only 1 to 3 mm. That mouth is 147 mm above the table, at the top of a console held loosely in its dock.
- **The catch resists the last few millimetres.** The nub is pushed in by a 3.5 mm bar on the way down. It needs a
  deliberate push of about 2 to 3 N on top of the Joy-Con's weight (estimate) before it springs into the notch.
  A Joy-Con let go above the catch slides down and stops on it: on the rail, but not locked (0.5).
- **The console is loose in its dock.** It must not be lifted out (over 5 mm) or tipped (over 10 degrees), and the
  pads must never touch the screen glass.

## How this differs from nearby tasks

- **TK-9DRT5J** (another team's pending task, a Switch lowered onto its dock's plug) uses the same console and dock.
  There, the console is lowered straight down into a slot onto a hidden plug, a blind vertical mate with no rail and
  no catch. Here the console stays in the dock throughout. The part being placed is a 52 g controller that has to be
  reoriented, started into a captive T-rail only at its top end, slid 100 mm and pushed past a sprung catch.
- **TK-L264XR and TK-UKX5MH** (merged rail-click tasks) slide a carriage along a T-rail lying flat on the table. The
  carriage starts upright, already oriented to the rail, and the rail's open end faces the arms. Here the rail stands
  vertical on the end of a console standing in a dock, its open end 147 mm up and facing the ceiling. The part starts
  lying on its side, in a different orientation in each lot, and must be turned 90 to 120 degrees to face the rail.
  Its catch is on the moving part, not the rail. Sideways entry onto the console's bare end, the natural mistake for
  this product, is modelled and scores 0.

## Scene (true size)

The console, Joy-Con and dock use Nintendo's published sizes and weights. The rest are estimates, listed under
Fidelity.

| Object | Model | Size | Mass |
|---|---|---|---|
| Console | Nintendo Switch HAC-001 body, left Joy-Con HAC-015 attached | body 173 x 102 x 13.9 mm (+33 mm Joy-Con on its left end) | 297 g + 49 g |
| Loose Joy-Con | Joy-Con (R) HAC-016 | 102 x 33 x 13.9 mm body + 2.9 mm slider (estimate) | 52 g (1.8 oz) |
| Dock | Nintendo Switch dock HAC-007 | 173 mm long, 104 mm tall, 54 mm deep | 327 g |

- **Console:** screen glass 137 x 77.2 mm, centred on the front face. A USB-C port opening sits in the middle of the
  lower edge: 10.0 x 3.6 x 7.0 mm, its edges rounded 1 mm. It stands in the dock with its port on the dock's plug.
- **Rail channel** (on the console's bare right end):
  - Open at the top, closed by a 3 mm stop at the bottom.
  - Two lips, 0.85 mm deep, leave a 4.7 mm slot for the stem. Behind them is an 8.7 mm wide cavity for the head,
    reaching 3.25 mm in from the end face.
  - At the mouth, the inner top edges of the lips and cavity walls are rounded 1 mm, and the console's top corner
    over the channel floor is rounded 3 mm.
  - The screen-side lip has a 5 mm notch, 3.5 to 8.5 mm below the console's top. The lip above the notch is the bar
    that pushes the nub in.
- **Joy-Con:**
  - The slider has a 4.0 x 1.2 mm stem and an 8.0 x 1.7 mm head. It runs from the top end to 3 mm above the bottom
    end, its lower ends rounded, so the Joy-Con ends flush with the console when it is on the stop.
  - The body's lower inner edge is rounded 3 mm.
  - The stick has a 16 mm cap, 8 mm proud of the front face, 62 mm below the top.
  - The catch is a 3 mm round nub, 0.8 mm proud of the rail face beside the stem, 6 mm below the top. It is a 10 g
    slider on a 2,500 N/m spring, preloaded 0.5 mm (1.25 N), with 1 mm of travel. That is a real joint in the
    simulation, not a scripted snap.
- **Dock:**
  - The slot is 16 mm wide and runs the dock's full length. Its floor is 45 mm above the table.
  - The front wall rises to 75 mm and the back wall to 104 mm; their inner top edges are rounded 4 mm.
  - A USB-C plug (8.25 x 2.4 x 6.5 mm) stands at the middle of the slot floor. The Joy-Con hang past the dock's
    ends, as on the real dock.
- **Friction** (estimates, static / dynamic): console and Joy-Con 0.5 / 0.4, dock 0.7 / 0.6.

Layout. x is ahead of the line through the arm bases and y is to the robot's left, in cm; heights are above the
table.
- The dock's footprint is centred at (31.0, -12.0), its lower front wall toward the robot. The console stands in it,
  screen toward the robot.
- The red Joy-Con lies on its slider, its centre 19.7 mm up.
  - **Lot A:** centred at (19.5, -16.5), long axis along x, top end toward the robot, stick facing the robot's right.
  - **Lot B:** centred at (17.5, -25.5), long axis along y, top end toward the robot's left, stick facing the robot.

## Scoring

Exactly as `task.py` computes it, at each control step:

- **1.0, attached:** all of these:
  - The Joy-Con is within 1 mm of its seated height on the stop.
  - Its catch is out in the notch: the nub's joint is pushed in no more than 0.25 mm.
  - It is within 0.6 mm of the rail's line front to back and in toward the console. The head's play is 0.35 mm, so
    this holds only with the slider in the channel.
  - It is parallel to the console within 2 degrees.
- **0.5, on the rail, not clicked:** the same 0.6 mm line tolerances, at least 30 mm of slider inside the channel and
  within 3 degrees of parallel, but not attached. In practice: stopped on the catch, or pushed partway down.
- **Both tiers also need:**
  - The console still docked: its lower edge within 3 mm of the slot floor, within 8 mm of the slot's centre line
    and 5 mm of its middle, and upright within 3 degrees.
  - Release: no point on any face of either arm's links 3 to 5, gripper or fingertips within 5 mm of the box around
    the Joy-Con, the console or the dock. Arm faces are sampled about every 3 mm, corners included, and a flat face
    0.5 mm away with no corner near is caught.
  - Stillness: the Joy-Con and the console each move no more than 0.2 mm and 0.2 degrees per control step for 20
    steps (1 s).
- **0** for anything else: the Joy-Con pressed sideways against the console's bare end (it rests 2.9 mm out on the
  lips), resting on the console's top, lying on the table, upside down, backwards, or still held.
- **0 for the whole trial** once any of these happens:
  - a gripper or fingertip surface within 1 mm of the screen glass;
  - the console's lower edge more than 5 mm above the slot floor, or the console tipped more than 10 degrees;
  - the Joy-Con moving down faster than 0.6 m/s (a free fall of about 18 mm);
  - an arm's state unknown (fail closed).

The trial's score is the score at its last step. The whole-trial record is cleared only by the harness's scene reset
between trials (an `on_reset` hook), never by objects returning to their start. Trials run on lots A and B in turn
(trial k uses lot k mod 2) and the task score is their mean.

A person can score a real trial from the video and a ruler. Attached means the Joy-Con flush with the console's top
and bottom after the click, and it does not lift off without its release button. The never rules can be seen in the
video.

## Run

From the task folder, with yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Attach the loose Joy-Con to the console." --scorer task_score --max-steps 1200 --epochs 2
```

Use `--policy dream` with the same arguments for Dream-1. `--epochs 2` runs lots A and B. Setting `TASK_LOT=B` in the
environment pins every trial to one lot, for debugging. Submissions run with `object_jitter=0`. Each trial is at most
1,200 control steps (60 s); the scripted demo finishes in about 30 s.

Three helper files sit next to `task.py` and patch the embodiment in the task's own process only; nothing is changed
on disk:
- `parts.py`: multi-shape parts and the catch's sprung slider joint;
- `lots.py`: the per-trial Joy-Con position and turn;
- `tilt.py`: lets the scripted demo use a tilted grip. It affects only the demo, never Dream-1 or scoring.

Offline, `python near_misses.py` scores every near-miss case without a simulator and reports any that differ from
their declared score.

## Demo

The demo runs open loop from the published layout. The only things it reads are the Joy-Con's place and turn for
the trial (`lots.current()`, both visible in the top camera) and fixed waypoints. It reads no object pose, joint
state or contact at run time.

The right arm:
1. takes the Joy-Con with a grip tilted 45 degrees, the pads across its 13.9 mm thickness, 12 mm in from its outer
   edge and 30 mm below its top;
2. lifts it and turns it upright (rail toward the console, stick toward the screen side) while carrying it beside
   the console;
3. lowers the slider into the top of the channel, 0.5 mm toward the console from the line;
4. slides it down over 5 s, pushing 8 mm past seated, and lets go;
5. backs out along its approach, closes its fingers and presses the Joy-Con's top end straight down, 6 mm past
   seated, until the catch clicks (where the Joy-Con sits in the pads varies by a few millimetres with the pick, and
   when it sits high the held push stops on the catch);
6. lifts away and waits clear.

The left arm stays parked. One fixed correction, 0.8 mm toward the back face (`CARRY_FIX`), makes up for the loaded
arm's carrying error. It was measured once in the first trace and is the same in both lots.

## Results

All runs are simulations (yam-isaac-lab-arena on Isaac Lab-Arena, GPU physics, `object_jitter=0`, 20 Hz); none is a
hardware result.

The final demo and Dream-1 runs used the same task.py (SHA-256
`03a8a1bd50684a011002b892195a335e0e461062ad3738c6a58d9e3655646fab`) and instruction, with `--epochs 2`.

- **Scripted demo: 1.0 in each of lots A and B** (task score 1.0). The trials ended with success after 490 and 582
  control steps (24.5 s and 29.1 s), together 1,072 steps, well inside the 1,200-step limit per trial.
- **Dream-1: 0.0 in each of lots A and B** (task score 0.0); both trials ran to the 1,200-step limit.

Earlier runs while building, kept for the record. In each, the scripted pick and the turn upright left the Joy-Con
within about 0.5 degrees of the rail's direction.
- **Trace 1:** in lot A the slide jammed at the mouth. The Joy-Con had been aimed 1.5 mm toward the console, its body
  landed on the console's top edge, and the stiff arm would not let it slide back out. In lot B it slid the full
  rail and stopped on the catch 7 mm short (0.5).
- **Trace 2:** the aim moved to 0.5 mm, with a 0.8 mm correction toward the back face. Lot A slid, clicked and scored
  1.0. Lot B again stopped on the catch, because the Joy-Con had settled about 6 mm higher in the pads at the pick
  and slipped further up them under the catch's resistance.
- **Trace 3:** the closed-fingertip press on the top end was added. It scored 1.0 in both lots with no never rule
  tripped: the console rose at most 0.3 mm in its slot and the Joy-Con never moved down faster than 0.06 m/s.

Offline, `near_misses.py` declares and checks 873 cases, including:
- each tier's limits on both sides;
- the sideways press (resting 2.9 mm out on the lips) and the Joy-Con upside down, backwards, on the console's top
  or still on the table;
- each never rule;
- the release check with a flat arm face 0.5 mm above the Joy-Con (no corner near: no credit).

Replaying lot A's motions in lot B closes the fingers 39 mm beyond the end of the lot B Joy-Con, on empty table.
That replay was not run in the simulator.

## Fidelity

- **Geometry.** The console, Joy-Con and dock are at Nintendo's published sizes and weights, built from rigid boxes
  and cylinders; no CAD model was used. Estimates: the rail profile (stem 4.0 x 1.2 mm, head 8.0 x 1.7 mm, 0.35 mm
  play a side), the mouth's rounding, the slot's width and the floor and wall heights.
- **The catch force is an estimate.** No published Joy-Con latch or attach force was found. The catch is a sprung
  nub with these values: 10 g, 2,500 N/m, preloaded 1.25 N, 0.8 mm proud, a 3 mm round nose, a 3.5 mm bar above a
  5 mm notch. Its geometry gives a push of roughly 2 to 3 N past the bar, on top of the Joy-Con's weight. It was
  chosen to be felt but easy for a hand, and well under the YAM's force.
- **Release.** A real Joy-Con needs its release button pressed to come off. This catch can be pulled off with about
  the same force it takes to push on. The task never asks for removal.
- **Not modelled.** Buttons other than the stick, the electrical contacts and the wrist strap.
- **Lots.** The two Joy-Con layouts stop one fixed motion from scoring in both. They do not stop a policy that has
  memorised both.

## Sources

- Nintendo support, how to attach the Joy-Con: "slide each Joy-Con down from top to bottom, until you hear a click".
  https://www.nintendo.com/en-gb/Support/Nintendo-Switch/How-to-Attach-Detach-the-Joy-Con-Controllers-from-the-Nintendo-Switch-Console-1379043.html
- Nintendo Switch tech specs: console with Joy-Con attached 4 x 9.4 x 0.55 in (102 x 239 x 13.9 mm), 0.66 lb alone
  and 0.88 lb with Joy-Con; Joy-Con (L) 1.7 oz, Joy-Con (R) 1.8 oz; dock 4.1 x 6.8 x 2.12 in, 0.72 lb.
  https://www.nintendo.com/us/switch/tech-specs/ . Each Joy-Con's 33 mm outside the console is (239 - 173) / 2; its
  2.9 mm slider depth is an estimate.
- USB Type-C plug 8.25 x 2.4 mm: USB Type-C Cable and Connector Specification, USB-IF.

Nintendo Switch and Joy-Con are trademarks of Nintendo, named for identification only. This task is not made,
endorsed or sponsored by Nintendo.

## Licence

Code MIT; documentation CC BY 4.0. All geometry is generated in task.py; no third-party assets.
