# Remove the middle feeder from a packed pick-and-place feeder rail

**Instruction:** Take the middle feeder off the rail and lay it on the blue mat. Leave the other two feeders in place.

**Category:** Impact on AI timelines.

## Why it matters

Robot builders assemble their motor-driver, encoder and sensor boards on pick-and-place machines; small makers and
labs use desktop machines such as the LumenPnP, and the SMT lines that build robot controllers and server boards use
the same tape feeders on rails. Each board needs a different set of parts, so every board change means taking feeders
off the machine's rail and putting others on. A robot that runs such a machine between jobs has to do this feeder
change itself. The feeders sit side by side at the rail's slot pitch, so the job is always
"take one unit out of a packed row without disturbing the ones next to it": a neighbour knocked off its slot loses its
slot address and its pick position, and the next board is placed wrong.

## What makes it hard for a robot

- **No grip is available at the start.** The three feeders sit at the rail's 15.5 mm pitch (estimate): 0.5 mm between the 15 mm
  bodies and 2.5 mm between the 13 mm reels. A YAM finger is 6.4 mm thick, so it cannot go down beside the middle
  feeder or its reel, and a pinch from above closes on all three. Nothing on the middle unit can be grasped until the
  unit has been moved out of line.
- **It is held by two features at once.** A round hook nose under the body sits in the slot along the rail's top
  face, and a yellow sprung lever on the clip hooks two teeth under the slot block on the rail's left face. The unit
  only comes free by turning about the hook (reel end up, pick end down) once the catch lets go, then lifting the hook
  out of the slot.
- **The release and the grasp need two hands.** Whatever turns the middle unit out of line has to keep it turned while
  the other hand takes the part that now stands proud of the bank. Let go, and the reel's weight drops the unit back
  into its catch.
- **It must leave the bank along its own plane.** The unit is 354 mm long (reel edge to pick end) and 0.5 mm from its
  neighbours. It can only slide out sideways once its pick end has passed their reels; any twist rubs the neighbours,
  and pushing or prying on a neighbour unseats it.
- **The wrong ways score 0:** pressing or pulling on a neighbour, lifting the bank, pulling the middle unit straight
  up while its catch or hook is still engaged, dropping it, or leaving it on the rail.

The maker's mounting guide documents both release routes for a feeder between occupied slots: reach under the spools
to the lever and "push forward to tip the feeder", or remove it "without depressing the lever arm" by pushing or
pulling the spool arm ([mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/)). Both are open here (the
catch is a sprung tooth that cams out under load). Neither is required, and the scoring only reads the end state and
the whole-trial failures.

**How it differs from accepted tasks.** Tipping a recessed book off a packed shelf (TK-E6ECW7) also has no grasp until
the item is tipped out, but the book is free: once tipped it stays tipped and is then taken with one hand. Taking one
bottle from a row (TK-HAH39Z) and swapping a module from a row of sockets (TK-KHHQD4) start with a graspable item. Here
the unit is latched by a hook and a sprung catch, falls back into its catch the moment it is let go, so one hand has
to hold it tipped while the other takes the reel that now stands proud; it is then lifted off its hook and slid out
along its own plane, 354 mm long, past two neighbours 0.5 mm away that must stay seated.

## Scene and parts (true size)

| Item | Real product | Source | Modelled as |
|---|---|---|---|
| Tape feeders (3) | Opulo LumenPnP 8 mm feeder with its spool arm | [maker page](https://opulo.io/products/feeder), [mounting guide](https://docs.opulo.io/feeders/4-mounting/mounting/), [overview](https://docs.opulo.io/feeders/1-overview/feeder-overview/), design files (the `opulo-inc/feeder` repository on GitHub) (frame-8 envelope 164.8 x 82.8 x 15 mm) | black body 158 x 15 x 42 mm, hook nose, clip, yellow sprung lever, spool arm, 0.29 kg with reel (estimate) |
| Reels (3) | Standard 7 inch (178 mm) reel of 8 mm component tape, EIA-481 | maker spec sheet ("standard 7 inch reel") | white disc 178 x 13 mm, fixed to the spool arm |
| Rail | 20 x 20 mm V-slot aluminium extrusion, 100 mm length, black | maker: feeders "mount onto any 20mm x 20mm v-slot extrusion" | core with its top-face slot, simplified to a 4.1 mm wide, 4 mm deep channel (a real V-slot groove is 6.2 mm wide at the face with 45 degree lips), two feet |
| Slot blocks (3) | LumenPnP feeder slot (printed block with alignment peg) | mounting guide photos | yellow printed PETG block 14.5 x 6 x 26 mm with a 5 x 5.5 x 6 mm peg (drawing in fixture/) |
| Rail feet (2) | Aluminium 10 x 20 mm post on a 26 x 60 x 6 mm plate, screwed to the rail end and clamped to the bench | custom, drawing in fixture/ | fixed |
| Mat | Anti-static bench mat, blue, cut to 300 x 420 x 3 mm | any electronics supplier | loose slab, 0.5 kg |

Estimates, each marked as such in `task.py`: the 15.5 mm pitch (15 mm feeder plus clearance), the hook, clip, peg and
lever dimensions (sized from the maker's photos; the exact printed geometry is not reproduced), the masses (printed
frame, two small motors and board, a loaded paper-tape reel), and the lever spring. In an offline MuJoCo check of this
geometry the catch let go at a few newtons pressed on the pick end, and the reel then stood 30 to 45 mm proud of its
neighbours at 13 to 19 degrees; in the GPU runs the demo tipped the middle unit to about 16 to 21 degrees.

Layout (lot A, the screenshot): positions in the `setup` field of task.json. Lot B moves the mat 30 mm toward the robot.

## Scoring (what the code checks)

Whole-trial zeros (0 at any moment, and it stays 0): either outer feeder is knocked loose (its origin, the point on
the rail's centre line under its hook, moves more than 5 mm, or its body turns more than 10 degrees from its starting
pose, which unhooks it or releases its catch); any feeder's body speed exceeds 1.2 m/s (as in a free fall from about
7 cm; the demo's set-down peaks at about 0.6 m/s); any feeder's origin drops 2 cm below the bench top. A missing or
malformed observation of the scene or the hands scores 0 while it lasts (fail closed). The record resets only at a
scene reset.

End state:

| Score | Condition |
|---|---|
| 0 | either outer feeder's origin more than 2 mm, or its body more than 3 degrees, from its starting pose; or any point of the middle feeder (every box of the body and the lever sampled at about 4 mm on its faces; the hook nose and reel sampled on their rims and axes) within 5 mm of the rail, its feet, a slot block or a peg |
| 0.5 | every point 5 mm or more clear of them |
| 0.75 | and: no gripper or fingertip surface point (face-sampled hulls) within 5 mm of any collider of the middle feeder (exact distance to each box and to each solid cylinder, reel faces included); the feeder's across axis within 20 degrees of vertical (lying on a broad side); its lowest point within 5 mm above the bench or the mat top; still (under 5 mm/s and 0.05 rad/s) continuously for 1 s |
| 1.0 | and every sampled point lies inside the mat's outline (seen from above) |

`success()` is `score() == 1.0`. Trials are run on lots A and B and averaged.

## Run

    inspect-robots run -E task=tasks/remove-the-centre-feeder-from-a-populated-bank/task.py -E object_jitter=0 \
        --policy yam_scripted --epochs 2      # the demo: lot A then lot B
    inspect-robots run -E task=tasks/remove-the-centre-feeder-from-a-populated-bank/task.py -E object_jitter=0 \
        --policy dream --epochs 2             # Dream-1

Trial k uses lot k mod 2 (A first). `TASK_LOT=B` in the environment pins every trial to lot B. Run with
`object_jitter=0`. One trial takes about 4 minutes of simulation on an L40S GPU.

## Demo

`demo()` is open loop from the published layout (the only thing it reads per lot is the mat's starting position). The
right hand's open finger presses the middle feeder's pick end down until its catch lets go and the unit turns about its
hook, reel end up. The left hand comes down over the three reels and pinches the middle reel where it now stands proud
of the others, while the right finger still holds the unit turned. The left hand then lifts the hook out of the rail's
slot, moves the unit 25 mm out, levels it with a 20 degree lean of the wrist, slides it out along its own plane until its
pick end has passed the neighbours' reels, carries it over the mat, leans it and lets go so that it settles on its
side. The demo uses `tilt.py` for leaned grips, `lots.py` for the two mat positions and `parts.py` for the jointed parts;
they patch the stock embodiment in this process only and change nothing on disk.

## Near misses

`python near_misses.py` checks every level and its boundaries: seated; lifted 3 mm; tipped 15 degrees with the hook
still in; lifted 30 mm with the clip still over the slot block (0); 4.9 mm and 5.1 mm clear (0 / 0.5); held clear
(0.5); lying flat on the mat released for 1 s (1.0) or 0.9 s (0.5); a fingertip 4 mm away (0.5) and exactly 5 mm away
(1.0); fingertips pinching the reel faces at mid radius (still held: 0.5); 0.5 mm inside and 10 mm over the mat edge
(1.0 / 0.75); flat on the bench (0.75); tilted 25 degrees (0.5); 8 mm above the mat (0.5); a neighbour knocked 6 mm
or tipped 11 degrees at some moment and put back (0); a neighbour nudged 4 mm and back (no zero); a neighbour ending
2.5 mm or 4 degrees off its seat (0) or 1.9 mm or 2.9 degrees (no zero); unknown or NaN hand geometry (0).

## Fidelity

See the `fidelity` field. In addition: the reel cannot turn on its hub (on a real feeder the threaded tape stops it
turning one way only), and the simulator spreads the 0.29 kg over the colliders by volume, which puts about three
quarters of it in the reel (a loaded 7 inch paper-tape reel is roughly 0.15 to 0.2 kg; estimate); the feeder's printed surfaces are given a friction coefficient of 0.9 against the gripper pads and 0.6 against the rail (estimates); the reel cannot spin on its thimble and the tape is not modelled, so a pull on
the reel rim acts on the feeder as a rigid pull; the printed frame's flex and wear-in are not modelled; the lever is
a single hinged link with a preloaded spring.

## Answers to the pre-review

| Pre-review point | How the task answers it |
|---|---|
| Importance: the link to AI hardware is not shown | The context ties the job to building robot electronics: the boards are a robot's motor drivers, encoders and sensors, and the feeder change is the hands-on step of every board change on these machines. |
| Close to the accepted camera quick-release, DIN-rail and carriage tasks (and the packed-shelf book task) | Those parts can be gripped from the start, and the book stays tipped once tipped. Here no grip exists until the unit has been turned out of a 0.5 mm-gap row, one hand must hold that turn while the other takes the part that stands proud, and the unit then leaves along its own plane between two neighbours that must not move. |
| The spool-arm route may make it trivial | It is allowed, and it is not trivial for a two-finger gripper: the spool arm and reel are in the same 15 mm slice as the neighbours', so neither can be pinched until the unit is out of line. |
| Finger access and extraction forces | The access is the point of the task and is shown by the demo; the catch is a preloaded spring that releases at a few newtons on the pick end (an estimate, stated as one). |
| Upright stability | A 15 mm-wide feeder with a 178 mm reel cannot stand upright on its own base; the task asks for it to be laid on its side, which is how a feeder is put down on a bench. No feet or stands are added. |
| Fixed revision, loading and pitch | One feeder revision (the maker's current 8 mm design), one full reel per feeder, one pitch, all fixed in the setup; slot block, peg, rail and feet are drawn in fixture/. |
| Speculative partial-credit stages | Levels are end states that hold after release (clear of the rail; on the bench; on the mat); none needs two separately observable latch stages. |
| No demonstration | Scripted demo at full score in both lots (see scores). |

## Licences

All geometry is modelled from scratch with primitive shapes; no third-party asset is used. The dimensions come from
the maker's public documents and design files, cited above.
