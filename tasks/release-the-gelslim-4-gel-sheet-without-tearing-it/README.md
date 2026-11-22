# Release the GelSlim 4.0 gel sheet from its original mold without tearing it

**Instruction:** Get the cured gel sheet out of its mold without tearing it and lay it flat on the counter, leaving the
sheet, the acrylic base and the outer ring apart.

**Category:** Impact on AI timelines. **Run limit:** 2,400 steps (120 s at 20 Hz) per trial; two trials (lots A and
B), averaged.

## The job and why it matters

GelSlim 4.0 is an open-source optical tactile fingertip for robot grippers (Panda Hand, WSG 50). Each fingertip's
contact skin is cut from a 3 mm silicone gel sheet that is cast in the project's own two-part mold: a 3D-printed
hexagonal outer ring with a clear 1/4 in acrylic hexagon pressed into it, paper backing down. The maker's gel-pad
instructions then say:

> "Press the exposed acrylic on the back of the mold to push the gel and acrylic out of the mold."

followed by peeling the sheet off the acrylic one corner at a time, "This will help prevent the gel from tearing."
A torn or holed sheet is wasted: the pads are cut from it after it is painted. O*NET 51-4072.00 (Molding,
Coremaking, and Casting Machine Setters, Operators, and Tenders) lists the task "Remove finished or cured products
from dies or molds, using hand tools, air hoses, and other equipment" (a supplemental task for that occupation).

Robots that can make the parts of their own touch sensors shorten the loop of building and repairing robot hands.
That is the reason for the category; no measured labour saving or speed-up is claimed.

What makes it hard for a robot:

- **Getting the acrylic out of the ring.** The acrylic is a press fit held from below by the ring's 5 mm ledge, so it
  can only leave upward. The demo lifts the 362 g mold level by the ring's 9.8 mm rim with both hands, sets the
  acrylic's exposed back on a stand narrower than the ring's opening, lets go, and presses a corner of the ring down
  so the ring slides off the acrylic, without touching the sheet.
- **Starting the peel.** The cured sheet is flush with the acrylic's edge. Nothing lifts until a fingertip is pressed
  into a corner's cut edge below the sheet's top face.
- **Peeling at the front only.** Each region lets go only when the region beside it is already free and its own edge
  is held up for a moment. Pulling the middle, yanking, dragging the stuck sheet sideways or pinching it hard tears it.
- **Holding the stack.** The acrylic only sits on the stand; the demo holds it with one hand while the other works the
  corner.
- **Laying a 170 mm floppy sheet flat.** The freed sheet hangs from one corner and has to be laid down without folds.

## Scene (lot A; lot B moves only the stand)

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the counter.

| Item | Product | Size used | Where (lot A) |
|---|---|---|---|
| Outer ring | printed from the original `gel_mold/outer_ring.STL` (GelSlim 4.0 hardware files), black PLA | 193.2 across corners, 167.3 across flats, 18 tall; 5 mm bottom ledge with a 117.3 mm (across flats) opening; 140 g estimate | on the counter, centre (380, 0), corners toward +x and -x |
| Acrylic base | 1/4 in clear cast acrylic cut from the original `hex_acrylic_base.DXF` | 170.1 across corners, 147.3 across flats, 6.35 thick; 145 g estimate | seated on the ledge (bottom 5 up), back exposed through the opening |
| Gel sheet | cured XP-565 silicone (Silicones, Inc.), 70 g base + 7 g activator, 16-18 h cure (maker's recipe) | same hexagon, 3 thick, 77 g | in the ring on the acrylic, top 14.35 up |
| Push-out stand | [Corning PYREX 3140-90 crystallizing dish](https://ecatalog.corning.com/life-sciences/b2c/US/en/p/3140-90), upturned | 90 across, 50 tall; 190 g estimate | centre (200, 0) |
| Millimetre rule | 150 mm steel rule | 150 x 18 | (560, -220) |
| Counter | sealed laminate prep counter with a grey upstand | 1100 x 1300 top; upstand x 711 to 729, 110 tall | fixed |

Lot B: the stand at (226, 0). The mold, rule and counter stay. Trials alternate A, B. The ring's section, the
fit allowance and the source of every size are in [fixture/](fixture/README.md).

## Scoring

Read from the overhead and side videos, a ruler, and before/after macro photos (1 mm rule in view) of both faces of
the sheet.

**Score 0 for the whole trial if, at any moment:**

- the sheet gets any tear, hole or missing piece 1 mm or longer (macro photos);
- the ring, acrylic, sheet or stand falls off the counter.

In the simulation a tear is a stated rule (next section): a still-stuck region pulled up more than 2.5 mm before the
peel front reaches it, a front region lifted more than 5 mm before it lets go, a stuck region dragged 3 mm along the
acrylic, or more than 15 N of robot contact on one 24 mm band. Once set, the zero stays for the trial.

**Otherwise score the end state.** Every score needs both hands at least 30 mm from the ring, the acrylic and the sheet
for the last 1 s. (The simulation checks 38 mm, as a sampling margin, on the fingers, the gripper body and the wrist
link; the rest of the arm is not checked.)

| Score | Result | How it is read |
|---|---|---|
| 0.25 | The acrylic and the sheet are both out of the ring: no part of either within 5 mm of the ring's outline (its outer hexagon, 18 mm tall); ring flat on the counter, acrylic flat on the counter or on the stand | side video, ruler |
| 0.5 | The acrylic and the sheet out of the ring's outline as for 0.25, and the whole sheet at least 5 mm from the acrylic; the ring and acrylic may be anywhere | side video, ruler |
| 0.75 | As 0.5, and the sheet lies flat on the counter: every part within 1.5 mm of it, no fold or curl (no part tilted more than 10 degrees), not moving | overhead and side video |
| 1.0 | As 0.75, and ring and acrylic at rest as for 0.25, nothing moving more than 1 mm for 2 s, and both hands at least 30 mm from all three for 2 s | video |

Otherwise 0, including a sheet left on the acrylic in the ring, a sheet peeled out of a ring that still holds the
acrylic, and anything still held. In `task.py` distances are taken between the parts' full convex shapes, the hands are
checked with point clouds sampled over the finger and gripper surfaces, and stillness with every part's corners.

## How the simulation models the mold and the sheet

Ring, acrylic and stand are rigid PhysX bodies at true size, collided as their own convex pieces (the ring as 24
sectors with its ledge, cavity and relief). The acrylic sits on the ledge with the 0.2 mm radial print allowance.

The sheet is seven 24.3 mm bands across the hexagon, joined by hinges at mid-thickness (0.001 N m/rad spring). It
folds between bands and does not stretch, so it can only be peeled along the line from one hexagon corner to the
opposite one. The two end bands are the corner triangles.

Adhesion and tearing follow a stated rule, checked at every 240 Hz physics step on each band's underside
centre in the acrylic's frame:

- Each stuck band is pulled onto the acrylic with 0.45 N and held where it was cast by a 300 N/m in-plane spring
  (0.9 N at the 3 mm tear limit), with equal reactions on the acrylic.
- A peel can only start at a corner: the corner band lets go when a fingertip presses at least 0.3 N into its cut edge
  within 8 mm of the corner and below the sheet's top face (at most 2.5 mm above the acrylic). Pressing on the top face,
  or any other contact, does not start it. A simulated fingertip cannot slide into a zero-thickness bond line, so the
  freed corner's hinge is then driven up (a 0.001 N m/rad spring toward 63 degrees; the 4 g corner settles near 45
  degrees), standing in for the fingertip lifting it. The spring is released as soon as the next band lets go.
  Everything else, from pinching the raised corner to laying the sheet down, is ordinary contact physics.
- A stuck band next to a free band (the peel front) lets go once its centre has been held 1 mm or more above the
  acrylic for 0.2 s. Lifted 5 mm before that, it tears.
- A stuck band that is not at the front tears if lifted 2.5 mm (pulled from the middle), and any stuck band tears if
  dragged 3 mm along the acrylic. More than 15 N of robot contact on one band tears it.
- The ring's press fit is a 2.5 N pull holding the ring onto the acrylic until the ring has been pushed 2 mm down past
  it; then the ring is free.

These numbers are estimates chosen to make the maker's warnings concrete, not measurements: the maker gives no peel
force. For scale, XP-565 is listed at Shore A 12-16 with 40 ppi tear strength. A real trial reads tears directly.
Because the bands only hinge across one axis, the simulated sheet can be peeled only from the two corners on that axis;
a real sheet can be started at any of its six corners.

The sheet's mass is the poured mass (70 g base + 7 g activator). At about 1.1 g/cm3 a 3 mm sheet over the 188 cm2
hexagon is nearer 62 g, so the simulation's 77 g is an upper value; part of the pour stays in the cup and syringes.

To check the numbers on real sheets (not done here): clamp the acrylic, peel a 24 mm strip cut from a cured sheet with
a spring scale held at 90 degrees and read the steady force (the model uses 0.45 N per band), and peel test strips
faster and faster to find the lift rate at which the front tears (the model allows about 20 mm/s at the front).

## Shortcuts and what they score

| Shortcut | Result |
|---|---|
| Do nothing, or move the whole mold | parts still nested: 0 |
| Lift the mold by its ring and set it down | the acrylic stays in the ring: 0 |
| Press the ring down on the stand, stop there | 0.25 |
| Grab or press the middle of the sheet and pull | no edge to hold; a stuck middle band lifted 2.5 mm tears: 0 |
| Pinch the sheet edge from above and pull up | the corner has not been started; lifting it tears: 0 |
| Peel the corner and yank the rest up fast | the front band passes 5 mm before letting go: 0 |
| Slide the stuck sheet off the acrylic | 3 mm of drag tears it: 0 |
| Peel the sheet out while the acrylic is still in the ring | 0: the acrylic is inside the ring's outline |
| Peel the sheet and leave it hanging or draped | at most 0.5 |
| Lay the sheet with a fold | 0.5 |

## One-hand check

The demo uses two hands: both pinch the ring's rim to carry the 362 g mold level, and one holds the acrylic on the
stand while the other starts and peels the corner. One hand alone was not tried. It is not forbidden: any route that
ends in the scored state counts, but a single pinch on one side of the 167 mm ring has to carry the mold off-centre.

## How this differs from close tasks

- TK-A8X2KJ (PR 1949) flexes a printer build plate to pop a rigid print off it; here the substrate is rigid and the
  fragile part is the flexible sheet, which has to be peeled progressively without tearing.
- TK-XTCCB5 (PR 2321) breaks away sacrificial supports; nothing here may break.
- TK-63RTRA (PR 2019) scrapes stuck debris off a griddle; here the stuck layer is the product and must stay whole.
- TK-KDXU22 knocks a spent espresso puck out of a portafilter: ejection by impact, and the puck is waste. Here the
  acrylic is pressed out gently and the cast part must survive.
- TK-VT2J8N peels a cucumber with a peeler, cutting the skin away; here nothing is cut and the peeled layer is the
  product.
- A muffin-liner peel (PR 2561, closed) peels a flexible cup from a soft food; here the sheet first has to be freed
  from a press-fit mold and is peeled from a rigid casting surface.

## Checks against likely objections

- *Could pushing or pinching the sheet hard count as fine?* Robot contact is summed over every rigid body of both arms
  (links, palms, fingers) on each band; over 15 N on one band is a tear.
- *Could a sheet just short of free, or freed but left in the ring, score?* Clearances are measured between the parts'
  complete convex surfaces, and every positive score needs both the acrylic and the sheet 5 mm clear of the ring's
  whole outline. `near_misses.py` pins both: a 4.9 mm gap scores no more than 0.25; a sheet laid inside the ring
  outline scores 0.
- *Are the materials specified?* See the scene table and fixture/dimensions.json (PLA print settings, paper backing
  kept on the acrylic's back, dry laminate counter, no release agent).
- *Does the robot use a tool the maker does not?* No: it uses its fingertips, as the maker uses fingers, and a stock
  glass dish as the push-out stand.
- *Is this just ejecting a part from a mold?* The scored core is the peel: starting it at a corner, keeping the peel
  at the front, holding the base, and laying the freed sheet flat. Ejection is only the 0.25 step.
- *Is the corner start real?* Partly. A fingertip pressed into the corner's cut edge below its top face is checked by
  contact (0.3 N, within 8 mm of the corner, lower 2.5 mm of the 3 mm edge); the lift
  that follows is a stated spring (see the rules above), because a simulated fingertip cannot slide into a
  zero-thickness bond line.

## Run and reproduce

From this folder, with Isaac Lab-Arena and the `yam_arena` embodiment:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --policy yam_scripted --scorer task_score --max-steps 2400 --epochs 2 \
  --instruction "Get the cured gel sheet out of its mold without tearing it and lay it flat on the counter, leaving the sheet, the acrylic base and the outer ring apart."
```

For the model run replace `yam_scripted` with `dream`. Trial 1 uses lot A and trial 2 lot B; the score is their mean.
`TASK_LOT=A` or `B` pins one lot. There is no random placement. The scripted demo is open loop from the visible
layout: it never reads the bonds, contact forces or object poses.

## Reset

1. Cast a fresh sheet for each trial: 70 g XP-565 base plus 7 g activator, mixed, centrifuged and degassed as the
   maker describes, poured into the assembled mold on a level surface and cured 16-18 h. Record batch, temperature and
   cure time.
2. Before the trial, photograph the sheet's top face with a 1 mm rule; reject a sheet with a defect of 1 mm or more.
   Each trial uses up one sheet, so two trials need two molds or two cure cycles.
3. Place the mold and the stand at their lot positions (fixture/dimensions.json).
4. After the trial, photograph both faces of the sheet; a used sheet goes on to painting and cutting. A rubber mat on
   the floor by the counter protects the glass dish if it is knocked off.

## Results

**Scripted demo: 1.0 in each of lots A and B** (mean 1.0). In both trials the ring's press fit let go when the right
hand pressed the ring's near corner (22.8 s), the corner started at 29.9 s, the seven bands let go one after another
between 37.3 s and 50.5 s with no tear rule triggered, and the sheet was laid flat on the counter and released. The
trials ended on full score at 72.2 s (1,443 of 2,400 steps). The largest robot contact on any band was 10.8 N (lot A)
and 6.9 N (lot B), from the hand holding the stack down; the limit is 15 N. The demo is open loop and never reads the
bonds, forces or object poses.

**Dream-1: 0.0** (0 in each of lots A and B). Both trials ran to the 2,400-step limit without refusing; the hands
moved near the mold but never lifted it, so the acrylic never left the ring.

These are simulation results under the stated rules; no physical robot trial or real-sheet calibration was done.
