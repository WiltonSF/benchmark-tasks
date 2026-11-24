# Release the GelSlim 4.0 gel sheet from its original mold without tearing it

**Instruction:** Push the acrylic and gel sheet out of the ring, lift one corner of the sheet off the acrylic and peel
it off without tearing it, then lay it flat and leave all three parts apart.

**Category:** Impact on AI timelines. **Run limit:** 2,400 steps (120 s at 20 Hz) per trial; two trials (lots A and
B), averaged.

## The job and why it matters

GelSlim 4.0 is an open-source optical tactile fingertip for robot grippers (Panda Hand, WSG 50). Each fingertip's
contact skin is cut from a 3 mm silicone gel sheet that is cast in the project's own two-part mold: a 3D-printed
hexagonal outer ring with a clear 1/4 in acrylic hexagon pressed into it, paper backing down. The maker's gel-pad
instructions then say:

> "Press the exposed acrylic on the back of the mold to push the gel and acrylic out of the mold."

followed by peeling the sheet off the acrylic one corner at a time, "This will help prevent the gel from tearing."
A torn or holed sheet is wasted: the pads are cut from it after it is painted. The people who do this step are the
builders of GelSlim 4.0 fingertips, as part of fabricating new tactile hardware for robot grippers (maker's gel-pad
steps: https://sites.google.com/view/andreasipos/projects/gelslim-4-0/gel-pad-fabrication).

Building tactile fingertips is part of building robot hands; that is the reason for the category. No measured labour
saving or speed-up is claimed.

What makes it hard for a robot:

- **Getting the acrylic out of the ring.** The acrylic is a press fit held from below by the ring's 5 mm ledge, so it
  can only leave upward. The demo lifts the 362 g mold level by the ring's 9.8 mm rim with both hands, sets the
  acrylic's exposed back on a stand narrower than the ring's opening, lets go, and presses opposite rim flats down together
  so the ring slides off the acrylic, without touching the sheet.
- **Starting the peel.** The cured sheet is flush with the acrylic's edge and stuck to it. The robot has to push a
  plastic pick into the corner's perimeter seam and lift the stuck corner itself, against the bond's opening resistance,
  while the other hand keeps the acrylic from sliding away. The right hand supports the raised flap until the left hand is staged nearby, then moves the pick sideways as the left advances and closes on the sheet. It stores the pick and returns to counterhold while the left peels. Nothing else lifts it.
- **Peeling at the front only.** Each region lets go only when the robot, through the already freed flap, has pulled
  it open against its resistance. Pulling the middle, yanking, dragging the stuck sheet sideways or pinching it hard
  tears it.
- **Holding the stack.** The acrylic only sits on the stand; the demo holds it with one hand while the other works the
  corner.
- **Laying a 170 mm floppy sheet flat.** The freed sheet hangs from one corner and has to be laid down without folds.

## Scene (lot A; lot B moves only the stand)

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the counter.

| Item | Product | Size used | Where (lot A) |
|---|---|---|---|
| Outer ring | printed from the original `gel_mold/outer_ring.STL` (GelSlim 4.0 hardware files), black PLA | 193.2 across corners, 167.3 across flats, 18 tall; 5 mm bottom ledge with a 117.3 mm (across flats) opening; 140 g estimate | on the counter, centre (380, 0), corners toward +x and -x |
| Acrylic base | 1/4 in clear cast acrylic cut from the original `hex_acrylic_base.DXF` | 170.1 across corners, 147.3 across flats, 6.35 thick; 145 g estimate | seated on the ledge (bottom 5 up), back exposed through the opening |
| Gel sheet | cured XP-565 silicone (Silicones, Inc.), 70 g base + 7 g activator, 16-18 h cure (maker's recipe) | same hexagon, 3 thick, 77 g upper estimate from the pour | in the ring on the acrylic, top 14.35 up |
| Push-out stand | [Corning PYREX 3140-90 crystallizing dish](https://ecatalog.corning.com/life-sciences/b2c/US/en/p/3140-90), upturned | 90 across, 50 tall; 190 g estimate | centre (200, 0) |
| Starter tool | [iFixit IF145-123-2](https://www.ifixit.com/products/ifixit-opening-picks-set-of-6), Delrin opening pick | 38 x 36 x 0.5; 8 mm exposed, 30 mm captured by a 30 x 18 x 26 handle body with four stops (30 x 26 x 26 overall);22 g total mass estimate | loose on counter at (350,-220), picked up after ring extraction; [mount](fixture/PICK.md) |
| Millimetre rule | 150 mm steel rule | 150 x 18 | (560, -220) |
| Counter | sealed laminate prep counter with a grey upstand | 1100 x 1300 top; upstand x 711 to 729, 110 tall | fixed |

Lot B: the stand at (226, 0). The mold, rule and counter stay. Trials alternate A, B. The ring's section, the
fit allowance and the source of every size are in [fixture/](fixture/README.md).

## Scoring

Read from the overhead and side videos, a ruler, and before/after macro photos (1 mm rule in view) of both faces of
the sheet.

**Score 0 for the whole trial if, at any moment:**

- the sheet gets any tear, hole or missing piece 1 mm or longer (macro photos);
- the ring, acrylic, sheet, stand or tool falls off the counter.

In the simulation a tear is a stated rule (next section): a still-stuck region pulled up more than 2.5 mm before the
peel front reaches it, a still-bonded front boundary lifted more than5mm before it lets go, a stuck region dragged 3 mm along the
acrylic, or more than 15 N of summed normal robot contact on one 24 mm band. Once set, the zero stays for the trial.

**Otherwise score the end state.** Simulation also requires recorded loaded ring-release history; moving separated parts into place without it earns zero. Every score needs both hands at least 30 mm from the ring, the acrylic and the sheet
for the last 1 s. (The simulation checks 38 mm, as a sampling margin, on the fingers, the gripper body and the wrist
link; the rest of the arm is not checked.)

| Score | Result | How it is read |
|---|---|---|
| 0.25 | The acrylic and the sheet are both out of the ring: no part of either within 5 mm of the ring's outline (its outer hexagon, 18 mm tall); ring flat on the counter, acrylic flat on the counter or on the stand | side video, ruler |
| 0.5 | The acrylic and the sheet out of the ring's outline as for 0.25, and the whole sheet at least 5 mm from the acrylic; the ring and acrylic may be anywhere | side video, ruler |
| 0.75 | As 0.5, and the sheet lies flat on the counter: every region's underside within 1.5 mm above it, no fold or curl (no part tilted more than 10 degrees), not moving | overhead and side video |
| 1.0 | As 0.75, and ring and acrylic at rest as for 0.25, nothing moving more than 1 mm for 2 s, and both hands at least 30 mm from all three for 2 s | video |

Counter placement also requires the complete ring/acrylic footprint inside its bounds. Dish support requires the dish within2 degrees of level on the counter, the acrylic normal within2 degrees of the dish normal, acrylic COM within20 mm of its centre and the whole90 mm support disk inside the hexagonal acrylic footprint. Here "flat" for the ring/acrylic means no more than 10 degrees from level, with the lowest point within 1.5 mm above the supporting counter or stand. The sheet check uses every band's underside and tilt, with no point above 4.5 mm (3 mm sheet plus 1.5 mm gap). Native contact penetration up to 2 mm is a numerical tolerance. "At rest" uses each part's reference-frame speed below 8 mm/s and angular speed below 0.12 rad/s; full score additionally follows every region's corners and requires no corner moving more than 1 mm over 2 s.

Otherwise 0, including a sheet left on the acrylic in the ring, a sheet peeled out of a ring that still holds the
acrylic, and anything still held. In `task.py` distances are taken between the parts' full convex shapes, the hands are
checked with point clouds sampled over the finger and gripper surfaces, and stillness with every part's corners.

## How the simulation models the mold and the sheet

Ring, acrylic and stand are rigid PhysX bodies at true size, collided as their own convex pieces (the ring as 24
sectors with its ledge, cavity and relief). The acrylic sits on the ledge with the 0.2 mm radial print allowance.

The sheet is seven 24.3 mm bands across the hexagon, joined by hinges at mid-thickness with a passive bending stiffness of 0.001 N m/rad toward flat (rest angle 0, never changed). It
folds between bands and does not stretch, so it can only be peeled along the line from one hexagon corner to the
opposite one. The two end bands are the corner triangles.

Adhesion and tearing follow a stated rule, checked at every 240 Hz physics step on each band's underside centre in
the acrylic's frame. The rule only applies forces and records what the robot's contacts have done; it never assigns a sheet pose,
lifts a corner or attaches the sheet to a hand, and no hinge target is changed.

- **Opening resistance.** The figures below were obtained by phone from a technician who works with this product (Nov 2026): about **0.5 N per 24 mm of strip WIDTH** peeled straight off unprimed acrylic; about **7 N ring push-out**, normally 5-15 N, or about 3 N for a worn loose ring. Practical corner starting uses a fingernail, plastic pick or flat blade. These are phone reports, not measurements on the submitted specimens or datasheet adhesion values.
- **Width-aware force.** The maximum transverse front widths within the bands are84.180mm at the two ends and147.315mm in the other five. The20% engineering envelope on the phone strip figure gives **2.105N / 3.683N**. When a robot contacts a front or its connected freed flap, the opposing load at the contacted material point rises over0.4mm of that bonded front's own underside-centre opening, capped at1.01 times these forces (**2.126/3.720N**). A freed flap resting on a pick does not preload the unopened next front. At rest the load does no work. This position law avoids the lagged velocity switch that oscillated at the physics timestep. Positive work is credited only while the actual applied load reaches the original2.105/3.683N envelope; the acrylic receives the opposite force and matching moment. The active front's old normal-opening spring is replaced by this peel load, avoiding double counting. Without qualifying contact it retains a0.4mm normal restoring ramp; non-front bonds retain that width-scaled restoring law. Sliding stiffness stays300N/m. There is no force-free or driven peel.
- **Area work.** For a90-degree inextensible peel, the [Kendall energy balance](https://bdml.stanford.edu/twiki/pub/Rise/AdhesionModels/Kendall75peeling.pdf) gives interfacial work per area approximately force per strip width. Applying the20% envelope gives **25J/m²**, an explicit model inference from the phone figure, not a measured XP-565 fracture energy. Each whole band needs that value times its full hexagonal area: **25.571,75.113,89.497,89.497,89.497,75.113,25.571mJ**. A1mm opening alone no longer debonds24.3mm of sheet. Actual robot contact work is capped by work against the explicit peel load at its material point; translation and rotation relative to the acrylic count. Negative work removes credit once: robot and resisted work describe the same energy flow. Both signed ledgers are combined across connected contacts/fronts before taking their minimum, and a shared flap's work cannot be reused at two fronts.
- **Letting go.** Only a corner or the next bonded front can fully release. It needs the complete area work, at least1mm underside-centre opening and actual contact >=0.05N on that front or connected freed flap. Work fraction represents peeled area; the moving front position is found from the hexagon's actual width profile. No object position or hinge target is assigned.
- **Tearing.** The still-bonded portion of a front fails if its most-open boundary rises over **5mm**. This uses the **previous substep's** peeled-area fraction, so a violent high-work jump cannot erase its prior damage state. Non-front regions still fail at2.5mm opening, bonded material at3mm shear, and any band atover15N summed normal robot/tool contact. The numeric tolerances are unchanged. Applying the front test to already peeled material would not describe a propagating front. These remain uncalibrated failure proxies; they do not establish real tear or pick-puncture tolerance.
- **Press fit.** Only the nominal standard-ring cohort is simulated. At least **7 N actual downward robot load** must accompany **2 mm accumulated push-out travel**, supplying at least **14 mJ**. Unloaded travel and a later stationary touch do not count. Closing travel removes credit. The equal reaction force is **8.3734 N**, including the 140 g ring's **1.3734 N gravity**, so gravity cannot substitute for the required robot load. Physical rings must enroll with 5-7 N peak plus instrument uncertainty; rings requiring over 7 N belong to a separately versioned higher-force cohort that needs new runs. The reported 5-15 N population is not all covered by a 7 N nominal trial. Worn 3 N rings are excluded, not silently substituted.

The0.7/0.55 static/dynamic gel friction,0.4mm resting-bond ramp,300N/m shear stiffness,0.001Nm/rad passive bending and damage proxies are uncalibrated engineering assumptions. The25J/m² inference assumes90-degree peeling without the elastic term; other angles, gel stretch and intact-sheet/start behavior still require enrollment. No intact-sheet/start, bending, damage or handled-tool hardware qualification is claimed. The [XP-565 datasheet](https://static1.squarespace.com/static/5b1ecdd4372b96e84200cf1d/t/5b608e1a758d46a989c0bf93/1533054492488/XP-565%20%281%29.pdf) lists Shore A 10 +/- 2 at 16-18 h, 16 +/- 2 at 7 days, and tear strength 40 +/- 10 ppi. None is acrylic adhesion or pinch tolerance, and none is converted into one here. The required enrollment includes intact-sheet force at the varying front and a held-pick damage check, not a strip test alone. Until those records exist the physical-force/damage gate remains incomplete.

Seven connected bands approximate a continuously bonded sheet; no tape gaps or initially freed region exists. Bonds cover all seven regions, but the resolved front advances in 24.3 mm increments and bending has one axis, so only two of six real corners are available. That is a material simplification of continuous peeling and successive-corner lifting, not proof of continuous-front equivalence.

The sheet's mass is the poured mass (70 g base + 7 g activator). At the datasheet specific gravity of 1.02, a 3 mm sheet over the 188 cm2
hexagon calculates to 57.5 g, so the simulation's 77 g is an upper value; part of the pour stays in the cup and syringes.

Required production-part measurements and enrollment records are listed in [fixture/SPECIMENS.md](fixture/SPECIMENS.md). Strip and intact-sheet peak/start forces must both be recorded, using a low-range peel scale and a separate calibrated instrument covering at least 15 N for ring push-out. No task-specific physical readings are supplied here.

## Shortcuts and what they score

| Shortcut | Result |
|---|---|
| Do nothing, or move the whole mold | parts still nested: 0 |
| Lift the mold by its ring and set it down | the acrylic stays in the ring: 0 |
| Press the ring down on the stand, stop there | 0.25 |
| Grab or press the middle of the sheet and pull | no edge to hold; a stuck middle band lifted 2.5 mm tears: 0 |
| Press down without opening the corner, then withdraw | no opening, no work: the corner stays stuck |
| Lift the corner with no robot contact on it (or touch only a far band) | the corner does not let go |
| Peel the corner and yank the rest up fast | the front band passes 5 mm before letting go: 0 |
| Slide the stuck sheet off the acrylic | 3 mm of drag tears it: 0 |
| Peel the sheet out while the acrylic is still in the ring | 0: the acrylic is inside the ring's outline |
| Peel the sheet and leave it hanging or draped | at most 0.5 |
| Lay the sheet with a fold | 0.5 |

## One-hand check

The demo uses two hands to carry and press the ring. The left hand pinches the acrylic/gel stack from its side while the right hand picks up the loose handled pick and lifts the bonded corner. The left hand first grips the supported flap. After the right hand stores the pick and counterpresses the middle over the dish, the left slides its open pads to the flap interior and closes again. The right maintains central support through the near peel, then grips exposed acrylic while the left finishes. One hand alone was not tried. It is not forbidden: any route that
ends in the scored state counts, but a single pinch on one side of the 167 mm ring has to carry the mold off-centre.

## How this differs from close tasks

- [PR 769](https://github.com/robocurve/benchmark-tasks/pull/769), TK-F8CKFL / revision TK-SMHUE6, also holds acrylic while peeling a flexible layer. It was closed after its four separately taped strips failed the claimed continuous-film construct. This task adds press-fit extraction and flush bonded fragile silicone initiation, preserving the peeled layer as the tactile product. All seven computational regions start bonded across their entire area, rather than intermittent tape joins. Nevertheless seven rigid bands still discretize a continuous peel and restrict corner choices; continuous-front equivalence is an open fidelity limitation, not a solved distinction based only on material names.

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

- *Could pushing or pinching the sheet hard count as fine?* Normal contact magnitudes are summed over every rigid body of both arms
  (links, palms, fingers) on each band; over 15 N on one band is a tear.
- *Could a sheet just short of free, or freed but left in the ring, score?* Clearances are measured between the parts'
  complete convex surfaces, and every positive score needs both the acrylic and the sheet 5 mm clear of the ring's
  whole outline. `near_misses.py` pins both: a 4.9 mm gap scores no more than 0.25; a sheet laid inside the ring
  outline scores 0.
- *Are the materials specified?* See the scene table and fixture/dimensions.json (PLA print settings, paper backing
  kept on the acrylic's back, dry laminate counter, no release agent).
- *Does the robot use a tool the maker does not?* A real named Delrin opening pick is provided, following the technician's plastic-pick method. Its grasp handle is a custom adaptation documented in [fixture/PICK.md](fixture/PICK.md). Pick and handle collide as a loose22 g estimated tool, physically pinched by the right hand. Tool work counts only with opposed same-arm fingertip contacts >=0.5 N each. It counts in final hand clearance and stillness; no follower or state capture exists. The handle and dish-supported procedure remain untested on hardware.
- *Is this just ejecting a part from a mold?* Ejection earns only 0.25; full score needs complete intact peel and flat separated placement.
- *Is the corner start real?* The bonded corner must be physically lifted by colliding pick contact against the width-scaled resistance. No spring changes its rest angle and no code raises the corner. Native revised-run evidence is reported below; hardware tool access, damage and dish stability remain unqualified.

## Run and reproduce

From this folder, with Isaac Lab-Arena and the `yam_arena` embodiment:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --policy yam_scripted --scorer task_score --max-steps 2400 --epochs 2 \
  --instruction "Push the acrylic and gel sheet out of the ring, lift one corner of the sheet off the acrylic and peel it off without tearing it, then lay it flat and leave all three parts apart."
```

For the model run replace `yam_scripted` with `dream`. Trial 1 uses lot A and trial 2 lot B; the score is their mean.
`TASK_LOT=A` or `B` pins one lot. There is no random placement. The scripted demo is open loop from the visible
layout: it never reads the bonds, contact forces or object poses at runtime.

## Reset

A sheet takes 16-18 h to cure, so sheets are never cast between trials. They are cast the day before in logged
batches and kept in their molds; [fixture/SPECIMENS.md](fixture/SPECIMENS.md) has the full procedure, the sources
and the kit prices.

1. Independently log batches of **1-4 sheets**, as reported by phone from a technician who works with this product (Nov 2026). For a minimum A/B pair, cast four sheets: two trial sheets, one strip witness and one intact-sheet/start witness. A ten-trial inventory needs five independent four-sheet batches, each with two witnesses: **20 sheets/molds**, not eleven sheets grouped across sessions.
2. Per mold use the maker's 70.0 +/- 0.5 g base and 7.0 +/- 0.1 g activator, one material lot, logged 20-24 C, weights and pour time. Enroll at 18-24 h with photos and both witness records. These are proposed limits; no yield or consistency records exist. The phone report gives about 16 h at room temperature, or 1-2 h near 60 C; the heated route is not adopted or qualified here.
3. Swap an accepted prepared mold into its lot position, record start/end time and inspect the tool/stand. A few minutes is an estimate, not a timed reset. Photograph both sheet faces afterward and reuse cleaned ring/acrylic in later batches.

**Reproducibility limit:** advance staging answers the board's request for a batch plan, but still consumes a hand-cast multi-hour-cure sheet per trial. It therefore remains outside the local ready-bought/minutes-reset specimen gate. No ready-made replica has been qualified to preserve this bonded-sheet task, and no complete physical batch/yield/force/reset record is claimed. The loose dish support is an uncited adaptation, not established maker workholding.

## Results

Round 2 phone-force/area-work/pick source `f1f2572716d2f17d7c8a33e37a21f5dffe60c472b471bff47b20a041b719b19d`: native demo `demo-20261124T065150` scored **A/B 0.0/0.0**, both stopped by the scripted policy, with zero errors or abstentions. Lot A released the ring at 21.05 s under 7.6 N over 2.00 mm, and the pick released band 0 at 40.55 s after 25.59 mJ; the interior clamp caused damage at 66.6 s (18.5 N on band 0). Lot B did not qualify ring release. Same-source Dream-1 `dream-20261124T074148` scored **A/B 0.0/0.0**, both reaching the 2,400-step limit, with zero errors or abstentions. Neither lot qualified ring or band release. All six Python source hashes and the unchanged host source timestamp match the demo and predate both runs. **No full-score demonstration exists for this revision.**

A motion-only retry used a lighter clamp, shallower counterpress and more lot-B ring travel. It passed the earlier clamp failure but damaged band 3 at 31 N during peeling, followed by NaN object state; that run was interrupted and has no completed final epoch score. The checked source above is retained as the honest fallback; the retry is not feasibility evidence.

Historical pre-review revised source `1302b2da931fc38f9dbd53ebb2228f91ac8f27b6db376bebd6eee4e081383b35`: demo `demo-20261123T223734` A/B 1.0/1.0, Dream-1 `dream-20261123T225333` A/B 0.0/0.0 (step limit). These runs had the lower estimated forces, fingertip start and scoring defects identified by strict review; they do not validate Round 2. M-LUWGCK predates even those runs and shows the original assisted task. Revised media linkage is the coordinator's publication step. No physical robot trial is claimed.

## Revision response to the November 22 verdict

> **Not accepted** (TK-WFE2Q4).

> Construct validity (not met): After a fingertip presses the corner, a simulated spring lifts it about 45 degrees instead of requiring the robot to lift the bonded edge.

The press-triggered release and driven hinge target have been removed. Every band starts bonded. Contact mechanics must open the corner against the width-scaled peel load, and release requires the full area work capped by signed robot work and actual resisted work. Equal reactions act on the acrylic. The only hinge stiffness is passive bending toward
flat; its rest angle never changes. `near_misses.py` checks press-only, no-contact lift, wrong-face work, insufficient
work and force/moment conservation. The phone-force/pick demo and same-source Dream-1 both score 0.0/0.0; see Results. Historical full-score evidence is superseded.

> Clarity and reproducibility (not met): Each trial consumes a hand-cast sheet requiring a 16–18 hour cure, without a specified quickly repeatable preparation or ready-made supply.

`fixture/SPECIMENS.md` specifies independent 1-4-sheet batches, with both strip and intact witnesses included. Four molds yield one A/B pair and two witnesses; twenty molds yield ten trials with two witnesses per four-sheet batch. Weights, temperature, pour/use age, photos, force and yield/reset logs are mandatory but not yet completed. This is a prepared-stock plan, not a ready-made or minutes-reset specimen; the stricter local gate remains unfulfilled.

> Feasibility (not met): The full-score demo uses an automatically lifted corner, so it does not demonstrate the real-world peel start within the action limit.

The automatic lift is gone. The new physical pick route and higher loads require fresh native evidence. Current status is in Results; earlier full scores are historical and cannot establish feasibility of this changed source.

> What to change: The tactile-sensor fabrication task is promising, but the simulated corner currently rises automatically after a press. Change the peel-start model so the robot must physically engage and lift the bonded corner itself, then provide a full-score demonstration and Dream-1 run on the revised task. Also specify a repeatable supply of prepared sheets for repeated trials, including how batches are made consistently and staged in advance of the 16–18 hour cure.

The mechanism, phone-based forces, real starter tool and separately logged batch procedure are changed as described above. Current same-source run status is in Results. Hardware corner/start/full-sheet force, damage, bending, dish workholding and batch yield/reset qualification remain open; no gate is claimed passed by disclosure alone.

> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can claim the idea.


