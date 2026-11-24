# Paint all 532 pin tips of a TacTip tactile skin white, bringing every side of its dome to the paint without smearing

Sensor technicians and researchers paint markers while fabricating cast optical tactile fingertips for robot hands.
The TacTip is one such open-source fingertip. Inside its black cast-rubber dome stand
hundreds of short pins; a camera inside the sensor tracks the white tips of those pins, and their movement is the
touch signal. Before the skin is bonded to the sensor, every pin tip has to be painted white and nothing else: paint
on the black skin between the pins puts false markers in the camera's view. The maker's step, from the Soft Robotics
Toolkit's TacTip fabrication page (section 2, Skin, "Painting pins"):

> "Insert your thumb in the skin and press on the white paint. Make sure all the pin tips are painted white."

Here the skin is already in its turned-out painting shape (pins outward, as in the maker's photo of this step) on a
printed former that takes the place of the thumb. Each trial uses a fresh unpainted skin cast from this task's mould
files (specimen/; procedure and checks in specimen/CASTING.md). The former hangs in a beaker. White enamel is spread on one end of a clear acrylic
bar lying on the table. The robot must paint every pin tip, keep paint off the skin, hang the skin back in the beaker
to dry and set the bar down.

## Why it matters

- TacTip skins have been built as fingertips for several robot hands (the Shadow, Pisa/IIT SoftHand, Open-Hand M2
  and GR2 hands among others; [Lepora, 2021, Table II](https://arxiv.org/html/2105.14455v2#S6.T2)). Cast skins remain one of the
  documented build routes ([Soft Robotics Toolkit, Design](https://softroboticstoolkit.com/tactip/design) specifies a moulded VytaFlex 60 skin).
  Those hand integrations use several different TacTip designs; they do not establish integration of this replica.
  Multi-material 3D-printed TacTip skins can have their markers printed in; cast skins, the kind used here, have
  their pin tips painted.
- The marker step is manual and slow. Of cast skins, [Ward-Cherrier et al. (*Soft Robotics* 5(2), 2018)](https://journals.sagepub.com/doi/abs/10.1089/SORO.2017.0052) say "the
  pins' tips are (painstakingly) painted white by hand". A robot that can do it can build the touch sensors for
  other robots' hands.
- [O*NET 17-2199.08, Robotics Engineers](https://www.onetonline.org/link/summary/17-2199.08), core task: "Build, configure, or test robots or robotic applications."

## What the robot has to do, and why each part is hard

1. **Hold the skin by its former and the paint by its bar.** The former's 20 mm square handle is the only safe grip
   on the skin; the bar is 38.1 mm square.
2. **Bring every part of the dome to the paint.** The 532 pins cover the whole 40 mm hemisphere, out to 87 degrees
   from its apex, all the way round. A pin only takes paint where its tip meets the wet face. A press on a face lying
   flat reaches only the pins within the wrist's lean of the apex (the YAM wrist leans about 45 degrees away from its
   base and turns through about 210 degrees). The rest of the dome has to meet a face tipped toward it, so the other
   hand holds the paint out at an angle, and the skin has to be gripped again half a turn round (or the paint turned)
   to reach the far side.
3. **Dab, do not wipe.** The paint goes on with straight presses (or a true roll). Sliding or twisting the pin field
   on the wet paint drags paint down the pins onto the skin between them; pressing too hard folds the pins over until
   the skin reaches the paint; too light a press paints nothing.
4. **Keep the wet pins off everything else** afterwards: the table, the beaker, either hand.
5. **Put both down**: the skin back on the beaker's rim by its flange with no pin touching the glass, the bar on the
   table with its wet end touching nothing.

## Scene (true size)

Millimetres; world frame from the midpoint between the arm bases, x forward, y to the robot's left; heights above the
table. Lot A is the screenshot's layout; lot B is its mirror image (y negated, the arms' roles swapped).

| Item | What it is | Size, mass | Where (lot A) |
|---|---|---|---|
| Skin | Unpainted black TacTip-type skin cast in Smooth-On VytaFlex 60 (Shore 60A) from this task's mould files (specimen/), in its turned-out painting shape, pins outward; the Open TacTip skin's size and pin count | 40 mm dome with a 25 mm skirt, 1 mm wall; 532 hexagonal pins (count: [Lepora 2021, Table I](https://arxiv.org/html/2105.14455v2)) 1.0 mm across corners and 2.0 mm long, tips on a 44 mm sphere, in 15 rings out to 87 degrees; layout in specimen/pins.csv and specimen/pin_atlas.svg (this task's own: the original mould's layout is not published); 6.4 g computed | on the former |
| Painting former | PLA print, fixture/painting_former.scad and .svg | 38 mm dome and body, 24 mm shank, 80 x 4 mm flange, 20 mm square handle 40 mm long; 40 g with the skin (estimate) | hanging in the beaker by its flange, the dome's centre 59 above the table, the handle's flats turned 55 degrees |
| Beaker | Corning Pyrex 1000-250, 250 mL low-form Griffin | 68 across, 90 tall (catalogue), 1.8 mm wall (estimate), 110 g (estimate) | centre x 235, y +195 |
| Paint bar | Clear cast acrylic square bar, 1.5 in (38.1 mm), cut to 6 in (152.4 mm), ends faced flat | 263 g calculated from [1.19 g/cm3 acrylic density](https://www.perspex.co.uk/Perspex/media/General/technical-library/Typical%20Physical%20Properties/Perspex-Acrylic-Typical-Physical-Properties.pdf) | lying along y, centre x 300, y -85, its painted end toward +y |
| Paint | Humbrol Enamel No. 22 White Gloss (14 mL tinlet) | about 0.15 mm wet film in a 32 mm circle centred on the bar's end (estimate) | on the bar's +y end |

The table is the station's bench. Nothing is fixed: the beaker, the former and the bar are loose. The bar is the
maker's "piece of acrylic" in a form one hand can hold out and tip toward the skin; its 38.1 mm end carries a 32 mm
paint patch, enough for the 40 mm dome.

## Scoring

Read from the overhead and both wrist videos (stepped frame by frame) and, after the trial, from macro photos of the
dome square-on to its apex and from six sides, of both grippers' finger pads, the beaker and the table, with a 10x
loupe for doubtful marks. Before the trial the setter photographs the enrolled, unpainted skin the same way; pins
are counted ring by ring against the numbered atlas (specimen/pin_atlas.svg; 1, 9, 14, 20, 25, 30, 35, 40, 43, 47,
50, 52, 54, 56 and 56 pins from the apex out, counted from the rim's index notch).

**Score 0 for the whole trial if, at any moment:**
- paint gets on the black skin between the pins, on the skirt or on the former: a white mark 0.5 mm or more across
  in the photos. Simulation proxies: an actual loaded wet-face dome contact over 19 N, more than 2 mm relative
  slide or 15 degrees accumulated relative twist under at least 0.1 N, a loaded dome contact centred past 92 degrees,
  or a wet-face contact on the skirt, shank, flange or handle under at least 0.1 N;
- a gripper touches the wet paint (simulation: actual wet-face contact under at least 0.1 N);
- after the first paint, a painted pin tip touches the table, the beaker or a gripper (simulation: actual contact
  under at least 0.1 N at a painted tip, with a 0.6 mm contact-point matching radius);
- the beaker is knocked over (tilted more than 30 degrees).

**Otherwise** count the pins whose tip is white (at least half of its end face) against the atlas. Every score
also needs, at the end: the former hanging on the beaker's rim by its flange (upright within 8 degrees, its flange
within 2 mm of the rim's height, its axis within 15 mm of the beaker's, no pin touching the glass; simulation uses a 0.3 mm final geometric clearance guard); the beaker upright
within 5 degrees, its base on the table, moved no more than 15 mm; the bar resting on the table (lying, or standing
on its dry end) with its wet circle at least 1 mm clear of it; both hands at least 30 mm from the former, the beaker
and the bar; everything still for 1 s.

| Score | Pins painted (of 532) |
|---|---|
| 1.0 | all 532 |
| 0.75 | 479 to 531 (90 %) |
| 0.5 | 266 to 478 (50 %) |
| 0.25 | 133 to 265 (25 %) |
| 0 | fewer |

Trials are run on lots A and B and averaged.

**Simulation scorer** (task.py, `record_press` and `score`). PhysX reports individual contact points, normals,
separations and normal loads. Only non-separated, positively loaded contacts on the wet end inside the 32 mm circle,
with contact normal aligned within acos(0.95) of the face normal, can change paint state. The 0.1 mm numerical
classification tolerance does not create contacts. Dry-side/handle loads are excluded from dome force. A dome contact
is classified by its nearest collision surface (dome hemisphere/edge versus skirt/shank/flange/handle); a loaded contact on the remaining former is contamination.
At F >= 0.4 N, a tip within sqrt(F / 8.3) + 0.15 mm of the face in the wet circle is painted. The 0.4 N painting
press, approximately 2 mm pin height, 1 mm tip diameter and 1 mm wall were obtained by phone from a technician who
works with this product (Nov 2026). The hexagonal pin section and ring layout are task designs.

The footprint law K_FIELD=8.3 N/mm2 (0.6 mm at 3 N), 19 N fold proxy, 2 mm slide and 15 degree twist limits are
uncalibrated estimates, not technician measurements. They remain explicit limitations. Both bodies receive the
physical contact reaction; the rule changes paint bookkeeping only. Relative slip and rotation use both body poses
in the bar frame, so rotating both together adds no smear. Slide is the larger of net drift and accumulated path
(steps <=0.1 mm ignored); twist accumulates absolute rotation about the face normal (steps <=0.3 degrees ignored).
Only loaded wet contacts >=0.1 N accumulate either. Three steps without a qualifying loaded wet contact reset a dab.
Hand/paint and post-paint contamination use continuous PhysX collision contacts, replacing sampled hand proximity.
Hand samples remain only for the sampled 30 mm final-clearance check. The hemisphere collider cannot resolve
individual flexible pins; contact-point matching and transfer remain rigid-envelope approximations. Simple photo
inspection is the real-trial score; simulator forces do not constitute hardware calibration.

## Bypass and shortcut table (near_misses.py)

| Shortcut | Result |
|---|---|
| One hard straight-down press | 44 pins, score 0 |
| Bar stood face up only, dome leaned up to 60 degrees (more than this wrist gives) | 420 pins, score 0.5 |
| Wiping the dome round the wet face | smear, score 0 |
| A light wipe whose force flickers under 0.4 N (5.2 mm, or 3.6 mm at 0.5 / 0.2 N) | smear, score 0 |
| Scrubbing back and forth 0.6 mm | smear (4.2 mm of path), score 0 |
| Twisting the dome 20 degrees on the paint, or wobbling it +-4 degrees | smear, score 0 |
| Pressing at 19.5 N | pins folded, score 0 |
| Actual loaded skirt contact with the wet face | score 0 |
| Zero-force or separated skirt/dome proximity | no paint-state change |
| Bar-only 20 degree turn after full coverage | smear, score 0 |
| Both parts co-rotate 20 degrees | no smear, full coverage remains 1 |
| Dry-side overload or 19.5 N away from the wet end | no painting or fold |
| Pressing on the bar's dry side | nothing painted |
| A reported force with the dome 20 mm off the face | paints nothing |
| An unpainted pin touching the table after the first dab | allowed (only painted pins mark) |
| Skin left in the hand, or held 3 mm above the rim | score 0 |
| Skin hung tilted 10 degrees, or pins touching the glass | score 0 |
| Bar stood on its painted end | score 0 |
| Beaker pushed 20 mm | score 0 |
| A true roll (no slide), a 1.5 mm creep, a 2 degree twist | allowed |

## How this differs from close tasks on the board

- TK-SHQ94B (calibrate a tool tip by touching one point from four angles): a held tool brought to one point at a few
  orientations. Here a curved field of 532 points must each meet a face, which needs dozens of orientations spanning a
  hemisphere, two hands to reach them, and contacts that may not slide or twist.
- TK-7P97ME (finish painting a panel a coworker started): selective coverage of a flat panel with a roller. Here the
  targets are separate raised tips on a held 40 mm dome, and the paint is carried on a face held by the other hand.
- TK-A8X2KJ (flex a build plate to pop a printed robot part free) and TK-XTCCB5 (remove tree supports from a printed
  robot part): robot-part fabrication steps; neither presents a part to a tool face in two hands.

## Fidelity

The 40 mm/532-pin legacy size and the phone-reported 2 mm pin height, 1 mm tip diameter/wall and 0.4 N press are
represented. The task-owned mould defines a replica cast in its outward painting shape, not a qualified sensor skin.
The phone-reported press concerns white silicone paint; transfer with this task's enamel, VytaFlex replica and rigid former has not been measured.
The rigid tip-envelope collider and uncalibrated sqrt(F/8.3) footprint, 19 N fold, 2 mm slide and 15 degree twist
limits approximate compliant paint transfer. The 19 N fold proxy corresponds to about 1.5 mm of field yield
in that estimated law; it is not a measured safe load. The demo's 10.62 N peak implies about 1.13 mm yield, so a
full simulated score does not establish that real 2 mm pins would remain straight. Only actual loaded wet-face contacts update these rules; reactions
act on both free bodies. Pyrex 1000-250 is 68 x 90 mm; wall, mass and omission of its pouring spout are estimates.
The 80 mm flange spans the nominal 68 mm rim; the nominal 64.4 mm bore clears 44 mm tips by 10.2 mm radially when
centred. Enrolment must check the actual rim, spout and pin clearance before use. The printed former replaces a thumb.

## Real-world trial

- **Trial skins:** one fresh enrolled skin per trial, taken from stock, so fitting an already qualified skin is intended to take minutes (not yet timed). Skins are cast
  ahead of time, outside the trials, in batches from the files in specimen/
  (procedure, materials with cited prices, and enrolment checks in specimen/CASTING.md). A printed resin master
  makes up to four Mold Star 15 SLOW silicone moulds; each mould casts one VytaFlex 60 skin per 16-hour cure, so
  four moulds could give at most four skins a day before rejects, two per evaluation (lots A and B); skins rest 7 days before enrolment, so the
  first batch is ready about 8 days after the moulds. This proposed skin is cast in the shape the maker's skin takes when it
  is turned inside out for painting (pins outward), so on the rigid former it has the simulated geometry exactly. Materials listed by Reynolds Advanced Materials on
  2026-11-24: Mold Star 15 SLOW 2 lb trial unit USD 36.77 (four moulds), VytaFlex 60 2 lb trial unit USD 26.00
  (about 80 skins, about USD 0.33 each), Ease Release 200 USD 13.88, black SO-Strong 2 oz USD 16.51 (partial materials USD 93.16); plus resin prints of the master
  and cores (printer or print service, not priced here) and a PLA print of the box. Each skin is enrolled before use:
  532 pins counted ring by ring against the atlas, no defective pin, pin and dome sizes, 6.39 +-0.4 g, snug on the
  former. Prepare at least 8 enrolled skins before trials (4 evaluations); cast a new batch when 4 remain. No skin
  has yet been cast from these files; specimen/check_specimen.py checks the files against the simulation (output in
  specimen/CHECK.txt), and the first cast skin from each mould must pass the enrolment checks before that mould's
  skins are used. The original casting mould files are generally shared only through direct academic
  collaboration (obtained by phone from a technician who works with this product, Nov 2026), and the original
  pin layout is not published, so this task supplies its own mould files.
- **Other parts:** Humbrol No. 22 enamel (GBP 1.89 in the maker's historical bill of materials; [current displayed retail GBP 2.99](https://stocktonmodeller.co.uk/products/humbrol-22-white-gloss-14ml)); a 1.5 in
  cast acrylic square bar cut to 6 in (any plastics supplier); a Pyrex 1000-250 beaker ([Zoro USD 8.39 each, minimum 2 / USD 16.78](https://www.zoro.com/corning-pyrex-250-ml-low-form-griffin-beaker-do-1000-250/i/G3300544/)); the former, printed from fixture/painting_former.scad (about
  33 g of PLA at 20% infill, an estimate). The simulation uses a rounded 40 g combined former/skin mass
  (its coarse skin-mass estimate is 6.5 g); the CAD skin volume instead gives 6.39 g. Neither is a weighed part.
- **Paint preparation:** stir the tinlet for 1 minute; brush about 0.3 mL onto the bar's end with a 6 mm flat brush
  through a 32 mm circular mask centred on the face. Remove the mask and excess paint (0.3 mL is dispensed, not
  retained; a 0.15 mm film over this circle is 0.121 mL); wipe the dry border clean; check the retained film
  with a wet-film comb (100 to 200 micrometres) at the circle's edge; start the trial within 2 minutes of brushing.
- **Safety:** brush and clean in a ventilated area (enamel and thinners are solvent based); cover the arms' finger
  pads with nitrile finger cots, changed when marked.
- **Reading the score:** after the robot stops, lift the former out by its handle, photograph the dome square-on to
  the apex and from six sides, count the black (unpainted) tips ring by ring against the atlas, and look for white
  marks of 0.5 mm or more on the black skin, the skirt, the former, the finger cots, the beaker and the table.

## Reset

1. Wipe the bar's end clean with Humbrol Enamel Thinners and re-paint it as above.
2. Fit a fresh enrolled skin from the batch on the former (pins outward, the skin's rim against the shank, the skin's rim notch
   under the middle of one of the handle's flats; the turn does not affect the score, it only lets pins be named by
   their atlas numbers) and photograph it before the trial.
3. Hang the former in the beaker by its flange, the beaker at its place, the handle's flats turned 55 degrees (lot A;
   -55 in lot B); lay the bar at its place with its painted end toward the beaker's side (+y in lot A, -y in lot B).

## Running it

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Paint every pin tip on the TacTip skin white without getting paint on the skin between them, then hang the skin back in the beaker and set the paint bar down." \
      --scorer task_score --max-steps 4000 --epochs 2

Trial k uses lot k mod 2 (`TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot). Submissions run with
`object_jitter=0`. The scripted demo is `--policy yam_scripted` with the same arguments; it uses tilt.py (a tilted
grip for the scripted demo only) and lots.py (the layout lots). About 200 s of simulated time per trial.

The demo is open loop from the visible layout (where the beaker and the bar lie, the handle's turn) and the skin's
published size. DAB_TABLE in task.py contains the 79-dab offline plan from `plan_dabs()` plus two verified
completion presentations of atlas pins 217 and 26: 81 dabs in lot A. Lot B adds four verified
presentations for residual traced gaps (85 dabs). The plans last 192.7 s and 198.7 s respectively; every dab is aimed no more than 78 degrees from the
apex. The base plan covers the nominal pin map with 13-degree caps; the two completion dabs address gaps found
in contact traces. The skin is indexed on the former as specified in the reset. Each dab is commanded past first touch (26 mm, or 20 mm with the bar tipped 40 degrees) and the arms' position
control limits the load; recorded wet-face peaks are 10.62 N in lot A and 7.54 N in lot B, under the 19 N fold limit;
the plan uses only poses both wrists reach
with the arms, the former and the bar clear of each other (offline IK and the arms' collision hulls). One hand holds
the bar at its middle and tips its painted end up 0, 22 and 40 degrees (its commanded pose is raised by the sag
recorded in a simulation trace, plus a -8 mm x correction in lot B from loaded alignment traces); the other dabs the dome straight in and out; halfway, the former is hung back in the beaker and
gripped again half a turn round; then both are put down.

## Scores

Scripted demo: **A=1.0, B=1.0**, mean 1.0; both lots 532/532 tips, no rule broken, former hung, bar down, hands
clear and still. Peak wet-face loads 10.62 N (lot A) and 7.54 N (lot B). Dream-1, with the same instruction:
**A=0.0, B=0.0**, mean 0.0; on both lots it reached the 4,000-step limit without bringing the skin to the paint.
The runner retried 164 expired Dream-1 requests during the run.

## Sources

- Soft Robotics Toolkit, TacTip: Design; Fabrication (bill of materials: Humbrol white paint, 50 x 50 mm acrylic);
  Fabrication, 2) Skin, "Painting pins". https://softroboticstoolkit.com/tactip
- B. Ward-Cherrier et al., "The TacTip Family: Soft Optical Tactile Sensors with 3D-Printed Biomimetic
  Morphologies", *Soft Robotics* 5(2):216-227, 2018 (open access): https://journals.sagepub.com/doi/abs/10.1089/SORO.2017.0052 .
- N. F. Lepora, "Soft Biomimetic Optical Tactile Sensing with the TacTip: A Review", *IEEE Sensors Journal*
  21(19), 2021, Table I (Open TacTip: molded hemispherical soft tip, 40 mm, 532 pins): https://arxiv.org/html/2105.14455v2 .
- Casting-material prices, Reynolds Advanced Materials (checked 2026-11-24):
  [Mold Star 15 SLOW, USD 36.77](https://www.reynoldsam.com/product/mold-star),
  [VytaFlex 60, USD 26.00](https://www.reynoldsam.com/product/vytaflex-series),
  [Ease Release 200, USD 13.88](https://www.reynoldsam.com/product/ease-release-200),
  [black SO-Strong 2 oz, USD 16.51](https://www.reynoldsam.com/product/strong-color-tints).
  Manufacturer technical data, rather than prices: [Mold Star 15 SLOW](https://www.smooth-on.com/products/mold-star-15-slow/),
  [VytaFlex 60](https://www.smooth-on.com/products/vytaflex-60/), [SO-Strong](https://www.smooth-on.com/products/so-strong/),
  [printed models](https://www.smooth-on.com/support/faq/210/).
- Corning Pyrex 1000-250 beaker; Humbrol Enamel No. 22; cast acrylic square bar (any plastics supplier).
- No third-party 3D assets are used; every shape is built from primitives in task.py, and the mould files are
  generated by specimen/make_mould.py.

## Revision 1 (review of TK-R4PGQ9)

The review, verbatim:

> **Not accepted** (TK-R4PGQ9).
>
> Clarity and reproducibility (not met): The fresh cast skin is a bespoke consumable, but the submission does not
> identify a reliably obtainable matching skin or a specific reproducible mold and acceptance procedure.
>
> What to change: Please make the trial skin reproducible: identify a reliably obtainable unpainted skin matching the
> task, or provide the exact mold files and a batch-casting procedure. Specify acceptance checks for its pin count and
> relevant dimensions so the 532-tip scoring and contact geometry can be reproduced.
>
> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can claim the idea.

There was no earlier review and no Flags note. Changes, point by point:

1. *"does not identify a reliably obtainable matching skin or a specific reproducible mold and acceptance
   procedure"*: the trial skin's digital design and proposed manufacturing route are now specified by files in this folder. Scene table, task.json
   `setup`, Fidelity, Real-world trial and Reset now name one route: a skin cast from specimen/ and enrolled before
   use. No ready-made unpainted skin with a published layout could be found to buy. The original casting mould
   files are generally shared only through direct academic collaboration (obtained by phone from a technician who
   works with this product, Nov 2026), so the review's mould-file alternative is taken. The words
   "estimate" and "not published" no longer apply to the trial skin's geometry: its pins are 1.0 mm across corners
   and 2.0 mm long and its layout is given pin by pin. Runtime geometry agrees with the digital mould, but
   manufactured dimensions, yield and reset time are unverified. The proposed stock is eight enrolled skins;
   no ready stock or matching ready-made purchase route is claimed.

2. *"provide the exact mold files and a batch-casting procedure"*: specimen/ holds the generator make_mould.py,
   which copies task.py's skin constants and pin-layout function verbatim, and its outputs: master.stl (resin
   master: the skin's outside with all 532 pins), core.stl (the skin's inside, which is the painting former's
   radius-19 dome and body, with a seating flange, sprue and vents), box_base.stl and box_tube.stl (pouring box,
   also the sleeve for each skin pour), reference meshes of the silicone mould and the cast skin, pins.csv,
   pin_atlas.svg, mould_section.svg and MANIFEST.sha256. specimen/CASTING.md gives the batch-casting procedure
   (Mold Star 15 SLOW moulds, VytaFlex 60 skins, release, pigment, mix amounts, cure times, four skins a day from
   four moulds), with Reynolds prices and manufacturer technical data. A current beaker price is linked; a bar quote remains unavailable.
3. *"Specify acceptance checks for its pin count and relevant dimensions so the 532-tip scoring and contact geometry
   can be reproduced"*: specimen/CASTING.md section 4 gives an enrolment table with limits and tools: 532 pins counted
   ring by ring against the numbered atlas, pin defects, pin size, the tip sphere (43.9 +-0.4 mm across the 87 degree
   ring, which sets the contact geometry), dome, skirt and wall, mass and fit on the former. The scorer counts painted
   tips against the same atlas. specimen/check_specimen.py imports task.py and checks that the generated files match
   the simulated skin: identical constants and pin directions, pins.csv within 1e-6 mm of task.py's pin roots and
   tips, every mesh closed and consistently oriented, exactly 532 pin tips on the master and the cast-skin reference
   (and 532 pin holes in the mould reference) each on task.py's tip point, and the skin's radii, rim height and apex
   tip equal to task.py's (output in specimen/CHECK.txt).
4. Whole-task re-read: relative-twist scoring and contact gating corrected; phone transfer force applied;
   beaker corrected to 68 x 90 mm and sources added. Final-source evidence appears in Scores.
   Casting yield, complete kit pricing and force/footprint calibration remain open; CAD checks do not close them.

Not done: no skin has yet been cast or measured from these files; the first cast skin must pass the enrolment
checks before a batch is used.

## Further source links (checked Nov 24, 2026)

- [TacTip fabrication and historical BOM](https://softroboticstoolkit.com/tactip/fabrication),
  [painting step](https://softroboticstoolkit.com/tactip/fabrication/skin).
- [Ward-Cherrier et al., 2018](https://journals.sagepub.com/doi/abs/10.1089/SORO.2017.0052).
- [Lepora review, Table I: sensor designs; Table II: hand integrations](https://arxiv.org/html/2105.14455v2).
- [Corning 1000-250 dimensions](https://ecatalog.corning.com/life-sciences/b2b/US/en/General-Labware/Beakers/Beakers,-Glass/PYREX%C2%AE-Beakers,-Low-Form,-Double-Scale,-Graduated/p/1000-250).
- [PERSPEX cast acrylic density](https://www.perspex.co.uk/Perspex/media/General/technical-library/Typical%20Physical%20Properties/Perspex-Acrylic-Typical-Physical-Properties.pdf):
  1.19 g/cm3 is a material estimate for this generic cast bar; its 263 g mass is calculated, not weighed.
- [Humbrol paint chart, No.22 / AA0240](https://d63oxfkn1m8sf.cloudfront.net/4516/0346/0243/Humbrol_Paint_Chart.pdf).
- [Mold Star 15 SLOW](https://www.smooth-on.com/products/mold-star-15-slow/),
  [VytaFlex 60](https://www.smooth-on.com/products/vytaflex-60/),
  [resin curing/sealing guidance](https://www.smooth-on.com/support/faq/210/).
