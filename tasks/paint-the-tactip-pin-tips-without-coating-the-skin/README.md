# Paint all 532 pin tips of a TacTip tactile skin white, bringing every side of its dome to the paint without smearing

The TacTip is an open-source optical tactile fingertip for robot hands. Inside its black cast-rubber dome stand
hundreds of short pins; a camera inside the sensor tracks the white tips of those pins, and their movement is the
touch signal. Before the skin is bonded to the sensor, every pin tip has to be painted white and nothing else: paint
on the black skin between the pins puts false markers in the camera's view. The maker's step, from the Soft Robotics
Toolkit's TacTip fabrication page (section 2, Skin, "Painting pins"):

> "Spread a layer of white paint on a piece of acrylic. Insert your thumb in the skin and press on the white paint.
> Make sure all the pin tips are painted white."

Here the skin is already turned inside out (pins outward, as in the maker's photo of this step) on a printed former
that takes the place of the thumb. The former hangs in a beaker. White enamel is spread on one end of a clear acrylic
bar lying on the table. The robot must paint every pin tip, keep paint off the skin, hang the skin back in the beaker
to dry and set the bar down.

## Why it matters

- The TacTip family is one of the most used optical tactile sensors in robot-hand research (fingertips for the
  Shadow, Pisa/IIT SoftHand, Open-Hand M2 and GR2 hands among others; Lepora, *IEEE Sensors Journal* 21(19), 2021,
  Table II). Its cast skin is still preferred where durability matters: "3D-printed skins would be less robust over
  the long term than their cast counterparts" (Ward-Cherrier et al., *Soft Robotics* 5(2), 2018).
- The marker step is manual and slow. Of cast skins the same paper says "the pins' tips are (painstakingly) painted
  white by hand". A robot that can do it can build the touch sensors for other robots' hands.
- O*NET 17-2199.08, Robotics Engineers, core task: "Build, configure, or test robots or robotic applications."

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
| Skin | Cast TacTip skin of the Soft Robotics Toolkit's Open TacTip (Smooth-On VytaFlex 60, Shore 60A), turned inside out | 40 mm dome with a 25 mm skirt, 1 mm wall; 532 pins (Lepora 2021, Table II), drawn 1.0 mm across and 2.0 mm long (estimates from the printed TacTip's 1.2 x 2.0 mm pins), tips on a 44 mm sphere, in 15 rings out to 87 degrees (the cast layout is not published) | on the former |
| Painting former | PLA print, fixture/painting_former.scad and .svg | 38 mm dome and body, 24 mm shank, 80 x 4 mm flange, 20 mm square handle 40 mm long; 40 g with the skin (estimate) | hanging in the beaker by its flange, the dome's centre 64 above the table, the handle's flats turned 55 degrees |
| Beaker | Corning Pyrex 1000-250, 250 mL low-form Griffin | 70 across, 95 tall (catalogue), 1.8 mm wall (estimate), 110 g (estimate) | centre x 235, y +195 |
| Paint bar | Clear cast acrylic square bar, 1.5 in (38.1 mm), cut to 6 in (152.4 mm), ends faced flat | 263 g | lying along y, centre x 300, y -85, its painted end toward +y |
| Paint | Humbrol Enamel No. 22 White Gloss (14 mL tinlet) | about 0.15 mm wet film in a 32 mm circle centred on the bar's end (estimate) | on the bar's +y end |

The table is the station's bench. Nothing is fixed: the beaker, the former and the bar are loose. The bar is the
maker's "piece of acrylic" in a form one hand can hold out and tip toward the skin; its 38.1 mm end carries a 32 mm
paint patch, enough for the 40 mm dome.

## Scoring

Read from the overhead and both wrist videos (stepped frame by frame) and, after the trial, from macro photos of the
dome square-on to its apex and from six sides, of both grippers' finger pads, the beaker and the table, with a 10x
loupe for doubtful marks. Before the trial the setter photographs the unpainted skin the same way and makes its pin
map (pins counted ring by ring).

**Score 0 for the whole trial if, at any moment:**
- paint gets on the black skin between the pins, on the skirt or on the former: a white mark 0.5 mm or more across
  in the photos (simulation: a press harder than 19 N; within one contact, the dome sliding more than 2 mm over the
  wet paint while touching it with 0.1 N or more, or turning more than 15 degrees on it in all; a press centred more than 92 degrees from the apex, where the skirt
  meets the paint; any point of the skirt, shank, flange or handle within 0.5 mm of the wet circle);
- a gripper touches the wet paint (a white mark on a finger pad; simulation: a hand point within 0.5 mm of the wet
  circle);
- after the first paint, a painted pin tip touches the table, the beaker or a gripper (white marks on them in the
  photos; simulation: a painted tip within 0.3 mm);
- the beaker is knocked over (tilted more than 30 degrees).

**Otherwise** count the pins whose tip is white (at least half of its end face) against the pin map. Every score
also needs, at the end: the former hanging on the beaker's rim by its flange (upright within 8 degrees, its flange
within 2 mm of the rim's height, its axis within 15 mm of the beaker's, no pin touching the glass); the beaker upright
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

**Simulation scorer** (task.py, `record_press` and `score`). Every control step the simulator reports the force
between the former and the bar. While it is at least 0.3 N and the dome's lowest point toward the bar's end is on
that face, every pin tip within (the pin field's yield + 0.15 mm of paint) of the face's plane and inside the 32 mm
wet circle is painted. The yield is sqrt(F / 8.3) mm for a press of F newtons: 0.6 mm at 3 N, from VytaFlex 60 pins
1 mm across and 2 mm long (E about 2 MPa, an estimate) on a 1 mm skin, so a firmer press paints a wider patch (about
15 degrees of the dome at 3 N), up to 19 N, where the pins would fold over. A contact lasts from the dome touching
the face (reported force over 0.05 N, or within 0.15 mm) until it has been off for 3 control steps; within it, the
slide is the larger of the net drift and the summed path (steps under 0.1 mm ignored) of the dome's material point at
the contact, measured on the face over every step touching the wet paint with 0.1 N or more (so a light wipe whose
force flickers keeps adding to it), and the twist is the dome's turning about the face's normal summed by size (steps
under 0.3 degrees ignored), so a back-and-forth wobble adds up too. The real check of all of
these is the photo: smears and twists leave white on the black skin.

## Bypass and shortcut table (near_misses.py)

| Shortcut | Result |
|---|---|
| One hard straight-down press | 44 pins, score 0 |
| Bar stood face up only, dome leaned up to 60 degrees (more than this wrist gives) | 420 pins, score 0.5 |
| Wiping the dome round the wet face | smear, score 0 |
| A light wipe whose force flickers under 0.3 N (5.6 mm, or 3.6 mm at 0.5 / 0.2 N) | smear, score 0 |
| Scrubbing back and forth 0.6 mm | smear (4.2 mm of path), score 0 |
| Twisting the dome 20 degrees on the paint, or wobbling it +-4 degrees | smear, score 0 |
| Pressing at 19.5 N | pins folded, score 0 |
| Laying the skirt on the paint, with or without force | score 0 |
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

True size: 40 mm VytaFlex 60 dome, 532 pins (layout and 1 x 2 mm pins estimated), Pyrex 1000-250 beaker, 38.1 x
152.4 mm acrylic bar. The skin is rigid; its pin tips form a hemisphere collider. Paint is a declared rule: F N paints
tips within sqrt(F/8.3) + 0.15 mm of the wet face (about 15 degrees of dome at 3 N; a thumb flattens a wider patch);
over 19 N folds the pins; 2 mm of slide or 15 degrees of twist smears. The 0.3 mL film is not used up. A printed
former replaces the thumb.

## Real-world trial

- **Parts and cost (November 2026):** one fresh cast TacTip skin per trial. The TacTip's authors supply cast skins "for
  a small fee" on request; the alternative is to cast them, as the toolkit describes, in Smooth-On VytaFlex 60 (the
  2 lb trial unit is about USD 45, an estimate, and makes dozens of 6.5 g skins) in the TacTip skin mould (the
  toolkit's design page says the skin is moulded and that the TacTip's STL and CAD files are in its Downloads
  section);
  Humbrol No. 22 enamel (GBP 1.89 in the maker's bill of materials); a 1.5 in cast acrylic square bar cut to 6 in
  (about USD 12); a Pyrex 1000-250 beaker (about USD 15); the former, printed from fixture/painting_former.scad (about
  30 g of PLA).
- **Paint preparation:** stir the tinlet for 1 minute; brush about 0.3 mL onto the bar's end with a 6 mm flat brush
  to an even film; wipe the outer 3 mm of the face clean with a swab so the wet circle is 32 mm across; check the film
  with a wet-film comb (100 to 200 micrometres) at the circle's edge; start the trial within 2 minutes of brushing.
- **Safety:** brush and clean in a ventilated area (enamel and thinners are solvent based); cover the arms' finger
  pads with nitrile finger cots, changed when marked.
- **Reading the score:** after the robot stops, lift the former out by its handle, photograph the dome square-on to
  the apex and from six sides, count the black (unpainted) tips ring by ring against the pin map, and look for white
  marks of 0.5 mm or more on the black skin, the skirt, the former, the finger cots, the beaker and the table.

## Reset

1. Wipe the bar's end clean with Humbrol Enamel Thinners and re-paint it as above.
2. Fit a fresh unpainted skin on the former (pulled over it inside out, pins outward) and photograph it for the pin
   map.
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
published size. Its dabs (DAB_TABLE in task.py, written offline by `plan_dabs()` from the same constants; set
`TACTIP_REPLAN=1` to recompute them, which takes about 100 s): 79 dabs, each aimed no more than 78 degrees from the
apex, that cover every direction on the dome out to the last pin ring within 13 degrees, so they do not depend on how
the skin sits on the former. Each is commanded 24 mm past first touch (18 mm with the bar tipped 40 degrees; the arms
give way, so the press is about 2 to 3 N), and the plan uses only poses both wrists reach
with the arms, the former and the bar clear of each other (offline IK and the arms' collision hulls). One hand holds
the bar at its middle and tips its painted end up 0, 22 and 40 degrees (its commanded pose is raised by the sag
measured in a trace); the other dabs the dome straight in and out; halfway, the former is hung back in the beaker and
gripped again half a turn round; then both are put down.

## Scores

- Scripted demo: 1.0 in each of lots A and B (all 532 pins painted, no breaking rule, the former hung on the rim and
  the bar set down, both released and still; each trial done at about 3,620 of the 4,000 steps; 79 dabs, peak press
  14.1 N in lot A and 11.9 N in lot B).
- Dream-1: 0.0 (mean of lots A and B, 0 in each). Both trials ran to the 4,000-step limit without refusing; it painted
  no pin, and in lot A it pushed the beaker about 190 mm late in the trial.

## Sources

- Soft Robotics Toolkit, TacTip: Design; Fabrication (bill of materials: Humbrol white paint, 50 x 50 mm acrylic);
  Fabrication, 2) Skin, "Painting pins". https://softroboticstoolkit.com/tactip
- B. Ward-Cherrier et al., "The TacTip Family: Soft Optical Tactile Sensors with 3D-Printed Biomimetic
  Morphologies", *Soft Robotics* 5(2):216-227, 2018 (open access).
- N. F. Lepora, "Soft Biomimetic Optical Tactile Sensing with the TacTip: A Review", *IEEE Sensors Journal*
  21(19), 2021, Table II (Open TacTip: molded hemispherical soft tip, 40 mm, 532 pins).
- Corning Pyrex 1000-250 beaker; Humbrol Enamel No. 22; cast acrylic square bar (any plastics supplier).
- No third-party 3D assets are used; every shape is built from primitives in task.py.
