# Release all 60 molded tablets from a Torpac tablet-mold plate onto parchment without crumbling any

**Instruction:** Release all sixty tablets from the mold plate onto the parchment without breaking any, and set down
anything you pick up.

**Category:** Other. **Run limit:** 1,600 steps (80 s at 20 Hz) per trial; two trials (lots A and B), averaged.

## The job and why it matters

Category Other, laboratory formulation and R&D. Molding small batches of tablets by hand is a routine bench step in
tablet formulation development, in compounding and in supplement labs: a new blend is molded, released, inspected and
tested many times before anything is scaled up. Torpac's ProMold RDTM2G is sold for exactly that work ("For Use in
R&D, Pharmacy Compounding, Nutritional Supplements"); the 75 mg model, X-MRDT60x075, makes 60 tablets at a time in a
black, PTFE-coated aluminium cavity plate with tapered through-holes. A robot that runs such formulation work at the
bench (as self-driving labs do for other materials) has to get each batch out of its mold intact; a crushed tablet
spoils the weight, hardness and dissolution checks that follow.
After baking and cooling, the cavity plate is lifted off its base plate, turned over onto parchment paper, and the
tablets are pushed out. The maker's brochure (2509) gives that step as:

> "Invert & rest Mold Plate at an angle and use Spreader to push out tablets."

and its instruction sheet (2001) adds "Hold Spreader at an angle and sweep down to carefully push out tablets",
"Do not use any metal tools", and "Rapid Dissolve Tablets are more fragile than standard tablets." No dose, active
ingredient, sterility or patient use is involved: the tablets are blanks of a commercial base.

What makes it hard for a robot:

- **A force window, not a position.** Each tablet is held in its hole until a push on its back face frees it. Too
  light a sweep (or a sweep that hovers just above the plate) frees nothing; pressing too hard, striking the plate
  or poking a tablet with a gripper finger crushes it, and one crushed tablet spoils the batch. The robot has to bring
  a thin plastic tool down onto a hard plate and keep a gentle, even press while it moves.
- **Cover all sixty holes.** The spreader's edge must pass over every hole while pressing; a sweep that misses a
  column or lifts early leaves tablets in the plate.
- **Don't knock the plate.** The plate only rests on two bars. Shoving it 10 mm or tipping it ruins the batch.
- **Leave the tool.** The spreader must be put down and the hands moved clear.

## Scene

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the counter.

| Item | Product | Size used | Where |
|---|---|---|---|
| Cavity plate | Torpac ProMold RDTM2G X-MRDT60x075 cavity plate (US$470 for the whole mold, brochure 2509), hard-anodized PTFE-coated aluminium, upside down | 120 x 152 x 3.7; 60 tapered holes, 5.5 mm narrow end up, 6.5 mm wide end down, 6 across x 10 along at 12.5 x 14 pitch (pitch estimated from the maker's photos); 166 g (estimate) | centre (300, 0), long side along y, resting on the bars; back face 16.6 up |
| Tablets | blank (no active ingredient) rapid-dissolve tablets molded in that plate, e.g. MEDISCA Medi-RDT Base 2772, one of the bases the maker lists | one per hole, 75 mg nominal (modelled as 8-sided tapered prisms 3.45 tall) | in their holes, back faces 0.1 below the plate's back face |
| Bars | two 1/2 in (12.7) square 6061 aluminium bars, 6 in (152.4) long | 66 g each | standing on the parchment along y under the plate's long margins, centres x 246.35 and 353.65, y 0 |
| Parchment | baking parchment sheet | 260 x 300 | centred (300, 0) |
| Spreader | the mold's white plastic Powder Spreader (rigid plastic card) | 90 x 70 x 3, rounded lower edge, 18 g (size, mass and friction estimated from the maker's photos) | standing upright in the block's slot, its 90 mm length along x, top 75 up |
| Spreader block | wooden block with a slot (a simple fixture, fixture/README.md) | 100 x 40 x 20, slot 4.5 wide x 15 deep | lot A centre (220, -200); lot B (410, -190) |
| Counter | the station's preparation counter | 1100 x 1300 | fixed |
| Load cell (real trial only) | 5 kg bar load cell with an HX711 board logging to a laptop, under a 300 x 350 platform (or a scale with peak hold) | platform at least 300 x 350 | on the counter at (300, 0) under the parchment; heights are then taken from the platform top; not modelled (the simulation reads the same push from contact forces) |

The plate, bars, tablets and parchment are in the same place in both lots; lot B moves only the spreader block and
spreader. Trial k uses lot A for even k and lot B for odd k; the screenshot shows lot A.

The bars stand in for the hand in the maker's step 10 ("Rest Cavity Plate on the fingers of one hand"): they hold the
plate up so a freed tablet can drop 12.7 mm onto the parchment.

## Scoring

Read from the overhead, side and end-on videos, a 5 kg load cell (or a scale with peak hold) under the parchment, tared
with the plate, bars and tablets in place and logging at 80 Hz or more so a short jab is not missed, a photo of the parchment taken after the trial (the
assessor lifts the plate off), and a 0.5 mm scale with a 10x loupe.

**Score 0 for the whole trial if, at any moment:**
- the load cell reads more than 612 g above its tare: more than 6 N pushed down on the plate by the spreader and the
  grippers together (pressing too hard, jabbing, or striking the plate);
- a gripper touches a tablet;
- any tablet is cracked, chipped (a missing piece 0.5 mm or more, loupe and scale) or crushed;
- the plate is knocked: it slides more than 10 mm or tilts more than 5 degrees on its bars (overhead and side video);
- a tablet falls off the counter.

The simulation applies the same rules (push and touch from contact forces at every physics step; plate and counter
from poses at every control step); the 6 N push limit and the no-touch rule stand in for crushing, which the
simulation cannot model.

**Otherwise** count the tablets that are out of their holes and lying still on the parchment with their top at most
5 mm above it, and score the end state. Every non-zero score needs both hands at least 30 mm from the plate, the
bars, the parchment under the plate and the spreader for the last 1 s (side video).

| Score | Tablets on the parchment | Read with |
|---|---|---|
| 0 | 0 to 14 | photo of the parchment over a numbered 6 x 10 grid |
| 0.25 | 15 to 29 | photo |
| 0.5 | 30 to 44 | photo |
| 0.75 | 45 to 59, or all 60 with something still moving | end-on video, photo |
| 1.0 | all 60, and nothing (plate, spreader, tablets) moving more than 1 mm over the last 1 s | end-on video, photo |

The tablets lie under the plate, so a third camera looks end-on along y through the 12.7 mm gap under the plate (at
parchment height): it shows tablets dropping, lying flat or on an edge, and still. The count is taken at the end of
the trial from that view; the photo after the assessor lifts the plate straight up confirms the count and that each
tablet is whole (a tablet moved by the lift still counts, since it was on the parchment in the end-on view).

A tablet still in its hole, standing on its edge, off the parchment, held, or resting on a fragment does not count.
Trials on lots A and B are averaged.

### Exact simulation rules (task.py)

- **Release rule (declared, not measured).** Each tablet is held up in its hole by a force of twice its weight
  until it is freed. It is freed at the first physics step (240 Hz) at which the spreader's rounded lower edge is on
  the plate (edge bottom within 0.3 mm of the back face along the stretch of edge over the hole), passes over the tablet's hole (the hole centre within 2.75 mm,
  the narrow end's radius, of the edge line, between the edge's ends), and the spreader presses at least 1.0 N on the
  plate and tablets together (PhysX contact force between the spreader and the plate and tablets, read at every
  physics step). Once the edge has moved on (5.25 mm past the hole centre) or lifted 2 mm off, the hold is removed
  and the freed tablet drops through the hole's widening taper onto the parchment under gravity. Only that working
  edge (the long edge standing in the block's slot) counts; the opposite long edge is where the spreader is held, and
  sweeping with it or with a corner frees nothing in the simulation. (Dropping it while
  the edge was still on it launched some tablets so hard that they tumbled; see Results.)
- **Why a press on the plate stands in for the push on each tablet.** A real tablet sits flush in its hole; the
  spreader's edge spans the 5.5 mm hole and rides on the plate around it, so the push it gives the tablet's face is a
  share of how hard it is pressed onto the plate there. The rule therefore uses the measured press of the spreader on
  the plate, applied only while the edge is on the plate over that hole.
- **Seat.** A tablet is 0.02 mm smaller than its hole all round and 3.45 mm tall, so when held up in the taper its
  back face is 0.1 mm below the plate's back face. The spreader's edge therefore rides on the plate and never pinches
  a 75 mg tablet against the hole's rim (in an earlier run a tablet seated 0.15 mm proud was pinched and thrown off the
  counter); the push on each tablet is the declared rule above, and the press it needs is real contact force on the
  plate.
- **Damage rule (declared).** More than 6.0 N pushed down on the plate and tablets by the spreader and both
  grippers together at any physics step (the vertical sum of the PhysX contact forces of the spreader and of both
  arms' wrist, gripper and finger links on the plate and tablets; the load cell in a real trial reads the same push),
  or more than 0.02 N between any gripper link (wrist link, gripper body, either fingertip) and any tablet: the score
  is 0 for the rest of the trial. A tablet below the counter top also scores 0.
- **Knocked plate:** the plate's centre more than 10 mm from its start, or tilted more than 5 degrees, at any control
  step.
- **On the parchment:** a tablet that has let go, whose lowest point is within 1 mm of the parchment's top, whose top
  is no more than 5 mm above it (lying on a face, not on its edge), inside the sheet, moving under 10 mm/s.
- **Hands clear:** sampled points on both arms' wrist links, gripper bodies and fingertips (faces sampled, not only
  hull corners) at least 30 mm from the box enclosing the plate, bars and the space under it, from the plate, and from
  the spreader.
- **Still:** the plate, the spreader and every tablet moved less than 1 mm over the last 1 s.
- **Fail closed:** unknown arm, object or contact state scores 0 for the trial. Scoring is over the whole trial, and
  the record is reset only at a scene reset.

### Why the thresholds

The maker publishes no release or crush force, so both thresholds are estimates, chosen so that a gentle sweep
passes and a firm press or a knock fails. The edge spans a row of six tablets, so 6 N is about 1 N per tablet, and
1 N is about 0.17 N per tablet: a light push, in line with the maker's "Tablet sticking virtually eliminated" and
"Cavity Plate has through holes so you can directly press out tablets" for its PTFE-coated plate. The same 6 N limit
is read on the load cell in a real trial (612 g above the tare), so the simulation and a real trial apply one
number. Before trials, both routes check one tablet: pushed by the spreader's edge alone on the load cell it must
free between 10 and 100 g (0.1-1 N) and must not crumble below 100 g; a batch that fails is remade. The edge must be within 0.3 mm of the plate over a hole: a card rolled so that only one end touches frees
only the holes under that end. The no-touch rule follows the maker, who allows only a human finger to push a stuck
tablet ("Do not use any metal tools"); a robot's hard fingertips are not a finger, so in this task the grippers may
not touch the tablets at all.

## Unintended strategies and what they score

| Strategy | Result |
|---|---|
| Sweep the spreader just above the plate (no press), or with the card rolled so one end touches | nothing (or only the holes under the touching end) lets go: 0 to 0.25 |
| Press the spreader hard or jab it down on the plate | over 6 N: 0 (checked on the GPU: a 25 mm press reached 6.05 N at touch-down) |
| Push tablets out one at a time with a fingertip, or dig them out | gripper contact on a tablet: 0 |
| Strike or tap the plate with a gripper | over 6 N in total on the scale, or the plate shifts: 0 |
| Tip the plate up or drag it off the bars | knocked: 0 (tablets would also still be held) |
| Sweep only part of the plate | the freed count only: 0.25 to 0.75 |
| Keep holding the spreader at the end | hands not clear: 0 |
| Do nothing | 0 |

These cases, and the rule's thresholds on both sides (0.99 N vs 1.0 N, 6.0 N vs 6.01 N, 2.74 mm vs 2.76 mm from a hole
centre, 0.29 mm vs 0.31 mm above the plate, a rolled card), are scored in near_misses.py.

**One hand is enough.** The demo uses only the right arm; the left arm stays parked. The maker's second hand only
supports the plate, which the bars do here.

## How this differs from close tasks

Why it is more than a scrape: a scraper task is judged on what is left on the surface; here 60 separate fragile
parts must each get a push inside a narrow force window, the press is capped for the whole trial by one number on a
scale, and one crushed tablet zeroes the batch. The bars replace the maker's supporting hand ("Rest Cavity Plate on the
fingers of one hand") so a freed tablet has room to drop and a person can repeat the trial exactly; the robot's work
is the maker's other hand, the spreader.

- TK-YTH6M4 (twist an ice-cube tray and tip the cubes out): there the tray flexes and gravity does the rest; here the
  plate is rigid and each of 60 small, fragile parts needs a controlled push through a hole, with crushing as the
  failure.
- TK-A8X2KJ (flex a 3D printer's build plate to pop a part free): one robust part released by bending its plate;
  here no bending is possible and the force window per part is narrow.
- TK-QQSJZE (count tablets on a pill counting tray): pharmacy setting, but sliding loose tablets with a spatula; no
  release from a mold and no crushing limit.
- TK-63RTRA (scrape debris off a griddle with a scraper) and TK-4T97N9 (pack and strike spackle flush with a putty
  knife): a blade dragged over a surface to move material; here the edge must press within a force window over a
  pattern of 60 holes, nothing is scraped, and the failure is crushing fragile parts.
- TK-GJ7LSU (pending: pop one tablet through a blister pack into a cup): one tablet pushed through foil by a
  fingertip; here sixty fragile molded tablets, a tool, a force window and a whole-batch crush rule.

## Real-trial route and cost

- Full route (the maker's process): Torpac X-MRDT60x075 mold, US$470 (brochure 2509 price list); a pack of
  no-active-ingredient rapid-dissolve base (MEDISCA Medi-RDT Base 2772 or PCCA Rapid Dissolve Tablet Powder
  #30-3269), roughly US$50-100 (estimate); a countertop convection oven that holds 110 C, about US$80; two 1/2 in x
  6 in 6061 aluminium bars, about US$10; baking parchment, about US$5; a scrap-wood block with a saw-cut slot; a 5 kg
  load cell with an HX711 board and a plywood platform, about US$15. About US$650 in all. Each trial uses one batch
  molded by the maker's steps 1-8 (fill, tamp, bake 30 min at 110 C, cool 5-10 min, loosen the thumb screws, lift
  the cavity plate off), about 50 minutes per batch with one mold, prepared before the session.
- Lower-cost reference route, no oven, same counts and limits: a resin print of the cavity plate from the dimensions in
  fixture/README.md (about US$20 from a print service) coated with PTFE dry-film spray (about US$10), filled with
  classic molded tablet triturates (lactose with 10-20% sucrose, moistened with 60% ethanol, pressed into the holes
  and air-dried 12 h at room temperature; about US$20 of ingredients), with the same bars, parchment, block and load
  cell: about US$80 with one plate, or about US$160 with five printed plates molded the day before, so each reset is
  only setting a filled plate on the bars (about 2 minutes). The one-tablet release check under "Why the thresholds"
  applies; adjust the moistening or the PTFE coat until a batch passes it. Triturates are a different material from
  the maker's base, but a batch inside the same 0.1-1 N release window that does not crumble below 1 N tests the same
  thing, so runs on either route that pass the check are comparable.

## Fidelity

True size from the maker's brochure (plate 120 x 152, holes 5.5/6.5 mm, 3.7 mm deep, 75 mg tablets). The hole pitch,
the plate, spreader and bar masses, all frictions, and the spreader's size are estimates. Holes and tablets are
8-sided tapered prisms (a tablet 0.02 mm smaller than its hole, seated 0.1 mm below the back face). The spreader is modelled rigid; the arm's own
compliance sets how hard it presses. The release and damage thresholds are the declared rules above, not measured
values. Heat, dose, dissolution and sterility are outside the task. The layout is fixed in each lot, so a memorised
replay of both lots cannot be ruled out; the lots stop a single fixed grasp habit.

## Run it

Yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), stock stack. `parts.py` and `lots.py` in this
folder patch the stock `yam_arena` scene and embodiment classes in the task's own process at import (parts built
from primitives and convex pieces; the two layout lots); nothing on disk changes. No third-party assets are used.

    inspect-robots run --policy yam_scripted -E task=task.py -E object_jitter=0 --max-steps 1600 \
        --epochs 2 "Release all sixty tablets from the mold plate onto the parchment without breaking any, and set down anything you pick up."

Trial k uses lot k mod 2 (A first); `TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot. Submissions run with
`object_jitter=0`. Runtime is about 3 minutes per demo trial on an L40S with video (the demo reaches full score at 27.2 s of simulated time). Setting `TABLET_LOG=<file>` writes a JSON line
every 5 control steps (tablets held, on the parchment, press force, peak, events).

## Results

- **Demo (scripted, open-loop from the published layout, right arm only):** 1.0 in each of lots A and B. The right
  hand takes the spreader from its block, lowers its edge onto the plate's far end margin, presses 8 mm below first
  contact (the arm's compliance gives 1.9-2.5 N in lot A and mostly 1.3-2.2 N in lot B while sweeping; peak 3.2 N),
  sweeps once along the plate over all ten rows at about 20 mm/s, lifts off and lays the spreader flat on the counter.
  All 60 tablets were freed and lay flat on the parchment by 20 s; full score at 27.2 s in both lots. No damage rule
  fired (no gripper contact on any tablet, the plate moved under 1 mm and did not tilt).
- **Dream-1:** 0.0 in both lots (mean 0.0). Both trials ran to the 1,600-step limit without refusing; the arms moved
  slowly back toward their bases, the fingertips never came lower than 98 mm above the counter, the spreader was
  never touched and no tablet was freed.
- **Naive hard press (GPU check of the same scene, lot A):** the same demo with the spreader commanded 25 mm instead
  of 8 mm below first contact pushed 6.05 N on the plate at touch-down (peak 7.0 N) and scored 0 for the trial before
  any tablet was freed.
- **Earlier runs on this design (development):** a 4 mm press gave only 0.4-1.3 N (a 1.5 mm spreader slipped in the
  fingers), freeing 42 of 60; tablets dropped while the edge was still on them were launched and some tumbled (58/60
  and 55/60 on the parchment); a tablet seated 0.15 mm proud was pinched under the edge and thrown off the counter
  (59/60). Each was fixed in the model as described above, not by loosening a threshold.
