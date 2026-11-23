# Coat the pan flutes without bare patches or loose flour

Home bakers grease and flour cake pans before filling them so a finished cake releases intact. Missed creases
can tear the cake on release; excess flour leaves a chalky crust. This task isolates that familiar preparation step
on a reusable, empty Nordic Ware Original 6-cup Bundt pan (51202), using the maker's shortening-and-flour method.

The brush must reach the outer flutes, rounded floor and centre tube. Flour adheres where shortening has been
laid; the pan is then lifted by its two handles and inverted over the half sheet to shed loose flour while
retaining the thin release coat. It finishes upright, tools outside its outline and both hands clear. Coverage,
retained loose powder and lumps determine the score. The two-hand inversion follows tool use on a concave surface;
placing the tools and pan alone earns nothing.

Instruction: "Grease and flour the Bundt pan for baking: no bare spots, no loose flour left in it. Leave it upright
on the sheet pan."

## Run

Run from the task folder:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 \
      --instruction "Grease and flour the Bundt pan for baking: no bare spots, no loose flour left in it. Leave it upright on the sheet pan." \
      --scorer task_score --max-steps 9000 --epochs 2

Trials use lots A and B in turn (trial k uses lot k mod 2) and are averaged; set `TASK_LOT=A` or `TASK_LOT=B` to pin
one lot. Run with `-E object_jitter=0`; no seed changes the scene. One trial is at most 450 s of simulated time
(9,000 steps at 20 Hz); the scripted demo (`--policy yam_scripted`) takes about 404 s. `task.py` uses three helper
files kept next to it: `parts.py` (parts built from boxes, cylinders and convex pieces, and drawn meshes), `lots.py`
(the two layouts) and `tilt.py` (leaned grips, used only by the scripted demo). They patch the running embodiment in
the task's own process and change nothing on disk. There are no third-party assets: every shape is built in
`task.py`.

## Scene and products

| Item | Product | Size and mass | Source |
|---|---|---|---|
| Cake pan | Nordic Ware Original Bundt pan, 6-cup (51202), 10-gauge cast aluminium, silver PFOA-free non-stick inside, about US$38 | 9 1/4 x 8 1/8 x 3 1/4 in (235 mm across the handles, 206 mm body, 83 mm tall); 0.55 kg (estimate: shipping weight 1.37 lb boxed) | WebstaurantStore listing 57A51202; Williams-Sonoma lists the same 9 1/4 x 8 x 3 1/4 in 6-cup pan |
| Its inside | 12 rounded flutes, about 6 mm deep at the outer wall and floor, shallower toward the tube; rounded floor; centre tube 32 mm across at the top with a 25 mm bore; 3.5 mm walls; holds 1.50 L to the brim | estimate from the maker's photos and the 6-cup (1.42 L) capacity | |
| Brush | Nordic Ware natural-bristle pastry brush, 02122, beech | 8 in (203 mm) long, head 1.5 x 0.5 in (38 x 12.7 mm), 0.2 lb (91 g); bristles 55 mm and handle 22 x 11 mm (estimates) | Nordic Ware product page and catalogue |
| Sheet pan | Nordic Ware Naturals half sheet, 43100, aluminium | 17.9 x 12.9 x 1 in (454 x 328 x 25 mm) | Nordic Ware customer service answers |
| Dredger | Tablecraft 159 stainless dredge, 10 oz | 2 13/16 in across, 3.7 in tall (71 x 94 mm); 90 g (estimate) plus 25 g of flour | KaTom listing |
| Shortening | Crisco All-Vegetable Shortening, 120 g (25 mm deep, levelled, at about 21 C) in a 4 oz white porcelain ramekin | ramekin 3.5 x 1.75 in (89 x 44 mm), about 180 g | commodity catalogue size |
| Flour | Gold Medal all-purpose flour, 25.00 g in the dredger | | |
| Spoon rest | white ceramic spoon rest with a raised pad (22 mm) under the brush handle's lower end and a notch (42 mm) near its far end; the brush lies on it rising 14 degrees toward the handle | 200 x 70 mm, 300 g (estimate) | commodity |

Layout (positions are centres in cm, forward from the line through the arm bases, left of their midpoint):

| Item | Lot A (screenshot) | Lot B |
|---|---|---|
| Sheet pan | (36, 0), long side across | same |
| Cake pan, on the sheet pan | (33, 0), handles left and right | same |
| Ramekin, on the sheet pan | (25, -18) | (25, 18) |
| Spoon rest with brush, tips away from the robot | (34, -40), tips at (44, -40) | (34, 40) |
| Dredger, on the counter | (30, 31.5) | (30, -31.5) |

Lots A and B are fixed, mirrored tool layouts. Report both lot scores and their mean; these are two repeatable
instances, with no randomized seed or added pose jitter.

## The maker's method

Nordic Ware's [commercial use-and-care instructions](https://nordicwareic.com/commercial/use-and-care/)
recommend brushing solid vegetable shortening evenly into the pan's recesses, dusting lightly with flour,
and inverting to remove the excess. The half sheet catches the flour in place of the maker's sink. The task
scores the finished coating; the simulation's dredging rule deposits flour over the upright pan.

## Declared shortening and flour rules (`Field` in `task.py`)

The inside of the pan is split into 4,992 cells (96 round x 52 along the profile, 3 to 23 mm^2 each, 663 cm^2 in
all). Each cell carries a shortening thickness, stuck flour and loose flour.

- **Loading the brush.** While bristles are in the ramekin's shortening the brush takes it up at 2 g/s (times the
  share of its bristle points under the shortening's top), up to 3 g.
- **Laying shortening.** The bristles are 90 points in a 38 x 10 x 50 mm block. A point counts as pressing on the
  inside when it is between touching the surface and 15 mm into it (bristles bend; they are not simulated, and the
  bristles have no collider, so the drawn block can pass into the wall where it presses hardest), part of the block is
  still in front of the inside surface, and the bristle's root, at the ferrule, is above the inside or above the rim's
  top out to 15 mm past its outer edge (so brushing the outside of the pan lays nothing inside). Every cell within 4 mm of a pressing point gets at least a 0.03 mm film, and gains 0.08 mm more for
  every second the bristles stay on it, while the brush holds any shortening; the shortening comes off the brush
  (0.9 g/cm^3).
- **Dredging flour.** When the dredger is turned more than 100 degrees from upright, flour leaves its lid at 0.8 g/s
  and falls straight down, spread evenly over a 40 mm radius. Over the pan's inside, while the pan is within 15
  degrees of upright, each cell takes the share that falls on its area as seen from above; the rest lands on the
  sheet pan or, beyond it, on the counter. (Flour falling into a tilted or held-up pan is counted as landing below
  it: the rule covers dredging over a standing pan, the way the maker describes it.)
- **Sticking and sliding.** Loose flour on a cell with at least 0.02 mm of shortening sticks, up to 2 mg/cm^2.
  Loose flour on any cell sloped 45 degrees or more (or facing down) moves each control step to the lowest
  neighbouring cell, off the rim, or down the tube's bore, so it leaves when the pan is turned over. Flour leaving
  the pan lands below where it left: on the sheet pan or the counter.
- **What is read.** A cell is coated when it has at least 0.02 mm of shortening and 0.5 mg/cm^2 of stuck flour.
  A bare spot is a connected patch of uncoated cells whose longest extent is 10 mm or more. A deposit is the
  shortening plus the stuck flour (0.5 mg/mm^3); more than 2 mm is a lump. Loose flour is the loose flour left in
  the pan at the end. The 36 zones are the 12 flutes (ridge to ridge), each split into the outer wall (from the rim
  down to 46 mm deep), the floor (below 46 mm deep) and the tube (from 46 mm deep up to the tube's top).

Simple checks of the rules on the real products (one-off calibration, not part of a trial): weigh the brush before
and after a 3 s dip (load, about 3 g); weigh the pan before and after brushing it (shortening laid, 6-9 g); dust a
greased pan, turn it over, tap it and weigh it again (stuck flour, about 2 mg/cm^2 of inside area, 1.3 g in all);
tilt a greased, floured plate until dusted flour slides (about 45 degrees); hold the filled dredger turned over above
a scale and shake it for 10 s (about 8 g).

These are stated estimates, chosen to match a careful baker's result (about 6-9 g of shortening and a light, even
dusting): they are not measured properties of these products.

## Scoring

Whole-trial rule: 0 if the cake pan's downward speed ever exceeds 0.6 m/s, or if the scene or either arm cannot
be read. The proposed video check is downward travel over 30 mm between frames at 20 fps. End check (all must hold, held still for 1 s): the cake pan upright within 10
degrees, its feet within 4 mm of the sheet pan's floor and its middle inside the sheet pan's walls; the brush and
the dredger out of its outline box (body and handles); no point of either hand (link 5, the gripper and both fingers, hull faces sampled every
3 mm) within 30 mm of it; it moves less than 5 mm/s and 0.05 rad/s. Otherwise 0.

Then, with Z the number of the 36 zones (12 flutes, ridge to ridge, each split into outer wall, floor and tube) with
no bare spot:

| Score | Condition |
|---|---|
| 1.0 | Z = 36, loose flour at most 0.5 g, no lump (deposit over 2 mm), at most 2 g of flour on the counter |
| 0.75 | Z = 36, loose flour at most 2 g |
| 0.5 | Z >= 24, loose flour at most 2 g |
| 0.25 | Z >= 12 |
| 0 | otherwise |

More than 2 g of loose flour, a lump, or more than 2 g of flour on the counter caps the score at 0.25. Trials are
run on each of lots A and B and averaged. `score()`, `readings()` and `observe()` in `task.py` implement this;
`success()` is `score() == 1.0`.

### Reading it in a real trial

Real-trial starting quantities: about 120 g of shortening in the ramekin (25 mm deep, levelled), 25.00 g of flour in the dredger (0.01 g
scale), a clean dry pan. At the end, from the overhead video (20 fps, stepped frame by frame): whether the pan was
dropped, where it stands, whether it was still for 1 s with the hands 30 mm clear (a 30 mm gauge block). Then:

1. Brush the counter outside the sheet pan into a tared cup and weigh it (counter flour).
2. Turn the pan over above a tared tray, tap its rim three times on the tray's edge and weigh what fell (loose
   flour). Turn it back.
3. Photograph each flute's outer wall, floor and tube from straight in, with a steel rule in the frame; 36 photos.
   White is coated; the silver metal shows through where it is bare (shiny, against matt white flour). A zone fails
   if it has a bare patch 10 mm or longer.
4. Lumps: slide a 2 mm feeler gauge beside any visible blob; a blob standing taller than the gauge is a lump.

For a shared rig, fit fresh snug nitrile covers over the fingers, contact pads and finger roots before every
trial, with seams and tape above the contact pads. Fit removable wrist sleeves and place a washable catch cloth
under the entire working area, including the brush rest and dredger. Replace a torn cover immediately. The half
sheet remains in its listed position; powder on the surrounding cloth counts as counter spill and is collected
before cleanup. Remove covers without releasing powder and clean the sleeves, tools and cloth between trials.

Proposed physical controls: a bare pan dusted and turned over (all flour falls out, Z = 0); a greased pan dusted but
not turned over (loose flour well over 2 g); a pan greased only on the ridges (bare flute bottoms).

## Unintended strategies in the simulation

| Strategy | What happens | Score |
|---|---|---|
| Do nothing | no coat | 0 |
| Dust and turn over, no shortening | no flour sticks, all of it falls out | 0 |
| Shortening only, no flour | nothing coated | 0 |
| Grease and dust, never turn over | several grams of loose flour stay in the flutes | at most 0.25 |
| Brush only the ridges or the upper wall | bare flute bottoms, floor or tube | 0 to 0.5 |
| Dab and hold the loaded brush in one place | a lump (0.08 mm/s for about 25 s) | at most 0.25 |
| Pour flour over the side of the sheet pan | counter flour | at most 0.25 |
| Leave the pan upside down, tilted, held, or off the sheet pan | end check fails | 0 |
| Let the pan fall while turning it | whole-trial rule | 0 |
| Leave the brush or dredger in the pan | end check fails | 0 |
| Brush the outside of the pan | nothing laid inside | 0 |
| Coat it, then pick it up again at the end | the 1 s hold restarts | 0 until still again |

`near_misses.py` checks loose flour at 0.50/0.51 g and 2.00/2.01 g, bare patches below/above 10 mm,
24/23/12/11 coated zones, total deposits of 1.99/2.01 mm, counter spill at 1.99/2.01 g, hand clearance
at 29/31 mm, 10/11 degrees of lean, 4/5 mm off the floor, a 0.95/1.25 s hold, unreadable arms and both lots.
The implemented lump threshold is `LUMP = 2.0` mm, which is used by the scorer and these checks.

## What the demo does

`demo()` plays an open-loop plan from the published layout (where the brush, the ramekin, the dredger and the pan
stand); it reads no layer, no score and no simulator pose. The arm on the brush's side takes the brush off the rest
with the handle crossing its fingers at 45 degrees, dips it in the ramekin, and brushes the inside in 42 passes round
the ring (five on the outer wall, eight across the floor, four up the tube, and 25 short arcs where the leaning brush
meets the surface edge-on or the arm reaches farthest: there the outer wall is pressed harder and the foot of the tube
more lightly), reloading six times; it lays the brush back on the rest. Where the leaning handle would come within
6 mm of the tube's top or the rim, the bristles are lifted. In GPU runs the brush settled in the fingers turned 2-4
degrees from the planned grasp, which put the bristles 4-9 mm off toward the arm's own side; every pass is therefore
asked a fixed 6-7 mm the other way (measured over two runs; a fixed correction, not feedback). The other arm takes the
dredger, turns it over above the pan and dusts round the outer and inner parts of the ring, then stands it on the
counter 44 cm out to its side, beyond its starting spot, which lies in that arm's way when it reaches for the pan's
handle. Both hands then come down beside the far end of a handle each with the fingers part open (a 32 mm gap),
slide them over and under the handle, close, lift the pan 9 cm, turn it over above the sheet pan (each wrist turns
half a turn about the line through both handles), hold it 3 s, turn it back and stand it down. The brush is held in
one pose for all its passes, found offline with the arm's
inverse kinematics so that every point is in reach; the left arm's plans are mirror images of the right arm's, apart
from that correction. The hands reach the handles along a straight line in joint space (forward kinematics), because
the straight line in the hand's own space passes poses out of reach.

## How this differs from close tasks

The distinctions below concern the manipulation and assessed result, rather than a change of kitchen object.
Statuses follow the linked PR verdicts checked on November 23, 2026.

- [Finish painting a panel, TK-7P97ME / PR 1530](https://github.com/robocurve/benchmark-tasks/pull/1530)
  (accepted): a roller covers bare regions on a flat panel while preserving existing paint. This task combines
  brush access inside concave flutes with a dependent second coating and selective removal of loose powder.
- [Patch nail holes, TK-4T97N9 / PR 1961](https://github.com/robocurve/benchmark-tasks/pull/1961)
  (accepted): pack local cavities and strike the fill flush with a knife. This task assesses continuous release
  coating across outer wall, floor and tube, followed by inversion to empty the unbound material.
- [Empty a portafilter, TK-KDXU22 / PR 1482](https://github.com/robocurve/benchmark-tasks/pull/1482)
  (accepted after a category revision): an impact releases a compact puck. This pan's loose flour leaves by
  tilting/inversion, while the adhered coating must remain across all 36 inspection zones.
- [Bread a chicken tender, TK-F8ZPEG / PR 3349](https://github.com/robocurve/benchmark-tasks/pull/3349)
  (not accepted; revision open): press and turn a wet food piece in flour while preserving its breading during
  transfer. Here a brush prepares a rigid reusable cavity before dredging; inversion removes surplus while the
  functional coating remains. The specimen is a bought empty pan, used again after washing and drying.
- [Salt half a glass rim, TK-UZ4LZW / PR 3043](https://github.com/robocurve/benchmark-tasks/pull/3043)
  (final rejection for relatability): wet an outer rim band and dip it into salt. Here the brush must reach down
  the outer flutes and up the centre tube before flour deposition and two-hand inversion. The household context
  is preparing a cake pan for release, without the cocktail task's half-rim restriction.
- [Crank a flour sifter, TK-SXWXZR / PR 3192](https://github.com/robocurve/benchmark-tasks/pull/3192)
  (final rejection for overlap and rig contamination): steady a supported tool and turn its crank. Here a held
  brush lays the base coat throughout a concave surface, followed by dredging and controlled inversion; the score
  checks coverage and powder remaining rather than how much flour passes through a mechanism.

## Real-trial kit and reset

Use the commercial pan itself as the repeatable specimen. Shortening and dry flour are shelf-stable stock;
no batter, baked cake, biological specimen, cure or freshly made part is required for any trial. Keep the named
pan and natural-bristle brush; changing the pan profile, brush type or coating materials would need separate
physical calibration.

Planning budget in US dollars, excluding robot, cameras, tax and delivery. These are purchase allowances,
not quotations; the linked listings identify the ready-made parts.

| Item | Quantity | Allowance |
|---|---:|---:|
| [Nordic Ware 51202 6-cup pan](https://www.webstaurantstore.com/nordic-ware-original-bundt-9-14-x-8-18-x-3-14-nonstick-cast-aluminum-bundt-cake-pan-6-cup-capacity-51202/57A51202.html) | 1 | $40 |
| [Nordic Ware 02122 natural-bristle brush](https://www.nordicware.com/products/natural-bristle-pastry-brush/) | 1 | $10 |
| [Nordic Ware 43100 half sheet](https://www.nordicware.com/products/naturals-bakers-half-sheet/) | 1 | $25 |
| [Tablecraft 159 handle-free dredger](https://www.tablecraft.com/10-oz-Dredge-Stainless-Steel) | 1 | $8 |
| 89 x 44 mm porcelain ramekin | 1 | $6 |
| 200 x 70 mm spoon rest plus reusable stock pads/notch blocks set to 22/42 mm | 1 set | $15 |
| 0.01 g resolution pocket scale, suitable for tared loose-flour cups | 1 | $25 |
| Steel ruler, 2 mm feeler gauge and 30 mm clearance block | 1 set | $15 |
| Tared collection tray and cups | 1 set | $10 |
| Nitrile finger covers, removable wrist sleeves, catch cloth, soft nylon cleaning brush and mild detergent | 1 set | $20 |
| Crisco All-Vegetable Shortening and Gold Medal all-purpose flour | 1 retail pack each | $15 |
| **Total allowance** | | **$189 (budget $190)** |

An optional second identical pan and brush adds $50 (about $240 total) and lets a clean, dry set replace the
used set while it is washed. The rest's pads/notch are made or assembled once, outside the trial, and reused.

Before enrolling a pan, check the model number, two handles, twelve flutes and centre tube against the stated
size and layout; reject damaged coating, dents or a loose handle. Check that the brush fits its rest with the
handle raised 14 degrees and tips clear of the counter. Keep a reference photograph and ruler measurements of
the enrolled pan so a replacement can be compared. Use room-temperature shortening at about 21 C and the same
commercial flour for both lots. Verify the 0.01 g scale with a check weight and tare the collection vessels.

Allow 5-8 minutes of active reset, including washing, drying and refilling; this is a planning estimate to time
during physical calibration. Collect the scored flour first. Wash the pan and brush with warm water, mild
detergent and a soft nylon brush; rinse and towel-dry every flute and the tube. Use the spare dry brush if the
bristles remain wet. Empty and clean the half sheet and catch cloth, replace finger covers and clean wrist
sleeves. Level the shortening to 25 mm, refill the dredger to 25.00 g and restore the selected lot's positions.
Start only with a dry, visibly residue-free pan, dry bristles, intact covers and clean handles. The same initial
inspection applies to every trial.

The proposed material checks above and the video/photo/weighing procedure are physical-trial instructions.
The recorded runs below are simulated; physical calibration, reset timing and robot results have not been measured.

## Fidelity notes

- Shortening, flour, bristle bending and adhesion are the declared rules above, not physics. Their numbers are
  estimates.
- The bristles and the brush's ferrule have no collider; the handle does. The flutes' shape and depth are
  estimates from photos; the pan is modelled as 216 convex wall pieces 2 mm behind the drawn inside, a rim, a tube
  top and two handles.
- The dredger releases flour by the declared rate; its holes are drawn, not modelled.
- Tapping the pan is not modelled: loose flour slides by the 45 degree rule alone.
- The scene's flour is drawn as the colour of the pan's inside cells (silver: bare, yellowish: shortening, cream:
  coated, white: loose); flour on the sheet pan and counter is counted, not drawn.

## Scores

Both trials of each run were scored by `task_score` (lots A and B, averaged).

| Run | Lot A | Lot B | Mean |
| --- | --- | --- | --- |
| Scripted demo (`--policy yam_scripted`, 9,000 steps) | 1.0 | 1.0 | 1.0 |
| Dream-1 (`--policy dream`, 9,000 steps) | 0.0 | 0.0 | 0.0 |

Demo, lot A: all 36 zones coated (99.8% of the inside area coated), 0 g of loose flour after the pan was turned
over, thickest deposit 0.97 mm, no flour on the counter, 6.5 g of shortening laid; the pan stood upright on the sheet
pan with both hands 66 mm clear; score 1.0 from 401.5 s of the 404 s plan. Lot B: 36 zones (99.98%), 0 g loose,
1.09 mm, no counter flour, 6.9 g of shortening, hands 66 mm clear, score 1.0 from 401.5 s. Dream-1 ran to the step
limit in both lots without refusing: its hands moved toward the dredger and the ramekin but never took the brush, and
the pan stayed bare (0 zones).
