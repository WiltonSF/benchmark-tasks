# Brush PETG residue off a Prusa MK4S heater block before printing robot parts, keeping the brass brush off its leads

Builders of the Upkie wheeled biped print its PETG legs, wheel hubs and torso plates on Prusa printers. The guide uses an i3 MK3S+ and says other printers can be used with the same settings. Prusa's guide for the current Original Prusa MK4S says to check the hotend before every print and brush old plastic off the nozzle and heater block so it does not fall onto the part. This task is that step: brush a standard PETG film off the heater block's underside while keeping the brush off the leads beside it.

**Instruction:** Brush the PETG residue off the heater block's underside with the brass brush, leaving the tray in place; then lay the brush back on the counter and clear both hands.

**Category:** Impact on AI timelines. **Run limit:** 2,400 steps (120 s at 20 Hz) per trial; lots A and B, averaged.

## Where this step sits in building a robot

- The [Upkie 3D-printing stage](https://upkie.github.io/upkie/printing-3d.html) documents PETG structural parts
  printed on a Prusa i3 MK3S+: femurs, tibias, knees, ankles, wheel hubs and torso plates. It also provides
  print-project files and settings, and allows other printers using those settings.
- The [SO-101 guide](https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/README.md) supplies arm-part
  print plates and names the Prusa MINI+ among example printers. Its material is PLA+, not this task's PETG.
- Prusa's [MK4/S hotend guide](https://help.prusa3d.com/article/regular-printer-maintenance-mk4-s-mk3-9-s_419000)
  says these points should be checked **"before every print"**, explains that debris can fall onto a print, and prescribes heating
  for 3–5 minutes at 250 °C and using a brass brush. Its cable instruction is: **"Avoid contact of the brush with
  the hotend cables, as this could cause a short circuit."**
- [Prusament PETG](https://www.prusa3d.com/product/prusament-petg-jet-black-1kg/) specifies 250 ± 10 °C nozzle
  temperature. Same polymer and starting temperature preserve the proposed hot-cleaning procedure. Coupon
  adhesion and removal resistance have not been compared with naturally accumulated residue.

- [Print & Place](https://hpi.de/oldsite/fileadmin/user_upload/fachgebiete/baudisch/projects/Print___Place/UIST_2026___Print___Place.pdf) (UIST 2026) is a related example from robot fabrication: the printer wipes its nozzle on a fixed brush before and after each pick-and-place step (section 3.1, pp. 3–4) while assembling a small vibrating robot (section 6.3, p. 10). It uses PLA on a Prusa XL and an automatic wipe, not a hand-held brush under the block.

These sources show PETG robot parts printed on Prusa printers and the maker's before-every-print hotend check on the MK4S. None of them lists this step inside a robot build guide.

Why it is hard for a robot:

- **A small, crowded target.** The residue is on a 13 x 20 mm block underside, 76 mm above the build plate, inside
  a fan shroud that is open only at the front, with a nozzle standing 6 mm proud in the middle of it.
- **Pressure inside a band.** Too light and the bristles slide over the film; too hard and the bristles that
  overhang the block's back left face spring up 5 mm or more and touch the leads. The arm yields several
  millimetres under the bristle load, so the depth has to be found, not just commanded.
- **Which way to stroke.** Brushing toward the lead exit pushes loosened residue into it; those strokes do not clean.
- **Hot metal.** Only the brush may touch the block and nozzle; the hands stay clear.

The scored residue is on the block underside, where most of it sits and where the leads make the depth matter. The
nozzle is not coated: a film on a 1.2 to 8 mm cone cannot be applied as one repeatable coupon, and the 4.5 mm ring
left bare round it keeps the coupon off the nozzle shoulder.

## Changes in this revision (answers to the review of TK-ESAUK2)

| Review text | Change |
|---|---|
| "Importance (not met): This scene does not test foreseeable harm to a person, but hotend maintenance for robot-part production could fit Impact on AI timelines." | Category changed from Safety and alignment to Impact on AI timelines. The burn warning is no longer the premise; refusal and safety-test wording is removed. |
| "If this cleaning is part of an actual robot-part fabrication workflow, change the domain to Impact on AI timelines and explain that workflow and the hotend's role in it." | New section above: the Upkie and SO-101 build guides (parts, material and printer family), Prusa's per-print hotend check on the MK4S (quoted), and why residue on the block spoils the next printed part. Print & Place is listed as a related automatic nozzle-wipe example, with its printer, material and wipe differences stated. |
| "The current scene does not meet the Safety and alignment definition: avoiding heat damage to robot fingers and contact with disconnected leads does not establish a foreseeable risk to a person." | Same change; the task no longer claims a safety test. The lead and hot-metal zeros stay as construct rules (maker's cable warning; the shared rig's fingers). |
| "Clarity and reproducibility (not met): Preparing and enrolling 60 individually placed PETG patches has no demonstrated repeatable preparation time and is excluded from the reset estimate." | The 60 hand-placed patches are gone. The residue is one standard PETG coupon, printed 24 per plate and transferred by a proposed stand during the preheat; the bond is unqualified. The reset is a numbered list of about 10.5 min per accepted trial with every step counted, including the batch print (62.5 s per trial); no step is excluded. These times are estimates and are labelled so. |
| "Also demonstrate a repeatable way to prepare and enroll the 60-patch specimens in a few minutes per trial, including batch-preparation time, or use a practical standardized residue that preserves the cleaning skill." | Second option taken: a standardized residue proposal (same polymer, starting temperature, brass brush and block; removal resistance still needs physical qualification), with a photo enrolment check (film over at least 95% of the outline, none in the lead strip). The simulation and scoring now use the same coupon (482 area cells of 0.5 mm), scored by cleared area. |
| "Feasibility (not met): A model can touch the 250 °C hotend and damage unprotected shared-rig gripper pads despite the scoring penalty." / "Provide a heat-resistant gripper protection or another safe trial method that retains the real brushing forces." | All four distal fingers have 1.5875 mm USA Industrials BULK-RS-S50FDA-226 silicone covers, with material and adhesive rated to 260 °C. Matching contact colliders are included. A touch still scores 0. The material rating is not proof of pad insulation; spare-pad thermal qualification is required before a physical robot trial. Hot PETG is retained rather than an unqualified cold substitute. |

## Scene (lot A; lot B moves the brush and the tray)

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left), heights above the
counter.

| Item | Product | Size used | Where (lot A) |
|---|---|---|---|
| Printer | [Original Prusa MK4S](https://www.prusa3d.com/product/original-prusa-mk4s-3d-printer-5/), switched off right after the pre-heat | 500 x 550 x 400, 7 kg (maker); frame, bed, rods, carriage, fans, power supply and control box kept; build plate top 100 up | fixed, centre (305, 0), front facing the arms |
| Heater block | MK4S Nextruder block (installed) | 13 x 20 x 12 (estimate from photos), turned 35 degrees | centre (270, 0, 182) |
| Nozzle | installed brass 0.4 mm nozzle | 6 below the block; 8 across at the shoulder, 1.2 at the tip (estimates) | tip (270, 0, 170), 70 above the plate |
| Fan shroud | [MK4S fan shroud](https://www.prusa3d.com/product/fan-shroud-mk4s/) | walls 25 either side of the block and 27 behind it, reaching 0.5 below the block; open at the front | fixed with the carriage |
| Leads | two heater leads (2.2 across), two thermistor leads (1.3 across) | leave the block's back left face square to it at mid-height (lowest point 4.9 above the underside), 4 to 9 left of the face's centre line; run 12 out, then 29 up | fixed |
| Residue | one standard dark PETG coupon, 0.2 thick, bonded to the block underside | covers the underside to 0.5 from its edges, except a 3 wide bare strip along the lead-exit face and a 4.5 radius hole round the nozzle; 120 mm² (fixture/coupon-cells.csv) | on the block |
| Brush | [Prusa Brass Nozzle Brush](https://www.prusa3d.com/product/prusa-brass-nozzle-brush/), beech, $4.19 | 172 x 12 x 12 wood, 34 mm head with 14 mm brass bristles, 35 g (sizes estimated) | bristles up along x, handle end x 170, head end x 342, y 265 |
| Catch tray | stainless instrument tray | 120 x 80 x 15, 1 mm sheet, 140 g (estimate) | loose on the plate, centre (250, -10) |
| Finger covers | [USA Industrials BULK-RS-S50FDA-226](https://www.fixsupply.com/bulk-rs-s50fda-226-fda-silicone-rubber-sheet-with-high-temp-adhesive-50a-1-16-thick-x-6-wide-x-6-long) silicone, 1/16 in | 1.5875 thick over each distal pad and tip | all four fingers; sliding carriages uncovered |

Lot B: brush handle end at x 90 (head end 262), y 255; tray centre (290, 10). The printer does not move. The
fixture drawing gives the hotend geometry, the coupon outline and the transfer stand.

The lead strip is left bare on purpose: the maker says to keep the brush off the cables, so full credit never needs
bristles at the lead exit. The coupon still runs to within 3 mm of that face, where overhanging bristles meet the
leads if pressed 4.9 mm or more.

## Proposed real-trial kit

A replica carriage holds a ready-assembled Nextruder hotend with the actual fan shroud, lead routing and build-plate
clearance, inside an extrusion frame that reproduces the MK4S frame and gantry obstacles at the listed positions (the
reach path passes through them). An existing MK4S can be used instead. The coupons are printed on any desktop FDM
printer that prints PETG; the budget assumes one is available, as it is wherever robot parts are printed.

Budget in US dollars, excluding tax and shipping:

| Item | Price basis | Budget |
|---|---|---:|
| One ready-assembled standard-flow brass 0.4 mm Nextruder hotend | [PartsBuilt catalogue](https://partsbuilt.com/original-prusa-nextruder-hotend-assembly-brass/), $68.38, checked Nov 23 | $68.38 |
| [Hotend heatsink](https://www.prusa3d.com/product/hotend-heatsink-3/) | Prusa shop, opened Nov 23 | $46.99 |
| [MK4S fan shroud](https://www.prusa3d.com/product/fan-shroud-mk4s/), [fan case](https://www.prusa3d.com/product/fan-case/), [hotend fan](https://www.prusa3d.com/product/hotend-fan-mk4-3-9-xl/) | Prusa shop, Nov 23: shroud $3.59, fan $9.39; fan-case $3.59 historical Nov 21 figure (page unavailable) | $16.57 |
| Prusa brass nozzle brush, two (one for the robot, one for the assessor's reset) | Prusa shop, opened Nov 23, list $4.19 each (sale $3.35) | $8.38 |
| 24 V supply and printer control board, e.g. SKR Mini E3 (stock firmware, thermal-runaway protection on) | Allowance: $25 + $40 | $65 |
| Extrusion frame, steel plate surrogate and printed carriage parts | Allowance: $25 + $10 | $35 |
| Catch tray | Allowance | $10 |
| USB microscope, inspection mirror, scale and heat-resistant gloves | Allowance: $50 + $15 | $65 |
| [Prusament PETG Jet Black, 1 kg](https://www.prusa3d.com/product/prusament-petg-jet-black-1kg/) for coupons | Prusa shop, Nov 23 | $23.99 |
| [USA Industrials BULK-RS-S50FDA-226, 1/16 in, 6 x 6 in](https://www.fixsupply.com/bulk-rs-s50fda-226-fda-silicone-rubber-sheet-with-high-temp-adhesive-50a-1-16-thick-x-6-wide-x-6-long) for four finger covers | FixSupply, opened Nov 23: silicone -80 to 500 °F; adhesive -100 to 500 °F | $9.23 |
| Coupon transfer stand: small laboratory jack, 1.5 mm steel plate, M4 rods and nuts, 0.1 mm PTFE film | Allowance: $25 + $15 | $40 |
| **Total** | **Planning estimate** | **$388.54 (about $390)** |

A spare hotend is optional ($68.38 extra); replacements after damage add their actual cost. Allowances are planning figures, not vendor quotes. Coupon printing requires an existing PETG-capable printer, excluded from this budget and explicitly not supplied by the replica kit. Each coupon uses well under a gram of PETG.

## Score (real trial), read from simple tools

Tools: the overhead and wrist videos, a 60 fps close video of the lead exit, before and after photos of the block
underside (USB microscope with an inspection mirror, 0.05 mm per pixel or finer, a scale in view, the coupon outline
overlaid), a light talc dusting on the leads, a chalk line along the top edges of the brush head, a ruler.

**Score 0 for the whole trial if, at any moment:**

- any part of the brush, bristles included, touches a heater or thermistor lead (close video; the talc shows any
  touch);
- a robot finger touches the hot block or nozzle (wrist and close video; the assessor stops the trial at once; the
  simulation counts any finger surface within 3 mm as touching);
- the brush's wood touches the block or nozzle (the chalk line shows any touch; close video);
- the brush or the tray falls off the counter.

**Otherwise measure the share of the coupon outline left bare** in the after photo (dark film thresholded on the
photo, inside the overlaid outline, registered on the nozzle's hex flats). Film pushed to another spot inside the outline, into the 3 mm lead strip or
onto a lead still counts as present.

| Score | Coupon area cleared | Read with |
|---|---|---|
| 1.0 | 95% or more | before/after photos with the outline overlay |
| 0.75 | 80% to under 95% | same |
| 0.5 | 50% to under 80% | same |
| 0.25 | 25% to under 50% | same |
| 0 | under 25% | same |

Every score above 0 also needs, for the last 2 s of the trial: the brush lying on the counter to the left of the
printer (y over 225, not on the printer or its plate), released; the tray on the build plate, upright within 5°,
its bottom within 2 mm of the plate and the nozzle axis at least 8 mm inside each floor edge; both hands at least
30 mm from the brush, tray, block and nozzle (ruler on the video); nothing moving more than 1 mm. Lots A and B are
averaged.

## Reset, per trial (about 10.5 minutes for an accepted preparation)

Heat-up from 20 to 250 °C takes about 70 s, extrusion of 30 mm about 12 s, and pressing onto the coupon about 8 s:
figures obtained by phone from a technician who works with this product (Nov 2026). These are component times;
complete reset, coupon bonding and repeatability have not been measured. These phone figures refer to the MK4S.
The replica has no powered filament feeder: the assessor hand-feeds 30 mm of marked filament through the cold
heatsink inlet, using 12 s as a planning allocation; its actual heat-up/feed/press times must be recorded.
Other durations below are estimates.

1. Take the after photo through the mirror (1 min).
2. Heat to 250 °C (70 s), hold at temperature for 3 min, then hand-brush remaining film with a second brush and
   heat gloves (1 min). The 3 min hold retains the lower end of the maker's 3–5 min preheat.
3. Extrude 30 mm (12 s). Remove the purge strand; load the new coupon on the PTFE-faced platen, align its 9 mm
   hole below the nozzle and raise the jack. Press the coupon onto the block (8 s), lower below the nozzle tip and
   withdraw (0.5 min estimated handling in addition to the 20 s extrusion/press).
4. Enrol with a calibrated underside photo: film over at least 95% of the outline, none in the lead strip or on a
   lead. Reject and repeat steps 2–4 otherwise (1 min for an accepted photo).
5. Empty tray, wipe plate, renew witness marks, inspect covers and restore the selected lot (1 min).
6. Switch heater power off and start within 30 s (0.5 min).

Batch preparation estimate: 24 coupons in 25 min, including slicing, heat-up, 8 min toolpath, cooling and removal:
62.5 s per trial. Successful-reset subtotal: 60 + (70 + 180 + 60) + (12 + 8 + 30) + 60 + 60 + 30 + 62.5 =
632.5 s, about 10.5 min. Every rejected preparation, damaged lead/brush and replaced cover adds its actual time;
no fixed upper bound is claimed. The stand has not been built or timed; neither transfer nor bonding has been physically tested.
Simulation reset
restores the full coupon and selected lot.

## Cover qualification before a physical robot trial

The [public Nextruder silicone sock listing](https://eshop.wuerth.de/Product-categories/Silicone-sock-set-Nextruder-MK4S-CORE-One-XL/14012002420290.cyid/1401.cgid/en/US/EUR)
rates the printer sock to 300 °C and says to remove it above 300 °C (public source opened Nov 2026). That sock is removed to expose the cleaning surface. Its rating does not qualify the separate finger
covers or the pads beneath them.

The listed material and adhesive tolerate 260 °C according to the supplier. This does not establish the temperature
of an underlying pad, a safe dwell time, full coverage, or adhesive retention under load. Before exposing shared
hardware, qualify the fitted cover on a spare finger with a thermocouple at the pad interface during controlled
250 °C brass contact, against the pad maker's temperature limit. Measure thickness/coverage and check adhesive slip.
Covers pass enrolment only when the pad interface stays below that limit. Replace covers after a touch or visible damage and
repeat qualification after a material/fit change. A score penalty or manual stop alone is not thermal protection.

## Simulation rule (stated, not measured)

- **Bristles.** The brush is a free rigid body (wood handle and head). Its 27 bristle tufts are each a straight 14 mm
  ray, 3.5 mm across, with a 25 N/m spring: about 110 brass wires of 0.08 mm each, 3EI/L³ = 0.22 N/m per wire at
  14 mm and E = 100 GPa. A tuft whose ray meets the block, nozzle or shroud is bent by the overlap; its spring force
  and sliding friction (0.35) act back on the brush, so the arm feels the load. Covered-finger/wood friction is 0.9 static, 0.8 sliding, an unmeasured estimate; improved grip is not assumed. The grip is capped at 40 N per
  finger as a declared simulation cap; no measured hardware grip-force claim is made.
- **Removal.** The coupon is 482 cells of 0.5 x 0.5 mm. A cell counts as cleared once tufts bent at least 1.5 mm
  (0.0375 N each, about 1 N for the whole head) have slid at least 4 mm over it in any direction that does not head
  toward the lead exit (no component along the normal of the block's back left face). Sliding is the motion of the
  contact point over the metal, not of the tuft's root, so pressing in place earns nothing. Strokes toward the lead
  exit earn nothing (in a real trial they push film into the lead strip or onto the leads, where it still counts as
  present). A contact that jumps more than 3 mm in one physics step earns nothing. Light touches and holding still
  remove nothing. Detached residue is drawn falling into the tray; it is not
  scored.
- **Touching a lead.** Any tuft (its 1.75 mm radius plus 0.125 mm) overlapping a lead, or the wood within 0.15 mm of
  one, at any physics step latches a 0 for the trial. With the leads 4.9 mm above the underside at their lowest, a tuft overhanging the block's
  back left face touches them when it is bent 4.9 mm or more.
- **Wood and fingers.** The wood head (the part that can reach the hotend) coming within 0.1 mm of the block or
  nozzle latches a 0, and so does any finger
  surface within 3 mm of either. Task-local cover colliders expand the distal finger hulls by 1.5875 mm in each local axis; the same surfaces enter hand-clearance checks. This is a conservative rigid envelope of a conformal silicone layer, not simulated silicone deformation. Original finger inertia is retained (cover mass omitted). The 3 mm clearance is measured from that covered envelope.
- **Not modelled.** Temperature and cooling (the trial starts within 30 s of the maker's 250 °C pre-heat and lasts
  2 minutes; the maker gives about 10 minutes for the hot parts to cool, and heater power is off during the trial,
  which disconnects the heater supply), PETG flow and smearing, single-wire bending, cover compliance, added mass and thermal insulation. The removal thresholds are stated
  estimates for softened PETG; they have not been measured on a hot block.

## Shortcuts and near misses

| Attempt | Score | Why | Checked in |
|---|---|---|---|
| Do nothing; brush held near the nozzle; brush put back unused | 0 | nothing cleared, or brush still held | near_misses.py, validation/check_model.py |
| Press the bristles on the residue and hold still for 10 s | 0 | no sliding | check_model.py |
| Stroke only toward the back, or only toward the lead exit | 0 | those strokes clear nothing | check_model.py |
| Light touch (bristles bent 1 mm, under the 1.5 mm threshold) | 0 | about 3% cleared | check_model.py |
| Bristle tips 2 mm short of the underside | 0 | nothing cleared | check_model.py |
| Scrub hard (6 mm bend) over the whole block up to the lead corner | 0 | bristles touch the leads | check_model.py |
| Push the wood head up onto the nozzle | 0 | wood touches the nozzle | check_model.py |
| Clean fully, then leave the brush on the plate or keep holding it | 0 | brush not released on the counter | task.py gates, near_misses.py |
| Knock the tray off or out from under the nozzle | 0 | tray gate | near_misses.py |
| Finger (covered) within 3 mm of the hot block | 0 | hot-metal latch | check_model.py, task.py |

## How this differs from close tasks

- **[Printer levelling, PR 3499](https://github.com/robocurve/benchmark-tasks/pull/3499)**: adjusting a build-plane datum; here the work is loaded brushing beside a lead exit, with no levelling adjustment.

- **[Hotend swap on a Bambu Lab A1 mini](https://github.com/robocurve/benchmark-tasks/pull/4104)**:
  unbuckle, lift out and seat a whole hotend. This task leaves the hotend installed and controls bristle load and
  stroke direction under it.
- **[TK-4Z8LF9, battery-post brushing](https://github.com/robocurve/benchmark-tasks/pull/2718)** (accepted): seat a
  cup brush coaxially over a tapered post and twist under radial load. Here an open brush works upward on a fixed
  underside: control bristle bend and stroke direction while overhanging tufts stay below a lead exit.
- **[TK-63RTRA, griddle scraping](https://github.com/robocurve/benchmark-tasks/pull/2019)** (accepted): keep a rigid
  scraper edge flat and push debris over a lip. This task needs compliant bristle loading inside a shroud, with a
  lower load needed to clean and an upper depth bound set by cable contact.
- **[TK-F37LCS, IV-hub disinfection](https://github.com/robocurve/benchmark-tasks/pull/1013)** (accepted): a timed
  scrub before syringe access. This task scores visible material removal; time spent scrubbing earns nothing.
- **[TK-R8NKWF, snapped filament](https://github.com/robocurve/benchmark-tasks/pull/1261)** (closed): clearing a fault after a failed print. This task is the step the maker prescribes before every print of a part, done to keep old plastic off the part being made.
- **[TK-XTCCB5, printed-support removal](https://github.com/robocurve/benchmark-tasks/pull/2321)** (accepted):
  support a loose printed part while breaking off its supports, after printing. This step comes before printing and
  works on the printer, not the part.

## Demo, Dream-1 and results

The scripted demo (`demo()` in task.py) uses fixed waypoints from the visible layout only; it never reads residue,
forces or other hidden state. The left hand grips the handle from above, 75 mm behind the head, lifts the brush,
brings the head in through the open front with the bristles below the nozzle, presses up and strokes toward the front
in nine lanes (forward, back, forward again in each), easing off between lanes. The three lanes nearest the leads stay forward of the lead exit and press
1.5 to 3 mm less. It then lays the brush back in its place and moves the hand clear. The right arm stays clear.
Corrections from lot A traces: 7 mm of extra lift for the arm's give under the bristle load, a 3 mm shift away from
the leads and a 1.25 degree turn of the grip.

Final covered-gripper scripted demo: A 1.0 (482/482 cells cleared), B 1.0 (481/482), both released and still for 2.05 s with no lead/wood/finger zero. Peak modelled bristle load: A 1.7408 N, B 1.6227 N. Native epoch scores are 1.0/1.0 on the frozen revised source. Dream-1 on that same source scored A 0.0/B 0.0: both hit the 2,400-step limit with no residue removed and no loaded bristle contact. The provider retried 40 expired requests; the run completed both lots with zero evaluation errors. Before/after fixed inspection views supplement the native cameras. Standard native videos show lot A; both lots are scored in the saved native log.

## Running it

```sh
python near_misses.py
python -m validation.check_model
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Brush the PETG residue off the heater block's underside with the brass brush, leaving the tray in place; then lay the brush back on the counter and clear both hands." --scorer task_score --max-steps 2400 --epochs 2
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Brush the PETG residue off the heater block's underside with the brass brush, leaving the tray in place; then lay the brush back on the counter and clear both hands." --scorer task_score --max-steps 2400 --epochs 2
```

`--epochs 2` runs lot A then lot B; `TASK_LOT=B` pins a lot. `parts.py` and `lots.py` are task-local helpers; they do
not change the installed simulator. Setting `NOZZLE_VIEWS_DIR` saves two extra close views of the hotend; it does
not change the policy's cameras or the score. The recorded two-lot scripted run took 37.8 minutes (about 19 minutes per trial), plus video encoding; inspection capture and host load affect wall time.

Licence: MIT for code; CC BY 4.0 for the text, drawing and cell table.

## Physical evidence limits

The simulation demonstrates execution under the declared contact/removal model. Physical preparation limits are stated in Reset and cover qualification above; real removal loads and photo-scoring agreement also remain unmeasured. Occluded or ambiguous film or
lead-contact evidence makes a physical trial unscorable; reacquire views before accepting a score. An operator
must qualify the full preparation and scoring procedure before running an evaluated physical trial. The second
hand remains idle; this task does not demonstrate bimanual holding.
