# Hold a polymer beam at 1 mm and then 2 mm of bend on a lever test stand, unloading in between

A white UHMW-PE bar lies on a specimen cradle beside a Mark-10 ES05 lever test stand set up for a three-point bend. The robot puts the bar across the two bend rollers, centres it under the loading nose, and works the stand's own spring-loaded lever to hold the bar's centre at 1 mm of deflection for a second while the force gauge reads the load. It lets the lever up until the load is gone, holds 2 mm for a second, lets the lever up again and returns the bar to a cradle. The hard part is holding a live load inside a ±0.2 mm window, steady to 0.03 mm per 50 ms, through a compliant arm and a lever, on a bar it placed itself, and unloading without disturbing the bar.

Materials labs run quick bend checks like this to compare stiffness between lots of a polymer before full tests. [O*NET 19-2032.00](https://www.onetonline.org/link/details/19-2032.00) (Materials Scientists) lists the task "Devise testing methods to evaluate the effects of various conditions on particular materials." The [attachment maker](https://mark-10.com/downloads/product-downloads/DataSheetGrips.pdf) states, "Use a base-anvil combination for 3-point bend testing." The [ES05 data sheet](https://mark-10.com/downloads/product-downloads/DataSheetES05.pdf) describes "a spring-loaded lever" with up to 38 mm of travel and a lever mechanism that "can be repositioned along the length of the column". Category: Other (materials-research automation). This is not an ASTM D790 test and gives no calibrated modulus.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Put the beam on the test rollers. Using the stand's lever, hold its centre at 1 mm deflection for a second, let the lever up to zero load, hold 2 mm for a second, then let the lever up and return the beam to a cradle." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames
```

Use `--policy dream` for the model run. Each layout has 2,400 control steps (120 s at 20 Hz). Both fixed layouts are run and averaged; no random jitter is used. Layout A starts the bar on the far cradle (X=200 mm), layout B on the near one (X=110 mm), so a single fixed command replay cannot pick it up in both.

## Scene

All parts are at full size; nothing is scaled down. X points away from the arm bases, Y to the left, Z up from the bench; the load axis is at X=320, Y=0 mm.

| Item | Specification |
|---|---|
| Mark-10 ES05 lever test stand | 162.1 × 99.1 mm base, 355.6 mm maximum height, 150 N capacity, 38 mm travel; column 92 mm behind the load axis |
| ES05 lever | Original spring-loaded lever on the right of the head: about 105 mm from hub to knob, Ø26 mm black knob, resting about 40° above horizontal toward the operator. Dimensions scaled from the maker drawing; the head is set on the column so the loading nose rests 6 mm above the bar |
| AC1060 base plate | 70.1 × 82.8 × 18.5 mm |
| G1095 lower bend fixture, G1097 upper anvil | 10 mm rollers; lower span set to 150 mm (maximum 152.4 mm) |
| M5-20 force gauge | 100 N × 0.02 N; its reading in the simulation is the actual nose-to-bar contact force |
| Deflection indicator | Mitutoyo 1044A-15 low-force dial indicator: 0.01 mm graduation, 5 mm range, 8 mm stem, 0.1 to 0.4 N measuring force, for use with the contact point downward only ([maker data](https://mitutoyoshop.eu/MITUTOYO_HU_20004/1044A.pdf)). It is held in front of the lower fixture by bracket C1, contact down on the pad of rocker R1 |
| Rocker R1 | A balanced 1:1 aluminium rocker bar, 76 × 6 × 10 mm, pivoted on a Ø3 mm pin in a clevis on the lower fixture: a rounded tip 32.0 mm behind the pivot touches the bar's underside at mid-span, and the dial rests on a pad 32.0 mm in front of it. When the bar's centre moves down, the pad lifts the dial's spindle by the same amount. The bar feels the dial's force through the rocker, at most 0.4 N upward, less than its 0.46 N weight, so it stays on both rollers |
| Bench display (simulation only) | A 190 × 90 mm display lying flat to the right of the stand that repeats the gauge force (N) and dial deflection (mm) in large digits, because the simulated instruments' own faces are drawn blank |
| Specimen | Mitsubishi Chemical [TIVAR 1000](https://www.mcam.com/mam/54452/PE-TIVAR%E2%84%A2%201000%20Virgin%20UHMW-PE_en_US.pdf) UHMW-PE, 200 × 25 × 10 mm; 46.5 g from the sheet's 0.93 g/cm³; flexural modulus 87 ksi (about 600 MPa); three small black witness marks on the front face, at mid-span and over each roller |
| Specimen cradles | Two machined aluminium bases, 70 × 220 × 10 mm, each with two 40 × 25 × 20 mm saddles 150 mm apart; side by side left of the stand, bar centres at X=200 (far) and X=110 (near), Y=130, Z=35 mm |

The bar rests on the rollers with its centre 131.1 mm above the bench; the roller contact lines are at Y=±75 mm. The complete column, head, lever, gauge, lower fixture and anvil are present and fixed. Mounting adapters, the head height and the lever dimensions are estimates in `fixture/DRAWINGS.md`, not a verified OEM assembly.

## Measurement

Force is the M5-20 reading. Deflection is the dial reading, zeroed with the bar resting unloaded on the rollers. The rocker's pivot and the dial's holder are fixed to the lower fixture, so the dial reads the bar's centre relative to the supports. In a real trial the robot reads the gauge screen and the dial through its cameras; both face the arms, and no reading needs to be typed or written. In the simulation the force is the native nose contact force and the deflection is the ideal centre-to-support chord of the bar, both shown on the bench display at 0.02 N and 0.01 mm resolution; the deflection is zeroed at the first step of each trial with the bar unloaded on its cradle, whose saddles are also 150 mm apart, which gives the same zero as the real reset. The simulation draws the pick-up schematically as a block and pin under the bar's centre and models neither the rocker, the dial nor the dial's push; the task code's comments call that drawn body a digital indicator, and it has no collision or force.

The dial is used contact point down because that is the only orientation the maker specifies for this low-force model; the rocker turns the bar's downward movement into the dial's upward spindle movement. With the bar resting, the dial is set about 0.5 mm into its 5 mm travel and zeroed, so a 2.2 mm bend puts it about 2.7 mm in and the dial's end is reached only past a 4.5 mm bend, beyond the 4 mm limit. Its push, 0.4 N or less (0.1 to 0.4 N over its travel), is carried 1:1 to the bar and is always less than the bar's 0.46 N weight, so the bar never lifts off a roller while its centre is near mid-span (within about 9 mm; placement credit needs 2 mm). At a given nose load the push changes the centre deflection by at most about 0.02 mm (0.4 N ÷ 17.8 N/mm = 0.022 mm), about a tenth of the ±0.2 mm windows, and less between the zero and a reading because the zero is set with the dial already pushing; the windows are read on the dial itself. The rocker arms are 32.0 ± 0.1 mm, so the reading is the centre deflection within 1% (0.013 mm at 2 mm). The rocker and its bracket are drawn estimates in `fixture/DRAWINGS.md`, to check on the real fixture.

Reading the dial: one pointer turn is 1 mm. Because the dial is zeroed about 0.5 mm into its travel, the small revolution counter sits about halfway between two marks at zero and moves one mark per turn; a reading of 1.0 ± 0.2 mm is one full turn past zero with the pointer within 20 graduations of zero, and 2.0 ± 0.2 mm is two full turns past zero, the same way. Video of the dial face, with the gauge screen in the same frames, establishes each reading and its 1 s hold.

A reading is the pair of displayed values over 21 consecutive 20 Hz samples (one second) inside the window. `captures/records-<process>-<layout>.json` keeps every displayed value of each qualifying second.

## Scoring

**Score 0 for the whole trial (latched; returning things to the start does not undo it) if:**
- the bar leaves the table, the stand moves more than 5 mm, the gauge reads over 50 N or the bend exceeds 4 mm;
- any arm surface comes within 5 mm of the bar, or touches it with more than 0.5 N, while the nose load is over 2 N;
- from the moment the bar first sits centred on both rollers (as for 0.25) until both readings are taken, an arm touches it with more than 0.5 N while the rollers carry more than 2 N between them (the bar alone puts at most 0.46 N on them); in a real trial this is judged from the gauge, the front and rear video and the roller witness marks;
- from the first nose load over 2 N until the second reading is saved, either end of the bar slips more than 1 mm on its roller, or the bar loses either roller or is lifted off; this includes the unload between readings.

**Otherwise:**

| Score | Observable condition | Checked with |
|---|---|---|
| 0 | No current valid placement and no saved reading | Overhead video |
| 0.25 | Bar released on both rollers, centre within 2 mm (horizontal) of the load axis, yaw and tilt within 2°, all arm surfaces more than 10 mm away, bar still for 1 s. Lost again if the bar is moved | Front and rear video, ruler, roller witness marks |
| 0.5 | First reading: deflection 1.0 ± 0.2 mm with nose load over 2 N, bar on both rollers, arms clear, still for 1 s | Gauge screen and dial in the same video frames |
| 0.75 | Both readings, in order: the 1 mm reading, then an unload to ≤0.2 mm and under 2 N, then 2.0 ± 0.2 mm held for 1 s | Same gauge screen and dial, with the unload frames between |
| 1.0 | Both readings saved, nose under 2 N, residual deflection ≤0.2 mm, bar on either cradle within 5 mm (horizontal) and 1 mm (height) of its cradle centre, all arm surfaces at least 30 mm away, bar still for 2 s | Overhead video, cradle witness marks, ruler |

Saved readings persist after the second one, through the unloaded removal and racking. A 2 mm reading without the earlier 1 mm reading and unload earns no reading credit. "Still" means the bar moves less than 2 mm/s and turns less than 0.02 rad/s between consecutive 20 Hz poses and its deflection changes by no more than 0.03 mm per 50 ms; holds count only over consecutive eligible samples. The bar is symmetric end for end, so it may be placed either way round. Where the lever is left at the end is not scored; with the bar back on a cradle the nose reads no load either way.

The simulation reads both roller reactions separately (over 0.02 N counts as contact), the native arm-to-bar contact force, and the distance from sampled arm surfaces to the bar. Missing contact or arm data scores 0, and a fresh state certificate stops a stale pose from earning credit. In a real trial the same checks use front and rear video of both roller contact lines, witness marks on the rollers and the stand, and the gauge screen and the dial.

## Declared response model

The bar is two 50 mm end pieces on one free body plus a 99 mm centre piece on a downward sprung slide, separated by 0.5 mm seams. The spring is the bar's nominal centre stiffness, k = 48EI/L³ with E = 600 MPa, I = bh³/12 and L = 150 mm: 17,778 N/m, so about 17.8 N at 1 mm and 35.6 N at 2 mm. These are model values, never the scored answer; no FEM, creep, residual set or material calibration is claimed. Damping 4 N s/m and friction 0.2 are estimates. Contact offset is 0.15 mm with 32/4 solver iterations. The rollers are passive 32 g revolute links. The lever drives the nose through an estimated 18 mm/rad pinion (38 mm over about 121° of lever); its return spring is estimated at 0.3 N m/rad with 0.3 N m preload at the rest stop, enough to lift the 0.65 kg gauge carriage back when the knob is released. Neither display is computed from the lever angle.

## Demonstration and shortcuts

The scripted demonstration is a feasibility replay, not a perception claim. It takes the trial's cradle (far in A, near in B) from the layout table. The left hand carries the bar to the left of the stand, comes in behind the load axis and slides it under the raised nose onto both rollers. The right hand then takes the lever knob from the operator side with its pads closing along the knob's arc, so pressing down and lifting are pad normal forces. It lowers the lever along the declared lever arc until the displayed force rises, then adjusts on the displayed deflection, holds, lifts to unload, repeats for 2 mm, lets the lever back to its rest stop and opens. The left hand returns the bar to its cradle. During loading the demo reads only the displayed force and deflection, never the stiffness, the lever angle or any scorer state. The arm is compliant: under load the commanded lever angle leads the actual one by up to about 0.35 rad, and the display feedback absorbs it.

| Shortcut | Result |
|---|---|
| Gripper bends or supports the bar during a reading | 0, latched |
| Push the bar down onto the rollers with a hand | 0, latched |
| Move or shorten the supports | Fixed fixture; not possible in the scene |
| Bar slips or is lifted during the unload between readings | 0, latched |
| Lose either roller while loaded | 0, latched |
| Bend past 4 mm or load past 50 N | 0, latched |
| Bar turned 90° or resting on one roller | No placement credit |
| Only a 2 mm reading, or no unload between readings | At most 0.5 |
| Keep holding the bar at the end, or leave it on the rollers | No 1.0 |

One hand can in principle do both the placing and the lever work; no two-arm necessity is claimed. `fixture/check_protocol.py` runs 46 synthetic scorer cases (all tiers, order, window edges, support loss, slip, hand contact, force limit, stillness, end-for-end placement and latching). They test the scorer, not the robot.

## How this differs from close tasks

- TK-K3R94E (load an ASTM D638 dogbone into wedge grips on a test stand) ends once the specimen is gripped; here the robot drives the load itself and must hold two deflection windows.
- TK-QF4WMY (bend a copper tube to 90° in a lever bender, allowing for springback) works a lever to a permanent bend; here the bend is elastic and must be held within ±0.2 mm, steady for a second, twice, with an unload between, on a bar the robot placed.
- TK-8AFCU4 (square a milling vise with an indicator) uses an indicator to align a fixed part, with no load control.
- TK-M47HJ8 (measure a pin with a caliper) and TK-YYGGX9 (measure a bar and enter its length) read a tool; nothing is loaded.
- TK-A8X2KJ (flex a 3D printer's build plate to free a print) bends to release a part, with no measured deflection.
- TK-XF6HD6 (set a bend fixture's spacing and centre a specimen) stops at placement.

## Reset, cost and access

Reset before each trial: (1) check the 150 mm span and the witness marks; (2) lay the bar unloaded across both rollers, set the nose 6 mm above its top (on the ES05, the lever on its rest stop; on the lower-cost press below, lower the nose until a 6 mm drill blank laid across the bar just slides out from under it, and the ram's gibs hold it there), and set the dial to zero; (3) lift the bar off the rollers and return it to that trial's starting cradle, the far cradle for layout A or the near one for layout B, so no trial starts with the bar on the rollers; (4) clear both arms and start the video. Run A, then repeat steps 2 to 4 for B. The score history resets only at the trial reset, never when the robot returns to a pose.

Named set: the [maker's 2026 price list](https://mark-10.com/downloads/catalogs-brochures-pricelists/Price_List_2026.pdf) gives the ES05 at $760 and the AC1060 at $95; with the M5-20 gauge and the G1095/G1097 attachments it costs a few thousand dollars.

Lower-cost real set: about $400 for the press, gauge and bend fixture, plus about $152 for the low-force dial and its magnetic base, about $550 in all before shipping and tax. It keeps the 150 mm span, the 10 mm rollers and 10 mm nose, the 6 mm nose gap at the start, the bar, the dial and rocker R1, so the same 1.0 ± 0.2 mm and 2.0 ± 0.2 mm windows and the ≤0.2 mm unload are read on the same dial. Prices are retailer and maker listings seen on 21 November 2026, not purchases; lines marked "estimate" are typical prices for small stock and hardware. Part dimensions are in `fixture/DRAWINGS.md` (L1 to L6). This set is a priced proposal for real trials: it has not been built or simulated, and the simulation and videos use the ES05 set. The total excludes the UHMW-PE bar and the two cradles, which are the same in both sets, and tools.

| Item | Part | Price |
|---|---|---|
| Press | [VEVOR 3-ton manual arbor press](https://www.walmart.com/ip/17476308669), plain-lever model (not the ratchet-and-handwheel model, whose handle ratchets back without raising the ram): a lever turns a pinion that drives the rack on the ram, as in the ES05; 12.2 in (310 mm) maximum height, 0.5 in ram bore, 6.5 in anvil plate; also listed at the same price by [FactoryPure](https://factorypure.com/products/vevor-arbor-press-3-ton-manual-cast-iron-12-2-max-height-new). Chosen for its 310 mm opening, which the gauge stack needs, not its capacity; stop L6 bounds the load well below the gauge's 100 N. Bolted to the bench | $255.90 |
| Force gauge | [AMF-100 digital push-pull gauge](https://www.walmart.com/ip/16323471339), 100 N capacity (resolution and accuracy as stated by the seller), hung from the ram on bracket L5 with its screen facing the arms | $72.00 |
| Bend fixture | L1 steel plate, 12.7 × 152 × 203 mm; L2 two 76 mm pieces of [80/20 1010](https://8020.net/1010.html) 1 in T-slot extrusion ($0.45 per inch plus a $3 cut charge each, $8.70) bolted across the plate with their top-slot centres 150.0 mm apart, with T-nuts, screws and end caps; L3 two Ø10 × 70 mm hardened dowel pins lying in those top slots as the rollers; rocker R1 on a small clevis on the plate; a 3/8 in to 8 mm sleeve for the dial's stem | about $55 (estimate) |
| Nose, gauge bracket, lever stop | L4 nose: a Ø10 × 40 mm hardened dowel pin in an aluminium holder threaded onto the gauge shaft; L5 bracket: 2 × 2 × 1/4 in aluminium angle held in the ram bore by a 1/2-13 bolt (measure the bore; use an M12 bolt if it is 12 mm) and drilled for the gauge's rear mounting screws; L6 a stop block under the lever's path | about $15 (estimate) |
| Press, gauge and bend fixture | | about $400 |
| Deflection indicator | Mitutoyo 1044A-15 low-force dial indicator, as in the named set ([MSI-Viking listing](https://www.msi-viking.com/Mitutoyo-1044A-15-DIAL-INDICATOR-0.01MM-5MM)) | $129.76 |
| Indicator holder | [WEN adjustable magnetic base](https://www.walmart.com/ip/399205731) with fine adjustment, on the steel plate in front of the bar | $22.03 |
| Total | | about $550 |

Each roller pin lies along the extrusion's top T-slot, whose two lips locate it across the span; the span is set by where the two extrusions are bolted, not by machined grooves, and is set to 150.0 ± 0.5 mm with a caliper before tightening. The pin tops sit about 9 mm above the extrusion and the bar's underside about 34 mm above the plate, which leaves room for the rocker. The stack on the anvil plate (fixture, bar, 6 mm gap, nose, gauge and bracket) is about 278 mm with a 150 mm gauge body, inside the press's 310 mm; check it with the real gauge. The magnetic base holds the dial on the same plate as the rollers, so the reading is still relative to the supports. Set the ram's gib screws so the ram, gauge and nose stay where the lever leaves them: the press has no return spring, so "let the lever up" means lifting it, and at the end lifting it until the nose is clear of the bar. Its lever is longer than the ES05's and its pinion ratio differs, so lever angles differ; the robot works from the gauge and dial readings, as the simulated demonstration does. Stop block L6 is set where the handle lands at 3.5 mm of bend (about 62 N by the declared stiffness), so a hard push on the long lever cannot overload the 100 N gauge or bend the bar further. Hands stay clear of the nose and rollers while the lever is worked.

## Results

Scripted demonstration: 1.0 in both layouts (mean 1.0). In layout A the first reading was 15.12 N at 0.85 mm and the second 33.16 N at 1.86 mm; in layout B 15.14 N at 0.85 mm and 33.00 N at 1.86 mm. Each trial then unloaded the bar to zero, returned it to its cradle and finished its 2 s still hold at about 101 s of the 120 s limit. The model values for these deflections would be about 15.1 N and 33.1 N, so the gauge readings follow the declared stiffness.

Dream-1: 0.0 in both layouts (mean 0.0). In both trials it ran the full 2,400 steps; its arms reached toward the stand and the cradles, but the bar never left its cradle, so no placement or reading was scored.

A scripted success shows the task can be done with these arms in this simulator; it is not a hardware result.

All primitive geometry and task code are locally authored. Code: MIT; documentation: CC BY 4.0. No third-party mesh assets are distributed.
