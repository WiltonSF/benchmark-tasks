# Enrolment and reset

Status: procedures and acceptance targets only. No cap has been acquired and no fixture has been printed,
assembled or measured. `TRIAL-LOG.csv` is blank intentionally. Run these checks before reporting any physical
trial. A failed check is a failed fixture; do not change a simulation score or threshold to accommodate it.

Tools: caliper, 20 N spring scale and steel rule in `BOM.csv`. Zero the scale in its test orientation; read force
at first motion. Record each reading, instrument and date. Resolution is not calibration or accuracy.

## Size the genuine cap once

The genuine tip's knob and rim dimensions have not been verified. The nominal simulation uses a 13 mm head,
10 mm axial head length and 6 mm neck, all estimates. Festo's public cap datasheet identifies the material and mass,
but supplies no dimensioned tip interface. An attempt to open the official cap CAD/product page on 2026-11-23
failed; no maker CAD has been used to confirm the knob.

1. Caliper the genuine DHAS-GA-B22-S knob's maximum head diameter, head length and neck diameter, and the rim's
   unstretched inside diameter, in mm. Photograph the tip and record whether it has a usable undercut. Do not infer
   an undercut or dimensions from the catalogue's outside view.
2. In a separate copy of this fixture folder (keep the submitted nominal files), run:

   ```bash
   python make_replica.py --knob-head-d HEAD --knob-head-len LENGTH --knob-neck-d NECK --rim-seat-d SEAT
   ```

   `SEAT` starts at measured rim ID + 1.0 mm (an untested fit allowance). The keeper's closed inner edge is the
   larger of 3.5 mm or neck radius + 0.5 mm. The cam's first slope changes with head/neck size so release remains
   at 1.5 mm sleeve travel; opening still reaches 4 mm at 2 mm travel. The pocket is head diameter + 1 mm.
3. All CAD checks must pass. The generator rejects a head without sufficient undercut, insufficient pocket wall,
   an overlong head, or an out-of-envelope rim seat. Print the resized coupling, sleeve, four cheeks and carrier.
   Record all inputs and the generated manifest. These fitted dimensions are separate from the frozen simulation's
   nominal knob and rim values. Do not overwrite its metadata with measurements from an adapted fixture.
4. If the genuine tip is not a knob with a usable undercut, or cannot pass the design envelope and the checks,
   stop the genuine-cap route. The mould files provide an experimental fallback with the nominal printed knob;
   it needs separate casting/force/cycle qualification. A cast cap does not establish genuine-cap compatibility.

## Assembly acceptance (repeat after any reprint)

| Check | Method | Acceptance / record |
|---|---|---|
| Printed fits | Assemble without cap; operate every moving part by hand | No binding, cracks or loose pin; no machining of working faces to bypass a failed travel check |
| Mount and collar | Scale pulls on the catch carrier / coupling axially, up to 20 N | Board, bracket, bonded coupling and rod collar stay fixed at their intended joints; no crack |
| Locked bayonet | Catch locked, cap absent; scale pulls ring axially | Ring cannot be drawn 2 mm forward at 20 N |
| Open bayonet | Turn to open mark, then draw off | Mark is 45 degrees from locked; lip clears lugs there. CAD stops allow about 3 degrees additional play; stop before 49 degrees. Record actual angles |
| Rod | Push coupling back with scale; steel rule reads stroke | Breakaway 5–7 N (6 N design target); stroke 55 mm; forward rod end u=199 mm. Set the brake, then repeat |
| Spring fit | Slide specified LC 036G 06 M on actual rod before bonding | Free sliding over full working length. Lee's recommended rod is 9.96 mm, so a nominal 10 mm rod needs this check. If it binds, reject and resolve the fit; no substitute spring rate is silently allowed |
| Sleeve force | Scale pushes sleeve toward rod; caliper depth rod reads travel | Record force at 0, 1.0, 1.5 and 3.0 mm. Design at 1.5 mm: 1.8 N elastic force (1.00 N/mm x 1.5 mm + 0.3 N preload), plus friction; initial screening range 1.5–3.0 N. Record full curve, not just pass/fail |
| Latch holding | Nominal knob stand-in seated, sleeve untouched; scale pulls knob | Holds 20 N without opening or damage |
| Latch release | Pull knob with 2 N; measure sleeve travel while pushing | Releases at 1.4–1.8 mm (nominal 1.5 mm, print tolerance); holds at 1.0 mm. Recheck using genuine cap with tip unloaded; record its force/travel separately |
| Latch return | Release sleeve after opening | Sleeve returns fully to its stop; knob retained on a subsequent 2 N pull. Physical cam re-engages below 1.5 mm; simulation remains open until below 1.0 mm |
| Fitted cap | Fit genuine cap; rule reads tip from mounting face | Tip u=224 +/-2 mm, index at locked mark. Record rim seating and any sag |
| Rim retention | Evert cap, sleeve not pushed; scale draws carrier | No rim slippage at 20 N; no tear. If it slips, reject/revise the seat and repeat all checks; no unpriced tie is assumed |
| Eversion | Scale draws open carrier along axis through the roll | Record force versus draw and peak force (simulation estimate 4 N). No Festo force measurement is available; discrepancy is reported, not adjusted away |
| Durability | Ten complete removal/reset cycles | No damage or looseness; holding, release and return checks still pass |

The force and fit bands above qualify this proposed replica; they are not measured Festo specifications. Passing
CAD clearance checks cannot establish these physical results. The additional spring preload, passive cam loading,
friction, cap sag, surrounding support geometry and fitted knob can differ from the simulation. Label physical
results accordingly. If force matching fails, no claim of validated physical equivalence follows.

## Reset before each trial (estimated 2–3 minutes, not timed)

1. Hold the sleeve pushed toward the rod, seat the cap's knob, then release the sleeve. Unlike the genuine DHEF,
   this cam latch has no snap-in action. Roll cap back, rod fully forward; align lugs with lip gaps, seat carrier and
   turn to locked mark.
2. Check u=224 mm tip position and locked index. Look for holes/tears against a lamp. Replace damaged caps.
3. Place loose mat at lot A or B coordinates. Record mat thickness (Desco's nominal 3.175 mm vs simulation 3 mm)
   and use the actual mat top for physical height checks. Remove assembly tools from the workspace.
4. Time three resets, report individual seconds and median. Record trial lot, specimen, damage check, final tier,
   hands-clear interval and photo/video filenames. A cap that fails inspection is not reused.

## Estimates and their derivation

- Rolling, `F_ROLL=4 N`: an order-of-magnitude design estimate, not a measurement. Assuming a 1 mm wall,
  effective modulus 1.5 MPa and a 3/18 hoop strain, energy per rolled area is E*t*strain²/2 ≈0.021 N/mm.
  Multiplying by an assumed 100 mm circumference gives about 2.1 N; the 4 N model allows roughly another 2 N
  for losses. Every material/geometry assumption here is unverified for the genuine cap. Do not treat this as
  a material datasheet or a validated shell model.
- Rod, `F_ROD=6 N`: the physical brake's target, still to be set/measured. Simulation exhausted-seal drag is an
  estimate, not a published cylinder specification.
- Sleeve: [Lee Spring LC 036G 06 M](https://www.leespring.com/product/compression-spring-lc036g06m-music-wire),
  opened 2026-11-23, lists 1.00 N/mm with 0.10 N/mm rate tolerance. Design spacer gives 0.3 N preload.
  Simulation has no preload and drives the keepers independently; keeper force is not charged to sleeve work.
  This published spring rate does not establish the assembled cam's loaded release force or the Festo release force.
  The [Festo catalogue](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2024/TR/DHEF_EN.PDF),
  p. 8, gives 2.3 N permissible dynamic transverse rod load at maximum cantilever, not sleeve force. Neither it nor
  the [operating instructions](https://docs.rs-online.com/d47e/A700000014068131.pdf), section 10.1, supplies a
  numerical tip-seating load. Measure tip seating and sleeve force simultaneously during release, recording axial
  force and displacement on each operated part; the independent unloaded-spring reading cannot close that gap.
- Stretch, `K_STRETCH=2000 N/m`: nominal simulation estimate only. The physical cap supplies its own response;
  the 20 N retention check tests holding strength, not this stiffness. Measure a separate force/displacement curve
  before claiming stiffness equivalence.
