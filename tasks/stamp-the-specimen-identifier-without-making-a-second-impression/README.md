# Stamp a specimen identifier with three registered strikes

This task is laboratory specimen preparation under **Other**. Permanent identifiers preserve traceability through materials testing. The scored skill is punch orientation and registration for three complete, irreversible characters, each made by one clean hammer strike. It does not claim a workforce count, a human baseline or robot hardware performance.

Partial submission candidate. The completed scripted demo scored0.0 in both layouts. No full-score demonstration or physical validation exists. See the results below.

## Run

Use the program's yam-isaac-lab-arena embodiment for Isaac Lab-Arena through Inspect Robots:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Stamp 123 on the brass specimen without making a second impression, then put the tools back." --scorer task_score --max-steps 3600 --epochs 2
```

For the recorded Dream-1 evaluation, replace `yam_scripted` with `dream` and use `--max-steps 2400 --epochs 2` (120seconds per layout). The recorded scripted demo used a3600-step cap and ended after about100seconds per layout. The vise was placed65mm nearer the arm bases after the first reach study. The two fixed visible layouts have coupon x centres225mm and243mm; the vise and all tool starts stay fixed. No moving articulation is repositioned between layouts. The original proposed20degree rotated-vise layout was replaced with a rigid stock translation. The scripted demo holds each punch with one constant75degree lean from pickup to strike, pads on two flats2mm above the punch centre of mass, with a fixed workpiece trim; the punch still rocks during impact, as reported below; the hammer is pinched across the flat-sided eye directly above its face, so the blow does not roll it in the fingers. The punch holder sits at x340mm with16mm sockets turned to each start yaw. The native runs have not qualified this plan. It is an open-loop plan from these published visible starts, with fixed measured tracking compensation. It reads no object poses, forces or material state. `tilt.py` supplies wrist rotations for the scripted demo only; Dream-1 and scoring use the ordinary embodiment. `parts.py` adds local contact geometry, contact measurements and callbacks; no installed simulator code is changed.

## Setup and reset

[Dimensioned setup and procurement](fixture/dimensions.md), [plan drawing](fixture/bench-and-backing.svg), [side section](fixture/side-section.svg) and [independent inspection protocol](fixture/inspection.md) specify the real setup. The native TEKTON HMA51108 hammer, Young Bros08091 punches and ordinary low-profile vise are not scaled. The complete08091 figure set supplies digits1/2/3; unused stamps stay outside the scene. Procurement is approximately$194 before tax/shipping, including estimated stock, backing, rule/tray and contingency. The hammer, stamp set and vise prices are sourced; other allowances are identified as estimates.

The operator bolts the vise, installs the solid backing and presets the jaws. Each reset replaces a ready blank, seats it at the marked A or B centre and restores the tools. No specimen casting, printing, curing or drying occurs between trials. Blank exchange is estimated under1minute, not a measured reset time. Enroll only flat, uncracked annealed brass50x50x3mm and confirm actual handle dimensions, die references, mass and jaw retention before real trials. The model's detailed hammer taper and vise casting are approximate, and the real holder must support the whole coupon underside. Wear the eye protection required by the stamp maker and keep people outside the impact area; no robot hardware trial has been performed.

## Score read from simple tools

Score0 permanently for a wrong character or slot, any extra separate impression, any repeated strike that makes an impression (even exactly overlapping), stock motion>3mm or tilt>5degrees, a fresh crack>=2mm, or hammer contact with the punch hand. The simulation's declared overstrike damage proxy is incident allocated work>0.40J; it is not a validated brass fracture model.

For each intended character, register its actual die reference by translation and rotation only to an overhead macrophoto with a ruler. At least90% of the reference centre-line must be continuously grooved, with no missing run>0.5mm. The groove must lie within0.15mm of the reference after registration. Require centre within4mm of its designated coupon-frame target(-12,0),(0,0),(12,0)mm and angle within20degrees for partial credit. An incomplete intended first mark earns nothing for that digit and cannot be repaired with another blow. Strike order may vary; spatial order must be123.

Every nonzero tier requires all tools released, hands at least50mm from every tool and the specimen, and a1second still hold (simulation: all origins remain within0.5mm and orientations within1degree of their hold anchors, linear speed<=5mm/s and angular speed<=0.1rad/s).

| Score | Completed result | Real reading |
|---|---|---|
|0.25|One complete intended character|Macrophoto/reference overlay|
|0.50|Two complete intended characters|Same check independently on both|
|0.75|All three complete, in123 slots|Macrophoto, ruler, strike video|
|1.00|All three; baseline range<=1mm, x pitch12+/-1mm, rotation<=10degrees; tools back within15mm of published origins|Ruler/photo and released stillness video|

The exact lighting, image scale, missing-groove and second-impression rules are in [inspection.md](fixture/inspection.md). No physical inter-rater agreement has been measured. These are proposed reproducible physical checks, not a validated recognition system.

## Impact and indentation model

The punch has physical sharp-face line colliders; the hammer head, shank, brass, jaws and backing collide. A callback samples native specific hammer-punch, punch-stock, punch-hand and stock-backing contacts at240Hz. A mark event can begin only at a real hammer-punch contact with a normal closing speed of at least0.35m/s. It needs concurrent punch-stock support, backing reaction and a counterholding hand contact. Gravity, placing the punch and force-free pose alignment produce no mark.

The estimated energy budget is the smaller of half the native normal impulse times incoming normal speed and the actual incoming rigid hammer kinetic energy (COM translation plus rotation). The latter uses the published226.796g head,345g whole tool and estimated18g collar/99.934g handle distribution. Native collision resolution supplies reciprocal loads; no kinematic weld, pose overwrite or external action moves the hammer or punch. The wooden handle uses an estimated10mm torsional contact radius with8mm minimum, based on roughly20x16mm of contacting pad area; it is a friction model, not a rigid attachment. [PhysX documents](https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/_api_build/class_px_shape.html) that its zero default supplies no torsional friction. That setting did not solve the earlier handle grasp. The recorded demo pinches across the native25.4mm flat-sided head eye, with the right gripper leaned30degrees for reach, and keeps that grip through the three blows. Native reaction impulses are sampled at240Hz; true high-frequency impact peaks and compliance are unresolved. Local subgrid plasticity allocates that collision energy to sharp-face indentation with `W=H*sum(line_length*depth^2)`, estimated fully plastic pressure H=0.6GPa and45degree edges. Solve depth at each actual die sample using the worst loaded tilt seen during the blow and its peak-force pose. A die-footprint shift>0.5mm during loaded impact permanently fails as a smeared impression; direct hammer-stock contact also fails. Only downward hammer motion qualifies, and the energy cap uses the preceding native substep, preventing a rising punch from receiving credit against a stationary hammer. A50micrometre depth is the simulation's visibility proxy. The result keeps the actual impact position and yaw permanently, and its dark groove geometry stays visible after tool removal. A second qualifying strike on a character permanently fails the trial.

The pressure is an engineering estimate consistent with a work-hardening brass contact, approximately3times an estimated0.2GPa mean flow stress during indentation, informed by [Copper Development Association C26000 properties](https://alloys.copper.org/alloy/C26000); it is not a fitted value from these tools. Exact Gothic face outlines, plasticity, roughness, cracking, hickory taper and high-frequency impact compliance are simplified. Native contact reaction is physically simulated; the permanent subgrid impression is a declared material model. It has **not** been validated against actual stamps and brass. The real reference mask and photo protocol also need qualification. These limitations answer the pre-review's calibration objection by making its present boundary explicit, rather than claiming a physical result.

## Bypass controls

| Attempt | Outcome |
|---|---|
|No action, or merely place a punch on its target|0: no hammer contact energy|
|Press or drag a punch without an incident strike|0: no qualifying impact|
|Drop a punch without holding it|0: counterholding contact required|
|Use the wrong digit, slot or upside-down face|0 by permanent mark record/registration|
|Strike a tilted die|Incomplete groove receives no character credit|
|Repair a weak mark with a second blow|0 permanently|
|Move stock back after it escaped|0: reset occurs only at official scene reset|
|Hold tools near their rests|0: face-sampled50mm hand clearance and1s anchor hold|

`near_misses.py` checks the law and scoring boundaries, including repeat marks, incomplete outlines, wrong slots, rotations, coverage and absent runtime observations. Face-sampled hand surfaces at3mm spacing cover palms and finger interiors; no vertex-only release shortcut is used.

## How it differs from close tasks

- **TK-W437WY (accepted stabilize-and-hammer nail):** axial nail insertion permits repeated advancing blows. Here the held die is removed after one blow; its yaw, complete outline and registered location create an irreversible identifier. Driving deeper cannot correct wrong registration or a doubled character.
- **robotwin-2 Stamp seal and TK-PQB7R5 / PR2792:** those press an ink seal on a target. This task separately controls a real hammer's impact and three independently oriented sharp-face steel dies. Neither an ink-press pose nor repeated stamping can earn a character.
- **TK-F3RTEA desktop embossing press:** a lever drives a fixed matched die; it does not test orientation and counterholding of loose character punches under a hammer.
- **TK-K3R94E ASTM specimen loading:** supports laboratory preparation under Other, but tests wedge-grip seating rather than irreversible identifier registration.

The nail and stamp-seal comparisons remain a material board-duplication risk. The distinction rests on the scored physical skill, not the laboratory story, material, tighter placement tolerance or the number of repetitions.

## Pre-review objections and present answers

| Objection | Build response and remaining limit |
|---|---|
|Unspecified impact/indentation law and reaction|Actual specific contacts, total incident energy and stated yield/work law; native reciprocal contact path. Real brass calibration remains undone.|
|Unreadable/incomplete/doubled mark ambiguity|90% reference length,0.5mm maximum gap,0.15mm registration corridor; strike-video repeat rule; ruler/photo tolerances. Physical scorer agreement remains undone.|
|Unspecified tools and mounting|Named native products, sourced dimensions, full-set procurement, exact starts/yaws, backing and bolt drawings. Detail estimates labeled.|
|No180s full-score run or retained punch|Native evidence is reported below when complete; no qualification is inferred from specifications.|
|Overlap with nail and ink-seal tasks|The scored skill and remaining duplication risk are stated above.|

## Sources and licences

- [TEKTON HMA51108](https://www.tekton.com/8-oz-ball-peen-hammer-hma51108):$27,0.76lb whole weight; [maker dimension drawing](https://images.tekton.com/assets/ball-peen-hammer-spec-table.jpg).
- [Young Bros Machine Made](https://www.youngbrosstampworks.com/machine-made/):08091 native geometry; [using stamps](https://www.youngbrosstampworks.com/using-stamps/) specifies solid support and a moderate hammer blow.
- [08091 complete figure set](https://www.zoro.com/young-bros-stamp-works-hand-stamp-kit-number-set-08091/i/G4599725/):$46.99, supplies all three actual characters.
- [Taytools465968 vise](https://taytools.com/taytools-3-inch-drill-press-vise):$34.99 and published native envelope/jaw sizes.

All geometry, drawings and approximate glyph paths are generated primitives; no third-party CAD, textures or catalog images are redistributed. Code MIT; documentation and generated data CC BY4.0. Product names identify the real-trial equipment.

## Results

This is an honest partial submission, not a qualified full-score task. The recorded runs use the same frozen source `f6c486e53790` and the unchanged instruction.

| Run | Layout A | Layout B | Mean | Recorded budget and outcome |
|---|---:|---:|---:|---|
|Scripted demo `demo-20261122T111117`|0.0|0.0|0.0|3600-step cap; both stopped by the scripted policy after about103seconds|
|Dream-1 `dream-20261122T225508`|0.0|0.0|0.0|2400steps/120seconds per layout; both reached the step limit|

The scripted blows reach about0.14J and stay clear of the punch hand, but the punch rocks1-13degrees during impact. Recorded outline coverage is only about25-34%, below the90% requirement. Both layouts first fail at21.9seconds when the die footprint slips more than0.5mm. Returned tools also fail their parking condition; coupon movement remains unresolved. Material and scoring thresholds were retained.

Dream-1 produced no scored character in either layout. The evaluation completed both trials and recorded no abstentions. These runs do not establish successful task execution or physical validity. No1.0 demonstration or measured robot result exists; brass calibration and physical scorer agreement remain unvalidated.
