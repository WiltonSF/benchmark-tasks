# Tighten and clip the C-A-T Gen 7 training windlass

Emergency-care trainees practise tightening a windlass tourniquet and maintaining compression while securing its rod. This task isolates that step on a reusable training cylinder. The category is **Other**. A gauge is a trainer indicator, not a clinical treatment threshold. Complete application, initial band routing, the security strap and recording time are outside the task.

Instruction: “Tighten the C-A-T trainer until its gauge is green, lock its rod in its attached clip, then move both hands clear.”

## Provenance and present evidence

This independently written task credits [PR 1026, TK-U4G6F6](https://github.com/robocurve/benchmark-tasks/pull/1026). Earlier versions are TK-NMSLU5, TK-NEMG7P and TK-EL693A. Original code and assets were not copied. Shared helper code is derived from the existing task kit. All four prior verdicts, including Flags, are preserved below.

**Estimated loads are explicit; actual C-A-T force measurements are absent.** The last verdict's measurement request remains unresolved. `fixture/LOAD-ESTIMATES.md` gives the source, equations and assumptions used instead during simulation development. `fixture/CALIBRATION.md` remains the hardware measurement procedure; its empty JSON template is not evidence. Native results and model scores are recorded separately from CPU probes and hardware results. No hardware trial has been performed.

Current estimated-model validation: Isaac Lab-Arena scripted trials score **1.0 in A and B**; the single final Dream-1 run scores **0.0 in A and B**. Both policies use the same frozen executable source, assets and instruction. Full results appear in the final validation section below.

## Product, fixture and lots

Use the [NAR blue C-A-T Gen 7 trainer 30-0033](https://www.narescue.com/combat-application-tourniquet-c-a-t-tb.html), with its **original attached clip and stabilization plate**. The [maker's trainer sheet](https://www.narescue.com/downloadable/download/sample/sample_id/2003/) identifies the model and NSN 6910-01-560-2972. Detailed rod/clip dimensions are estimates, not maker CAD. The [maker's procedure](https://www.narescue.com/cat-instructions-for-use) says: “Secure the rod inside a clip to lock it in place.” No clip is glued to the training limb.

The reusable cylinder is 127 mm in diameter and 400 mm long, centred 230 mm forward of the arm-base midpoint with its axis 65.5 mm above the table. Its axis lies across the table along +y. Four 30 x 40 x 35 mm wooden chocks and two 20 mm webbing end straps secure it. Screw the chocks to a wooden tabletop; pass each end strap over the cylinder and beneath the tabletop, tensioning it down into the chocks. The simulator fixes these anchored supports. The central blue band is 38 mm wide; the approximate 150 x 12 mm rod starts level at azimuth 126 degrees, with its centre 150.5 mm above the table. The two factory clip channels lie approximately 50 mm from the windlass at azimuths 156 and 204 degrees, with a 13 mm channel, 10.7 mm mouth and approximately 21-degree seated tilt. `task.json` and `fixture/layout.svg` document enrollment.

The demo uses two reachable winding-grip branches within the wrist range. The other hand stays at ready through the first stroke, then counterholds and turns the short section through the wrist reset before the right hand regrips. Both hands are still required by the demonstrated clipping motion: one maintains winding torque while the other takes the opposite end and pushes it through the clip lips. The demo does not read simulator state, hidden flags or exact-pose feedback. Fixed positional lag compensation is part of its open-loop motion; jaw angles follow the intended rod angle. This must be checked in native traces. Public-layout IK computes fixed joint endpoints; the local demo helper interpolates joints to avoid an unreachable Cartesian wrist-reset path. It reads no simulator feedback.

Only the rigid gauge/stand moves between lots: its face centre is (360,-180,105) mm in A and (400,+220,105) mm in B. The articulated tourniquet, clip joints, cylinder and winding start pose remain identical. Run both lots and average. Zero object jitter is required; seeds do not alter the fixed layouts. These two layouts do not exclude memorizing both motions.

## Reusable bladder and cost

Use a 470 mm section of a Sunlite 26 x 1.95–2.125 bicycle inner tube. A [catalogue supplier lists this tube family around $6](https://www.hermosacyclery.com/accessories/tubes/). Enroll a flat width of 48–54 mm; tyre-size markings alone do not prove width. Fold one end twice by 15 mm and clamp its full width with a 51 mm binder clip. Insert a 1/4-inch barb 20 mm into the other end, gather the rubber and secure it with two zip ties and tape. This leaves approximately 420 mm active length, enough for the 399 mm circumference plus 21 mm overlap.

Run the port through tubing and a Y to an [MDF Calibra 808M aneroid gauge/bulb kit](https://mdfinstruments.com/collections/calibra-series), listed by the maker around $42. The modeled 52 mm face is an estimate to verify on enrollment. Pump only under the snug band. Close the bulb branch with a pinch clamp, detach the bulb and cap the open branch. Require <=2 mmHg loss in 5 minutes at 60 mmHg and <=4 mmHg in 2 minutes at green before enrollment. Keep the original band tail fastened and taped, and fold the time strap aside.

A stock [5-inch round foam bolster](https://actionup.com/products/round-foam-bolster-54-long-4-10-diameter-medium-compression-cylinder-foam.html), F15036RB05, can be cut once to 400 mm. The supplier describes medium-soft upholstery foam. The scene uses rigid cylinder geometry; pressure and winding compliance remain declared estimates requiring enrollment and matched curves on the actual foam. It is reused. Approximate budget before tax/shipping: trainer $35, gauge kit $42, tube $6, fittings/clips/tubing $20 and reusable cylinder/chocks/straps $25, total **$128**. Hardware and fixture amounts beyond the named catalogue figures are estimates. A shared force instrument is additional. Nothing was purchased. Initial bladder preparation is planned at approximately ten minutes; a normal reset should take a few minutes, but neither duration has been measured.

## Physical model and independent score

The winding joint has an implicit 0.529 N m/rad spring and 0.20 N m s/rad damping, giving 1.94 N m at 210 degrees. The 0.10 N m/rad tilt spring represents woven-band bending. Clip lips have physical colliders, 50 kN/m springs, 40 N s/m critical damping and 8 g effective masses. Their floors and walls collide with the rod. Pressure is the declared proxy min(300,60+absolute winding degrees), not simulated blood flow or a measured pneumatic calibration.

Every credited snap needs native rod–lip contact of at least 10 N, at least 1 mm downward contact traversal and at least 1 mJ contact work during one advancing entry with contact witnessed on each lip. Finger-to-lip pressing cannot set that record. The historical energy/proximity shortcut from the first prototype has been removed. Held credit needs native hand–rod contact; proximity alone does not count. The rod has no scripted pose writes, weld grip or force-free retention rule. Contact springs and constraints transmit reactions through the band fixture and both hands.

For a real trial, use gauge/clip photos and video for timing. After the scored hold, the assessor confirms retained seating with a 2 N upward pull along the clip normal at the clipped rod end for 1 s, recording any extraction. This fixed gentle verification load is approximately a 200 g weight; the complete insertion/extraction-force measurement remains part of calibration. Below-green retention takes precedence: it earns **0.25 even with an arm nearby**. Otherwise, a retained green rod earns **1.0** after all arm surfaces remain at least 10 mm from the rod and both clips and the winding and tilt angular speeds each remain at most 1 degree/s for one second. Green retained or level-held progress earns **0.5** without that release hold. Everything else earns **0**. Missing observations fail closed. The numeric green range is 250–300 mmHg. Reading the gauge while contacting the thigh, bladder or gauge does not count as green.

The release audit samples broad arm faces and includes a 6 mm conservative sampling margin. The thigh, bladder and gauge contact guard uses that same conservative margin; a contact between sampled surface points cannot authorize a green reading. Native telemetry records the sampled body gap and release clearance. Fingertip contact with the cylinder or attached clip during manipulation is physically allowed; it does not waive arm-link collisions or the final clearance gate. `DEMO_TOUCHES` documents that narrow offline-check exception. Native collisions and forces stay enabled. A current-state certificate prevents substituting an arbitrary state for observed evidence.

| Bypass or near miss | Required score |
|---|---|
| No action, no known contact observations, or loose unlocked rod after release | 0 |
| Early below-green lock with hands near or clear | 0.25 |
| Green rod still held level | At most 0.5 |
| Green retained rod with an arm still near it | At most 0.5 |
| Independent finger pressing of clip lips | No verified snap credit |
| Pressure above 300, or a reading while pressing the gauge/bladder/thigh | No green credit |
| Retained state without representative contact work | 0 |
| Full-score request before a complete second released and still | At most 0.5 |

## Run and reset

In the installed Isaac Lab-Arena / Inspect Robots environment, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --epochs 2 \
  --instruction "Tighten the C-A-T trainer until its gauge is green, lock its rod in its attached clip, then move both hands clear." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir demo-output
```

Use `--policy dream` once on the final frozen source with the same instruction after the full-score demo. Per-trial run limit is 120 seconds of control time; native runtime and actual scores must come from run records. MuJoCo CPU probes are development evidence only. `near_misses.py` contains synthetic score-boundary and face-clearance checks; these are not robot results.

Between trials, lift the rod from its own clip, untwist to the level 126-degree start, inspect the band, chocks and end straps, and restore the gauge to 60 +/-2 mmHg if needed. Isolate and remove the bulb again. Move only the rigid gauge/stand for the selected lot. Inspect wear and seals and repeat enrollment every ten trials or after leaks. No per-trial casting, printing or curing is required.

## Distinctness and licensing

Live PR searches found source PR 1026 and PR 917, which tests releasing a phlebotomy tourniquet before withdrawing a needle. Loaded winding and clip retention are a different step. Fingerprint screening also returned ratchet-strap tensioning, repeated handle turning and lever-carrier tasks. A later same-core claim must be checked again before runs and handoff; this folder is not an exclusive reservation.

Task code and original drawings follow the task repository's MIT release. No original-PR asset, third-party mesh or font file is included. The dial is a procedurally rendered label using Pillow's default font. Referenced product sheets and articles remain external sources.

## Complete prior verdicts and point-by-point disposition

2026-11-12T09:34:25Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6819993757

> **Not accepted** (TK-NMSLU5).
> 
> Construct validity (not met): The simulated latch holds the rod down, but the specified clip's 18 mm gap between its lips cannot retain a 12 mm rod.
> 
> Clarity and reproducibility (not met): The specified 12 by 22 cm cuff bladder cannot wrap once around the approximately 40 cm circumference of the thigh.
> 
> Feasibility (not met): The real clip's lips do not overlap the rod, so the specified snap-in retention cannot be reproduced.
> 
> What to change: Please revise the trainer geometry so the windlass rod can snap past the clip lips and be retained: the current 4 mm clearance on each side and 1 mm lips leave an 18 mm opening for a 12 mm rod. Specify a cuff bladder long enough to wrap around the approximately 40 cm thigh circumference, and update the pressure calibration and simulated lock as needed to match the corrected build.
> 

2026-11-14T05:50:29Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6872656235

> **Not accepted** (TK-NEMG7P).
> 
> Flags: Prior review feedback for the earlier version was not provided
> 
> Importance (not met): Applying a windlass tourniquet is emergency first aid rather than a scenario most people know firsthand, although its lifesaving impact supports the Other domain.
> 
> Construct validity (not met): The setup removes the tourniquet's own clip and glues a custom clip to the thigh, changing how the windlass is secured in real use.
> 
> What to change: Use a standard windlass tourniquet with its own attached clip rather than replacing it with a clip glued to the training limb; update the setup, simulation, calibration and full-score demo accordingly. Change the domain to Other and explain the task's emergency-care impact. Clarify whether a rod locked below green with an arm still nearby earns 0.25 or 0.5.
> 

2026-11-16T21:11:30Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6929474142

> **Not accepted** (TK-EL693A).
> 
> Construct validity (not met): The simulated clip offers no snap resistance and the assumed green-range torque is only 0.22 N m versus the submission's 1–2 N m real-device estimate.
> 
> Clarity and reproducibility (not met): The custom heat-sealed bladder has dimensions but no repeatable quick preparation method or source for equivalent bladders.
> 
> What to change: The earlier clip, domain, scoring-order and bladder-length changes are addressed. Two new points were not raised before: measure the winding resistance and clip snap force on the specified trainer, update the simulation so the robot performs comparable work when turning and seating the rod, and provide an updated full-score demo; also specify an inexpensive source or a quick, repeatable preparation procedure for the 5 × 42 cm bladder, including its tube and sealed port.
> 

2026-11-19T00:27:30Z — https://github.com/robocurve/benchmark-tasks/pull/1026#issuecomment-6995781191

> **Not accepted** (TK-U4G6F6).
> 
> Construct validity (not met): The trainer's winding resistance and clip snap force have not been measured, so the revised simulation's force fidelity remains unverified.
> 
> What to change: The trainer-force measurements requested in the previous review are still open. Measure the winding resistance and clip snap force on the specified C-A-T trainer, report the results, update the simulated loads to match them, and provide a full-score demo using those loads. The revised resistance model and the bladder preparation procedure address the other points.
> 

| Requested correction or flag | Current disposition |
|---|---|
| Clip opening must overlap and retain the 12 mm rod | 10.7 mm mouth and physical spring lips; retention must be established in final native results |
| Bladder must encircle a 400 mm circumference | 420 mm active reusable bladder with approximately 21 mm overlap |
| Original attached clip rather than a limb-glued replacement | Named C-A-T 30-0033, factory clip and stabilization plate preserved in the real setup; geometry is an explicit estimate |
| Domain and emergency-care impact | Other, with a training-step context and no invented worker count |
| Below-green lock with an arm nearby | Explicit 0.25 precedence, tested synthetically |
| Prior review feedback flag | All four verdicts reproduced above, with this disposition table |
| Realistic winding and snap work; updated full-score demo | Estimated mechanical loads and native contact gating implemented; qualified native results must be reported separately |
| Quick, inexpensive bladder including tube and sealed port | Stock tube, full-width folded seal, barb, Y, clamp, cap and leak-enrollment checks specified |
| Measure actual C-A-T trainer winding and snap forces | UNRESOLVED: no physical readings; documented estimates are an alternative for simulation development, not fulfillment of this request |
| Match simulation to those measurements | UNRESOLVED until hardware data exists; current loads are explicit, source-based estimates |

## Revised contact and retention model

The band-anchor surrogate now sits within the band rather than above it. The prior root block could contact the rod near 9 degrees of tilt; that extra rigid stop was inconsistent with the declared elastic-band hinge. The windlass axis is estimated at 150.5 mm above the table, giving the rod clearance over the existing stabilization plate. Both lots use the same articulated pose. Rod, anchor, thigh, plate, clip, lips and hands keep their physical colliders.

The clip's estimated 13 mm interior carries lateral winding reaction; 12 mm sidewalls support the rod below the overlapping 10.7 mm mouth. Effective lip modes use 50 kN/m springs, 8 g mass and critical damping 2sqrt(km)=40 N s/m, while retaining the 40 N drive cap. Internal joint-pair contact is excluded for the connected molded lip/base, as in the task kit; external contacts remain active. These are unmeasured reduced mechanical parameters.

The production snap monitor now requires an above-mouth approach, the same aligned rod end, an advancing downward contact traversal, a 10 N peak, 1 mm net contact descent and 1 mJ work. It counts new depth using the lower force at consecutive contact observations. Asymmetric bevel engagement can load one lip before the other; the 10 N threshold applies to the complete clip reaction, with a physical rod-contact witness on each lip. The rod-axis approach plane is 8 mm above the seated axis; this is an observer condition, not a capture constraint. Reversals more than 0.2 mm above the deepest observed point, alignment loss or end changes discard the attempt; below-lip oscillations cannot arm it. Verified entry does not create a latch constraint. A retained rod may rest against the channel wall or rise below the lip centre, so the geometric check includes that physical clearance.

A coupled MuJoCo CPU test uses the full winding spring, tilt spring, actual rod/clip dimensions, spring caps, anchor, cylinder and stabilization plate. Physical pads perform the entry and withdraw. The revised model retains the rod under load at about 21 degrees, with about 13 N upward lip reaction. A 20 Hz replay through the actual monitor verifies the entry; no-press and no-lips controls receive no snap credit. Friction can leave the no-lips control resting in the channel, which does not satisfy the contact-history gate. These are CPU development results, not a native full-score demo or hardware measurements. New native results must be recorded separately.

The retained-state audit also reads the actual slider positions: the current mouth must remain at least 0.2 mm narrower than the 12 mm rod after a witnessed entry. This prevents a spread clip or a friction jam from being mistaken for a closed clip. The earlier native trace on the 10.9 mm mouth estimate showed an approximately 0.2 mm loaded lip displacement after release, leaving that mouth approximately 11.1 mm wide. This is a simulator observation, not a hardware measurement.

## Final frozen-source validation

| Policy | Lot A | Lot B | Final state |
|---|---:|---:|---|
| Scripted, Isaac Lab-Arena | 1.0 | 1.0 | Green, retained past both lips, both arms clear and rod still for 1 s |
| Dream-1 | 0.0 | 0.0 | Both 2,400-step limits; neither green nor locked |

The physical winding spring remains 0.529 N m/rad, approximately 1.94 N m at the lock. The final 10.7 mm estimated mouth gives observed upward rod-lip peaks of 11.44 N in A and 10.76 N in B. Each entry contacts both lips; advancing contact descent is 4.41/4.37 mm and conservative contact work 22.75/24.86 mJ. Final winding is 209.38 degrees, tilt about -21 degrees and the pressure proxy about 269.38 mmHg in each lot. The loaded lip openings are about 10.99 mm around the 12 mm rod.

At the final hold, all-arm sampled rod/clip clearance is 135.09 mm and the sampled thigh/bladder/gauge gap is 43.70 mm. After the 6 mm conservative allowance these bounds remain 129.09 mm and 37.70 mm. Both trials finish the complete one-second hold; no contact-observation error occurs. Articulated starts are identical in both lots. Forty synthetic controls pass, including below-lip oscillation, cumulative reversal, isolated force spikes, no force, one-lip-only histories and spread-lip retention. Coupled CPU retention/replay and no-press/no-lips controls are separate development evidence.

Dream-1 performs two 120-second trials on the identical executable source and instruction. Both final scores are zero, with no evaluation error or recorded abstention. Neither reaches green or verifies a clip entry. The instruction is unchanged. These are simulated robot-policy results; no C-A-T hardware force measurement or physical robot trial has been performed.

Geometry provenance: the [maker](https://www.narescue.com/combat-application-tourniquet-c-a-t-tb.html) publishes a 6.5-inch packaged length and 37.5-inch open strap length. These are scale checks, not rod or clip measurements. The source PR supplies the nominal 150 x 12 mm rod prior, and the 150 mm length fits within the maker's 165.1 mm packaged length. Rod diameter, hook locations and lip/channel dimensions remain explicit mechanical estimates requiring enrollment on the specified trainer. The original measurement request remains open; published sibling-device forces and the material prior support the estimated simulation loads.

The assessor-style 2 N confirmation was additionally checked in the coupled CPU model after both physical pads withdraw, with the full winding spring retained: one second at 2 N preserves verified seating and a closed mouth, with the rod axis below 0.006 mm above the nominal seat. This is a CPU development check of the stated protocol. Actual C-A-T insertion/extraction curves and the physical enrollment checks remain open.
