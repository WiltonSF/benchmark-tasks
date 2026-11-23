# Secure a cable in a HUMMEL M20 gland with a 24 mm spanner

Electricians fit enclosure cable glands to keep pulls on incoming cables off electrical terminals: [O*NET 47-2111.00](https://www.onetonline.org/link/summary/47-2111.00) reports 821,000 US employees in 2025, and [City & Guilds electrical wiring training](https://www.cityandguilds.com/-/media/productdocuments/engineering/mechanical/4505/centre-documents/4505-l2-engineering-operations-skills-qualification-handbook-v4%2C-d-%2C2-pdf.pdf) includes cable glands and grips. The occupational count describes electricians; it is not a count of operators of this particular gland. The operation combines maintaining insertion depth in a flexible cable with turning a loaded spanner to compress its enclosure seal, then checking retention after release.

Category: **Impact on jobs**. The scene uses a full-size, unpowered electrical enclosure clamped to a bench, a commercial HUMMEL gland and a compatible cable. The left hand operates the spanner; the right positions the cable. The enclosure clamps react the tightening load, while the open supports leave the cable free to move.

## How this differs from nearby tasks

Board-cache searches and live GitHub PR searches/verdicts were read on November 23, 2026. The local board cache contains 990 tasks; current PR states below come from GitHub. Comparisons concern the manipulation and outcome, not the enclosure's appearance or a different layout.

- **Compression joint, [PR 1562](https://github.com/robocurve/benchmark-tasks/pull/1562), TK-3TAU7U, accepted:** assembles a rigid pipe, separate olive and nut in the correct order, then turns a spanner past snug. Here the gland is already mounted; the coupled skill is keeping a flexible cable at its sheath mark while compressing a captive elastomer seal, followed by an independent cable pull. Both use loaded spanner strokes and a bounded tightening range. The earlier compression-joint review required an established snug reference; the gland's calibration status is recorded below.
- **Cable-lug crimp, [PR 741](https://github.com/robocurve/benchmark-tasks/pull/741), final rejection:** inserts stripped conductor into a lug and closes a crimping die to deform metal. Here the tool turns an enclosure gland around intact outer sheath; insertion depth must survive seal compression and a pull test. Its reviews requested electrician job context and affordable matched hardware; its final verdict found overlap with existing tasks. A different electrical component alone does not establish distinctness.
- **Drip loop, [PR 17](https://github.com/robocurve/benchmark-tasks/pull/17), TK-2UDNS3, final rejection as a variant of TK-YDWX58:** routes a hanging cable over a hook to establish drainage geometry. This task works at the gland's compression cap and scores insertion, rotation and axial retention.
- **Prestarted lens retaining ring, [PR 2867](https://github.com/robocurve/benchmark-tasks/pull/2867), TK-ZDNLJA, final rejection:** tightens a specialist optic retainer. This operation includes feeding and retaining a cable in ordinary electrical installation work, with a named large occupation and matched commercial specimens. The lens-ring verdict found specialized assembly insufficiently broad and required an accessible measurable retention check.
- **Earlier cable-gland task, [PR 2145](https://github.com/robocurve/benchmark-tasks/pull/2145), TK-Y6YDGB / TK-8NNP2L, open after two revisable verdicts:** tests the same core operation. This implementation supplies a named gland/cable pair, a physical 24 mm spanner and a native resisting brake, addressing its force and sourcing objections. Those are implementation changes, not a claim of a new core skill. Its original verdicts are preserved below; no measurements from that implementation are transferred.

## Real-trial kit and setup

All working specimens are ready-made commercial stock. Only reusable supports, a grip sleeve and a witness dial require one-time fabrication. Use the exact gland/cable pair for enrollment; a look-alike gland or different cable diameter can change the torque and retention behavior.

| Item | Specification and source | Budget allowance, USD |
| --- | --- | ---: |
| Enclosure | One [Hammond 1591ESBK](https://www.hammfg.com/part/1591esbk), ABS, lid removed; 190.4 x 110.6 x 60.8 mm, 2.4 mm walls | 12 |
| Glands and locknuts | Four [HUMMEL HSK-K 1.209.2000.50](https://www.hummel.com/en/cable-glands/products/plastic-cable-glands/hsk-k/1209200050-hsk-k-grey-m20x1-5/) plus M20 locknuts: polyamide body, NBR seal, M20x1.5, 6-12 mm cable range, 24 mm flats, 27 mm height, 9 mm mounting thread | 20 |
| Cable | 1 m [Miguelez BARRYFLEX-MAN H05VV-F 82010312-50](https://www.miguelez.com/descargas/productos/H05VV-F%20EN.pdf), 3G2.5, nominal 10.0 mm diameter; cut four 220 mm pieces and prepare 20 mm cores on each, leaving 200 mm sheath | 8 |
| Spanner | One [GEDORE 894 24 / 6575570](https://shop.gedore.com/en-de/6575570/), 24 mm, 215 mm overall length (217 mm nominal), 15-degree jaw set, 9 mm head, 154 g | 20 |
| Workholding | Two small padded screw clamps for the enclosure | 25 |
| Reusable fixture materials | PETG or smooth wood supports/rests, 50 g grip sleeve, M3 screws/nuts/washers, 50 mm witness dial | 10 |
| Pull measurement | Spring scale with at least 50 N capacity and 1 N or finer graduations | 20 |
| Torque measurement | Low-range torque wrench covering 4.5 Nm and a 24 mm crowfoot; record adapter orientation and torque correction | 60 |
| Dimension and witness tools | Caliper, millimetre ruler and paint marker | 20 |
| **Total allowance** | Excludes tax, shipping, robot/cameras and one-time fabrication tools | **195** |

These are planning allowances, not a purchased or quoted kit. Price anchors checked November 23: [Mouser lists the enclosure at $11.51 singly](https://www.mouser.com/en/ProductDetail/Hammond-Manufacturing/1591ESBK?qs=%252Bn8HNu1qIodFbLJVEG%252B9tA%3D%3D); [EVG lists the exact gland individually at EUR 0.976 before quantity discounts](https://www.evg.de/en/1.209.2000.50-hummel_et_kabelverschraubung_hsk-k_m20x15_grau); [GEDORE Tools UK lists the spanner at GBP 9.92 including VAT](https://www.gedoretools.co.uk/product-category/hand-tools/spanners/open-ended-spanners/page/8/). The USD allowances include provision for locknuts and small-order sourcing; they are not currency conversions or guaranteed delivered prices. No casting, curing, drying or perishable specimens are required.

The [manufacturer page](https://www.hummel.com/en/cable-glands/products/plastic-cable-glands/hsk-k/1209200050-hsk-k-grey-m20x1-5/) and [VDE certificate 40012033](https://www.sealconusa.com/images/certificates/pdf/HSK-K.pdf) specify 4.5 Nm for the M20 6-12 mm cap. At the sleeve's 165 mm lever, 4.5 / 0.165 = 27.3 N tangential force. This is a load calculation; demonstrated transmission through the robot's grasp is reported separately below.

Coordinates are millimetres, x forward, y left, z above the table. Drill a 20.5 mm hole in the near enclosure wall at x=280, y=15, z=35; fit the gland body and locknut. Orient the enclosure 110.6 mm along x and 190.4 mm along y. Two padded clamps restrain its side walls; clamp feet are at x=355, y=-100 and 130. Verify no body spin or enclosure motion under 4.5 Nm.

Place the cable reference origin at (115,15,35), with sheath end at (205,15,35). Open U supports at (25,15) and (195,15) have 18 x 35 mm feet, floor height 29 mm, lip height 36 mm and a 15 mm open slot. Check cable centre height 35 +/- 1 mm; the supports must not pinch it. The cable remains unpowered and unconnected to terminals.

The spanner's jaw centre starts at (205,50,35), handle toward +y. A reusable sleeve centred 165 mm from the jaw has finished outside dimensions 24 x 60 x 50 mm and nominal 10 x 18 mm handle channel. Make two hollow PETG halves with 3 mm walls/ribs, two M3 through screws at 34 mm pitch, nut traps and washers; fit the channel to the actual handle. Proof-load with 35 N transverse force and reject sleeve slip above 1 mm. The 50 x 75 x 10 mm sleeve support is centred at (205,200); the 60 x 40 x 8.2 mm head rest at (195,50). These locate the idle tool and do not turn the cap. Check that the prepared tool stays within 1 mm and 1 degree for 5 s. Layout B moves only the spanner 20 mm toward -x.

The 50 mm witness dial has 36 ten-degree divisions and a cap pointer. Simulation marks are first contact at 0.75 turns, green from 1.25 to below 1.55 turns, and red at 1.75 turns from reset. Thus green appears at 90-198 degrees after one complete turn, and red at 270 degrees on the second turn. Video must establish the preceding turn count because the dial sectors repeat.

## Enrollment and reset

Enrollment is one-time qualification of exact stock, separate from resetting an enrolled trial. Three gland/cable pairs support repeatability checks; reserve the fourth pair for damage inspection. No physical enrollment has yet been performed.

1. Label the specimens and lots. Measure cable diameter at several positions; enroll only 10.0-10.2 mm intact sheath. Check seal, threads, enclosure clamping and sleeve attachment. Record scale and torque-wrench uncertainty and crowfoot correction.
2. Record cap angle versus torque in 15-degree increments and cable displacement under an axial pull. Define and record a numeric, repeatable measured drag threshold for first contact C before marking each gland. That threshold has not yet been established for this stock.
3. Record the earliest angle passing the proposed 50 N retention test and the angle reaching the maker's 4.5 Nm endpoint. Use the sacrificial pair to inspect for dents, splits, seal extrusion or thread damage after additional increments. Photograph the marks and ruler. Do not transfer a damage angle or torque curve from unspecified hardware.
4. Establish the green band only where the retention test passes without exceeding maker torque; establish a rejection line conservatively before observed damage. Reconcile these measured limits with the simulated bands before treating real and simulated scores as equivalent. Demonstrate the robot/tool can transmit the required load before physical evaluation.

Between enrolled trials: return the cap to zero with the seal open; inspect sheath, seal and threads; replace damaged stock; restore the cable on both open supports and the tool on its rest; check for sleeve or enclosure slip. Layout B shifts only the tool 20 mm toward -x. Estimated reset is **2-3 minutes**, not a measured reset time. Calibration and fixture fabrication are excluded from that estimate. Intact commercial stock is reusable.

## Run and reproducibility

Trials cycle deterministically A then B and average the two lot scores. Object jitter is zero; only the loose spanner's starting position differs. This is a two-layout evaluation, with no claim of unseen-layout generalization. The instruction used for the recorded runs remains:

> Feed the cable into the box to its sheath mark, tighten the gland through one full turn into the green band, park the spanner, and release both hands.

Run from the task directory:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 --policy yam_scripted --scorer task_score --max-steps 3200 --epochs 2 --instruction "Feed the cable into the box to its sheath mark, tighten the gland through one full turn into the green band, park the spanner, and release both hands."
```

The scripted run limit is 3,200 steps, or 160 simulated seconds per lot. The recorded Dream-1 invocation used the same instruction and scorer with `--policy dream --max-steps 2400 --epochs 2`, or 120 simulated seconds per lot. Native diagnostic throughput was about 3-5 control steps/s, plus startup/rendering. Earlier traces using shorter instruction wording are diagnostic only; current results are tabulated below.

## Mechanism and scoring

The cap rotates on a hinge with a coupled 1.5 mm/turn slider. Four radial mimic sliders approximate the sealing cone. Cap/body contacts are stiff; NBR-pad contacts use estimated compliance of 50,000 N/m and static/dynamic friction 0.8/0.75. The native elasto-plastic brake has stiffness 180 Nm/rad and yield limit 4.5 Nm. Its target follows behind actual cap travel and never advances ahead to tighten it; the joint reacts into the installed gland body. Pads physically contact the cable.

The cable has a rigid leading 100 mm and two 50 mm tail links with estimated 0.12 Nm/rad springs. The spanner and sleeve are one rigid body; sleeve fastener preload is not resolved. Finger actuator caps are 40 N each and do not cap transient collision impulses. The scripted controller uses fixed waypoints; a separate per-step observer supplies scoring state.

Every nonzero endpoint needs a current matching observation, cable centreline crossing both bore ends, sheath at least 5 mm past the inner gland end, hands released, and 1 s stillness. The real-trial clearance criterion is more than 5 mm; simulation checks sampled hand surfaces more than 8 mm from cable, gland and tool. Stillness uses linear speed at most 3 mm/s and angular/joint speed at most 0.03 rad/s. Tool parking is within 35 mm of its lot's starting reference and still.

| Score per lot | Additional condition |
| --- | --- |
| 0.25 | Cap below 0.75 turns from reset |
| 0.5 | Cap at least 0.75 and below 1.25 turns |
| 0.75 | Cap at least 1.25 and below 1.55 turns, plus the configured brake-work check |
| 1.0 | The 0.75 conditions, tool parked, and independent axial pull passed |
| 0 | Invalid or mismatched observation, wrong cable path, hand contact, insufficient stillness, failed pull, or any other endpoint outside the ranges above |

The work check integrates positive cap travel beyond first contact against the configured brake load and requires at least 4.5 x pi - 0.15 J (about 13.99 J). It is a model-based work estimate; measured projected cap reaction is diagnostic and does not gate work from the spanner's contact pair. Turning at or above 1.55 earns zero at that endpoint; reaching 1.75 turns latches the angle-based damage proxy permanently at zero, even if subsequently loosened. Sheath damage from other causes is not fully detected.

After release and 1 s stillness, the independent pull ramps to 50 N over 1 s, holds for 1 s and requires maximum cable displacement below 2 mm. A native external wrench loads the cable against the seal's physical contact reaction. A failed pull invalidates success. Returning hands cancels the pull; later cable displacement of 2 mm or cap motion above 0.02 rad invalidates an earlier pass. The 50 N criterion and turn bands are provisional benchmark choices, not asserted product standards or calibrated hardware limits.

A physical evaluator would use side video, turn marks, ruler and spring scale after enrollment. A final photo can show cable penetration, tool parking and hand clearance; video is necessary for turn count, stillness, damage history and the pull. The current angle/work proxy does not establish actual sheath integrity or IP protection.

The scorer controls cover do-nothing/wrong-path endpoints, held parts, invalid observations, turn-history damage and synthetic released endpoints. Seven endpoint cases and eleven synthetic observed-history/release cases exercise the contract, including a broad hand surface only 0.5 mm away. These controls do not establish physical feasibility or that every physical bypass is excluded. Tool-use history, contact-specific work and gripper-induced damage need further validation; strict two-hand necessity has not been established by an ablation.

## Current recorded results

| Policy | Layout A | Layout B | Limit and outcome |
| --- | --- | --- | --- |
| Scripted demo | 0.0 | 0.0 | 3,200-step limit; stopped after 2,381 actions / 119.05 simulated seconds in each lot |
| Dream-1 | 0.0 | 0.0 | 2,400 actions / 120 simulated seconds per lot; both hit the step limit; no abstentions or errored trials |

Exactly one Dream invocation produced the two recorded epochs with the frozen instruction. Videos and setup show layout A; runner logs and force audits cover both lots. The matching-source scripted recording includes the existing 8 mm layout-B sleeve-pickup TCP correction. Earlier media are not reported as current results. Both videos decode completely as H.264/yuv420p and are under 50 MB: scripted 1920 x 360, 20 fps, 2,382 frames; Dream-1 960 x 176, 20 fps, 2,401 frames.

Scripted maximum cap travel was 0.177718 rad (10.18 degrees) in A and 0.647273 rad (37.09 degrees) in B. Peak projected cap reactions were 5.263 and 4.501 Nm; these brief peaks do not establish sustained tool transmission. Scored compression work remained 0 J, stillness remained 0 s and neither retention test ran. Sampled contact peaks were 44.96 and 48.07 N, including collision transients; these are not actuator-force or hardware measurements. Peak and final scores were 0.0. Dream-1 recorded no correct insertion, meaningful cap rotation, scored work or retention result in either lot.

Status: implemented native simulation prototype with offline checks, **no full-score demonstration, hardware calibration or physical robot results**. The existing independent pre-packaging review recorded NO-GO. Remaining evidence gaps are sustained tool/load transmission, the torque-angle and damage calibration, a numeric first-contact criterion, tool-use/contact-work validation, physical sleeve proof load, cable-mark reconciliation and physical stillness. Factory IK, collision and scorer checks do not resolve those gaps. The current qualification package records source/runtime provenance, per-lot scores, force audits and media checks.

## Earlier cable-gland review findings (verbatim)

> **Not accepted** (TK-Y6YDGB).
> 
> Construct validity (not met): The specified turn bands and crushing threshold are not established for a real gland and cable, so the scored result may not correspond to a correctly secured or damaged installation.
> 
> Scoring (not met): Real-trial scoring depends on a first-contact reference and damage threshold without a reproducible calibration for the specified hardware.
> 
> Clarity and reproducibility (not met): The custom gland has no identified real equivalent or procedure for establishing the required turn marks before a trial.
> 
> What to change: Specify a readily available gland and compatible cable, and provide a repeatable pre-trial procedure to establish the first-contact reference and turn marks. Validate the snug and damage thresholds against that hardware, adjust the real and simulated scoring accordingly, and verify that the specified robot can apply the required tightening force; provide a suitable tool if needed.
> 

> **Not accepted** (TK-8NNP2L).
> 
> Construct validity (not met): The simulated nut turns at a small fraction of the real tightening torque, removing a defining physical demand of the job.
> 
> Feasibility (not met): A wrist torque rating does not establish that parallel grippers can transmit the required torque through the 24 mm nut without slipping.
> 
> What to change: The tightening-force verification remains open: a 7 Nm wrist rating alone does not show that the grippers can transmit the gland's required torque without slipping, and the simulated nut turns at a small fraction of the real torque. Measure or otherwise substantiate the required torque across the scored snug range, make the simulated nut require approximately that physical effort, and demonstrate the task within the action limit; provide and use a suitable spanner if direct gripping cannot transmit it. Update the fidelity note and calibration details to match.
> 

## Response to the earlier verdicts

| Requirement | Present implementation and evidence |
| --- | --- |
| Named compatible gland and cable; reproducible specimens | Exact HUMMEL 6-12 mm gland and Miguelez nominal 10 mm cable specified, with stock quantities, budget and enrollment/reset procedure |
| First-contact, snug and damage calibration | Procedure supplied; physical measurements and a numeric first-contact criterion remain pending |
| Representative tightening effort | Native 4.5 Nm brake implemented; maker/VDE supports the cap endpoint, while the stock's torque-angle curve remains unmeasured |
| Gripper transmission and suitable tool | Named 24 mm physical spanner and sleeve supplied; full-range transmission remains unverified in the recorded 0.0 demos |
| Full score within the action limit | Not demonstrated in either lot |
| Fidelity and scoring match the evidence | Configured work and angle-based damage proxies disclosed; physical equivalence remains unvalidated |

Geometry is authored from primitives; manufacturer CAD is not redistributed. Code: MIT. Documentation: CC BY 4.0.
