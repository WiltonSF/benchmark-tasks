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
| Glands and locknuts | Four [HUMMEL HSK-K 1.209.2000.50](https://www.hummel.com/en/cable-glands/products/plastic-cable-glands/hsk-k/1209200050-hsk-k-grey-m20x1-5/) plus M20 locknuts: polyamide body, NBR seal, M20 x 1.5, 6-12 mm cable range, 24 mm flats, 27 mm height, 9 mm mounting thread | 20 |
| Cable | 1 m [Miguelez BARRYFLEX-MAN H05VV-F 82010312-50](https://www.miguelez.com/descargas/productos/H05VV-F%20EN.pdf), 3G2.5, nominal 10.0 mm diameter; cut four 220 mm pieces and prepare 20 mm cores on each, leaving 200 mm sheath | 8 |
| Spanner | One [GEDORE 894 24 / 6575570](https://shop.gedore.com/en-de/6575570/), 24 mm, 215 mm overall length (217 mm nominal), 15-degree jaw set, 9 mm head, 154 g | 20 |
| Workholding | Two small padded screw clamps for the enclosure | 25 |
| Reusable fixture materials | PETG or smooth wood supports/rests, 50 g grip sleeve, M3 screws/nuts/washers, 50 mm witness dial | 10 |
| Pull measurement | [Eisco PH0036GALPRM](https://www.eiscolabs.com/collections/spring-scales), 50 N capacity, 1 N graduations; price allowance only | 20 |
| Torque measurement | [TEKTON TRQ50402](https://www.tekton.com/1-4-inch-drive-dual-direction-micrometer-torque-wrench-trq50402), 20-200 in-lb, $85 including 1/4-to-3/8 adapter, and [WCF14124 24 mm crowfoot](https://www.tekton.com/3-8-inch-drive-x-24-mm-crowfoot-wrench-wcf14124), $19; keep crowfoot 90 deg to wrench and record orientation/correction | 104 |
| Dimension and witness tools | Caliper, millimetre ruler and paint marker | 20 |
| **Total allowance** | Excludes tax, shipping, robot/cameras and one-time fabrication tools | **239** |

Except for the torque tool/crowfoot prices explicitly quoted, these are planning allowances, not a purchased or fully quoted kit. Price anchors opened November 24: [EVG lists the exact gland individually at EUR 0.976 before quantity discounts](https://www.evg.de/en/1.209.2000.50-hummel_et_kabelverschraubung_hsk-k_m20x15_grau); [ChadsToolbox lists the exact spanner at $10.88, usually shipping in 2-5 days](https://chadstoolbox.com/gedore-6575570-single-open-ended-spanner-24-mm-894-24/). The USD allowances include provision for locknuts and small-order sourcing; they are not currency conversions or guaranteed delivered prices. No casting, curing, drying or perishable specimens are required.

The [manufacturer page](https://www.hummel.com/en/cable-glands/products/plastic-cable-glands/hsk-k/1209200050-hsk-k-grey-m20x1-5/) and [VDE certificate 40012033](https://www.sealconusa.com/images/certificates/pdf/HSK-K.pdf) specify 4.5 Nm for the M20 6-12 mm cap. At the sleeve centre 165 mm lever, 4.5/0.165=27.3 N tangential force; the scripted grasp is 5 mm toward the head at 160 mm, requiring 4.5/0.160=28.125 N. This is a load calculation; demonstrated transmission through the robot's grasp is reported separately below.

## Revision status and limits

This revision changes the cap resistance, mark scoring and pull check. On the current frozen source the scripted demonstration scores **A/B 1/0.5** within 3200 steps (2231 and 2381 actions) and Dream-1 scores A/B 0/0. In layout A all three load signals stayed at or above 4.5 Nm for 3.30 s (peaks: modeled 4.574, contact 4.782, reaction 4.758 Nm, all below 5 Nm), then three 50 N, 1 s pulls passed with 0.447 mm residual. In layout B the cap peaked at 148.40 deg (modeled 4.279 Nm), short of the 150 deg / 4.5 Nm certificate, so B earns the 0.5 tier; no damage occurred. A full score is therefore demonstrated in one layout only. Historical pushed-source recordings scored **A/B 0.0/0.0** for both policies; they do not validate this changed simulation. The 90 deg first-grip-to-4.5 Nm turn, 0.3 Nm free-running drag and 50 N pull with <=2 mm movement at 4.5 Nm were obtained by phone from a technician who works with this product (Nov 2026). The intermediate torque-angle curve, retention-versus-angle interval and damage onset remain model estimates or evidence gaps.

## Layout, mark and numeric references

Coordinates are millimetres: x forward, y left, z above the table. The unpowered enclosure is 110.6 along x and 190.4 along y, with its near wall at x 255 and gland axis (255, 15, 65). Drill a 20.5 mm hole and use the body locknut; two padded clamps at x 330, y=-100/130 hold the enclosure. Tighten the body against its locknut to 4.5 Nm, and bolt a reusable printed 24 mm key engaging the body flats to the enclosure fixture to counter-hold the cap torque. Proof-check the assembly at 4.5 Nm; reject body rotation over 5 degrees. The simulated fixed body represents this counter-hold. This fixture has not been physically proof-tested. A 110 x 190.4 x 30 mm reusable wood block raises the enclosure 30 mm above the bench; the clamps reach 95 mm. This gives the sleeve room for negative-angle re-seating without changing the gland load.

Cable reference origin is (90, 15, 65), sheath end (180, 15, 65). Supports at (0, 15) and (170, 15) have 18 x 35 feet, floor 59 mm, lip top 66 mm and 15 mm open slot. They support without pinching. Paint a 4 mm wide green band with centre **50 mm from the sheath end**, and a 1 mm white pull witness 90 mm from that end (the rigid lead reference point). Use two orthogonal ruler/video views to measure the white witness displacement norm, matching the simulated 3D residual. Both bands are visual only and do not restrain the cable. At the endpoint its centre must be **within +/-3 mm of the cap's outer face**, measured along x with a ruler. Code follows that face as the cap advances; wrong bore alignment or less than 5 mm sheath beyond the inner body thread earns zero. The primitive model places the reset cap face at x 228 and inner thread end at 264:36 mm apart, using HUMMEL's nominal H 27+GL 9 envelope. Exact cap/body internal dimensions are not measured; the mark target always follows the modeled or real outer face.

The native first-grip calibration observations and exact isolated probe are in qualification/first-grip/. They distinguish failed ramp data from completed full-load holds. They are not hardware enrollment.

The spanner starts with jaw centre (215, 40, 35), handle toward +y. Layout B shifts only its x position to 195. A 24 x 60 x 50 mm, 50 g sleeve sits 165 mm from the jaw, with channel fitted to the real handle (nominal 10 x 18), 3 mm walls and two M3 bolts at 34 mm pitch. Proof-load 35 N and reject >1 mm slip. The sleeve rest is 50 x 10 x 10 at (215, 235); head rest 60 x 40 x 8.2 at (205, 40). Verify <=1 mm/1 deg drift for 5 s. These rests never hold the working head on the cap. The nominal working lever is 160 mm, 5 mm toward the head from the sleeve centre. A 3 mm jaw-throat bias shifts the pickup TCP to about 162.9 mm along the handle; native grasp slip changes that value under load, so 160 mm is not claimed as an exact measured lever. B shifts the pickup 8 mm across the sleeve for reach. Per-stroke transforms are fixed unloaded native calibrations; the centered third-stroke pinch does not retain that pickup offset. No guide rails constrain the working tool.

The 50 mm witness dial has 5-deg ticks. Current **model** reset is 60 deg before operational first grip: the first 15-deg grid point passing three 10 N/1 s pulls with <1 mm motion. Isolated native calibration at 45 deg failed during the ramp, before reaching 10 N (32.66 mm movement at 1 N); 60 deg completed all three holds, with 0.0611 mm peak and 0.0530 mm maximum post-unload residual movement. Geometric touch occurs earlier, at 50.4 deg; it is not the scoring reference. A cubic interpolation of the phone endpoints reaches 4.5 Nm at 150 deg and 5.0 at 153.44 deg. The green sector is 150 <=q<153.44; red starts 153.44. These are model-derived angles, not real stock readings. Physical stock receives independently enrolled torque marks, rather than blindly copying the model dial.

## Real-trial procedure

Every number below is one concrete step with the kit listed above: the click torque wrench with the 24 mm crowfoot held at 90 deg to the wrench, the 50 N spring scale, a millimetre ruler, a caliper, a paint marker and the 50 mm witness dial (5-deg ticks) fixed behind the cap. The 0.3 Nm free drag is below the wrench range and is not measured locally.

**A. Enroll each gland/cable pair once (about 10 minutes per pair).** Use three exact pairs for trials and keep a fourth for damage inspection.

1. Cable: caliper the sheath at three points; accept 10.0-10.2 mm. Paint the 4 mm green band centred 50 mm from the sheath end and the 1 mm white witness 90 mm from the sheath end.
2. Fit the cable through the mounted gland until the green band centre is level with the cap's outer face (ruler, +/-1 mm). Hand-start the cap until it turns freely by hand.
3. First contact C: turn the cap in 15-deg steps on the dial. After each step hang 10 N axially on the cable for 1 s with the spring scale, unload, and read the white witness with the ruler. C is the first step at which the witness moved less than 1 mm. Mark C on the dial.
4. Green mark G: set the wrench to 4.5 Nm (the maker's installation torque) and turn the cap slowly from C until the wrench clicks. Mark G. Acceptance: G is 60-120 deg past C (the phone figure is about 90 deg); otherwise reject the pair as a wrong or faulty specimen.
5. Retention at G: three 50 N axial pulls of 1 s each; the white witness must return to within 2 mm of its start after each. Otherwise reject the pair.
6. Red mark R: set the wrench to 5.0 Nm and continue from G until it clicks. Mark R. 5.0 Nm is a benchmark limit set 10 percent above the maker's 4.5 Nm (4.95 Nm rounded); no measured damage onset exists. On the fourth pair only, continue in 0.5 Nm steps and record the first thread skip, crack, seal extrusion or sheath dent seen under a loupe; this records damage onset but does not change R.
7. Reset mark: loosen and mark the dial 60 deg before C. Inspect seal, threads and sheath; reject any visible damage.

**B. Reset before each trial (2-3 minutes).** Loosen the cap to its reset mark. Lay the cable on the two open supports with the sheath end at x 180 mm (ruler from the enclosure's near wall at x 255). Place the spanner on its rests with the jaw centre at (215, 40) for lot A or (195, 40) for lot B. Check the enclosure clamps and sleeve bolts by hand.

**C. Score after the robot stops (about 2 minutes).**

1. Both hands at least 8 mm from cable, gland and spanner, and nothing moving for 1 s (video).
2. Mark: green band centre within +/-3 mm of the cap's outer face (ruler along the cable). Otherwise the trial scores 0.
3. Cap angle on the dial: before C scores 0.25; from C up to G scores 0.5; at or past R, or any visible damage, scores 0.
4. Snug check, only if the cap is between G and R: set the wrench to 4.5 Nm and apply it in the tightening direction. It must click before the cap turns 5 deg further. A pass gives 0.75.
5. Retention: three 50 N axial pulls of 1 s each, white witness residual no more than 2 mm after each, and the spanner within 35 mm of its rest position: 1.0.

Physical and simulated angle tables stay separate: each physical pair uses its own enrolled C, G and R, not the simulation dial. Enrollment, damage inspection and the snug check have not been performed on real stock; the phone figures below are the only real measurements.

## Published values and estimated model

[HUMMEL's exact article datasheet](https://www.hummel.com/fileadmin/pim_import/Datenblaetter/1209200050_EN.pdf) specifies 4.5 Nm. The full [VDE 40012033 certificate](https://www.sealconusa.com/images/certificates/pdf/HSK-K.pdf), PDF page 5, appendix page 1/17, lists the M20 x 1.5/6-12 mm PA/NBR row as cap torque 4.5 Nm, 9 mm thread and anchorage Type A. At the nominal 160 mm lever, 4.5/0.160 = 28.125 N tangential force; the actual loaded grasp position can differ. This calculation alone does not establish robot transmission.

[A cable-gland manufacturer's reproduction of IEC 62444 tables and methods](https://www.securecablegland.com/introduction-to-cable-gland.php) gives Type A anchorage 42 N for >8-11 mm cable. The full method uses 50 one-second pulls and <=2 mm residual slip. The phone figure sets the task to **50 N**, higher than that class minimum. This task uses **three** pulls, so a pass is a shortened benchmark check, not certification. The separate 15 N/5 min retention test and 0.15 Nm twist test are not implemented.

Single named constants at the top of task.py keep estimates replaceable:

| Constant | Value and basis |
| --- | --- |
| CAP_TORQUE_NM | 4.5, exact maker value |
| TORQUE_TOL_NM | 0.0, no certificate below the maker's 4.5 Nm installation torque |
| DAMAGE_TORQUE_NM | 5.0, 1.1 x 4.5=4.95 rounded to 0.1 Nm; benchmark rejection line above maker torque, no measured damage onset |
| FREE_DRAG_NM | 0.3, obtained by phone from a technician who works with this product (Nov 2026) |
| FIRST_CONTACT_DEG | 60, numeric 10 N/<1 mm/1 s grip criterion, calibrated on 15-deg grid in native simulation; physical reset is enrolled 60 deg before measured C |
| FIRST_GRIP_PRELOAD_M | 0.00016 m estimated radial preload, derived as 0.41 mm - 6 mm x 15/360 = 0.16 mm after independent calibration showed that 45 deg already gripped on the refined contact model. Reset gap 0.84 mm; geometric touch at 50.4 deg; independent numeric first grip at 60 deg |
| FIRST_CONTACT_PULL_N / FIRST_CONTACT_SLIP_MM / FIRST_CONTACT_STEP_DEG / FIRST_CONTACT_PAIRS | 10 N /1 mm /15 deg /3 pairs, benchmark enrollment protocol: one fifth of 50 N, half of 2 mm, one sixth of 90 deg |
| SNUG_DEG | 90 additional degrees, obtained by phone from a technician who works with this product (Nov 2026);1.5 mm axial lead implies 0.375 mm axial advance |
| SEAL_EXPONENT | 3, estimated nonlinear seal stiffening, estimated interpolation between phone endpoints |
| RETENTION_N / PULL_COUNT / PULL_HOLD_S / PULL_SLIP_MM | 50 N /3 /1 s /2 mm; phone figure, shortened benchmark check |
| MARK_FROM_SHEATH_END_MM / MARK_TOL_MM | 50 /3 mm, benchmark depth and ruler tolerance |

The following is a **model calculation**, not a measured torque-angle or retention range:

| q from reset, deg | 60 | 90 | 120 | 135 | 148.55 | 150 | 153.44 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Estimated brake torque, Nm | 0.30 | 0.46 | 1.54 | 2.73 | 4.30 | 4.50 | 5.00 |

The estimated torque curve is 0.3 Nm below operational first grip, then T(q)=0.3+4.2*((q-60)/90)^3. Actual assembly torque also includes contact/mimic reactions. The native brake stiffness is 180 Nm/rad. Its rest target follows behind the cap and never advances it; it reacts into the mounted body. Four pad sliders, estimated 30,000 N/m compliance, 35 Ns/m damping and 0.8/0.75 friction approximate NBR contact. The 24 mm cap is an estimated sharp regular hexagon: six 13.856 mm flat slabs, 27.713 mm across corners. Exact HUMMEL corner radii are unmeasured; no ISO fastener-geometry equivalence is claimed. Native task bodies and articulations use 64 position/8 velocity solver iterations; robot/harness solver settings are unchanged. The spanner bridge is 19 mm from the jaw centre; a 13.856 mm corner radius and 3 mm seating bias leave about 2.14 mm modeled throat clearance. Real GEDORE throat dimensions are unmeasured. The maker lists the complete gland at 9 g; the primitive moving gland links total an estimated 40 g. No inertia equivalence is established. The cable has one rigid 100 mm lead and two 50 mm tail links with 0.12 Nm/rad estimated springs. The tool/sleeve is a rigid bolted assembly; finger actuator caps are 40 N each. Collision impulses are not capped at that value.

## Scoring and bypass checks

Every nonzero score requires a fresh matching state observation, correct bore path/depth, the mark within +/-3 mm, hand surfaces more than 8 mm from the gland/cable/tool and 1 s stillness. Pose differences over control steps must be <=3 mm/s and 0.03 rad/s; joint motion <=0.03 rad/s. Unknown observations, a latched overtorque violation or failed pull score zero.

| Tier | Physical check | Simulated check |
| --- | --- | --- |
| 0.25 | Released cable at its mark, cap before enrolled C | Same depth/release checks; q<60 |
| 0.5 | Cap at or past its enrolled C, before G | q>=60; snug contact-work certificate incomplete |
| 0.75 | Cap between its enrolled G (4.5 Nm) and R (5.0 Nm), and the 4.5 Nm wrench check clicks within 5 deg; hands released | Tool peak>=150 deg, >=90% modeled seal work, >=1 s continuous native spanner-cap torque and cap reaction>=4.5 Nm, no excess loosening; each credited increment requires both signed spanner-cap contact torque and native cap reaction>=80% modeled resistance |
| 1.0 | 0.75 plus three 50 N/1 s pulls, white witness residual <=2 mm, spanner within 35 mm of its rest | Same loads/count/residual limit, complete pull history and parked still tool |

Every credited work increment requires signed native spanner-cap torque and cap reaction each at least 80% of modeled resistance. The snug certificate additionally requires modeled resistance, signed native spanner-cap torque and cap reaction magnitude all at least 4.5 Nm continuously for 1 s. Jaw-centre distance is diagnostic only: it rejected real jaw-corner contact in a trace and is replaced by the stronger pair-specific load measurement. Final-source sustained-load evidence is reported below; configured brake work is never reported as a measured physical torque curve. Model work integrates new maximum angle only; repeated oscillations do not add duplicate credit. Excess loosening after the tool peak removes snug credit. Untouched-tool controls cannot earn full score. These synthetic controls establish scorer logic, not contact physics.

After release and 1 s stillness, the independent cable pull ramps 0.5 s to 50 N, holds 1 s and unloads 0.5 s, three times. Native external force loads the cable and physical seal contact takes the reaction. Residual motion >2 mm fails. The original pull-start-to-current residual must remain <=2 mm. A second freshness bound checks <=2 mm translation, <=1 deg cable rotation and <=1 deg cap change relative to the certified post-test endpoint. Rotation inside an otherwise passed pull does not invalidate that endpoint. Subsequent motion beyond these limits cancels a pass. Hand return interrupts an incomplete pull. No weld or cable pose write holds the specimen. A reset-only simulation sweep (qualification/retention-sweep/) passed all three 50 N pulls at 120, 135 and 150 deg, with 0.37, 0.44 and 0.59 mm residual; these are sampled simulation points, not a measured physical range.

## Run and recorded evidence

The changed instruction is:

> Feed the cable until its green mark meets the gland cap and hold it there, tighten the cap with the spanner into the green sector, return the spanner to its rest, and release both hands.

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 --policy yam_scripted --scorer task_score --max-steps 3200 --epochs 2 --instruction "Feed the cable until its green mark meets the gland cap and hold it there, tighten the cap with the spanner into the green sector, return the spanner to its rest, and release both hands."
```

Lots cycle A/B with zero jitter and scores averaged. The scripted controller uses fixed published waypoints and fixed grasp/tracking constants measured once in unloaded simulation; it reads no simulated state during the run. Dream-1 uses the same source/instruction with --policy dream --max-steps 2400 --epochs 2. No unseen-layout generalization is claimed.

| Evidence | A | B | Status |
| --- | --- | --- | --- |
| Historical pushed-source scripted | 0.0 | 0.0 | 2381 actions each, different runtime/instruction |
| Historical pushed-source Dream-1 | 0.0 | 0.0 | 2400 each, step limit, different runtime/instruction |
| Historical intermediate revision | 0.5 | 0.5 | 1316 actions each, different runtime and 4.3 Nm gate; no sustained load or pull pass |
| Current-source scripted | 1 | 0.5 | 2231 and 2381 actions, 3200 limit; qualification/current-runner-receipt.json |
| Current-source Dream-1 | 0 | 0 | 2400-step limit, same files and instruction; qualification/current-dream-receipt.json |

The earlier packaging flag stated that the folder was "28.1 MB, and the limit is 25 MB". Large videos stay outside the task folder; native observations are compressed. Current and historical source/results are explicitly separated.

Historical source hashes in qualification/historical/provenance.json and qualification/historical/final18/ describe superseded packages only. qualification/current-source-manifest.json gives the current source hashes. These are simulation results, not physical robot results.

## PR 4087 verdict (verbatim)

> **Not accepted** (TK-7TY83W).
> 
> Scoring (not met): The real-trial first-contact, acceptable tightening and damage thresholds have not been established, so equivalent physical scores cannot yet be assigned.
> 
> Clarity and reproducibility (not met): The cable-mark target and a numeric procedure for establishing first contact are not specified sufficiently to reproduce the stated real trial.
> 
> Feasibility (not met): Both scripted layouts score 0.0, so a full-score run and sustained transmission of the required tightening load are not demonstrated.
> 
> What to change: Calibrate the specified gland and cable: define a numeric first-contact criterion, measure the torque-angle and retention range, and establish a conservative damage threshold. Align the physical and simulated scoring bands with those measurements, and specify the sheath-mark target and its scoring tolerance. Provide a scripted or teleoperated full-score demonstration for both layouts within 3,200 steps, showing sustained tightening through the required load and the retention check.

## Point-by-point revision response

| Verdict requirement | Change or remaining evidence gap |
| --- | --- |
| Real first-contact threshold | Real-trial procedure A3: first 15-deg step where a 10 N, 1 s pull moves the cable <1 mm, on each of three pairs; phone first-grip reference and 90-deg endpoint supplied; native numeric first grip calibrated 60 deg, separate from geometric touch 50.4 deg |
| Acceptable tightening | Maker 4.5 Nm: enrolled green mark G at the 4.5 Nm wrench click (A4, 60-120 deg past C) and a 4.5 Nm wrench check at scoring (C4); simulation needs a 1 s, three-signal 4.5 Nm certificate; estimated torque-angle curve disclosed |
| Damage threshold | 5.0 Nm benchmark red line (A6: enrolled R at the 5.0 Nm click, 10 percent above 4.5 Nm), plus visible-damage rejection and a recorded onset on the fourth pair; this verdict sentence remains OPEN because no measured damage onset exists |
| Cable-mark target | 4 mm green band centred 50 mm from the sheath end (A1); scored centre within +/-3 mm of the cap outer face by ruler (C2); same rule in code |
| Sustained tightening load | Reactive native brake, physical spanner and contact-history gate; strict certificate needs modeled, contact and reaction torque all >=4.5 Nm for 1 s and <5 Nm. In layout A all three load signals stayed at or above 4.5 Nm for 3.30 s (peaks: modeled 4.574, contact 4.782, reaction 4.758 Nm, all below 5 Nm), then three 50 N, 1 s pulls passed with 0.447 mm residual. |
| Torque-angle and retention range | Model formula and numeric procedure given; phone 90-deg tightening endpoint and 50 N pull supplied; simulated 50 N retention passes at sampled 120, 135 and 150 deg (qualification/retention-sweep/); physical intermediate points and passing interval not measured |
| Equivalent physical/simulated score bands | Side-by-side table; each physical pair scored against its own enrolled C, G and R (procedure C3-C5); physical equivalence not established until enrollment is done |
| Full-score A/B within 3200 | Scripted A/B 1/0.5. In layout B the cap peaked at 148.40 deg (modeled 4.279 Nm), short of the 150 deg / 4.5 Nm certificate, so B earns the 0.5 tier; no damage occurred. This sentence remains OPEN for layout B. |
| Fresh Dream-1 and media | Dream-1 A/B 0/0 on the same frozen files and instruction; fresh scripted and Dream-1 media |


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


Geometry is authored from primitives; no manufacturer CAD is redistributed. Code: MIT. Documentation: CC BY 4.0.
