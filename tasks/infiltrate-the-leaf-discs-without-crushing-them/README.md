# Vacuum-infiltrate ten leaf discs for a photosynthesis assay without crushing them

Biology teachers and students use syringe infiltration to establish the starting state of the floating-leaf photosynthesis assay. [College Board Investigation 5, teacher page T101](https://secure-media.collegeboard.org/digitalServices/pdf/ap/bio-manual/Bio_Lab5-Photosynthesis.pdf) identifies successful disc sinking as essential before students design their experiments. This is a published teaching-lab procedure, independent of any syringe brand. [Carolina 744676](https://www.carolina.com/traditional-ap-biology-kits/leaf-disk-assay/744676.pr) supplies 32 students in pairs. For educational scale, College Board reports [over 280,000 AP Biology exam takers in 2025](https://apstudents.collegeboard.org/about-ap-scores/score-distributions/ap-biology); this is exam participation, not a measured count of students performing this assay. Category: **Other**, teaching-lab specimen preparation.

The physical skill is to keep a fingertip seal while pulling and holding a loaded syringe against its restorative force, restore pressure to infiltrate the discs, then remove the piston and transfer every disc intact. Score the freely sunken specimens after release. The task ends before illumination; photosynthesis rate is outside its scope.

## How this differs from nearby tasks

| Nearby task | Work tested here |
|---|---|
| [Dose withdrawal, PR 1095 / TK-UDCFTR](https://github.com/robocurve/benchmark-tasks/pull/1095) | A fingertip-sealed expansion and loaded hold, rather than liquid withdrawal from a vented vial. |
| [Air purge, PR 1151 / TK-YHWYSG](https://github.com/robocurve/benchmark-tasks/pull/1151) | Gas removal from tissue, pressure restoration and intact recovery, rather than expelling a free air pocket. |
| [Vacuum desiccator, PR 687 / TK-Y4ACC5](https://github.com/robocurve/benchmark-tasks/pull/687) | Hands sustain the syringe seal and plunger load; no chamber lid, stopcock or pump. |
| [Membrane filtration, PR 2911](https://github.com/robocurve/benchmark-tasks/pull/2911) | Sealed expansion followed by specimen recovery; no membrane or measured filtrate. That PR was closed for overlapping ability. |
| [Empty plunger to a mark, PR 2012](https://github.com/robocurve/benchmark-tasks/pull/2012) | All ten specimens must be recovered intact; credit depends on how many qualify. A plunger stroke alone earns nothing. That PR was closed for omitting the claimed dose work. |

Different syringe contents alone do not establish a distinct ability. The claimed distinction is a sustained fingertip seal against plunger load, pressure restoration and recovery of intact, freely settled specimens.

## Run and fixed layouts

Use the program's YAM embodiment for Isaac Lab-Arena with Inspect Robots:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --max-steps 2400 --epochs 2 --instruction "Prepare the ten leaf discs for the flotation assay and leave them with the solution in the clear cup, then put the syringe down and let go." --scorer task_score
```

Use policy `dream` for the model run. The instruction above is unchanged from the recorded runs. Each trial allows 120 s at 20 Hz; the open-loop scripted plan lasts 88.5 s. Trial k selects A/B cyclically; `TASK_LOT=A` or `TASK_LOT=B` pins a layout. Object jitter is zero. Both layouts use identical syringe, piston, bearing and holder starts. Only the rigid cup moves: A (280,170,0) mm; B (250,180,0) mm, relative to the tabletop origin. The syringe centre and piston origin are (290,0,90) mm with syringe axis along +y. The tray is at (200,-240,0) mm. The setup image shows A. The demo uses published start poses without force, state or score feedback.

## Priced real-trial kit and specimen enrollment

The apparatus uses the needleless [BD 309653 50 mL Luer-Lok syringe](https://www.bd.com/en-us/products-and-solutions/products/product-page.309653), also sold in older 60 mL listings, with its original piston and stopper. The larger original syringe exposes an 18 mm stem to the gripper. The [Carolina 2023 procedure](https://knowledge.carolina.com/discipline/life-science/biology/photosynthesis-and-floating-leaf-disks/) supplies the fingertip-seal, approximately ten-second vacuum-hold and pressure-restoration method. Its recipe is used throughout. Carolina specifies three 10 s cycles after expelling trapped air. This scene starts horizontally with approximately 30 mL trapped gas, and its demo uses one 12 s hold under the declared tissue thresholds below. These are explicit departures from the published procedure; neither the single-cycle tissue response nor the larger trapped-gas setup has been physically qualified.

Prices checked November 23, 2026; USD. Listed prices are distinguished from planning allowances. An existing counter, timer and assessor camera are assumed. Robot hardware is outside this apparatus budget.

| Item | Cost | Basis |
|---|---:|---|
| Carolina 744676 Leaf Disk Assay kit | $55.00 | Budget allowance; opened maker listing $54.30; cups, bicarbonate, straws and stock syringes for 32 students in pairs |
| BD 309653, box of 40 | $40.70 | [Retail listing](https://www.obgynsupply.com/bd-309653-60ml-syringe-luer-lok-tip-sterile.html); selected syringe replaces the kit's 10 mL syringe |
| Wood/acrylic holder materials | $20.00 | Allowance; build once to dimensioned drawing, version 2 |
| Ordinary hand tools for holder | $30.00 | Allowance |
| 6 mm hole punch | $15.00 | Allowance |
| Ready-washed bagged baby spinach, detergent and measuring supplies | $25.00 | Allowance; fresh leaves are not supplied with the kit |
| 1 mm ruler, caliper and force-check reserve | $20.00 | Allowance |
| Tray, matching clear cup and PPE | $10.00 | Allowance; check cup dimensions before use |
| Tax and delivery | $35.00 | Allowance |
| Removable pad covers, absorbent pads, wipes, rinse bottle and waste bags | $50.00 | Allowance; cover fit and real purchase price require checking |
| **Total** | **$300.70** | Apparatus and consumable estimate; no purchase recorded |

Syringe engineering estimates: 26 mm bore, 29 mm outside diameter, 135 mm barrel, 14 mm tip, 50 mm finger flange, 18 mm crossed stem and 45 mm thumb flange; 32 g dry barrel and 18 g piston. The barrel is polypropylene with an elastomer stopper. The cup model is 64 mm diameter and 58 mm overall height, with a 2 mm bottom and 56 mm internal height. Its 12 g dry mass plus 40 mL solution starts at 52 g. The barrel plus 2 mL solution starts at 34 g. The plastic tray is 130x110x4 mm. The wood/acrylic holder has an open side and the supplied drawing specifies every block. Caliper-check the purchased syringe, cup, bore travel and holder fit before a physical trial; these dimensions and masses are model estimates, not OEM measurements. The holder supports the barrel but supplies neither the fingertip seal nor the piston pull.

Use a labeled retail bag of ready-washed, smooth baby spinach from one batch, with storage and use date recorded. Punch 6 mm discs away from major veins; enroll only intact, initially floating specimens 0.2-0.5 mm thick. Keep the initial top/side photos with a ruler. Fresh specimens are consumed in every trial. The supplier kit explicitly excludes plant leaves; it is not a ready-made specimen. No suitable ready-made disc or shelf-stable replica was identified in the supplier sources checked. Weighted or already-sinking replicas would remove the infiltration endpoint. Fresh tissue and its variable response remain an unresolved reproducibility risk; no replacement specimen has been physically qualified.

Proposed physical enrollment tolerances, not validated operating ranges: use one supplier bag/lot and one solution batch for each A/B pair, with ten fresh discs per trial. Check the punch at 6.0 +/-0.2 mm; check 0.2-0.5 mm thickness on sacrificial adjacent tissue without compressing enrolled discs. Reject visible veins through a disc, ragged edges, holes, tears or any initially sinking disc; do not replace a bad specimen after the trial starts. Set 2.0 +/-0.1 mL in the syringe, 40 +/-1 mL in the cup and approximately 30 +/-1 mL trapped gas. Use a ruler/template to restore the stated rigid-body poses within 2 mm and 1 degree. The simulator keeps its exact fixed poses and dimensions; none of these proposed physical tolerances changes its scoring. Before each trial, verify all ten intact discs float in the initial solution, the syringe passes snapback, covers are intact and dry underneath, and the batch's control record is present. Reject the setup before starting if any check fails. Record measurements and rejection/preparation time; no such physical enrollment has yet been demonstrated.

Before scoring a new tissue batch, use sacrificial matched discs in the same syringe, solution and lighting. Record an open-tip control over the 120 s trial interval and a sealed hold with pressure restoration at the stated stroke and duration. The open-tip control must remain afloat; all ten sealed-control discs must sink intact and remain freely settled through the 15 s reading interval. Check the seal by snapback and check draw/hold force with a force gauge before robot use. Reject a batch or apparatus that fails these checks. This is a physical qualification protocol, not a report of completed physical tests. No-op soaking, forced submersion, weighted replicas and already-sinking specimens cannot establish infiltration.

## Preparation and reset estimates

One-person planning allowance: **3-6 min per trial**, including punching, inspection, loading, photos and cleanup. Allow a further **6-10 min per new leaf/solution batch** for preparing solution, sacrificial specimens and the two qualification controls above: **9-16 min for the first trial of a batch**, after one-time holder assembly. These estimates have not been timed or demonstrated. First-use dimensional, force, covered-pad and photo qualification adds untimed work, as does any failed batch or apparatus check. Record all preparation, rejection and qualification time; this is not the preparation time for the complete classroom lesson.

1. Punch and inspect ten fresh discs from the enrolled bag. Replace all used tissue; never reuse previously infiltrated specimens.
2. Use one labeled solution batch: 1 drop concentrated dish detergent and 1 g sodium bicarbonate per US gallon water. Load ten discs and 2 mL solution ahead of the piston, leaving approximately 30 mL trapped gas at the stated starting pose.
3. Restore barrel, piston and cup to A or B; put 40 mL of the same solution in the cup and empty the tray. Keep direct assay illumination off.
4. Verify syringe snapback, the ten initially floating intact discs, liquid volumes and starting poses. Record the initial photos and reject leaks or damaged specimens before enrollment.
5. Discard used discs into the waste bag, rinse and dry the syringe/cup/tray, replace wet or torn protective covers, and inspect the shared rig before the next trial.

Keep a reset kit beside the rig: the enrolled labeled leaf bag and solution bottle, 6 mm punch, spare BD syringe/piston, 2 mL measure, 40 mL measure, ruler, timer, rinse bottle, absorbent pads, disposable covers, dry wipes and lidded waste bag. There is no growing, casting or curing per trial. Simulator reset starts a fresh process to restore specimen states, liquid inventory, poses and persistent failure flags.

For physical trials, cover the shared rig's gripper pads with removable powder-free nitrile finger cots and protect exposed surfaces below the wet workspace with absorbent spill pads backed by a waterproof sheet, clear of arm travel. [Honeywell North nitrile cots](https://automation.honeywell.com/gb/en/products/personal-protective-equipment/hand-protection/finger-cots/north-nitrile-finger-cots) are commercially available at 3 mil thickness (about 0.076 mm); select fit on the actual fingers. Covers must leave finger travel free, without a loose flap, adhesive on the gripping face, or an outlet cap that replaces fingertip contact. Before robot trials, verify covered-finger grip, outlet seal, slip and draw/hold force with the same syringe and solution; inspect for leaks and pad wetting. Stop a physical trial if a cover tears or liquid reaches the rig, isolate the spill, and follow the rig maker's cleanup procedure before reuse. No hot, sharp or biological-hazard reagent is required. This protection route is proposed for physical trials, not simulated or physically validated; the recorded forces and success do not qualify covered pads.

## Force and declared tissue model


The score counts intact sinking discs; liquid amount earns no credit. A native hand must hold the actual piston against a vacuum force while the other hand's finger is at the hollow outlet. Bore area is531mm2. At seal closure the model records gas volume, so capping after an open-tip draw creates no vacuum. Boyle pressure difference times piston area gives a signed restorative load, capped32N and smoothed with0.12s time constant. At35mm expansion from30mL gas the load is about20.6N, before at most1N seal drag. Equal/opposite external force and moment are applied to barrel/piston with set_external_force_and_torque at each20Hz control step and held through physics substeps.

Seal drag depends on actual displacement-derived piston velocity, smoothed and bounded by half the18g piston momentum per control interval so it cannot launch the light piston. This is an explicit numerical passivity limit, not a measured seal-friction curve. An open outlet vents the vacuum immediately. Cup and barrel native masses change only when the declared liquid inventory changes.

A native prismatic joint approximates the barrel's bearing while the stopper is inside. It connects two loose bodies, never the world or either hand. It is disabled after actual78mm axial withdrawal, so the piston really separates and can be left on the tray. It is reenabled before a new reset settles. This bearing approximation removes radial play/rotation while engaged; stopper friction and pressure are still external loads rather than a motor. The holder bridge and its side post are25mm behind the barrel centre, exposing a central purchase at local y=+5mm. The open-side holder limits barrel motion during the draw; the barrel must be slid40mm sideways and lifted free for pouring.

Ten independent declared tissue states stand in for infiltration. Each receives exposure only while still inside the loaded syringe, the finger cap is present, the stopper is aligned and inside the bore, actual withdrawal is30-55mm, sealed expansion is at least30mm, and pressure reduction is at least25kPa. Continuous exposure thresholds are8.2,8.4,...10.0s. Pressure restoration, either venting or returning the piston, converts qualified exposure into infiltration. Interrupted short holds do not add; cycling an empty syringe never changes discs in the cup. These thresholds are stated model parameters, not tissue calibration or measured biological capability. One12s loaded hold precedes pressure restoration; its response is a declared model parameter.

The actual rear opening must be downhill. Its gravity ray must intersect the actual rotated cup rim within23mm of the rim centre, with0-72mm vertical fall to that plane; all stopper and stem geometry must be more than16mm from the mouth centre, keeping the26mm rear aperture clear. A previously removed but reinserted piston blocks drainage. Only then can declared draining transfer the discs. Infiltrated discs render at3mm above the floor, others at12.5mm, spread18mm from cup centre in its local floor plane. Cup rotation is applied to every disc position. This is a declared visual state model: fluid dynamics, wall adhesion, leaf deformation, permeability, photosynthesis and viability are not simulated. It never credits a piston stroke alone or a forced floating disc near the bottom.

## Whole-trial failure and photo scoring

Score0 permanently for a missing disc, new tear>=1mm, hole/crushing, represented apparatus failure or spill>2mL, whether deliberate or accidental. All ten initial discs remain the denominator. The simulated liquid ledger conserves 42 mL between syringe, cup and spill. Open-mouth drainage and the tilted cylinder's free-surface capacity determine spill; this is a declared inventory rule, not CFD. A declared3mm-radius vertical drop path must also clear the barrel, removed piston, holder, tray and sampled hand surfaces before the rim. Obstruction pauses transfer; it does not teleport discs through a tool. The same local hand-intrusion damage proxy applies to loaded tissue and cup tissue. Native mass includes the carried solution. Native mass readback preserves 34 g/52 g initial barrel/cup masses and 32 g/54 g after complete transfer. Gross apparatus failures represented are a body falling below the table, leaving the60cm tabletop bound or nonfinite inputs; rigid fracture is not simulated.

The1N crushing limit is an uncalibrated declared parameter. In-barrel damage uses direct axial hand contact when the piston face intrudes, plus the local1000N/m piston-to-disc compression proxy. The latter includes loads transmitted through the barrel or a supported piston; a remote/unrelated grip does not count. Exposed tissue uses a declared1000N/m compliant contact proxy at the disc's3mm radius, with damage above1N. Force is sampled at20Hz, so substep impact peaks are not qualified. Actuator effort, fingertip load, vacuum load and tissue proxy are distinct quantities. Any unknown/nonfinite consumed force, geometry or timing permanently fails closed. A native finger load>40N also invalidates a trial. Physical damage is assessed by post-trial photos, and the force threshold needs a non-scored force/photo check before physical use.

After the pour, leave the cup upright within5degrees and undisturbed15s. A cup that overturns to90degrees permanently fails. Read a top and side photo with a1mm ruler. A passing disc must be intact, freely settled at most3mm above the floor, at least3mm from the wall, unstacked and untrapped. Ambiguous/occluded tissue is not passed. All ten must be accounted for in the cup.0 for none;0.25 for1-4;0.5 for5-7;0.75 for8-9;1.0 for ten. Every positive tier requires the apparatus down, released and all hand/fingertip surfaces30mm clear. Full score also requires1s still for the syringe,piston and cup. Still means position within1mm and orientation within1degree of a fixed pose throughout the interval, checked from video/ruler; the15s cup interval includes that hold. The cup interval depends on cup motion, not instantaneous contact-solver velocity noise in the set-down piston. Average A/B. The simulator separately enforces the explicit declared state/force model above.

## Bypasses and controls

| Shortcut | Result |
|---|---|
| Open-tip draw; cap only after drawing | No pressure-reduction exposure;0 |
| Short stroke; interrupted short holds | No sufficient continuous exposure;0 |
| Pump empty syringe after pouring untreated tissue | Cup tissue cannot change;0 |
| Force a floating disc to bottom; omit/crush/discard tissue | Initial denominator retained;0 |
| Hold the finished apparatus, hover it, or skip15s wait | Release/clearance/wait gate;0 |
| Restore a damaged/spilled trial to its starting pose | Failure remains latched;0 |
| NaN, stale state certificate or missing hand/contact data | Fail closed;0 |

The endpoint and causal controls check every count tier, threshold edges, unknown/held/airborne/damaged states and late-cap/empty-chamber/interrupted-hold histories. Endpoint fixtures alone are not native validation. Direct grips, supported grasps, regrasping and one-hand attempts remain legal; either hand can seal when the matching fingertip actually contacts the tip; the provided demo uses the other hand for the cap. No compelled relay or handover is added.

## Assets and physical scope

Primitive geometry and the holder drawing are supplied under MIT (code) and CC BY 4.0 (documentation). No third-party meshes are used. The bearing and pose helpers operate within the task process. Fixed layouts permit memorized replay. Physical qualification must establish dimensions, seal force, tissue response and photo repeatability; no measured specimen, human baseline, biological viability or physical robot success is reported.

## Current validation record


Frozen task.py SHA256:216401f6e29bdeb7d42f4d07a7a803d9d48877067ac5499bd518f7ffbf63523e. The matching-host normal scripted demo scores A1.0/B1.0,mean1.0,ending successfully at82.65s/83.65s. Both retain ten intact declared infiltrated states and all42mL in the upright cup,put the apparatus down,release it,clear their hands and complete the15s cup wait. Neither latches damage,spill or another whole-trial failure. Peak sampled finger loads are39.68N/29.49N; peak modeled restorative loads are23.78N/22.90N. These are simulation measurements,not calibrated hardware forces or measured biological outcomes.

The matched Dream-1 run scores A0.0/B0.0,mean0.0. Both trials reach2400 control steps (120s) and terminate at max_steps. No disc qualifies for infiltration or transfers to the cup in either lot; both end with2mL in the syringe and40mL in the cup. The preparation therefore remains unfinished. The log contains65expired-response warnings,which must be considered when interpreting policy performance. There is no evaluation error,pool-full response or explicit task-content refusal. This is a scored model-policy simulation result,not a physical robot result.

The runtime,helpers,fixtures and instruction are identical across the paired runs.

The offline checker covers both layouts, all four arm reach/collision plans, 20 endpoint cases and causal controls. The physical dimensions,seal behavior,tissue response and photo repeatability remain uncalibrated. No physical specimen,human baseline or hardware success is claimed.

Verified simulation stack: isaaclab-arena 0.3.0, inspect-robots 1.0.0, inspect-robots-agent 0.30.0, NumPy 2.3.1 and Torch 2.11.0+cu128. The source hash above identifies the paired demo and Dream-1 results. Historical results from different source hashes are excluded from this current record.
