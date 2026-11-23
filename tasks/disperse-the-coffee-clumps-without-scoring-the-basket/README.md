# Disperse the coffee clumps without scoring the basket

Break up the coffee clumps, leaving an even dose in the basket and the tool back in its stand. Keep the basket and needles undamaged.

Category: **Societal relatability**. This is a specialist espresso preparation task. It tests dry tine-mediated deagglomeration while retaining the dose. It does not claim that most households use WDT, that a large occupation performs this exact step, or that a level surface proves internal density, extraction quality, taste or channeling. Familiarity outside espresso users remains an importance risk.

The final native reference on this runtime scored A1.0/B1.0. The completed matching Dream-1 scored A0.0/B0.0 at120s per lot; earlier credential-failed runs are preserved separately. These are native simulation results for the declared granular surrogate; no real-material or hardware qualification is claimed.

The support/drop repair passes its native reference and offline regressions; physical qualification remains unperformed.

## Named parts and complete affordable trial route

Prices checked November21,2026; taxes/shipping excluded. Robot/cameras and an existing counter are the shared evaluation rig, not purchased for this task. No purchases have been made.

| Item | Dimensions/procedure | Price and source |
|---|---|---|
| Normcore WDT v3, complete original stand | Maker 103 mm overall,31 mm body,50mm304SS wires. Nine supplied0.25 mm needles installed by operator before reset; no loose-wire manipulation. Body65 g estimate, stand100 g estimate. | $36.99 [maker](https://www.normcorewares.com/products/normcore-wdt-distribution-tool-v3) |
| Breville SP0027237 original portafilter | Maker 54 mm; native handle28 mm diameter estimated. Empty assembly430 g plus20 g dose gives450 g estimated moving mass; steel/polymer construction estimate. | $39.95 [maker](https://www.breville.com/en-us/product/sp0027237) |
| Breville SP0001520 single-wall double basket | Maker 54 mm nominal. Modeled interior50 mm, floor17 mm above counter, rim41 mm,1.5 mm floor, estimated from product photographs. Verify with ruler before using this model on hardware. | $13.95 [maker](https://www.breville.com/en-us/product/sp0001520) |
| Hario Mini-Slim+ MSS-1DTB hand grinder, operator reset only | Maker 150x72x220 mm,24 g capacity. Can produce fresh fine grounds; no claim that this grinder always makes six qualifying natural clumps. Reject reset batches that fail the protocol below. | $40 [maker](https://www.hario-usa.com/products/ceramic-coffee-mill-mini-plus) |
| Full-size passive machine/grinder enclosures | Dimensioned enclosure drawings in `fixture/`. Retain complete outer bodies, tray, head, hopper and wand clearance. Thin plywood/printed shell construction; about 2 kg material plus fittings, $50 estimated. No brewing, powered grinder, or custom basket holder. | $50 estimated materials allowance; [BES870 shape reference](https://www.breville.com/en-us/product/bes870), [BCG600 reference](https://www.breville.com/en-us/product/bcg600) |
| AWS PowerBank100 g 0.01 g scale with calibration weight; Carson TV15 loupe; ruler/straightedge, glass tray and coffee | [$20 scale,100 g x0.01 g with100 g calibration weight](https://awscales.com/powerbank100g-rechargeable-digital-pocket-scale/), [Carson TV15 5x/10x/15x loupe$14.42](https://www.fishersci.com/shop/products/carson-optical-inc-triview-folding-loupe-magnifier/S01928), plus$25.58 allowance for commodity ruler, glass inspection tray, straightedge and coffee. Use the shared phone camera through the loupe for macro images. | $60 estimated allowance; actual chosen models and checkout total must be recorded before a real trial |

**Total: $240.89 estimated**, including all task parts, passive surroundings, reset grinder and inspection consumables. The $699.95 original BES870 and discontinued BCG600 need not be bought. This cost route deliberately substitutes passive full-size enclosures for those unpowered background appliances; the scored basket, native tool and supplied stand remain original products. Enclosure dimensions are construction estimates, not maker CAD. Their purpose is to preserve occupied counter space; no robot action inside an appliance is claimed. No miniature machine, isolated group head, added vise, grip adapter or custom WDT holder is used.

Maker authenticity: Normcore's [V2.1 product description](https://www.normcorewares.com/products/0-23mm-wdt-distribution-tool-v2-1-with-stand) says “break up clumps, loosen compacted grounds”. This establishes the procedure, not household prevalence or calibrated force. The v3 page specifies the actual submitted tool geometry. All meshes and code here are original procedural geometry; no maker images/CAD are redistributed. Code MIT, documentation CC BY4.0; vendor names identify products only.

## Geometry and layouts

World x is away from the arm bases, y to the robot's left, z above the counter. Table top is0.7475m in the station world frame. Zero object jitter is required. Units below are mm above the counter.

| Object | A | B |
|---|---|---|
| Tool body origin | (210,160,84.5), upright | same |
| Original stand base origin | (210,160,0), upright | same |
| Portafilter base origin | (270,-15,0), yaw0 deg | (290,25,0), yaw30 deg |
| Passive espresso enclosure | (660,100,0) | same |
| Passive grinder enclosure | (650,-235,0) | same |

The tool body spans53 mm, with straight rigid-cylinder approximations to tines extending from26.5 to76.5 mm below its center. One central wire and eight wires on an estimated10 mm-radius pattern are used; wire splay and supplied stand geometry are estimates. Stand base radius25 mm and 5 mm thick; native side stem rises69 mm and supports an open C cradle at 69.5 mm (4 mm thick). Two22x5 mm cradle prongs flank the21 mm tool neck, supporting its31 mm shoulder at 71.5 mm. These unlisted stand dimensions are estimates from the maker close-up, not a custom holder. The body main cylinder is39.5 mm long above its13.5 mm reduced neck; the full body remains53 mm. The portafilter has a physical circular floor/wall, two native spouts on the counter, metal neck, and 112 mm-long native polymer handle. Native spout/handle support and legal one-hand solutions are allowed. No forced handover or two-arm requirement.

All moving parts are rigid bodies; no articulated part changes pose between layouts. B changes only the rigid basket. Initial settled poses must be verified from each native trace. The scripted plan uses only the published visible geometry, with no force, material, joint or exact-pose feedback. A fixed correction accounts for the measured5.44 deg native-support lean and about 1.5 mm settling shift: basket-local(0,-4.2,0)mm plus global(1,-1,0)mm to the working TCP. The measured free basket also creeps on its native spouts/handle support. The reference grasp is 15 mm above the body origin, placing the estimated body centre of mass below the pad contact. Working TCP is raised by the same 10 mm so the needle penetration does not change. The crossed raster is clipped to a 12.5 mm circle to preserve wall clearance while covering the stroke endpoints. Fixed feedforward velocities are (−0.00000083,−0.000056,0) m/s for the straight-handle layout and (+0.000049,−0.000078,0) m/s for the visible 30-degree handle layout, multiplied by elapsed plan time during loaded sweeps only. These are published open-loop calibrations from diagnostic traces, not online exact-pose tracking.

## Reset and real material acceptance gate

The reusable enclosures and hardware are assembled once; only20g dry coffee is consumed per trial, with no per-trial printing, casting or drying. Per-trial specimen reset has a four-minute acceptance limit, including grinding; this is a target protocol, not a measured timing result. Use a bean/grind batch already accepted by the batch controls below. Start the continuous timer before emptying and dry-brushing the basket/grinder. Budget up to120s for cleanup plus grinding20g,30s to weigh/loosely spread the base,45s to select/place six natural clumps, and45s for placement, ruler photo and logging. Include rejected sourcing/placement attempts in this same elapsed total; do not restart the timer to hide them. Reject a reset that exceeds240s or cannot supply six natural clumps within45s; do not spend hours constructing, drying, gluing or shaping specimens. Refill from fresh grounds, clean/dry the reusable parts and log failures.

1. Record bean lot, roast age, grinder model/setting,20.00 g dose (within 0.02 g), room humidity, elapsed time after grinding and operator. Clean/dry basket and needles; photograph both before reset under the same light and against the same ruler template.
2. Grind without added water, pressing or hand-shaping pellets. Select six naturally occurring3–5 mm cohesive aggregates using a wide spoon. Do not squeeze, glue, wet or heat them. Arrange six sectors centered16 mm from basket center,60 deg apart. Match visible size to an overhead macro ruler image; record the exact arrangement. Remaining grounds form a loose base with 1.5–1.8 mm relief before the six natural clumps are placed; do not tamp. The modeled base spans 1.72 mm and the initial aggregates have about 4.72 mm maximum span including proxy-grain diameter. A reset without six natural clumps is invalid, not a trial success.
3. Batch enrollment is performed once per bean lot/grind setting/humidity session before any trials, not repeated for each specimen. Accept a bean/grind batch only after five untouched10s controls retain all six clumps, followed by the gentle inspection procedure below retaining all 30 known original aggregates across the five controls, with no change across the 3 mm size threshold. A shallow above-bed tool-motion control must also leave six. Reject a batch if it collapses, settles flat or cannot be inspected reproducibly. Adjusting grinder settings requires recording a new reset batch and repeating the controls.
4. Place the complete WDT upright in its original stand; set basket/handle and passive surroundings to A or B. Photograph the setup. No helper acts during the trial. Cold, unpowered, dry scene; no heating, pressure, liquid, tamping or replacement-part handling.

**These are operational acceptance tests, not completed hardware measurements. Natural-coffee cohesion and witness repeatability remain unverified.** A simulation reset has exact finite20 g inventory and six bonded aggregates, and cannot settle away without tine contact; that is not evidence that a chosen real grinder batch passes the gate.

## Scoring for a real trial

Score0 permanently for a dropped basket, basket tilt over 15 deg, lost dose below 19.00 g at any time, punctured/bent needle, new basket gouge, any gripper/fingertip inside the50mm basket opening between floor and rim, including above the grounds, or tool-body crushing of the dose. An intact aggregate over 3 mm leaving the basket is also whole-trial zero, even if retained mass stays above 19 g. For a macro witness its center crossing the 25 mm opening radius, falling below the basket floor, or rising more than 3 mm above the rim counts as leaving. Merely returning an object or clump to start never clears a violation.

Every nonzero tier requires tool and basket released, every gripper/fingertip at least5 mm clear of basket, tool and stand, and one continuous second with every scored body moving at most 1 mm and turning at most 1 degree during that second. Read this from the side/overhead video. Evaluate the highest satisfied tier at the end:

| Score | Visible/physical result and measuring tool |
|---|---|
| 0.25 | Fewer than six surviving cohesive clumps have maximum span over 3 mm. Count all the dose, including buried aggregates, using macro ruler images and the gentle whole-dose inspection. |
| 0.50 | At most two clumps over 3 mm, with whole-bed peak-to-valley relief at most4 mm. |
| 0.75 | No clump over 3 mm, with whole-bed relief at most2 mm. |
| 1.00 | The0.75 result and the original tool upright in its supplied stand: body center in the native C cradle seat (−1 to +5 mm along its open direction, within 1.5 mm across it, and within 2 mm vertically of its supported shoulder height), lean at most3 deg. Macro ruler photos and side video. |
| 0 | Every other outcome, including untouched reset, unknown hand/pose data, or failed release/stillness. |

Inspect relief **before** disturbing the bed: film a straightedge/depth ruler traversing the entire50 mm opening in1 mm lateral increments. Scan continuously, including perimeter and all local peaks/valleys; use macro video to read the largest and smallest heights. Four isolated readings are insufficient. Maximum span, rather than average height, sets the tier.

Then gently spoon the complete dose into indexed thin layers on a glass tray without pressing or shaking; inspect every layer with10x macro ruler photos. An aggregate that remains a connected mass over 3 mm counts, including any that was buried in the basket. The unchanged-reset controls above test whether this destructive witness breaks the clumps itself. Also require ten matched control doses after recorded needle treatment, with known target aggregates in the 2.7–2.9 mm and 3.1–3.5 mm ranges. Photograph each target in an exposed shallow layer before gently covering it with loose grounds, then apply the complete inspection once. These controls include the complete relief measurement followed by whole-dose inspection. Every target must preserve its count and side of the 3 mm threshold; any breakup or threshold crossing invalidates the batch/procedure. Untouched controls alone do not establish preservation of fragile treated aggregates. These treated controls have not been performed. Any inspection-induced lost clump or size-threshold crossing invalidates the batch; it cannot be used to award a favorable score. The robot does not get inspection assistance or see an internal-density signal during the trial.

Retained mass: tare the removable dry basket alone (verify basket plus20 g is below 100 g), and use the0.01 g scale before and after; video records any intervening spill. Inspect needles against a backlit original-centerline template after release and again after10s unloaded: permanent centerline departure at least1 mm, a kink, fracture or missing tip is damage. Photograph the cleaned basket with10x macro and raking light from four directions against its pre-trial reference: a new continuous scratch/groove at least1 mm long or any puncture is a gouge. A washable coffee streak alone is not damage. Hardware observers must agree using these saved images; the declared simulation damage approximation below is not a calibrated metal wear model.

## Declared granular rule and physical work

The bed is finite20 g:1 mm conservative columns at estimated530 kg/m3 bulk density plus six finite aggregates made from0.8 mm proxy grains. Each aggregate starts roughly4 mm wide. Neighbor bonds are explicit; connected components with actual longest grain-center separation plus0.8 mm over 3 mm remain clumps even below the surface. Empty corners of a bounding box do not inflate the measured clump span. Neither a smooth visible top nor completion of prescribed waypoints clears a bond.

Only the actual swept0.25 mm needle shafts intersecting a bond's contact neighborhood can sever it. A bond requires at least0.8 mm accumulated local shear and 50 nJ accumulated work. Motion above the bed, stationary proximity, wrong tools and hand entry do not qualify. Work is credited only against the prior applied opposing reaction and the next observed relative shaft displacement; requesting a force after a sweep cannot retroactively earn bond credit. Native forces persist between20 Hz updates. Loaded shaft motion of at least 0.01 mm mobilizes contacted columns. A contacted 1 mm column exchanges 18% of each positive height difference over 0.05 mm with its twelve nearest columns (at most 2.83 mm away at the perimeter), capped at 10% of its current volume per pair per 50 ms step. Each exchange conserves mass. These short-range granular-yield parameters are stated estimates. Stationary contact or an unapplied reaction cannot redistribute the bed; there is no timer-based global smoothing. Surviving cohesive proxies remain in the basket frame until loaded rupture; free translation of intact clumps is not calibrated. A separate escape/history guard rejects discarding intact aggregates; disappearance or burial cannot replace bond rupture. The dose is finite and loss is recorded separately; no new coffee appears. A fast immersed sweep above 0.12m/s loses finite mass; above 0.4m/s is an invalid violent sweep.

Estimated per-needle drag is4mN at 20 mm submerged depth, ramped with depth and speed, opposing the measured shaft motion. Equal/opposite forces and moment arms act on the free tool and portafilter, through native external wrenches; no pose teleport or attachment weld holds either. Maximum full-depth resultant is36mN. This is a declared small-force model, not the research card's unmeasured1–5 N guess. Fine304SS wire is far more compliant than a blunt kitchen utensil. Using E193 GPa,d0.25 mm,L50 mm gives3EI/L^3 about 0.89 N/m; a4mN end load predicts about 4.5 mm deflection. [304 wire maker datasheet](https://www.fagersta.marcegaglia.com/wp-content/uploads/2023/07/304-R350.19-ENG-2025-Edition-1.pdf) lists190–200 GPa modulus and at least180 MPa proof strength. The actual wire's heat treatment, supported length, coffee drag and bond energy have not been measured. Force/geometry correspondence remains a fidelity limitation.

Rigid tines and the physical basket retain contact colliders with0.1 mm contact offsets. The candidate checks the entire swept shaft against floor/wall slabs with a0.25 mm penetration margin. An airborne basket without original-geometry support needs hand geometry within0.5mm and inferred upward COM reaction at least80% of its weight; otherwise a historical drop violation latches, including an upright release and landing. Finite loose-ground transport now follows measured basket linear/angular acceleration with the declared slip rule below; elastic/plastic needle bending is not solved, and the solid-contact guard is an uncalibrated conservative damage proxy; tool-body entry into the dose is independently invalid; this rule is not a substitute for real unloaded bend/raking-light inspection. No heating, liquid, brew, flavor, channeling or hidden density is simulated. Native traces record applied load, accumulated work, bond count, clumps, mass, coverage, damage, released hold, poses and sampled finger effort. Sampling at 20 Hz is not a hardware force calibration.

## Bypasses and controls

| Attempt | Result |
|---|---|
| Untouched reset / tool returned without stirring | Six clumps remain;0 |
| Circles above bed | No loaded contact, no severed bonds or smoothing;0 |
| Cover/bury intact clumps | Connectivity still counts;0 |
| Tamp or press with gripper/body | Whole-trial invalidation;0 |
| Tip or empty basket | Whole-trial invalidation/mass failure;0 |
| Return after damage/spill or discarded intact clump | Historical violation persists;0 |
| Hold a smooth result still in fingers | Release fails;0 |
| Gripper flat face within 0.5 mm of tool | Face-sampled hand surface fails release;0 |
| Safe supported one-hand WDT | Allowed; skill is retained-dose deagglomeration, not arbitrary bimanuality |

`near_misses.py` tests contact controls, buried-clump counting, all tier boundaries, conservation and unknown/face-contact release. Native trace/demonstration outcomes remain distinct from these offline controls.

## Prior objections and closest tasks

The screening objections concern natural-clump reproducibility, hidden aggregates, full-field relief, damage and importance. The rules below address them; native surrogate results do not establish hardware calibration or organizer acceptance.

- Full demonstration: the final native A/B reference reached1.0 in both layouts with the declared reactions applied; the measured results appear below.
- Natural-clump reset: the five-control acceptance protocol rejects unstable or manufactured pellets; correspondence remains unperformed.
- Hidden clumps: connected components are retained independently of surface height; real witness examines the complete dose and must pass its no-op preservation control.
- Four-point relief: full-field sweep replaces sparse readings.
- Damage: unloaded template and before/after macro protocol define the hardware witness; uncalibrated simulator damage is declared.
- Material contact: bond rupture requires actual shaft sweep, local work and opposing reactions; no force-free completed-path flag.
- Societal importance: specialty risk remains stated; neither appliance context nor a source is treated as proof of broad familiarity.

How this differs: TK-TLLPCQ (accepted665) axially compresses then bayonet-locks a portafilter; this task neither tamps nor installs it. TK-KDXU22 (accepted1482) impact-ejects a spent cohesive puck; this task retains and disperses a fresh dry dose. TK-A2NMP5 (accepted2319) inserts a reusable filter and folds its bail. Accepted1668 meters/transfers formula scoops rather than breaking retained cohesive aggregates. Thickener2444 and polymer1712 illustrate adjacent deagglomeration and reproducibility/importance objections; dry coffee alone is not claimed to guarantee novelty. Current rival checks are required before handoff.

## Reproduction

Use stock Isaac Lab-Arena with the program's YAM/Inspect Robots embodiment. The task-local primitive kit and layout helper require no simulator source edits. Two epochs, zero jitter,20 Hz and 2400 steps are required:

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --epochs 2 --max-steps 2400 --scorer task_score --instruction "Break up the coffee clumps, leaving an even dose in the basket and the tool back in its stand. Keep the basket and needles undamaged."
```

Use the identical instruction/epochs/limit with `--policy dream` for the single Dream-1 evaluation. No rewording around a refusal. Final source hashes and run provenance belong to the build handoff.

## Measured build status

The final normal native reference on this runtime scored A1.0/B1.0, mean1.0, at114.00/114.00s within120s per layout. Both dispersed all six clumps, parked the tool and completed the released one-second pose hold, with no history, hand-entry or damage flags. Relief was1.852/1.870mm and retained dose19.984/19.964g. Peak applied needle reactions were27.691/28.044mN; material work39.831/41.391mJ. The reference motion is unchanged from the preceding runtime.

The support/drop and forbidden-hand-entry witnesses each recorded27360/27360 native ticks in A/B, exactly12 per20Hz action at240Hz, covering every episode tick. Minimum inferred upward reactions were3.885/3.885N, above the3.532N gate; support came from actual counter-surface contact. AABBs alone never supply support. Each layout's final-second pose window stayed within1mm/1deg and hands remained5mm clear.

The completed Dream-1 run `dream-20261122T102057` used the identical frozen runtime and instruction, two epochs and2400 steps per lot at20Hz. Both lots reached120s and scored0.0; native logs contain no errored trials and no abstentions. The log contains98 service expiries and0 pool_full messages, which limit conclusions about clean policy difficulty. Dream did not reach a nonzero scored outcome; the available evaluation logs establish its scores, without a separate material-history observer for this invocation. The earlier run scored A0.0 with B unscored after a credential fault, and two subsequent attempts also failed before scoring; those remain separate from this completed result.

Packaging recovered this completed run from the host after access returned. A redundant invocation was started before the complete run list was checked and was promptly interrupted; it is not used as evaluation evidence. The frozen runtime and the unchanged instruction were retained throughout; host task.py matches local.

All94 offline controls passed, preserving all90 earlier cases including the75 original ones, rejection of the10.725mm spout/base gap, genuine stand contact, a brief hop wholly between20Hz endpoints, and the physics dispatcher/reset boundary. A supported-basket between-control hand-entry sequence also latched zero. These are offline state sequences through the same history handler, not demonstrated native robot exploits.

An unaltered early top-camera frame at2s shows the prepared dose before tool contact; the native first frame is retained. The ordinary20Hz video alone cannot show unsupported intervals shorter than a frame; the native physics logs provide that simulation evidence. A physical drop/contact witness, complete reset timing, natural-clump availability, treated-clump inspection and material/damage correspondence remain unqualified. No hardware trial is claimed.

## External benchmark comparison

The 1,015-name BEHAVIOR-1K list includes make_coffee, making_coffee and preparing_existing_coffee. The official [make_coffee definition](https://behavior.stanford.edu/knowledgebase/tasks/make_coffee-0.html) produces drip coffee in a mug; [preparing_existing_coffee](https://behavior.stanford.edu/knowledgebase/tasks/preparing_existing_coffee-0.html) fills a mug and adds milk/sugar. Neither listed definition requires a cohesive retained bed, selective needle-bond rupture or preserving an unscored beverage-free dry dose. This is a mechanism/outcome distinction, not a claim that narrower coffee scope alone establishes novelty. Broad coffee-activity overlap and specialist importance remain review risks.

Side-cradle return uses its open side: lower35 mm ahead of the cradle while the neck is6 mm above its supported seat, slide the neck between the original prongs, and lower the shoulder. It avoids threading nine fine tines vertically through the cradle. Whole-dose finishing uses seven-row rasters at all six published reset sectors:6x6mm in A and7.75x6.4mm in B; these are fixed camera-visible reset locations, with no hidden-material feedback. The preceding native pair measured this unchanged motion at1.0 in both layouts, and the repaired source has passed its own matched A/B reference.

## Transport and damage guards

The450g rigid moving-mass estimate includes20g coffee (430g empty portafilter/basket estimate); inertia stays fixed over the <=1g dose loss allowed in a valid trial. Loose columns respond to actual basket linear/angular acceleration and gravity. A Coulomb coefficient0.6 (about31deg repose) is an unmeasured batch estimate, not a claim of coffee calibration. Slip velocity integrates tangential effective acceleration with Coulomb deceleration; conservative upwind1mm-cell transfers use at most0.5 cell per substep. Only outward rim columns above41mm lose mass, recorded permanently. Over50mm unresolved displacement per50ms fails closed. Intact clump graphs remain basket-bound until actual loaded needle rupture; shaking never breaks bonds or earns a pathway flag. Internal momentum reaction beyond the fixed assembly inertia is omitted. [Ground-coffee measurements](https://doi.org/10.1590/1807-1929/agriambi.v20n6p581-587) show that repose depends on grind, roast and storage; they do not calibrate this batch or the chosen coefficient.

The entire observed shaft, swept in0.4mm relative-motion increments, is checked against the basket, native stand, counter and actual convex gripper solids. Penetration beyond0.25mm latches the declared damage proxy; this catches shaft/rim scraping, needle/prong catching and needle pinching that distal-tip-only checks missed. A supported hand-held basket remains legal; release without geometric support from the counter or original scene parts (0.2mm exact-surface query tolerance) requires hand geometry within0.5mm and inferred upward COM reaction at least80% of weight; failing either latches a historical drop, even after upright landing. These approximations do not demonstrate permanent wire deformation or wear correspondence; the real unloaded/template/raking-light witness remains required.

Finishing TCPs include a fixed -4.5mm world-x correction inferred from the observed held-tool lean and shaft registration; the seven-row sector rasters (A6x6mm,B7.75x6.4mm) still depend only on published reset sectors and visible basket yaw. No material-height or bond-state feedback enters the scripted plan.

Additional BEHAVIOR comparisons: [making_coffee](https://behavior.stanford.edu/knowledgebase/tasks/making_coffee-0.html) and [brewing_coffee](https://behavior.stanford.edu/knowledgebase/tasks/brewing_coffee-0.html) have brewed-drink goals; [iced espresso](https://behavior.stanford.edu/knowledgebase/tasks/make_an_iced_espresso-0.html) combines existing espresso with milk/syrup/ice; [cold brew](https://behavior.stanford.edu/knowledgebase/tasks/cold_brew_coffee-0.html) puts grounds and water in a refrigerated jar. These are explicit goal comparisons; final novelty is still a reviewer judgment.

## Current support and history implementation

The basket support query uses the actual box and cylinder geometry, with a0.2mm distance tolerance. AABBs only reject distant candidates; they never establish support. A point on a basket solid must touch the finite counter, or a convex support-mapping distance query must find real contact with an original stand/appliance/tool-body solid. Fine needles cannot support the basket. Round bases retain circular boundaries. The prior basket(251,190,5)mm case has a10.725mm spout/base gap and is unsupported.

After every native240Hz physics step during an action, the witness reads fresh PhysX rigid-body poses and COM velocities. Original-surface contact or true convex gripper contact within0.5mm must accompany inferred upward reaction m*(delta COM vz/dt+9.81)>=80% of the450g weight. Unknown data fail closed. The load estimate is a force-balance witness, not a contact-force sensor; rapid downward carries can conservatively fail. Every unsupported interval seen at physics resolution latches zero, including intervals wholly between20Hz control observations. The control scorer verifies all12 substeps were covered. Reset/layout settling is excluded; history clears only at episode reset.

Forbidden hand entry is tested against the opening between floor and rim using complete convex gripper solids after every240Hz physics tick, even when the basket has counter support. A between-control entry latches zero permanently; clearing the hand cannot restore credit. Coverage requires both support history and hand-entry checks on all12 ticks. The older20Hz entry check remains a redundant guard. Four additional controls exercise this supported-basket hand-entry case and persistence after legitimate progress.

The full-body/dose sweep, needle/solid damage guard, mass/bond rules, reference motion and one-second release hold are unchanged. The94 offline controls retain all90 earlier cases, including the75 original cases, real round-base contact, the10.725mm gap after progress, a24.5ms ballistic-hop state sequence, and physics-dispatch/reset gating. These are offline state controls, not native robot exploits or hardware validation. A35s native trace recorded8400 physics ticks,12 per action, no history violations, and minimum inferred upward reaction3.922N versus the3.532N gate; loaded needle work was9.307mJ. The final source with240Hz support/drop and hand-entry coverage completed its own matched normal A1.0/B1.0 reference.

BEHAVIOR-1K already contains coffee-making activities. This task narrows that family to retaining a dry dose while disrupting cohesive clumps, without tamping, pouring or brewing. Its needle contact and retained-clump outcome differ from the listed brewed-coffee goals and the accepted tamping/ejection tasks, but a narrower subprocedure or a different tool alone does not establish novelty. A reviewer may judge it to repeat the same underlying coffee-preparation activity; that overlap and specialist relevance remain open risks.

Complete reset timing, including rejected attempts and cleanup, must meet the240s acceptance limit. Natural-clump availability, treated-clump inspection preservation, needle drag and damage correspondence remain unmeasured.
