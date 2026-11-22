# Widen the bagel hole without thinning one side

Shape the prepared bagel ring with an even open centre and leave it on the greased sheet.

This task tests controlled permanent deformation of a free cohesive food ring. Opening one axis once leaves a narrow opening across the other axis or a thin neck. The endpoint is the unloaded food, with no groove, mandrel, mold or supporting fingers determining its shape. Cold preparation happens on an ordinary complete kitchen counter before the sheet enters an oven. No boiling, proofing or baking is performed during the trial.

## Status and evidence

**Final native scripted demonstration on the corrected scorer: A=1.0, B=1.0, mean=1.0.** Both finished at80.85 s after the2 s hands-clear/still gate, with no latched tear,cut or counter violation. The native-camera sheets and endpoint readings are in`validation/`.

| Layout | Score | Hole min–max | Wall min–max | Minimum local height | Outside diameter |
|---|---|---|---|---|---|
| A | 1.0 | 48.81–53.69 mm | 22.13–24.40 mm | 24.10 mm | 100.95 mm |
| B | 1.0 | 47.94–52.08 mm | 22.53–24.54 mm | 24.57 mm | 99.61 mm |

These dimensions come from the authored native geometry. The exported top/left-wrist/right-wrist camera sheets demonstrate motion and release;they are not substituted for calibrated real-trial measurement images.

**One Dream-1 beta invocation recorded A=0.0, B=0.0, mean=0.0.** Both hit the2400-step limit with no refusal or abstention. The arms moved but no fingertip-to-food load was recorded. Each layout returned50action chunks,dry-run off,with49recovered session expirations;those renewals limit interpretation as a continuous policy rollout.

That invocation preceded a grading-helper correction. The unchanged30mm opening gate rejects all4800retained states,so post-hoc regrading remains0.0/0.0. This is not a fresh execution under the updated helper. `task.py`,reset,material dynamics,demo andRecord are unchanged;the main file matches the host and predates both evaluations. The helper-version distinction and isolated source change are recorded in`validation/grading-change-provenance.json` and`dream-regrade.json`.

The corrected scorer rejects a supported9.84mm local neck that the earlier whole-cell height range had incorrectly credited at0.5. All five offline geometric tiers and release/damage regressions pass. An independent linear-programming check agrees with the new local-height bound within0.00018mm on three test geometries. These are computational checks,not dough calibration.

No real dough trial,rheology calibration,film-traction test,sanitation validation or human threshold-repeatability study has been performed. Source recipes establish an authentic operation;they do not establish household prevalence. Societal importance and the public Make a Donut overlap remain reviewer judgments.

[O*NET Bakers 51-3011.00](https://www.onetonline.org/link/details/51-3011.00) lists the core task: “Roll, knead, cut, or shape dough to form sweet rolls, pie crusts, tarts, cookies, or other products.” This corroborates the operation, but does not establish household prevalence or a job-count threshold. The category remains Societal relatability.

## Products and reset

- [King Arthur Bagels recipe](https://www.kingarthurbaking.com/recipes/bagels-recipe):480g bread flour,303g water,9g instant yeast,12g table salt and9g non-diastatic malt. Eight large portions from813g ingredients give approximately102 g each. Weigh the actual finished dough. The maker says to stretch the hole until it is “about 1 1/2\" to 2\" in diameter.” The [shaping guide](https://www.kingarthurbaking.com/blog/2020/01/15/how-to-shape-bagels) describes opening a preshaped round; the simulation begins after the normal initial finger hole.
- [Nordic Ware Naturals Baker's Half Sheet43100](https://www.nordicware.com/products/naturals-bakers-half-sheet/):listed $20 (maker price checked Nov21),17.9x12.9x1inch exterior,17.1x12.1inch interior. Reconstruction455x328x25 mm,434x307mm clear floor;750g estimated from the aluminum sheet/rim class. It rests freely on its own bottom and is never lifted by the demo.
- [OXO Good Grips73281 scraper](https://www.oxo.com/shop/kitchenware/utensils/multi-purpose-scraper-chopper.html): listed $14.99 (maker price checked Nov21), 6x1.5x4inch,stainless blade with original raised handle;maker weight0.545lb≈247g.152.4x101.6x38.1 mm flat envelope;38.1 mm handle diameter is an envelope-based estimate. No new handle is added. The scraper is available but optional. It is not a mold or ring support.
- Preparation equipment, removed before the trial:scale, mixer, thermometer, humidity meter, camera and300mm ruler. Measurement equipment is not manipulated by the robot.

Use one fresh102±2g source-recipe portion per trial. Record ingredient lots, mixer make/model and speed, room temperature22±2C and relative humidity45±10%. Knead10min at the mixer's dough setting, bulk rest90min, divide and round, then rest30min. Start with one connected ring,25±3mm clear hole,75–85 mm outside diameter and22–35 mm height. These are controlled preparation settings, not a repeatability study. Reject out-of-window resets instead of changing thresholds. Start within5min of making the hole and finish within120 s. Do not substitute cooked bagels, rubber rings or play dough, re-knead used dough or recycle an earlier stretched portion.

Spread1.0g food-grade canola oil over the clean sheet before placement; no loose pool remains. Fit food-contact polyethylene film≤0.2mm tightly over the original pads, retaining their native shape. Inspect film before and after; any tear/exposed pad contact during the trial scores0. Food and film are discarded. Clean and sanitize original contact surfaces between trials according to the rig's food-contact procedure. There is no claim of validated sanitation or measured film traction. Physical execution requires that protection procedure to be in place; raw dough is never eaten.

## Reproducible layout

Surface materials: the station worktop and rear upstand are represented as sealed wood with laminate faces; their friction is estimated. The native metal grippers retain their original elastomer pads, covered by the stated polyethylene film; pad traction is unmeasured. The pan is aluminum,the scraper blade stainless steel,and its soft handle is represented as an elastomer estimate.

Coordinates in millimetres:x forward from the arm bases,y left,z up. Station counter top747.5;arm bases at(0,±305,750). Full table remains present; rear upstand25x1100x180 is centred(720,0,837.5).

| Object | LotA | LotB |
|---|---|---|
| Full sheet origin |(260,0,748.5),455 mm axis alongx |same |
| Food centre |(230,+45,764.7) |(230,-45,764.7) |
| Scraper body-frame origin |(400,-300,766.55) |(400,+300,766.55) |

The optional scraper rests on the actual counter beside the sheet,within400mm reach of its nearer arm;the native shaping approach leaves it clear. The scraper handle axis is31.75mm toward−y from its body-frame origin;the blade is centred inx/y atz−17.5mm. The ring starts on the sheet in both lots. This deliberately replaces the keeper's later counter-start/transfer proposal: the scored core remains permanent free-ring reshaping, and the extra rigid transfer is omitted. The complete station worktop is1100x1300mm at747.5mm, not the supplemental IKEA VADHOLMA20359154 island at900mm. The island is not reconstructed or claimed as native furniture; the true-size pan, scraper and food remain unchanged. That work-height difference is a declared scene limitation. Its40 mm initial radius is safely inside the434x307mm floor. The pan is not rotated. These published base-relative coordinates replace both earlier proposed layouts. Trials cycleA,B with no jitter. The screenshot showsA. The task does not impose a specific hand, regrasp count or twirling method. A one-hand solution is legal. The scripted plan reads only the published visible starting position, never an internal material state or joint feedback.

## Score from video, photographs and a ruler

Score 0 permanently for a through-tear breaking the ring, any new tear longer than5 mm, removed/cut-away dough, any food crossing the counter edge or falling20 mm below it, or uncovered/ruptured-pad contact with raw dough. Damage stays0 even if the food is moved back or the opening is later improved.

All nonzero tiers require the entire ring supported on the original sheet, no gripper or tool within5 mm of the food, all grippers also clear of the scraper by5 mm, and2 continuous seconds of stillness after release. The simulator adds4mm conservative margin for sampled hand surfaces; stillness means sector centres move≤0.5 mm between50ms samples and pan/scraper move<5mm/s and<0.05rad/s. In addition, no food sector or pan/scraper bound may drift more than1mm over the entire release window;otherwise the window restarts. Compare the final photographs only after this window. A ring held open or a good shape that immediately shrinks earns0.

Place an overhead camera normal to the pan; use four coplanar known-length ruler/corner fiducials to rectify perspective. Calibrate to1 mm per pixel or finer in the measurement images, independent of the policy camera. Trace the complete inner and outer silhouettes as polygons. The hole centre is the signed-area centroid of its inner polygon. Angular zero is the pan's455 mm axis. Measure128 equally spaced rays22.5/8degrees apart and inspect all visible local necks. d andD are the minimum and maximum across64 opposite ray pairs; wall width is the radial separation between silhouettes. Inspect all boundary vertices as well. Non-star-shaped, folded, overlapping or unmeasurable rings score0. Maximum outside diameter is the maximum separation of any two outer silhouette points. Minimum height is the smallest full vertical span of a radial cross-section around the ring, including the narrow interfaces between portions of material, from four calibrated orthogonal side views; inspect the complete visible profiles, not four isolated spots. Do not press or move the food for measurement. Tear length is the longest visible crack path read against the same scale in top/side images and continuous video.

| Highest applicable tier after zero/release gates | Photo/ruler condition |
|---|---|
|0 |d<30 mm,D>65 mm,minimum wall<12 mm,or minimum height<12 mm |
|0.25 |30≤d<40 mm |
|0.5 |40≤d<45 mm;or d≥45 mm withD>55 mm |
|0.75 |d≥45 mm,D≤55 mm,all walls/heights≥12 mm |
|1 |0.75 and max/min wall width≤1.35and outside diameter≤130 mm |

A flat pancake or a thin local neck fails every tier, strengthening the proposal's low tiers. Threshold comparisons include1 micrometre numerical tolerance only; the real measurement is read to1 mm. Recheck borderline photographs with a second rater rather than inventing subpixel precision. Human repeatability at the boundaries has not been measured. Report each lot and their arithmetic mean.

## Declared material model

Sixteen free convex material sectors total102 g;each is6.375 g. Six-axis soft cohesive joints couple neighboring sectors, including the closing bond. These are free bodies, not radial sliders tied to an invisible centre. Translational stiffness60N/m,damping0.3N·s/m and weak0.008N·m/rad bending are estimates for an exploratory discrete material, not measured dough properties. Each bonded rest length creeps toward its actual stretched length at 0.2/s, limited to 5 mm/s, while the ring has measured fingertip-to-food contact load ≥0.05 N, hand proximity <4 mm and local material motion. The contact gate is ring-wide; tensile strain and motion select each bond, so an indirectly tensioned bond may also flow. This is an explicit discrete-material rule. Removing load stops plastic growth. Native joints supply elastic recovery and reaction forces. Extension beyond1.8times the current rest length irreversibly breaks cohesion. An original scraper edge swept through at least5 mm of the food core is a declared cut and also scores0. Supporting the underside does not cross that core. These are declared tear/cut rules, not calibrated fracture laws.

A separately retained compression ratio follows counter-supported vertical displacement under direct downward load on the affected sector, at 0.2/s;it reduces height and widens the local wall at fixed volume. This permits flattening independently of tensile extension. This law is also an estimate.

Each cell begins with cross-section axes reduced by the square root of local centreline extension. A coupled correction then solves the actual16convex-cell volumes against their original values,including shared-cap averaging and convex-hull filling. It fails closed above0.1%relative cell-volume error;uniform1.0–1.6x offline checks were below0.001%. The visible faces are exactly the authored convex collision hulls. Collider mesh vertices and the visible surface change together; no free body is teleported or welded to a finger. Cross-section and material laws do not read a target hole size or score. Contact-induced flow has an approximate control-rate update; continuum gluten mechanics, tack, skin formation and proofing are absent. These limitations require calibration against the stated real recipe before any physical performance inference.

`score()` and `success()` use the same final geometry. Minimum height comes from vertical radial-plane intersections of the actual convex hulls, including shared interfaces. Surface-edge intersections are monotone between angular events; conservative interval bounds are refined to a0.01mm bracket,with0.1micrometre numerical margin and10nm interface tolerance. Unresolved profiles fail closed. Overall cell bounding-box heights are not used. Each sector is projected as its actual convex collision hull;ray entry into the union gives the hole and ray exit gives the outside silhouette. The area centroid is obtained from512boundary rays before the fixed128measurement rays. This includes inward convex-hull chords rather than scoring an idealized centreline torus. Runtime records are bound to the supplied object states; missing hand/load/geometry data fails closed. Reset occurs only through the simulator's actual reset callback. Face-sampled original gripper/pad hulls determine release, including faces whose vertices are far away. A240Hz native post-physics-step guard uses uncached body/link transforms and conservative whole-hand bounds to interrupt release and latch damage between20Hz control samples. The scraper must also be clear of the complete food. Settled reset validation and history initialization happen after lot placement and before policy actions. Measurements use the actual pan coordinate frame. Film is assumed intact in simulation; the real trial uses continuous visible inspection and before/after barrier checks. The material model has no independent film-puncture model. Blade and food motion are interpolated in2.5 mm increments between50 ms samples;stationary food on an embedded edge also fails. The settled reset is checked against the declared initial hole,outside diameter,height and sheet support before any credit. Tears and off-counter events are latched for the whole trial. The unplaced mathematical reset has23.53–23.99mm opening;the native settled reset is separately checked to22–28mm,so either no-op scores0.

## Bypass checks

| Attempt | Required outcome |
|---|---|
|Leave it alone,touch,translate or rotate an unchanged ring |0:opening stays below30 mm |
|Hold a large opening on the fingers |0:release window missing |
|Release briefly then touch it again |hold resets;no retained credit |
|One large pull that creates a neck or tear |0 for wall<12 mm or any declared tear |
|Press flat to make a large hole |0 for height<12 mm;not credit for diameter alone |
|Cut out a larger centre or remove food |permanent0 |
|Move food off counter then put it back |permanent0 |
|Hide a local neck between eight diameter lines |full outline and128 rays reject it |
|Unknown gripper state or synthetic endpoint without runtime certificate |0 |

No two-arm necessity is claimed. The single-arm native-finger demonstration is a one-hand check of retained plastic shape after unloading,without elastic capture or placement into a receiver.

## How this differs from close tasks

The source-family neighbor is **TK-NT76KH(PR1576),Fit an O-ring onto a hydraulic plug**. That task elastically stretches a ring over a thread and leaves it captured in a groove; permanent set is damage. Here permanent set is required,and the free ring must retain a uniformly thick45–55 mm opening after every support is removed. There is no receiver to determine the final shape. This is a different core control objective,not a food-colored version of elastic installation.

The accepted kitchen-family tasks **Peel the cucumber, leaving the ends (TK-VT2J8N,PR1899)** and **Lock the spring tongs and put them away (TK-D88DTA,PR1830)** have different core skills. Peeling removes a surface along repeated tool strokes; this ring must retain all material while redistributing it. Tongs require elastic spring compression and catch engagement; this ring must retain deformation without a catch. Reusing a kitchen counter does not reproduce either manipulation.

[BEHAVIOR make_bagels](https://behavior.stanford.edu/knowledgebase/tasks/make_bagels-0.html) has real/cooked/on-sheet predicates and ingredient-to-food transitions, not a measured hole or wall-uniformity endpoint. [Make a Donut](https://qq456cvb.github.io/projects/donut) includes dough-to-ring shaping using rolling and hole-poking tools, so the broad claim that dough-ring shaping is new is not supported. The proposed distinction is redistributing an already open free ring under native fingers while retaining its narrowest wall after release. Whether this is sufficient beyond a starting-state/quality variant remains an independent importance risk.

The newly pending **Press a corn tortilla in a cast-iron tortilla press (TK-W5XE32, PR3381)** uses plate closure and lever force to compress a ball into a disc. Its apparatus determines the shape; it does not test distributed tension or a free ring's local neck uniformity. It remains a close soft-food category neighbor, not accepted evidence.

The dough-sheet spacer-pin and chopped-log cases mentioned during screening were closed,not accepted evidence. A pipe bender has tool-constrained permanent curvature; spackle filling redistributes material against a fixed cavity. Those do not test redistribution around a free cohesive ring. A title search alone cannot prove novelty against every task in every public suite; the independent review must assess the remaining overlap and importance risk.

## Public capability comparison

The following primary-source check was completed Nov21. It supplements the independent review; it is not an organizer novelty ruling.

| Public set and checked scope | Actual goal or manipulation | Capability comparison |
|---|---|---|
| [LIBERO,130 published BDDL definitions](https://libero-project.github.io/) | Goals use On/In,Open/Close,Turnon/Turnoff predicates. | No plastic-shape or unloaded-profile goal appears in those definitions. |
| [RoboCasa365 official task descriptions](https://robocasa.ai/docs/build/html/tasks/composite_tasks.html) | CookieDoughPrep retrieves egg/butter;MixCakeFrosting loads/starts a mixer;PrepForTenderizing places a rolling pin;ToastBagel loads/starts an oven. | These published descriptions concern preparation,placement or appliance operation,not controlling free cohesive geometry. This checks descriptions,not every implementation/version. |
| [SimplerEnv25 mapped variants](https://simpler-env.github.io/) | Rigid picking,relative placement,drawer operation,stacking,and food-to-container placement. | The published environment map has no plastic-shape objective. |
| [RoboDojo43 simulation pages](https://robodojo-benchmark.com/doc/sim-tasks/) and18real tasks | [Make Bread](https://robodojo-benchmark.com/doc/real-tasks/make-bread/) inserts bread and stacks bowls;[Make Food](https://robodojo-benchmark.com/doc/real-tasks/make-food/) arranges food/tools/pot/lid;[Make Toast](https://robodojo-benchmark.com/doc/sim-tasks/make-toast/) inserts bread and operates a lever. | These food tasks are assembly/placement/control,not plastic ring forming. Cloth folding and pouring are different material/control regimes. |
| [DataCenterBench public ten tasks](https://robocurve.org/datacenterbench/) | Six equipment controls and four connection operations. | No free plastic-food geometry objective is among those ten;unpublished private tasks cannot be cleared. |
| [BEHAVIOR make_bagels](https://behavior.stanford.edu/knowledgebase/tasks/make_bagels-0.html) | Cooked real bagels on a cookie sheet,with ingredient/mixer/oven transitions. | Overall bagel making already exists. The claimed additional implemented primitive is contact-driven free-ring deformation,not the activity name. |
| [Make a Donut,author project](https://qq456cvb.github.io/projects/donut) | Rolling-pin compression followed by pole-induced hole creation;point-cloud/EMD target shaping. | Plastic toroidal shaping already exists. The proposed distinction is distributed tensile extension around an intact annulus while preserving its neck under unloading,rather than compressive spreading and hole creation. Starting shape,tools or tighter tolerances alone do not establish novelty. Whether the physical-control distinction is sufficient remains a reviewer judgment. |

The specific own-family comparisons above remain stronger:material removal,capture in a receiver,and spring/catch operation have different core objectives. Broad toroidal dough-shaping novelty is not claimed. Checked catalogue revisions and coverage counts are retained in`validation/public-coverage.json`;public primary sources were read,not executed.

## Responses to the proposal's independent pre-review

1. Importance: authentic source shaping and its large opening are established; the distinct free/plastic endpoint is specified above. No adoption or worker-count claim is invented. Eating bagels is not treated as evidence that shaping them is universal. Importance remains at risk.
2. Construct validity: native true-size food,pan,tool and counter remain.16 free sectors avoid sub5g sprung bodies. Contact/loaded plastic flow and volume conservation are explicit; no invisible mandrel or idealized target-radius rule. Film traction and recipe calibration remain unmeasured. GPU traces must demonstrate access and unloading. Pancakes and thin necks get0 even at partial tiers.
3. Scoring: polygon centroid,pan-fixed axis,perspective rectification,outside diameter,tear length and full-profile height definitions are supplied.128 rays and all local boundaries replace eight rays/four height spots. Threshold-repeatability remains an actual unperformed human check.
4. Reproducibility: portion,mixing/rest windows,temperature/humidity log,grease amount,polyethylene protection and single-use reset are specified. LotB now starts completely on the sheet;pan rotation and150mm lateral offset are removed. Maker mass247g supersedes the scraper's earlier mass estimate. Contact contours and mixer-dependent dough behavior remain estimates.
5. Feasibility: both native layouts now demonstrate a full-score unloaded shape using one original gripper. Native-camera evidence and ruler-scale dimensions are retained below. A computer model cannot perform the proposed hand-shaped real-food check; no real hand trial is claimed. Dream-1 was evaluated once across both layouts with the same instruction;it earned0.0/0.0 without refusal,under the session-renewal condition stated above.

## Runtime revision

Isaac Lab-Arena checkout `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`;Python3.12.16,inspect-robots1.0.0,inspect-robots-agent0.30.0,inspect-robots-dream0.1.0,IsaacLab17.0.2,NVIDIA L40S/driver565.57.01. A local asset-resolver configuration differs from that checkout;its SHA256 and Dream package versions are in`validation/runtime-pinned.json`. This task uses original primitive reconstructions and no library assets.

## Run

Use yam-isaac-lab-arena,the program's YAM embodiment for Isaac Lab-Arena.

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --policy yam_scripted --instruction "Shape the prepared bagel ring with an even open centre and leave it on the greased sheet." --scorer task_score --max-steps 2400 --epochs 2
```

Use`--policy dream`with the identical instruction for the policy evaluation. `TASK_LOT=A`or`B`pins a diagnostic layout. The two layouts prevent one fixed absolute trajectory;they do not establish generalization to unseen dough or arbitrary positions.

Code and original reconstruction:MIT. Documentation/drawings:CC BY4.0. No third-party CAD or texture is redistributed;linked product/recipe pages are references.
