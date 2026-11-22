# Open the bonded cassette without cutting its gel

Category **Other**: laboratory research preparation. Frozen scripted demo: A 1.00/B 1.00; Dream-1: A 0.00/B 0.00, both at the full step limit without refusal or runtime error. No physical trial is claimed.

Open the native precast cassette, leaving its intact hydrated gel supported on either original plate and the empty plate separate. The cassette is not opened in advance; there are no added handles, release tabs or substituted gel. The knife is a real rigid tool prying a real part. No intended material cutting earns progress.

The [maker's opening procedure](https://documents.thermofisher.com/TFS-Assets/LSG/manuals/blotmod_pro.pdf), printed page 11, directs opening all three bonded sides and warns against “excessive pressure towards the gel.” This task stops before gel removal, well trimming or blot assembly. O*NET [Biological Technicians, 19-4021](https://www.onetonline.org/link/summary/19-4021.00) covers specimen and laboratory equipment preparation; no occupation-size or AI-timeline claim is made.

## Real parts, cost and surrounding apparatus

| Item | Size and material | Purchase route / allowance |
|---|---|---|
| [NuPAGE NP0321BOX](https://www.thermofisher.com/order/catalog/product/NP0321BOX), original 4–12% Bis-Tris precast gels, ten 10-well cassettes | 80 × 80 × 1 mm hydrated polyacrylamide in 100 × 100 mm styrene cassette | Maker page US$165.65 per ten; fresh cassette consumed each trial |
| [EI9010 Novex gel knife](https://www.thermofisher.com/order/catalog/product/EI9010) | Maker FAQ: 130 mm handle, 90 × 45 mm blade | [Maker US price table](https://www.thermofisher.com/lt/en/home/life-science/protein-biology/protein-gel-electrophoresis/protein-gel-electrophoresis-chamber-systems/xcell-surelock-mini-vertical-electrophoresis-system.html): $77.65 |
| [PYREX 3140-100 dish](https://ecatalog.corning.com/life-sciences/b2b/US/en/General-Labware/Dishes/Dishes%2C-Glass/PYREX%C2%AE-Crystallizing-Dishes/p/3140-100), inverted | 100 mm OD × 50 mm tall borosilicate | [Individual retail unit](https://www.zoro.com/corning-pyrex-crystallizing-dish-100-x-50-mm-3140-100/i/G3247974/): $31.99; no case purchase necessary |
| Rule, backlight and 10× loupe | 150 mm steel rule, ordinary LED light/phone inspection | $30 estimated allowance |
| Complete inert surrounding cell envelope | 110 × 120 × 160 mm housing, core and wedge from fixture drawing | $15 estimated printed-material allowance |

Approximate complete mechanical-opening kit **$321 for ten consumable trials**, before tax/shipping, excluding the benchmark's robot/cameras. Nothing has been purchased. The original powered cell costs more and is unnecessary for the enrolled off-machine opening action. A full-size inert copy preserves the unused parent envelope; it is not electrical equipment. [Current XCell SureLock user guide](https://documents.thermofisher.com/TFS-Assets/LSG/manuals/MAN0000739-XCellSureLockMiniCell-UG.pdf), page 6, gives the cell size. The older 145 × 140 × 110 mm dimension belongs to the blot module, not the whole cell. Fixture drawings describe only the affordable inert surroundings and the estimates used by the simulation.

## Fixed start and reset

World metres: x forward, y left, z up; tabletop z=0.7475. Arm bases x=0, y=±0.305. Cassette centre (0.325,0). Dish base rests on the counter and top is 50 mm above it. Lower plate centre z=0.7985, gel centre z=0.8000, upper plate centre z=0.8015. Well edge +x; bonded edges -x and ±y. Plates are estimated 2 mm thick, with a 10 mm native border and notched/slotted well/foot relief. Original plate thickness, relief and blade cross-section must be caliper-checked for a physical replication, rather than printed as replacement parts.

Knife root is the handle centre: A (0.185,-0.220,0.7545); B (0.215,-0.255,0.7545). Axis +x, blade tip +155 mm from handle centre. Estimated handle section 22 × 14 mm, blade 1.2 mm; underside of blade is level with handle underside. Estimated masses: lower plate 24 g, upper 22 g, native-density gel 6.4 g, knife 50 g, dish 220 g. No sprung tiny body is used. Cassette, gel and dish have the same start in both layouts; only the rigid knife moves. Jitter zero; deterministic A then B, averaged, no random geometry seeds.

1. Use a fresh, in-date original cassette; record SKU, manufacturing lot, expiry, temperature, storage and hydration. Mechanical qualification uses an unused native gel at 20–25°C, out of its bag for no more than five minutes, comb and foot tape removed without opening any bonded edge. This explicitly differs from a post-electrophoresis specimen; transfer performance is not claimed.
2. Inspect the entire gel and both plate faces backlit; reject an initially damaged cassette. Keep the reference image and a 1 mm scale in the gel plane. Keep all three bonded margins intact. Do not select a retaining plate in advance.
3. Put it notched plate up on the inverted ordinary dish at the stated coordinates. Place the knife flat at A or B. Set the disconnected inert cell behind it, with no electrical leads in the manipulation area.
4. Run the episode without a setter. Replace the consumed cassette after any edge release; resealing is not an equivalent reset. Native bond and adhesion consistency are uncalibrated batch properties and must be logged in real trials.

The dish only raises a source-normal free cassette; it does not restrain it, open a seam or acquire a tool for the robot. A direct supported one-hand strategy is legal if it meets the same damage and endpoint rules. There is no enforced bimanual sequence or handover.

## Scoring and real inspection

The numerical model is documented in task.py; the practical inspection witness remains unqualified. No physical inspection result has been measured.

Score 0 for the entire trial after a new gel slit/tear/hole of at least 1 mm, a plate crack entering the gel window, loss of gel support from both original plates or any part/tool off the counter. A repaired or returned object does not erase an earlier violation. Otherwise .25 for one full native bonded edge released, .5 for two, .75 for all three and an intact retained gel with plates at least 10 mm apart. Every nonzero endpoint needs hands at least 30 mm clear and the scene still for one second; 1.0 also requires plates and tools supported down. Exact numerical proxies and endpoint definitions are listed below.

Side video and a rule witness seam release; the full edge must be unbonded, not just a touched corner. Within the simulator, a local release needs the actual blade perimeter within 14 mm, at least 0.20 N of blade/plate contact on that same edge, within 14 mm of its bond plane and between the actual moving inner faces (0.3 mm contact tolerance), and at least 0.6 mm plate opening. A released edge also needs a visible 2 mm opening across two adjacent 17.5 mm-spaced sites. The load and local proximity thresholds are material-law parameters; a physical assessor measures released edges and damage with the stated simple tools, rather than measuring blade force. For a real episode, record close overhead/side video and before/after backlit images, inspect the exposed gel face and the empty plate with a 10× loupe. Inspect through the retaining plate from below without peeling the gel. Qualification must first show detection of sacrificial 1 mm slit witnesses in multiple orientations, including a slit against a plate. If the witness cannot resolve those slits, the real trial is unscorable; the submission does not assert a completed physical witness test.

## Material model and bypasses

The plates, original knife and native-size hydrated gel collide. The gel has a blue visibility tint and is one conserved supported slab with explicit irreversible damage states; this does not model biochemical quality or calibrated gel deformation. Fifteen local seam sites progressively release a native implicit D6 restraint: translation 1500 N/m in-plane and 700 N/m normal, damping 2 N s/m and 12 N drive limit; angular 0.2 N m/rad, 0.01 N m s/rad damping and 0.15 N m limit. Remaining stiffness scales with unreleased sites, except normal stiffness remains at least 140 N/m until the final site releases. This is a declared aggregate plate-compliance model, not fifteen calibrated material springs. Local release requires nearby knife contact load and actual plate opening. Equal estimated wet adhesion acts on both gel faces; contact and gravity choose the retaining plate. There is no hidden designation of a good plate and no gripper weld.

| Shortcut | Expected result |
|---|---|
| Do nothing / touch a seam | 0; no bond release |
| Move a blade in air or replay a pose with no plate load | 0; local release requires measured loaded contact |
| Yank an unworked margin | permanent 0 after the stated tear/crack threshold |
| Drive the knife into the gel | permanent damage, no credit |
| Free only a corner, or one anchor on each side | 0 until a whole edge is released |
| Open and then return damaged plates to start | remains 0 |
| Carry the plate/knife at the endpoint | cannot earn 1 |
| Supported direct or one-hand opening that preserves gel | legal; no compulsory sequence bonus |

## How this differs from close tasks

- TK-5VTWYW / paint-tin lid: a rigid lid/seat fit, without an adhering fragile sheet next to three bonds. This distinction is a hypothesis about contact control, not proof of novelty.
- TK-V7P9X9 / protected carton slit: cuts a seam with a protected item below it; this task wedges and debonds margins while preserving the adhering sheet.
- TK-W7HHVG / cutting a tie beside cables: selective removal at one localized restraint rather than three bonded plate margins.
- TK-F4ZU9A / scraper under a stuck print: releases an adhered object from one plate; here the gel must remain attached to one original plate while a surrounding welded enclosure is opened.
- GelSlim sheet demolding removes an entire silicone sheet from its mold. No sheet peeling is scored here. Blot-transfer cassette rolling/closure (TK-S2WQ7R) is a different downstream assembly action.
- TK-PA9SKU / clamping two precast gels into an electrode assembly was closed final for overlap. This task opens the native bonded enclosure after removal from the cell; another laboratory phase alone does not establish importance.
- TK-9KJVV4 / switching power off before lifting the tank lid is a pending safety revision. This task starts off-machine with disconnected inert surroundings and scores local mechanical debonding, with no electrical sequence claim.

Board overlap and category importance remain review risks. Native product identity alone is not the novelty argument.

## Running

Use yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), Inspect Robots and the bundled pure-Python parts/lots/tilt helpers. The demo is open-loop from the visible authored start, with no hidden bond or exact-pose feedback. Two layouts stop one fixed pickup location, not memorized replay of both starts. Per trial: 2,400 control steps, at 20 Hz.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Open this precast-gel cassette and leave its intact gel supported on one original plate, with the other plate separate." --scorer task_score --max-steps 2400 --epochs 2
# Repeat with --policy dream for the model run. TASK_LOT=A or B pins one layout.
```

Code and generated custom geometry: MIT. Documentation/drawings: CC BY 4.0. Product pages/manuals are references only; their images are not redistributed in the task. No third-party meshes are included.

## Exact numerical proxy and endpoint definitions

The simulation declares damage when an unworked native seam site opens more than 4 mm or displaces more than 8 mm relative to the other plate; this is a conservative plate/gel strain proxy, not a measured styrene fracture threshold. Blade travel at least 1 mm inside the gel causes an irreversible cut state; accidental cutting is resisted by a smooth, estimated 0.6 N drag opposite relative blade motion, with equal reaction on the gel. No intended cut earns credit. A gel displaced more than 3 mm from both original face adhesion anchors is damaged in this simplified model. All gel mass remains in the scene; no biochemical quality is inferred. Wells and the gel foot are not resolved as deformable shapes in the slab approximation.

Off-counter means a scored part's lowest point more than 10 mm below z=0.7475, or its body origin outside x=(-0.170,0.880), |y|<=0.630 m. Each released edge requires all five local sites released and a previous >=2 mm normal opening at two adjacent sites (17.5 mm apart). Once released, an edge may close loosely without becoming bonded again. Every nonzero endpoint requires known arm state, hands >=30 mm clear, and all scored corner positions within 1 mm of their common reference for one second. The face-sampled numerical hand margin is 36 mm to guard the 4 mm samples/2 mm thinning.

Gel retention means its centre within 3 mm laterally of either original plate, its face within 1.5 mm of that plate's supporting face, and the face normals aligned within 5 degrees. The ten-millimetre plate separation is a certified separating-axis surface gap, not a centre distance. Full-score support means plate/knife/dish lowest face -1 to +1.5 mm from the counter, or a plate centred within 8 mm of the dish’s current XY position, with lowest face -1 to +1.5 mm from the dish’s actual highest supporting surface. Supported objects must be within 5 degrees of level, moving less than 8 mm/s and 0.15 rad/s.

The original XCell II opening source permits the gel to adhere to either plate. The endpoint allows either original plate and does not claim a blot-ready bottom-plate preparation. Support is measured against the movable dish's current position and top surface.

## Native support and approach

The lower plate now retains its original full well margin; the upper notch exposes an estimated 10 mm lip, derived from the maker's 100 mm cassette and 80 mm gel envelopes. A sideways open left gripper presses one flat fingertip on that original lip, with the other fingertip beyond the cassette. It braces the lower plate through the front and right pries, then withdraws before the last-side reach; the support and cassette weight resist the remaining seam. No extension, pickup handle, gripper weld or pre-opened seam is added. The ordinary support dish is PYREX 3140-100, 100 × 50 mm; the larger footprint keeps this native margin over the support.

The declared bond-pressure patch uses actual blade/plate contacts within the inter-plate gap on the same edge. Native convex manifolds expose a few discrete contact endpoints; measured edge load is distributed over the rigid blade's actual footprint. A site still needs the blade perimeter within 14 mm and actual normal opening >=0.6 mm. Top-face loads, wrong-edge loads, no-load poses and untouched margins cannot release it. Resistance and all damage/hold rules remain active. The scripted grip has a fixed B pickup correction (0.45-degree nose-up initially, 0.65-degree nose-down after its first loaded pry) and 0.8-degree compensation per completed front pry, with no additional pitch change after a side pry, plus a B first-entry height correction of -0.6 mm, a measured +0.8 mm second-front entry correction and -0.3 mm side-entry correction relative to the first front pass; it does not read hidden states. After prying, the blade flattens and withdraws horizontally outside the margin before lifting. At B, the first 5 mm of outward withdrawal occurs during flattening so the sharp edge stays outside the gel as it levels. The counterholding arm stays on the lower lip through the first side extraction, then moves clear before the last seam.


The joint has been checked in an in-air force-identification test with collisions and gravity disabled. All 36 signed translation/rotation displacement and velocity cases produced opposing reactions; capped translation reversed between +12 N and -12 N. This validates the implemented force direction, not the native bond parameters.

The reproducible diagnostic is `fixture/drive_sign_probe.py`, with recorded reactions in `fixture/drive_signs.json`. It is separate from the scored scene. On the same simulator stack, run from this folder with `COUPON_OUT=/tmp/cassette-signs.jsonl inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=fixture/drive_sign_probe.py -E object_jitter=0 -E control_hz=20 --instruction "Identify joint force signs in air." --scorer task_score --max-steps 100`.

The residual normal stiffness estimate is 140 N/m: a styrene strip 10 mm wide and 2 mm thick with 75 mm free span and E≈2.96 GPa gives 3EI/L³≈140 N/m. [INEOS Styrolution GPPS 1700](https://eshop.ineos-styrolution.com/Product/GPPS_Styrolution-PS-1700_SKU200300060597.html) lists tensile modulus 430,000 psi (about 2.96 GPa). This is a reference material and dimensional estimate, not identification or calibration of the cassette grade/weld. Other drive stiffnesses continue to scale with remaining sites; all drives disable after full release. The in-air test uses the fully bonded settings.

The loose upper plate is acquired from its negative-y border after all three pries. The blade first enters outside the gel footprint, lifts the free border 12 mm, then advances under the plate above the gel. Both plates and the knife are set down before the hands withdraw. No retention state is read by the open-loop controller.

## Simulation validation

The normal A/B scripted run reached 1.00 in each layout at 1,898 control steps (94.9 s). Both trials released 15/15 sites, witnessed all three openings, left the gel retained on an original plate and the other plate supported on the counter, and met the hands-clear/still gate. No model cut, support-loss, unworked-margin or off-counter damage flag occurred. The same frozen runtime was used for the scripted and model runs.

A native unsuccessful-trajectory control, with identical physics/scoring, recorded 2.65 mm cumulative blade travel inside the model gel volume and therefore scored 0 for the whole trial. It later freed all 15 sites, but the damage state persisted. `fixture/native_cut_control.json` records this control; it is a simulator damage check, not qualification of the physical hidden-slit inspection.

Standard combined media show layout A; the per-layout native records and camera videos retain both layouts. Object jitter is zero. Actual evaluation reset seeds are retained in those records. Physical material and inspection correspondence remain unqualified; simulation feasibility is not a measured hardware result.

Dream-1 scored **0.00/0.00** on the two layouts, reaching the full **2,400 control steps** in each, without refusal or runtime errors. It released no seam sites. Neither model trial recorded a material damage flag.
