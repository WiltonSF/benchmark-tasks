# Clean a potato ready for cooking

Validation status: the scripted reference scores 1.0 in both layouts on this source; each trial ends at 175 s of the 180 s limit. Dream-1: A 0 and B 0, both trials at the 180 s limit. No physical trial has been run.

A loose potato with twelve marked soil windows lies in a kitchen catch tray beside a free vegetable brush. Every window must be brushed clean without damaging the skin; then the potato goes on the clean plate and the brush stays beside it. The windows sit on all four sides of the middle 60 mm of the potato (x = -24, 0 and +24 mm along its length), about a tenth of the skin; the two end caps are not scored. At any moment some windows are under the fingers or face the tray floor, so the potato has to be turned and regrasped, or held in some other way that leaves them exposed. The brush is not attached: it lies bristles up on the tray floor in the tray's corner. It starts 176 mm from the plate centre, inside the 105-200 mm "beside" range, so leaving it in the tray meets that condition. The faucet is off and is not used. This is visible soil removal, not a food-safety or microbial test.

## Products and real setting

The [OXO Good Grips 33781 Vegetable Brush](https://www.oxo.com/vegetable-brush.html) has a maker envelope of 3.3 x 2 x 2.5 in (83.8 x 50.8 x 63.5 mm), nylon bristles and a non-slip grip; maker price checked Nov 21: $9.99. The maker says: “Clean harder vegetables like potatoes and carrots with the Flexible Vegetable Brush.” Its model mass is an estimated 90 g, between the retailer-listed 0.17 lb (77 g) net and 0.25 lb (113 g) shipping figures ([retailer](https://www.webstaurantstore.com/oxo-good-grips-green-vegetable-brush-33781/29733781.html)), allowing for water held in a used brush. The model has the curved rubber body and a separate compliant bristle field.

The [IKEA KALAS plate](https://www.ikea.com/us/en/p/kalas-plate-mixed-colors-80461380/) is polypropylene, 190 mm diameter, $1.99 per six at the checked maker page. Its 4 mm floor, 14 mm rim and 50 g mass are estimates; its underside is secured to the counter. The potato is a Russet Burbank sold as a U.S. No. 1 russet, selected by caliper and scale to 95 x 65 x 60 mm within 3 mm and 200 g within 20 g (real-trial procedure below). It is neither scaled nor clamped. A custom polypropylene catch tray (drawing: `fixture/catch-tray.svg`) is fixed to the worktop, with a backsplash and faucet behind it.

[USDA guidance](https://ask.fsis.usda.gov/article/How-should-fresh-produce-be-washed-before-eating) describes brushing firm produce. [O*NET 35-2021.00](https://www.onetonline.org/link/summary/35-2021.00) (Food Preparation Workers) lists “Wash, peel, and cut various foods, such as fruits and vegetables, to prepare for cooking or serving.” The category is household relatability; no worker count is claimed.

## Starting poses

Coordinates are metres in station world coordinates; x points away from the arms, y to their left. The worktop is z = 0.7475 m; heights below are relative to it.

| Object | Centre, size and pose |
|---|---|
| Catch tray | (0.285, 0), outer 300 x 240 x 35 mm, 4 mm floor and walls; fixed |
| Potato, layout A | (0.245, -0.035, +0.034), long axis along x; red dot up, blue dot on the side facing the plate (+y); all twelve windows coated |
| Potato, layout B | Same centre, turned 70 degrees counter-clockwise seen from above |
| Brush | Body centre (0.1809, 0.0906, +0.027), bristles up, free on the tray floor against the tray's -x and +y inner walls; no attachment or stand |
| Clean plate | (0.255, 0.250), 190 mm diameter, 4 mm floor, estimated 14 mm rim from 2 to 16 mm above the counter; fixed |
| Backsplash | x = 0.560, centre y = 0, centre height 0.150; 14 x 600 x 300 mm |
| Faucet | Base (0.540, -0.180), 180 mm high, 80 mm spout; off |

Tray and plate are held down with four 25 mm squares of removable mounting tape each, outside every manipulation surface; they are fixed bodies in the simulation. Nothing is attached to the potato or the brush. Trials alternate layouts A and B; the screenshot shows A.

## Score

Score 0 if any of these happens:

- at any time, the potato falls more than 10 mm below the worktop (whole-trial video);
- at any time, the skin is torn over more than 5 mm or a patch over 10 mm is peeled (ruler after the trial; the whole video shows when);
- at the end, fewer than three windows are clean, or the release gate below fails.

Otherwise the score is the highest tier met at the end of the trial:

| Score | Result at the end | Measured with |
|---|---|---|
| 0.25 | at least 3 of the 12 windows clean | six registered photographs |
| 0.5 | at least 6 clean | six registered photographs |
| 0.75 | at least 9 clean; all 12 without the final placement also scores 0.75 | six registered photographs |
| 1.0 | all 12 clean; the potato entirely inside the plate with its centre within 40 mm of the plate centre; the brush outside the plate, on the tray floor or counter, its centre 105-200 mm from the plate centre | photographs and a ruler |

A window is clean when at most 2 of its 25 grid cells (5 x 5, about 2.4 mm each) still show any coating in its assigned photograph, that is at most about 10% of the window.

Release gate, for every nonzero tier: every hand and fingertip at least 30 mm from the potato and the brush; both objects still for 2 s; the potato resting on the tray floor, the plate floor or the counter; the brush resting on the tray floor or the counter. A brush left on the plate or on the tray wall scores 0. Video and a ruler establish the gate. The score is the state at the end of the trial, not the best earlier state. The two layouts are averaged.

In the simulation, still means speed below 10 mm/s and spin below 0.5 rad/s. Resting means the lowest point of the curved geometry is within 5 mm of that surface; for the brush, a native upward contact force above 0.25 N must also come from the tray floor or the counter. Outside the plate means the brush body and bristle collider boxes, at their actual pose, lie outside the plate's 190 mm circle.

## Sector map and photographs

Use `fixture/soil-map.svg`. x follows the potato's 95 mm axis; phi runs around x from +y toward +z. The twelve window centres are x = -24, 0, +24 mm crossed with phi = 0, 90, 180, 270 degrees. Each window spans x +/-6 mm and phi +/-11.09 degrees (about 10-12 mm along the curved skin), split into a 5 x 5 grid. The windows do not overlap. A blue dot at x = +40 mm, phi = 0 and a red dot at x = -40 mm, phi = 90 degrees register the axes.

Photograph the baseline and the end state from +x, -x, +y, -y, +z and -z beside a 1 mm grid. Register the two dots and the silhouette to the baseline. Assign each cell to the view most aligned with its surface normal and count it once; ties go to x, then y, then z. If a view is blocked, reposition the released specimen after the trial, without further cleaning, and use the registered opposite view. The simulation applies the same 23-of-25-cells rule to each window. The whole-trial video establishes damage, drops and release; end photographs alone cannot.

## Real-trial procedure

Every quantity below is fixed before the trial. The only judgement is whether coating is visible in a 2.4 mm grid cell. This procedure is proposed; no physical trial has been run, and the simulation does not calibrate the coating.

1. **Specimen.** Russet Burbank potatoes sold as U.S. No. 1 russets; record the variety on the bag. Select with a digital caliper: length 92-98 mm, width 62-68 mm, height 57-63 mm. Weigh on a 0.01 g scale with at least 300 g capacity (for example a 500 g x 0.01 g pocket scale), used for every weighing below: 180-220 g. Reject any with a cut, green patch, soft spot or a sprout over 2 mm. Use a fresh specimen for every trial. Marked specimens are not eaten.
2. **Marks.** Wash and towel-dry the specimen. With fine permanent markers put a 6 mm blue dot at x = +40 mm on the +y side and a 6 mm red dot at x = -40 mm on top. Mark the twelve window centres with a flexible ruler and a printed copy of `fixture/soil-map.svg`.
3. **Soil batch**, one per session: 5.00 g kaolin clay powder, 3.00 g Crayola Washable Kids' Paint (black) and 2.00 g tap water, stirred for 30 s. It dries to a dark grey coating that stands out against tan russet skin.
4. **Coating.** Cut a 30 x 30 mm stencil from 0.5 mm flexible PVC sheet with a punched 12 x 12 mm hole. Hold it on each window centre with tape, fill the hole level with soil using a 10 mm spatula, and lift it off. Weigh before and after: 1.30 +/- 0.25 g for all twelve windows (about 0.11 g each). Dry 20 min at 20-24 C until dry to the touch, then weigh again (m1).
5. **Batch check**, once per batch on a spare coated specimen, each on its own window: (a) rinse 10 s under cold running water and rub five times with a wet gloved fingertip; (b) rub the window five times, 25 mm each, on the dry tray floor at about 2 N (the specimen on a kitchen scale reading about 200 g more than its mass); (c) brush with the OXO brush at 1-5 N (the scale reading 100-500 g more) for 25 mm of total brush travel, as two 12.5 mm strokes. Checks (a) and (b) must leave at least 23 of 25 cells coated; check (c) must leave at most 2. If (a) or (b) fails, remake the batch with 4.00 g of paint; if (c) fails, with 2.00 g. Record the paint mass used. If the remade batch still fails, no scored trial is run with that kaolin and paint; the procedure stops there.
6. **Photographs.** Phone camera on a stand 200 mm away, two daylight LED lamps at 45 degrees, specimen on a 1 mm grid card. Six views, registered by the two dots, before and after the trial. Overlay the 5 x 5 grid on each window; a cell is dirty if any coating is visible in it.
7. **Weight check.** After the trial, dry the specimen 20 min and weigh it (m2). Record m1 - m2 beside the photo count as a consistency check; it does not change the score.
8. **Damage check.** Measure any skin tear or peeled patch with a ruler and review the whole video for drops.
9. **Reset.** Rinse the brush 10 s and shake it dry, wipe the tray, fit new nitrile finger covers on the gripper pads, wet the new specimen and the brush for 5 s, and place every object at its listed pose with a ruler and an angle template. The faucet stays off during the trial.

## Simulation model

The potato is a native convex rigid body with twelve axial divisions and eight broad cross-section faces. It keeps the 95 x 65 x 60 mm envelope; its radial profile is within 2.65 mm of the nominal ellipsoid. The broad, uneven faces approximate an irregular russet and let it rest on different sides. This is a declared geometry estimate; no specimen was scanned. The 75 g rubber brush body carries a 15 g bristle pad: a 72 x 42 x 17.5 mm collider on a 12 mm prismatic joint with a 1500 N/m spring and 12 N s/m damping. The pad is one rigid slider, so single bristles do not bend. Native contact gives equal reaction loads on the potato and the brush. Friction estimates (static/dynamic): potato 0.65/0.5, rubber body 0.8/0.65, bristles 0.3/0.25. Both bodies have 3/s angular damping as an estimated rolling loss. No weld, kinematic attachment or pose correction holds either object.

The soil is a declared state. A cell clears only after 25 mm of cumulative sliding of the bristles over it, with consecutive native contact readings between 1 and 5 N. Sliding is measured at the contact: the motion over the skin, in the potato frame, of the bristle point under the contact between 20 Hz samples. Rocking or rolling the potato on the bristles without slip clears nothing, and so does sliding slower than 10 mm/s. The rule has no minimum stroke length: a sub-millimetre back-and-forth faster than 10 mm/s still counts, although real bristle tips would bend rather than slide at that amplitude. The cell must also lie inside the actual 72 x 42 mm bristle rectangle, face the bristles, lie within 4.5 mm of the bristle plane, and be more than 5 mm from every sampled hand surface. Water, fingers, the tray floor, hovering and stationary pressing clear nothing, and soil never moves to another cell.

A bristle load over 12 N declares skin damage for the rest of the trial and shows a 12 mm pale patch. Overload and drop violations are latched after every 240 Hz physics step; missing or decimated history fails closed. Cleaning and hand clearance are sampled at 20 Hz. The force, travel and damage thresholds are estimates, not calibrated properties. The model does not claim tissue mechanics or food hygiene.

## Reference demo

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 --instruction "Clean the dirt from the potato and leave it in the clean dish, with the brush beside it." --scorer task_score --max-steps 3600 --epochs 2 --store-frames
```

Use `--policy dream` for Dream-1. Each trial has at most 180 simulated seconds; a scored run is two trials, A then B. `TASK_LOT=A` or `B` pins one layout. The layouts are fixed, with no jitter or seed dependence. `parts.py` supplies spawning and contact hooks, `lots.py` selects the layout and `tilt.py` sets scripted gripper tilts.

The reference uses only the left arm. It grips the potato from above, scrubs its underside over the brush, sets it down rolled 45 degrees, regrasps, rolls it 45 degrees more and scrubs the next face, four faces in all, then places it on the plate. The fingers close to 3.5 mm inside the declared flat width of whichever face is between them (65, 63.5 or 60 mm), so every face gets the same squeeze. Loaded passes slide the potato 40 mm along the bristles while rocking it 12 degrees each way along its length. The second and fourth faces add one 50 mm stroke per end at a constant 18-degree tilt to reach the end windows. Without a located brush, the path runs 3 mm toward the long tray wall behind the brush.

Between faces the reference parks the empty hand and uses the top camera's RGB image. It segments the visible potato at 320 x 176 pixels, projects its centroid and long axis through the published camera calibration, and rounds the position to 5 mm and the yaw to 5 degrees for the next grasp. It also fits the beige 72 x 42 mm bristle rectangle at 640 x 360 pixels, with a pinhole model, the known brush envelope and a support plane 1 mm above the counter. The next scrub is centred on that rectangle's centre (rounded to 5 mm) and turned to its heading (rounded to 5 degrees, capped at 10). The fitted roll, pitch and height are not used: the fit reports 5 degrees of pitch on the level reset image, and following its tilts moved the held potato sideways and pressed it outside the 1-5 N window. A missing image estimate, or a fit worse than 6 pixels, keeps the nominal segment and is logged. Each segment then runs open loop. No simulator object pose, force, joint or soil state selects an action; telemetry reads them only to audit scoring. This follows the guide's allowance for coarse poses visible to a camera. The plate target is biased 10 mm along +x for release slip; the potato must still meet the same 40 mm condition.

## How this differs from close tasks

- **TK-VT2J8N** (peel a cucumber in strips): a blade on a held body, its ends left untouched. Here every marked window is required, including those under the fingers and on the tray floor.
- **TK-4Z8LF9** (brush a battery post): the brushed part is fixed. Here the food is loose and rounded and the brush lies free on the tray floor.
- **TK-SANNN9** (scratch a flat card): one flat face, one tool stroke.
- **TK-EFZTFV** (brush a denture over a basin, pending): a loose object is brushed, but teeth under the holding grip are excluded from the required set. Here every grasp-covered and tray-covered window is required.
- **BEHAVIOR `clean_potatoes`** ([definition](https://behavior.stanford.edu/knowledgebase/tasks/clean_potatoes-0.html)): four potatoes are "no longer covered in dirt" as a symbolic state, with no window map, load window, skin-damage rule or plate placement.

The subject is not new. The difference claimed is covering all four sides of a loose convex food under a bounded brushing load, including the regions the grip and the tray hide. In the reference, the first grasp's scrub clears 3 of the 12 windows; the other 9 need the three turns and regrasps. An end grasp that leaves the windows exposed is allowed; regrasping is not claimed necessary for every strategy.

**Construct validity.** In the simulation only loaded sliding of the bristles cleans. In the real trial, the batch check rejects a coating that water, a fingertip or the tray floor would remove. Bristle reaction is native contact; an overload shows a pale damage patch and scores 0 for the rest of the trial. The skin-damage thresholds are estimates.

**One hand.** The reference uses one arm: the tray floor and walls hold the brush while the hand moves the potato over it. A second hand may hold the brush; the score is the same.

| Bypass | Scorer outcome |
|---|---|
| Put the dirty potato on the plate | 0 |
| Hover over the brush, or press without sliding | 0 |
| Rock or roll the potato on the bristles without sliding | 0 |
| Rub slower than 10 mm/s | 0 |
| Fingers, tray floor or water alone | 0 |
| Overload, then resume gentle cleaning | 0 for the rest of the trial |
| Drop below the worktop, then recover | 0 for the rest of the trial |
| Still holding an object at the end, or hand state unknown | 0 |
| Leave the brush on the plate or on the tray wall | 0 |
| Scrub only the original underside | At most 0.25 |
| Replay the fixed layouts from memory | Not ruled out; no generalization is claimed |

`near_misses.py` checks the score ladder, the 23-of-25 cell boundary, the 2 s hold, hovering, missing state, latched damage and drop, rubbing at zero force, below 1 N and above 5 N, stationary pressing, rocking and rolling without slip, sub-10 mm/s jitter, a positive 25 mm sliding control, the brush at its start pose on the tray floor, a brush lifted to the tray wall, and a broad hand face 0.5 mm away. These are synthetic scorer controls, not task runs.

## Evidence

Scripted reference on this source, two trials (A then B): A 1.0 and B 1.0, mean 1.0. In each layout all twelve windows were clean by 156 s, with 25 of 25 cells in every window. Peak bristle loads were 4.97 N (A) and 5.42 N (B), against the 12 N damage limit. The potato was released 11.8 mm (A) and 10.6 mm (B) from the plate centre. The brush stayed on the tray floor, 174.3 mm and 175.4 mm from the plate centre. Both hands were clear and both objects still for more than 2 s before each trial ended at 175 s. The first grasp's scrub cleared 3 windows in each layout; the other 9 needed the three turns and regrasps. `setup.png` and `demo.mp4` show layout A.

Dream-1 on the same source, unchanged instruction, two trials: A 0 and B 0, mean 0. Both trials ran to the 3,600-step limit with no refusal and no scorer error. In both, the robot pushed the potato across the tray floor (53 mm in A, 34 mm in B) but never pressed it on the bristles, so no window cleared; there was no damage or drop, and the brush did not move. `dream.mp4` shows layout A.

These are simulated results, not measured robot results.

## Licences

Source geometry is procedural; no maker meshes are redistributed. Code: MIT. Documentation and drawings: CC BY 4.0.
