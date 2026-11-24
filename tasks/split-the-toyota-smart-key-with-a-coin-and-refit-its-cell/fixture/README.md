# Fixture and enrollment, revision 7

Units mm unless stated. No case half, screwdriver or cell is attached to a support or robot; the two case halves connect only through the modeled snaps while engaged. The left hand lifts the case fully clear of supports for the loaded split: lowest point >5 mm above the tallest support (stand top 65.5 mm), also >50 mm above table. It then returns the base onto the stand with its grip maintained for cover/cell work; no case attachment. The passive supports only present parts for grasping and carry released parts. Stock wood/acrylic blocks are made once and reused. Reset target under two minutes is not measured.

## Sources and estimated figures

One original HYQ14FBA key was measured Nov 23 using calipers, ruler, toothpick probe and a kitchen scale (1 g). Readings are for that one key, not a five-shell enrollment or aftermarket-shell validation. Sources refreshed Nov 24:

- [Stallion R-SHELL-79](https://stallionkeys.com/products/2012-2020-toyota-3-button-smart-key-shell-hyq14fba-aftermarket): USD 8, four buttons, empty aftermarket HYQ14FBA-pattern shell. Trunk SKU shows backorder; confirm availability. No published notch, snap or internal seat dimensions were found on this page. Separate readings below come from an original key with the same FCC ID, not this aftermarket shell. Reject a shell whose clip positions differ from 5.2 mm from each end or whose notch differs from 7 mm width / 2.6 mm depth.
- [Toyota OM33A28U p 382](https://techinfo.toyota.com/t3Portal/document/om-s/OM33A28U/pdf/OM33A28U.pdf?locale=en&siid=1733337255502&sisuffix=ff): protected flat screwdriver and removable cover; small flat screwdriver for depleted cell; CR2032 positive face up. It does not establish the aftermarket shell's notch or selected blade fit.
- [Wera 05118012001 maker datasheet](https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05118012001.pdf): 0.6x3.5x80 blade, 97 handle, 177x13x13 overall, 20 g. [RS71410342](https://us.rs-online.com/product/wera-tools/05118012001/71410342/): USD 6.25, page has conflicting in-stock/backorder notices.
- [Energizer CR2032](https://energizer.com/wp-content/uploads/2025/01/cr2032.pdf): 20 diameter, 3.2 thick, 3 g.
- [I2RT SDK](https://doc.i2rt.com/products/yam#gripper-force-limiting): 50 N default stall-force control limit. This is not a hardware force rating. Use 50 N as the simulator diagnostic comparison, pending physical calibration. [YAM linear 4310 model documentation](https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/i2rt_yam/README.md), derived from I2RT SDKv1.3.6, describes a 50 N per-pad hardware force limit; no independent maximum-rating datasheet is claimed.

| Named constant | Value and basis | Enrollment measurement |
| --- | --- | --- |
| KEY_MODEL | HYQ14FBA label read on one key; R-SHELL-79 is the proposed replacement specimen | Photograph label/SKU, button count, vehicle model/year |
| CASE_L/W/T | 71 length measured without emergency key; 40/15 width/thickness estimated from proportions and cell clearance | Calipers on five closed shells, emergency blade removed |
| KEY_MASS / CASE_MASS | 30.4 g measured key mass; model assumes cell included: 27.4 g halves/board plus 3 g cell. Half allocation 15.4 base/12 cover is estimated | Kitchen scale, each half and assembled; confirm cell inclusion and emergency-blade exclusion in the reading |
| COVER_DETACHES | True, two fully separate halves observed on measured key | Photograph/video both loose halves; reject hinged specimen |
| SNAP_COUNT / SNAP_X | Two clips measured 5.2 mm from each end; x=-30.3/+30.3 for 71 mm length | Count and locate actual clips from inner photographs and ruler |
| SNAP_FORCE / SNAP_TRAVEL | 10 N at 1.2 mm, estimates. Assumed thumb-sized snap load per end and ABS lip deflection. Native stiffness 8,333 N/m, damping 2 N s/m | Instrument opening force/torque and displacement on each shell; three repeats, record lever length |
| Opening torque/work | Recorded diagnostics, no coin/hinge torque threshold; model now has fully separating snap halves | Tool handle pull force at measured lever length, three repeats; record displacement and torque |
| CLOSE_FORCE | Five centre scale readings 711/864/923/843/799 g; mean 828 g = 8.1 N, range 7.0-9.1 N. Retained 12 N gate/6 N per end is conservative, not a measured force | Kitchen-scale peak at the centre, five repeats and press location; record any end-press comparison separately |
| CLOSED_GAP | 0.5, grading tolerance | Feeler gauge/ruler photographs at all four corners before inspection |
| NOTCH_W / NOTCH_DEPTH | 7 / 2.6 measured by calipers and toothpick/ruler | Calipers and depth probe/toothpick with ruler |
| NOTCH_GAP / NOTCH_MOUTH | 1.2 / 2.4, estimates: clearance for 0.6 blade and protective wrap, flared entry only | Measure thickest entering gauge; photograph notch; record wrap thickness and actual insertion depth |
| TOOL_BLADE_T/W | 0.6/3.5, Wera maker dimensions | Calipers; photograph SKU; include protective wrap in fit test |
| TOOL_LEN/HANDLE_LEN/SHAFT_D | 177/97/4, Wera maker dimensions | Ruler, check unmodified stock tool |
| TOOL_FLAT_LEN | 10, design estimate for the flattened tip transition; not a maker axial dimension | Check that the actual flat land spans the protected insertion depth; replace geometry if it does not |
| TOOL_HANDLE_FLATS / TOOL_MASS | 13 maximum envelope / 20 g sourced; uniform hex cross-section is an approximation | Calipers at grasp location and scale |
| CELL_RING_ID/H / SEAT | 21 / 1.6; centre x=12, z=-2.3 in case frame, estimates: 0.5 radial clearance around 20 mm disc on inert board | Photograph retention, invert opened case, record whether cell falls or needs prying; measure seat and positive-face orientation |
| CELL_RETAINED | Full perimeter pocket; resists lateral slide, gravity can remove on inversion. Unmeasured, not asserted equivalent to OEM clips | If cell requires prying, replace this approximation and demo before physical equivalence is claimed |
| Board | 46x26x1.6 inert rectangle, estimate to fit shell and 20 mm cell | Measure actual board and access; record whether board must be lifted |
| Friction | case 0.7/0.55, tool 0.8/0.6, cells 0.3/0.25 static/dynamic; estimates for dry polymer/metal contacts | Tilt/slip tests with actual finger pads; no calibrated coefficient claimed |

A small flat screwdriver and quarter/penny/dime all entered the measured 7 mm slot and opened it by twisting. Interior slot gap and mouth were not measured; a coin entering a flared mouth does not establish the 1.2 mm interior-gap estimate. The screwdriver was not identified as this Wera SKU, so wrapped-Wera fit still needs enrollment. Toyota supports the tool class only. Enroll a protected blade without force; record the actual width, thickness and depth. Do not enlarge a real notch to match the model. New readings replace named task.py constants and require fresh physics evidence if geometry or force changes.

## Nominal layout and reusable supports

World x away from robot bases, y left, z above table. Case local x maps to world+y; local y maps to world-x.

- Mat 440x500x1, centre(320, 0).
- Case seam at(340, 0, 73). Base thickness 7.5 below seam, cover 7.5 above. Long axis y, notch at y=-35.5. Stand centred x=340, y=-13.5: 30x39 plan, top 65.5. Fix stand to mat with removable tape. The far base grip overhangs it; no case attachment.
- Cover rest centre(400,-60), base 30x64x2, pedestal 24x56, top 30. Fix with removable tape. Cover is placed loosely over it.
- Screwdriver grasp origin A(290,-125, 50), B(310,-125, 50), tip 132 mm toward +y. Ballasted 150 g rest base 30x160x2 centred y=-55; three 20x8 support blocks under handle y=-157/-93 and shaft y=-30. Block tops set the handle/shaft axis at 50. These are passive supports, not a clamp.
- New-cell tray A centre(440,-130), B(460,-130), 64x64x2 base, 6-high walls, 8x8x32 pedestal starting 2 above tray origin. Cell nominal centre 35.6 above table, positive up. Ballast tray 150 g. Gravity settles tray onto mat.
- Spent tray centre(400, 100), 100x80x2 base, inside 94x74, 3-thick/ 15-high walls. Tray origin 1 above table. Fix with removable tape. Old cell must finish with its centre inside the inner footprint.

The obsolete coin/hinge drawings have been removed from the trial package. task.py and this revision 7 document define the current scene.

## Reset and endpoint inspection

One-time prepare and photograph the blocks, tray and mat with ruler. Mark red/green positive faces on the cells without changing thickness. Before each trial place old red cell positive up in the pocket and snap the case shut, emergency blade removed; return all free parts to selected lot. Empty spent tray. Confirm no worn/broken clip, no cell trapped on a rim, and both ends flush.

After the final closed-case photo, the operator reopens using the same screwdriver and takes a top photograph and two orthogonal side photographs of the replacement green positive face with a ruler. Centre must lie within 2 mm of the seat centre, height within 1 mm and face normal within 12 degrees. Preserve the earlier photo proving closed seam<=0.5 mm. Optional working-key button/LED recording is an extra check, not scored. No electronics/RF operation is simulated. Reset worn specimens from the enrolled pool rather than altering the latch force.
