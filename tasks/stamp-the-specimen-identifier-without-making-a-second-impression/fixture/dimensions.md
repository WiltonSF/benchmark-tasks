# Fixture and enrollment, version1

All dimensions millimetres; bench top z=0. Taytools465968 drill-press vise ($34.99 quoted), native closed envelope234.95x146.05x53.975mm;76.2mm jaws,28.575mm jaw height,76.2mm maximum opening. Center x225,y0. Model base235x146x10, solid steel backing100x50x38 fromz10to48. Brass50x50x3,z48to51; center x225(A) or243(B). Jaw faces y+/-25,76.2mm wide x29high,z25to54. Four M8 bolts at x+/-100,y+/-60 through existing11.1mm-wide mounting slots and bench, washers and nuts below. Screw axis y atz32,140mm rod and75mm cross handle; internal casting and screw details estimated. Operator inserts the ready steel backing and coupon, then presets jaws; operator presets jaws, no robot screw turning. Jaw detail is an estimate, not maker CAD. Trial rejects stock displacement>3mm or tilt>5deg. Check the actual jaws hold stock before enrollment.

Punch stands32x32x4; centers(130,140),(205,140),(280,140). Upright punch centersz40.5125. Full73.025mm length with12.7square shank;9.525mm Gothic characters.90g individual mass estimated from steel volume minus taper. Hammer grip center(150,-200,110); head center+50mm along y. Maker dimensions A281.94mm overall, B83.82mm head length, C25.4mm head width, D248.92mm handle, flat face22.86mm, peen20.32mm, collar20.32mm. Native head extends41.91mm below grip and41.91mm above; hickory handle grip25x22mm cross-section remains a photograph estimate. Grip50mm from head axis; exact longitudinal taper/eye detail estimated within the published envelope. Enroll actual dimensions and adapt cradle, never scale tool. Hammer whole mass344.73g sourced. Cradle65x245 base10mm thick, two20x40 supports. Brass mass63.75g at8500kg/m^3.

Procurement: TEKTON HMA51108 $27 maker quote; Young Bros08091 complete figure set $46.99 Zoro quote; use only1/2/3 and store unused stamps outside the trial;75-100mm Taytools465968 vise$34.99 quote; annealed brass coupon$15 allowance; steel backing$15 allowance; rule and tray$15 allowance; contingency$40. Approximately$194 before tax/shipping. Allowances are estimates, not checked retail quotes.

Vise: https://taytools.com/taytools-3-inch-drill-press-vise
Hammer drawing: https://images.tekton.com/assets/ball-peen-hammer-spec-table.jpg
Maker: https://www.tekton.com/8-oz-ball-peen-hammer-hma51108
Punch maker: https://www.youngbrosstampworks.com/machine-made/
Method: https://www.youngbrosstampworks.com/using-stamps/
Digit2: https://www.zoro.com/young-bros-stamp-works-steeltypecharacter-individualcharacter-08091-2/i/G608841996/

Punch start character yaws: -20deg, +20deg, -30deg about upright z (1,2,3). Orange thumb stripe and top digit labels are visual geometry. All other starts are identical across layouts. These replace the proposal rotated-vise layout: rigid coupon translation18mm avoids moving any articulated assembly at reset. Both arrangements use actual vise/backing contacts.

Full-set procurement (resolves individual1/3 quotes without changing stamps): https://www.zoro.com/young-bros-stamp-works-hand-stamp-kit-number-set-08091/i/G4599725/

Hammer cradle repair, version2: back support40x20x12mm at localy=-100mm; front support40x12x12mm at localy=+78mm; both tops11mm below grip axis. The front block stops before the20.32mm steel collar. Base65x245x10mm, centre localy=-20mm,z=-33mm. The complete hammer COM is at[0, 74.9, -3.3]mm approximately; exact mass distribution is in task.py. This replaces the unstable+50mm front block; no trial starts from the tipped pose.
Blue ink crosses on coupon identify the three target centres; draw them with a fine marker using a reusable ruler before each trial (seconds, not drying/curing). Blue ink is excluded from dark-groove inspection. Work card65x35x1mm at(90,0,1)mm above bench, displaying123;120x14x1mm rule at(120,-84,1)mm.

Final choke grip shifts the complete unchanged hammer geometry70mm along its handle relative to the body origin. The head is localy=+50mm; the native tool remains281.94mm long. Hammer cradle base centre localy=-90mm; rear block localy=-170mm and front block localy=+8mm, width12mm alongy; all support tops11mm below the grip axis. This supersedes the version2 +78mm-front cradle, which applied to the120mm-from-head grasp.

Frozen final layout: vise centrex225mm; coupon centrex225(A)/243(B)mm. Earlier x290/308 and265/283 positions belong only to historical traces. Work-card centrex60mm. Full current shape geometry and both layouts are in layout-v1.json; its source SHA256 matches the frozen native build. Punch pinches are6mm above the body centre, approach45deg toward+x at pickup and90deg at the strike. These poses are implementations under failed qualification, not validated hardware feasibility.

Version3 (supersedes the punch-stand, grip and frozen-layout notes above): punch holder bases32x32x4mm at(340,90),(340,160),(340,230)mm, each with a16mm square socket16mm deep (four4mm walls) turned to its punch start yaw (-20,+20,-30deg). The hammer eye is modelled as the maker head width C25.4mm square section; the robot pinches it across the eye directly above the face. Vise and coupon positions are unchanged (x225 A, x243 B). The punch pinch is2mm above the punch body centre with a constant75degree wrist lean.
