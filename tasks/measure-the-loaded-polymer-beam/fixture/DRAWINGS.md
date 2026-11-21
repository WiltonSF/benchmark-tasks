# Apparatus and mount schedule, version 3

All dimensions in mm. World X points away from the arm bases, Y to the left, Z up from the bench surface. The commercial components are at full scale. This is a proposed, reproducible assembly, not a verified OEM fit.

| Component | Size and location | Source and status |
|---|---|---|
| ES05 base | 162.1 X by 99.1 Y by 20 Z; load axis X320, Y0; base centre X359.55 | Maker footprint; 20 mm height estimated from the drawing |
| ES05 column | Ø18, axis X412, Y0; from Z20 to the 355.6 maximum top | 92 mm load-axis offset and 355.6 mm from the maker drawing; diameter estimated |
| ES05 head | 60 x 60 x 75 block on the column, centre X395, Y0, Z242.5 | Estimated; the maker states the lever mechanism can be repositioned along the column |
| ES05 lever | Hub X380, Y-70, Z242.5; 105 mm hub to knob centre; knob Ø26 x 28, centre at rest X299.6, Y-70, Z310.0; rest 40° above horizontal toward the operator | Scaled from the maker drawing (data sheet page 2, using the 162.1 mm base as the scale) |
| AC1060 plate | 70.1 x 82.8 x 18.5; #10-32 holes on a 12.7 mm grid, central 5/16-18 | Maker drawing; long side along Y |
| G1095 lower fixture | 211.6 long (Y), 57.2 wide (X), 87.6 maximum height; Ø10 x 51.4 rollers 150 apart | Maker drawing |
| G1097 upper anvil | 57.2 wide (X), 33 thick (Y), 49.5 high; Ø10 roller | Maker drawing |
| M5-20 force gauge | 100 N x 0.02 N; #10-32 male output stud; 145 x 71.1 x 35 body envelope | Maker capacity and resolution; envelope nominal |
| Deflection indicator | Mitutoyo 1044A-15 low-force dial indicator: 0.01 mm graduation, 5 mm range, Ø8 mm stem, 0.1 to 0.4 N measuring force, for use with the contact point downward only; contact down on the pad of rocker R1, spindle axis X256, Y0 | Maker data sheet (Series 1 compact dial indicators); mount drawn here as an estimate |
| Rocker R1 | Pivot X288, Y0, about Z118; rounded tip at X320 under the bar's centre; pad at X256 | Drawn below; estimate |
| TIVAR 1000 bar | 200 long (Y) x 25 wide (X) x 10 thick; 46.5 g at 0.93 g/cm³ | Maker sheet: density and 87 ksi (about 600 MPa) flexural modulus |
| Two specimen cradles | Machined aluminium, 70 x 220 x 10 mm base, two 40 x 25 x 20 mm saddles at ±75 mm; bar centres X200 (far, layout A) and X110 (near, layout B), Y130, Z35 | Drawn dimensions |
| Bench display (simulation only) | 190 x 90 x 6, lying flat, centre X395, Y-155; force (N) top row, deflection (mm) bottom row | Repeats the two readings because the simulated instrument faces are blank; in a real trial the robot reads the instruments' own screens |

Eye-end adapter A1: 2024 aluminium plate 70.1 X by 82.8 Y by 12 thick, four Ø5.2 holes on a 50.8 x 50.8 square, central Ø8.4 hole for the 5/16-18 fastener, Ø14 x 6 counterbore underneath. The lower fixture's eye end is held in a split 303 stainless clevis, 30 x 25 x 25, by an 8 mm pin through Ø8.2 ears. The plate and clevis sit inside the bottom 12 mm of the 87.6 mm fixture envelope; they do not add height to the stack. Check the ear gap against the real fixture eye before machining. It must not rotate under 50 N; witness marks and a ruler check that the apparatus moves less than 5 mm.

Eye-end adapter A2: #10-32 female coupler, Ø12 x 18, threaded 12 deep, joining the M5-20 stud to a matching 303 stainless clevis (30 x 25 x 20) that holds the G1097 eye on an 8 mm pin. Check the eye bore before machining; adjust only the bore and the spacer length. Neither adapter is a robot action.

Rocker R1 (both set-ups): 6061 aluminium bar 76 long (X) x 6 wide (Y) x 10 high (Z), lying along X under the bar's centre. Ø3.0 pivot hole across its 6 mm width at mid-length, 5 below its top face, on a Ø3 x 14 steel dowel pin in a two-ear clevis (ears 3 thick, 6.5 apart). Tip: an M3 button-head screw with jam nut in a tapped hole in the top face 32.0 ± 0.1 from the pivot, head up, touching the bar's underside at mid-span (top face about Z123 in the named set). Pad: the top face 32.0 ± 0.1 from the pivot on the other side, under the dial's contact point. A matching M3 screw and nut under the pad end balance the tip screw, so the rocker rests level on its pivot with no dial. Stop: an M3 set screw under the pad end limits its fall so that, with no bar, the tip stands 0.5 mm above the line of the roller tops. With the bar resting, the tip is pressed down 0.5 mm and the pad lifts the dial spindle about 0.5 mm into its 5 mm travel; zero the dial there. At 1 mm and 2 mm of bend the spindle is about 1.5 mm and 2.5 mm in, and the dial's end is reached only past a 4.5 mm bend. Through the 1:1 rocker the bar feels the dial's measuring force (0.4 N or less) as an upward push at the centre, always below its 0.46 N weight, so the bar stays on both rollers. The arm tolerance keeps the ratio within 1%.

Bracket C1 (named set): an aluminium bracket clamped to the front face of the G1095 bar (X291.4) that rises above the fixture top to carry the R1 clevis at X288 and, above the pad, an Ø8 split clamp for the dial's stem with the dial face toward the arms. Because R1 and the dial are both held by the lower fixture, the reading is the bar's centre relative to the supports. The simulation draws the pick-up schematically as a block and pin under the bar's centre and models neither R1, the dial nor its force. Check C1 against the real fixture before a trial.

Lever: the head is set on the column so the nose rests 6 mm above the bar with the lever on its rest stop. The nose meets the bar about 19° below rest; the 2 mm reading is near 25°, with the knob centre near X278, Z268. The return spring (0.3 N m/rad, 0.3 N m preload at the rest stop), damping (0.05 N m s/rad) and the 18 mm/rad pinion (38 mm over about 121° of lever) are estimates. Check hub position, lever length and rest angle on the real stand before a trial.

Checks before a real trial: span 150.0 ± 0.5 mm; both rollers at the same height; the 200 mm bar clears the column 92 mm behind the load axis; unloaded bar centre 131.1 mm above the bench; gauge and anvil stack within the 179 mm clearance; a 45 N test through the lever with hands clear of the roller pinch; the gauge screen, the dial and both roller witness marks visible on video. No purchase or measured assembly is claimed.

## Lower-cost real set (L1 to L6)

Same frame as above, with X_c the nose axis. Prices and sources are in the README.

| Part | Size and location | Status |
|---|---|---|
| Press | VEVOR 3-ton manual arbor press, plain-lever model (not the ratchet-and-handwheel model): 12.2 in (310) maximum height, 0.5 in ram bore, 6.5 in anvil plate; bolted to the bench | Retailer listing |
| L1 base plate | Steel plate 12.7 (Z) x 152 (X) x 203 (Y), long side along Y, lying on the anvil plate, centred in Y and set forward in X (X_c-114 to X_c+38) with the nose axis X_c on the ram axis; held to the press base with two C-clamps. Four Ø6.6 clearance holes for L2 at Y ±75.0, X_c ±19, counterbored from below so 1/4-20 button-head screws sit flush and go up into 80/20 T-nuts in each extrusion's bottom slot; two M4 tapped holes for the R1 clevis at X_c-32 | Drawn estimate |
| L2 roller supports | Two 80/20 1010 (25.4 square) pieces, 76.2 long, along X, centred on Y ±75.0 (top-slot centres 150.0 ± 0.5 apart, set by caliper before tightening), bolted from below into T-nuts; 80/20 end caps on both ends keep the pins from sliding out | Maker profile |
| L3 rollers | Two Ø10 x 70 hardened dowel pins lying along the top T-slots of L2; the slot lips locate each pin across the span. Pin tops about 9 above L2, about 47 above the anvil plate; the bar's underside about 34 above L1 | Drawn estimate; level both tops within 0.1 |
| R1 and dial | R1 as above on a clevis on L1, pivot at X_c-32; dial on a WEN magnetic base on L1 (about X_c-100 to X_c-45, Y +20 to +80), arm over the pad at X_c-64, with a 3/8 in to Ø8 stem sleeve | Retailer listing; drawn estimate |
| L4 nose | Ø10 x 40 hardened dowel pin along X, held by two M3 set screws in a 10 wide x 5 deep slot along the bottom of a 6061 block 20 x 20 x 40, so half the pin projects; block tapped at its top centre for the gauge shaft thread | Drawn estimate |
| Gauge | AMF-100 100 N digital push-pull gauge, screen toward the arms, shaft down on the nose axis | Retailer listing; body length to be measured |
| L5 gauge bracket | 6061 angle 2 x 2 x 1/4 in, 76 long. The horizontal leg lies under the ram end, held by a 1/2-13 x 2 in hex bolt whose plain shank sits in the 0.5 in ram bore (measure the bore; use an M12 bolt if it is 12 mm) and is retained by the ram's drift set screw (fit one if the ram has none); the vertical leg is drilled for the gauge's rear mounting screws, placed so the gauge shaft lies on the ram axis under the bolt | Drawn estimate |
| L6 lever stop | Hardwood or aluminium block about 50 x 50 x 100, clamped under the handle's path and set so the handle lands on it at 3.5 mm of bend (about 62 N by the declared stiffness) | Set on the real press |

Stack on the anvil plate: L1 12.7 + L2 25.4 + pin about 9 + bar 10 + 6 gap + L4 25 (block plus the projecting half pin) + gauge shaft and body about 175 + L5 6.4 + bolt head about 8, about 278, inside the 310 maximum height. Set the ram's gib screws so the ram, gauge and nose stay where the lever leaves them (the press has no return spring). Checks before a lower-cost trial: pin centres 150.0 ± 0.5 apart by caliper; pin tops level within 0.1; nose parallel to the pins within 0.5 over the bar's 25 mm width and centred over the span within 0.5; the nose 6 mm above the resting bar (a 6 mm drill blank just slides out under it); the dial about 0.5 mm into its travel with the bar resting; L6 stops the handle at 3.5 mm of bend; a 45 N test with hands clear of the nose and rollers.
