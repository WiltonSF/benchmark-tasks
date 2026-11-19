## Fabrication-ready custom fixture G1

This construction uses the existing custom sliding-contact proxy, not an obtainable HIWIN bearing assembly. All dimensions below are **mm**, with mounting surface z=0, x along the rails and y across them. The rail, carriage and beam contact envelopes are unchanged. The linked drawings, hole tables and complete editable `fixture.scad` specify the parts; select `PART`, render and export a machining reference. Units are mm. Original CAD, SVG/PNG drawings and DXF assets are MIT; the construction prose is CC BY 4.0. No vendor CAD is included. The build and its friction/mass have not been physically measured.

### Stock and repeatable manufacture route

The economical route is cut stock and ordinary drilling/tapping/bench-mill operations, rather than a bespoke steel extrusion or a purchased recirculating bearing system. Order six 260 mm steel strip blanks, four acetal cheek blanks, two cap blanks, one 400 mm tube and the small fixings below from a local stockholder/machine shop using this cut list. Reuse a flat bench or a sacrificial fixture board with underside access. Its **upper mounting face is z=0**; base thickness and nuts sit below that datum and must not raise the exposed rail geometry relative to the chosen robot work surface.

| Part / quantity | Finished size and source/manufacture route |
|---|---|
| Track feet / 2 | 1018 or equivalent low-carbon steel, 250×12×3. Finish-machine a 260 mm blank from 1/8×1/2 in stock (3.175×12.7), e.g. [Online Metals item 4802](https://www.onlinemetals.com/en/buy/carbon-steel/0-125-x-0-5-carbon-steel-rectangle-bar-1018-cold-finish/pid/4802). Stock dimensions are not finished tolerances. |
| Track necks / 2 | Same stock/route, finish to 250×8×2. |
| Track heads / 2 | Same as feet,250×12×3, with flush fixing-head seats. |
| Carriage cheeks / 4 | POM-C acetal,45.4×9.25×10 bounding blank, integral lip as D2. [Curbell acetal](https://www.curbellplastics.com/materials/plastics/acetal/) is a stock-material source; this is an original profile, not a supplier bearing. |
| Carriage centre caps / 2 | POM-C,45.4×12.5×4.75, bolted between two cheeks. |
| Beam / 1 |20×20×400 aluminium square tube, nominal 2 wall, e.g. [6060 metric stock item 22467](https://www.onlinemetals.com/en/buy/aluminum/20mm-x-2mm-aluminum-square-tube-6060-metric-60-length/pid/22467). Request a 400 mm offcut/cut-length locally; the linked multipack is a dimensional source, not a low-cost purchase recommendation. A T-slot profile is not silently substituted for the smooth tube. |
| Internal nut blocks / 2 | Aluminium,15.5×12×6, two tappedM3 holes per block, at the carriage stations. |
| Ballast / 1 | Low-carbon steel,40×50×40; drill/tap twoM4 attachment holes. Outer envelope and placement as D4. |
| Stops / 2 | Low-carbon steel,10×30×20; adjustable mounting slots from D1. |
| Beam crush spacers / 2 | Aluminium or steel,Ø8 OD/Ø4.5 ID×16 long;90° entry reliefØ5.3 toØ4.5 over0.40 facing the countersunk wall. |
| Standard fasteners |6 M3 countersunk track bolts,4 M3 countersunk stop bolts,4 M3×12 countersunk beam bolts,8 M2×12 countersunk cheek screws,2 M4×30 countersunk ballast screws, matching nuts/washers for bench fixings,4Ø2×10 registration dowels. M2 screws use DIN965 head≤3.8; M3/M4 head seats allow ISO10642 heads up to 6.72/8.96 respectively. [Accu M3 dimensional example](https://www.accu.co.uk/countersunk-socket-head-screws/5430-SSK-M3-25-A2). Choose bench-fixing lengths for the measured support thickness with full nut engagement; the CAD does not invent a bench thickness. |

Supplier links establish stock/material or fastener dimensions, not stock availability, delivered price or a quote. No order or contact has been made. Labour/finishing dominates this route; no guaranteed total cost is claimed. Before ordering, a local shop can quote the same stock cut list and two-sided cheek operations. Do not replace the captured profile with an easy open slider to lower the cost.

### D1: steel track section, plan and mounting

```
End section, centred at each rail y=+/-170
                         y=-6                 +6
 z=8                     +---------------------+  head 12×3
 z=5                     +---+-------------+---+
                             | neck 8×2    |       edges y=+/-4
 z=3                     +---+-------------+---+
 z=0                     +---------------------+  foot 12×3
                         upper bench mounting datum

Plan: x=30 +-----------------------------------------------+ x=280
            fixing 55     dowel 105  fixing 155  dowel 205 fixing 255
      all holes on the rail centre-line; paired rails 340 apart
```

Each foot/neck/head strip is 250 long. Gang-align their centrelines; the neck leaves 2 mm shoulders each side. Drill fixing through-holesØ3.4 at strip-local x=25,125,225; drill/reamØ2 registration holes at 75,175. Pins are flush to 0.05 below rail top, never proud. Give the foot/neck registration holes Ø2.00..2.02 clearance; finish each head registration hole for 0.005..0.010 diametral interference relative to its measured Ø2 h6 pin. Press from above to the measured recess, rather than driving to a blind-hole bottom. The selected Ø2×10 pin extends about2 mm below z=0: drill the bench/base at those four registration locations Ø2.2×3 deep minimum. This clearance does not locate the rail and prevents pin bottoming; the head press fit retains the pin. Confirm retention and height after tightening. The CAD represents nominalØ2 bores; these fit requirements govern finishing. Countersink only the head strip at the fixing holes,90°, nominal mouthØ6.8; finish to the actual screw so its head is flush to 0.05 below z=8. All fixing/registration holes are omitted from the enclosing native collision solid, as disclosed.

Finished widths: foot/head 12.00±0.025, neck 8.00±0.025; thicknesses 3.00±0.025,2.00±0.025,3.00±0.025; lengths 250.00±0.10. Keep running edges straight within 0.03 over 250, flat/smooth and deburred, edge break≤0.10 on captured surfaces. These are manufacturing/inspection requirements, not measured stock tolerances. Mill/finish the stock to them; do not claim a saw-cut strip is automatically suitable.

Use one spacing jig/indicator to set rail centrelines 340.00±0.03 apart and parallel within 0.03 over 250. Bolt the stacks down and verify these dimensions again. Registration pins retain stack centring; clamping the strips loosely and allowing the 8 mm neck to drift is unacceptable.

The two stops have rear contact planes x=280, span y=rail centre±15 and z=0..20; their front/back thickness is 10 alongx. Mount with twoØ3.4×4.4 long slots alongx, centred at stop-local(x=5,y=±8). Countersink the slot mouths 90° to the received M3 heads (nominal 6.8 wide×7.8 long), flush below z=20. Set both contact planes against one straightedge at x=280; tighten without shifting. Require stop coplanarity≤0.02 and face squareness/flatness≤0.02 across 30×13 contact area. The 0.05 mm trial feeler must be able to test both corners; no rounded or resilient stop substitute.

### D2: captured acetal carriage section

```
Symmetric around its rail centre; all coordinates below absolute z above bench

 z=13    y=-13.5 +----------+------------+----------+ y=+13.5
                  cheek    | centre cap|  cheek
 z=8.25             +------+------------+------+
                    |       rail head          |
 z=5                |      y=-6 ... +6         |
 z=4.75             +--+                    +--+  upper lip faces
 z=3.25             +--+    neck y=+/-4     +--+  lower lip faces
 z=3       +-----------+                    +-----------+
                  lip inner edges y=+/-4.25
                  main cheek inner faces y=+/-6.25
```

This is a three-part bolted carriage; its union is the same 45.4×27×10 envelope/profile as the runtime. Each full-height cheek occupies y=6.25..13.5,z=3..13 (mirror for the other side) and has an **integral**2×1.5 toe projecting to y=4.25..6.25 at z=3.25..4.75. The centre cap fills y=-6.25..6.25,z=8.25..13. All three span x=-22.7..22.7. Do not glue a loose thin lip to a plain block.

Machine each cheek from its 45.4×9.25×10 blank: from the top remove the inner 2 mm strip down to z=4.75; flip on a parallel datum and remove 0.25 from that same inner 2 mm strip at the bottom, leaving the 1.5 mm toe. These are accessible straight milling cuts; no proprietary bearing/undercut extrusion is required. Finish the capture surfaces to±0.025 and leave edge breaks≤0.10; inspect thin toes for damage. Do not file the toes away to cure binding.

Each cheek has twoØ2.2 horizontal fixing holes, axes alongy, at x=±16,z=10.625. Countersink at its outer y=±13.5 face 90° to fit the received DIN965 M2 heads, maximum mouth 3.9; heads flush or below. The centre cap has matching M2 tapped holes,1.6 pilot,5 deep from each side. Use fourM2×12 screws per carriage: approximately 4.75 thread engagement after 7.25 cheek width. Seat the joints square without distorting the acetal; inspect the profile after tightening.

The cap has two verticalØ3.4 beam-fixing holes at x=±5,y=0. Countersink upward from its underside z=8.25,90°, nominal mouth 6.8, to seat M3 heads flush to 0.05 **above** the underside (recessed). No screw may protrude toward the rail head at z=8.0. Bolts are installed before the captured assembly slides onto the rail ends.

Nominal capture gaps are 0.25 at the rail-head sides/top and retaining-lip/neck/shoulder interfaces. Inspect with feelers and calipers: accept 0.20..0.30 before mounting; with full reset yaw±0.45°, verify no hard interference while still captured. The specified tolerances must be achieved, not inferred from the nominal drawing. The carriage can only be fitted from an open rail end and must not lift out once engaged.

### D3: rigid beam-to-carriage fixings

```
Cross section at each carriage station y=+/-170
 z=33  +--------------------+ 20×20 tube, nominal 2 wall
 z=31  |   hollow cavity    |
 z=21  | +----------------+ | internal aluminium nut block 15.5×12×6
 z=15  | +----M3----M3----+ | tapped holes x=+/-5
 z=13  +------|-----|-------+ tube underside / carriage top
 z=8.25       ^     ^         two M3×12 countersunk bolts from below cap
```

Drill the beam bottom wallØ3.4 at beam-local(x=±5,y=±170). Do not drill the upper wall for these holes. The two nut blocks sit on the tube's internal lower wall at z=15..21, centred at y=±170; tap their two holesM3 through,2.5 pilot. Slip them in from the open tube ends and hold position with the bolts before final seating. Bolts pass through cap 4.75 + tube wall 2, leaving approximately 5.25 engagement in the 6 mm nut block. Use the actual tube internal dimensions to inspect fit; nominal 16 mm cavity is not a guaranteed vendor measurement. Nut blocks may be machined to give 0.15..0.30 side clearance while retaining the stated external contact geometry. Relieve their lower longitudinal corners to clear the actual tube internal fillets; `NUT_CORNER_RELIEF=1.0` is a machining allowance, not a vendor radius. Inspect the received tube with the block/template. Increase relief only as needed, up to2 mm, retaining at least3 mm full thread engagement around eachM3 hole. If that cannot fit, select a tube with the same20×20 outer/2 wall specification that can accept these fixings; do not force/spread the tube or alter the external beam geometry. Record the actual fit and relief.

The tube outer section 20×20, length 400 and rail-centre spacing 340 remain unchanged. Its centre is z=23, underside 13. Fixture both carriages square to the tube with one rail-spacing jig before seating bolts. Inspect paired carriage centre pitch 340.00±0.03, relative yaw≤0.02° and top coplanarity≤0.03. No hinge, spring joint, loose slot or flexible strap connects the beam to its carriages. Validate rigidity in a separate setup check; dimensions alone are not a load measurement.

### D4: toolhead ballast and tube protection

Keep the steel outer block x=10..50,y=-25..25,z=13..53 relative to beam centrex 0 and its local y 0. Drill/tap M4,3.3 pilot,12 deep from x=10 at y=±15,z=23. Two M4×30 countersunk bolts enter from the opposite beam face x=-10 throughØ4.5 holes to the block. Finish those head seats 90°,nominal 9.2 mouth, to the actual heads flush with x=-10. PutØ8/Ø4.5×16 crush sleeves inside the tube, x=-8..8 at those axes, before tightening. The sleeve entry at x=-8 has a90° reliefØ5.3 toØ4.5 over0.40: the beam countersink is2.35 deep through a nominal2 mm wall and its cone isØ5.2 at that sleeve plane. Mark/inspect the relief orientation and finish the actual sleeve length to the received tube cavity. An unrelieved/reversed sleeve prevents flush head seating. Do not crush the hollow tube or replace the fixed ballast with a swinging mass.

The moving assembly mass 0.8 kg and friction 0.2 remain **simulation estimates**. The physical hollow tube, acetal, nut blocks and fixings will not reproduce the simulator's volume-weighted inertia exactly. Weigh and record the finished assembly and measure breakaway/sliding/off-centre binding before claiming hardware comparability; do not invent those results. Physical manufacturing holes/hollows/flush fasteners are omitted from native collision volumes. No native-runtime value or prior score is changed by this fabrication description.

### Assembly, inspection and reset

1. Produce and inspect strips/cheeks/caps/holes to the stated datums. Reject a proud screw, loose toe, incorrect neck centring or stock not finished to tolerance. Keep mating surfaces clean and dry; no lubricant is introduced to force a match to the uncalibrated friction coefficient.
2. Assemble the three-part carriages and bolt them to the tube using the internal nut blocks; fit ballast with crush sleeves. Verify both capture sections using the same go/no-go/feeler set and a 340 mm jig.
3. Align and fasten the two tracks/stops to the flat mounting surface. Slide the assembly onto the rear ends **before** installing any temporary rear keeper. Remove setup jigs before the trial; they cannot constrain yaw or drive the beam square.
4. Check capture, clean sliding over the working range, natural off-centre binding, no lift-out, fixed stops and rigid beam fixings. Record moving mass, measured force traces, gap/height results and tool resolution. These setup checks have not been performed. The CAD is not evidence that hardware succeeded.
5. Reset the beam origin to A:(216,0,23),yaw-0.45°, or B:(205,0,23),yaw+0.45°, with both lips captured. Allow the 0.25 mm upper running clearance to settle naturally. A pin/template/ruler may set the start but must be removed. The first settled observation defines travel baseline as in the runtime; do not wedge a carriage in a square start.
6. Physical scoring retains the existing movement, engagement, stop-gap, release and stillness checks. Use a 0.05 mm feeler at **both face corners of both carriages**, a dial indicator for seated height, ruler/caliper/video for travel and lateral registration, and the stated independent finger-force verification. Manufacturing acceptance is not a replacement for any scoring condition.

### Editable CAD and drawing files

The complete editable CAD is [fixture.scad](fixture.scad). Set PART to the named component, render and export a machining reference in millimetres. Threads, finish, tolerances and assembly inspections remain governed by the notes above. No OpenSCAD render or manufactured result is claimed.

Dimensioned vector sheets: [G1 tracks](G1-tracks.svg), [G2 carriages](G2-carriage.svg), [G3 beam fixings](G3-beam-fixings.svg), [G4 assembly](G4-assembly.svg). [Cut list](cut-list.csv) and matching millimetre DXF profiles are included in this folder. The stop DXF locates slot centres; fixture.scad and D1 define the complete rounded slots/countersinks.
