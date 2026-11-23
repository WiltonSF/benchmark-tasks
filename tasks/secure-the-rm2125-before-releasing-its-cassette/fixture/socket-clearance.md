# Frame fitting clearance, fabrication revision7

The frame keeps the native tube axes. The first corner uses Y/Z bores, the second X/Y bores. The third X/Z coupling is integral with the spring bridge; a rotated cube socket must not be used there.

| Connection at reset, mm | Printed envelope, mm | Bores and fasteners |
|---|---|---|
|First axis(339,94,243.25)|X323..348,Y82..110,Z227.25..259.25|13.1 Y/Z; two X-directed M4 x30 DIN965 countersunk screws, at local(Y,Z)=(−7.5,0),(0,−9)|
|Second axis(339,−122,243.25)|X323..348,Y−138..−106,Z227.25..259.25|13.1 X/Y; two Z-directed M4 x40 DIN912 bolts at local(X,Y)=(−9,0),(0,9)|
|Third axis(294.5,−122,243.25)|X277.5..311.5,Y−127..−106,Z227.25..259.25|13.7 X/Z saddles; Z-directed M4 x40 at local(X,Y)=(9,26), X-directed M4 x40 at local(Y,Z)=(21,26.3); Z coordinates here are relative to closed arm top|

All fittings move vertically with the head by−44.25..+14.75 mm from reset. The housing begins at X360; the first two fittings end at X348. The fixed upper brace starts at X354; the first fitting ends at X348. Its Y extent82..110 stops1.03 mm before the guide envelope at Y111.03. Its countersinks open at local X+9 with8.4 mm diameter and2.05 mm depth. Fit both screw heads flush, with washers/nuts on the −X face. A30 mm countersunk screw passes the25 mm block and leaves5 mm for washer/nut/thread; use the specified countersunk heads. The second fitting has space above/below for its Z fasteners. The first fitting retains2.45 mm minimum wall beside the13.1 mm bore, before the intentional countersinks.

The third coupling's open −Y face begins at Y−127, leaving1 mm before the roof/front cheek's Y−128 face. Its two4 x8 x34.8 mm ribs reach from the bridge foot into the coupling. They are part of the same print and remain2 mm clear of the cheek/roof. The old foot screw holes are removed so they do not sever these ribs. Its X/Z13.7 mm saddle cuts continue through the nearby foot/cheek as needed for12.7 mm tubing, giving0.5 mm radial clearance. The guide bores, spring seats, roller and clamp geometry are unchanged. No rotated cube, separate foot coupling or glue joint is needed at this corner.

Use DIN125 M4 washers with9 mm OD and M4 nuts at the frame bolts. The four M4 x40 DIN912 screws cross32 mm bodies and leave8 mm for washer/nut/thread; their7 mm diameter,4 mm tall heads stay clear of the bridge. The third-coupling bolt axes are at world Y−119 and−124,5 mm apart, so their4 mm shanks have1 mm separation. The X bolt crosses a34 mm body and leaves6 mm for washer/nut/thread. Its head/nut are outside X277.5..311.5, with at least0.5 mm clearance from the bridge sides X278..311, even where washers project past the fitting’s open rear edge. The Z bolt crosses32 mm and leaves8 mm for washer/nut/thread. Both holes remain within the12.7 mm tube cross-sections at their3 mm and−2 mm transverse offsets. Align the tube axes, miter their ends, drill the tubes through the4.3 mm printed guides, then bolt them. The frame's first two bores give0.4 mm diametral allowance around12.7 mm tube. All hardware is included in the stated fastener budget allowance.

The envelope check now includes all three fitting bodies, integral ribs, heads, shanks, nuts and washers against every fixed native body (including both guide braces), plus the co-moving bridge roof/cheeks. Run `python fixture/clearance_check.py`; its result and source hash are also recorded in `clearance_check.json`. This is analytical clearance, not STL export, a print, a load test or a robot execution with printed fittings. Printed fittings and fasteners are not separate native colliders; task.py, all native forces, scoring and arm plans remain frozen. Before enrolment, check the complete assembled stroke, loaded frame rigidity and robot access; reject any failed fitting.

## Rerouted frame and pad connection

The former rail at x350 intersected fixed guides and the upper brace. The current upper rail and upright axes are x339,11 mm forward. The cross-member is44.5 mm between x294.5 and339; cut a51 mm rough blank to allow its outer miter tip. The down-member needs a42 mm rough blank for its34.8 mm end/axis span and upper miter tip. The277 mm Y member stays continuous; the upright and cross-member have fishmouth ends at their T junctions. Physical12.7 mm tube has1.68 mm clearance before guide envelope x347.03 and8.65 mm before brace x354 throughout59 mm vertical travel. Native box surfaces also clear. The pad bore at(339,94,C0−16) is fully contained: x332.45..345.55 inside pad x315..355, so no side cap is used. An M4 x50 cross-bolt along X passes through the pad and upright, with washers/nut outside the pad.

These are analytical clearances complementing the current native qualification recorded in the main README; no hardware print/load test is claimed. The clearance probe includes the actual upper rail, upright, cross-member and down-member in addition to fitting/hardware envelopes.

## Pad guide relief and bolt washer

The guide bore at(351,115) continues through the mounting pad:8.3 mm diameter over80 mm, centred atC0−48, spanningC0−88..C0−8. The pad spansC0−22..C0−10, so the relief exits2 mm beyond it. With7.94 mm guide stock, radial clearance is0.18 mm; it is a deliberate bore, not solid overlapping material. The other guide lies outside this pad. The native slider is the guide constraint; native external box geometry does not explicitly tessellate the small guide bores.

The M4x50 bolt's head-side washer sits on a10 mm-diameter,1 mm-deep flat seat in the +X pad face. Washer thickness0.8 mm and head height4 mm give maximumX358.8,1.2 mm before the shell. Include both in envelopes; a flush seat without the recess would leave only0.2 mm.
