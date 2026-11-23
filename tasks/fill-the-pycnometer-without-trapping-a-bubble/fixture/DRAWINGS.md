# Fixture v7, dimensions in millimetres

Current drawing:fixture-v7.svg;current model manifest:model-v9.json (contact/force-established wet-seal proxy,normalized pocket pinning,covered-pad contact footprint and capillary drainage;v7 geometry unchanged). V2 has superseded post spacing;use the21.4mm inner radius specified here. The v1 drawing is retained as the superseded shelf-only layout.

All dimensions are full scale. Polypropylene/acrylic stock,silicone nonslip sheet; no tiny moving parts. Axes are world x/y/z. Edges deburred,R2 minimum external corners. Tolerances±1mm except final-rest post locations±0.2mm and bath waterline±1mm. Confirm positive radial clearance against the actual bottle at8mm above its base before use. Parts may be cut from stock once and reused; no per-trial fabrication.

Bath: clear rectangular inner envelope460x340x90 above4mm floor; nominal outside464x344x94. Frame center(290,120),bottomz0. Waterlinez80. Four ordinary locating blocks20x30x20 at offsets(x,y)=(±245,±130) locate bath outside walls on nonslip mat; no bottle constraint.

Start shelf:75x150x24 solid stock with4mm feet;center(180,30);bottomz4,topz28. Open on all sides. The two lot starts are(180,30) and(180,75). Shelf does not enclose bottle. Robot can lift above shelf then translate135mm in+x and lower to bath floorz4.

Stopper rack:65x100x4 base centered(270,-150);two12x50x20 blocks centeredx±23,z14. The code spawns the tip atz6;normal gravity settles it onto the4mm base before a trial. Physical tip enrollment isz4 +/-1mm in both lots,without a spacer. The posts do not suspend the23.4mm head;the native head remains accessible. LotB stoppery-180 is on the same open base edge; confirm stability in native simulation before readiness.

Inspection pad:110x100x4 centered(180,-190),rubber-coated stock. Optional final inspection position;not a bubble-clearing or seating aid. Catch tray under whole kit at least650x650mm,waterproof robot sleeves and absorbent sheets.

Bottle/stopper geometry is not a custom manufactured fixture. Use the named stock product without enlarging its grips. Model internals are explicit uncalibrated estimates described in README;native product photos/CAD are not redistributed.

Final placement rest v3: center(315,30),bath floorz4. Four posts6x12x10 centered at radial offsets(±24.4,0),(0,±24.4),short dimension radial,post bottomz4/topz14. Clear inner faces are21.4mm from center. Two-millimetre resilient pad Ø26 sitsz4–6. Estimated contact stiffness20kN/m,damping40Ns/m. Robot lifts18mm above this rest before tilting;post geometry never clears air or inserts the stopper. Cut stock pieces once;reset uses unmodified bottle/stopper. Include within existing35USD fixture/sleeve allowance.
