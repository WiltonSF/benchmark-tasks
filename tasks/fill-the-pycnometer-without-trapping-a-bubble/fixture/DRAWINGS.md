# Fixture v 9, dimensions in millimetres

Current drawing:fixture-v12.svg (metered replication of the first-fill shoulder bubble; fixture geometry unchanged); current model manifest:model-v21.json (open post-top-up first-fill start with one 0.25 mL shoulder bubble; declared wetting proxy; contact/force-established wet-seal proxy, covered-pad contact footprint and capillary drainage unchanged). V 2 has superseded post spacing; use the 21.4 mm inner radius specified here. The v 1 drawing is retained as the superseded shelf-only layout.

All dimensions are full scale. Polypropylene/acrylic stock, silicone nonslip sheet; no tiny moving parts. Axes are world x/y/z. Edges deburred, R 2 minimum external corners. Tolerances±1 mm except final-rest post locations±0.2 mm and bath waterline±1 mm. Confirm positive radial clearance against the actual bottle at 8 mm above its base before use. Parts may be cut from stock once and reused; no per-trial fabrication.

Bath: clear rectangular inner envelope 460 x 340 x 90 above 4 mm floor; nominal outside 464 x 344 x 94. Frame center(290, 120), bottomz 0. Waterlinez 80. Four ordinary locating blocks 20 x 30 x 20 at offsets(x, y)=(±245, ±130) locate bath outside walls on nonslip mat; no bottle constraint.

Start shelf:75 x 150 x 24 solid stock with 4 mm feet; center(180, 30); bottomz 4, topz 28. Open on all sides. The two lot starts are(180, 30) and(180, 75), with the stopper on its rack; bottle mouth z 98 stays above the 80 mm waterline. Shelf does not enclose bottle. Robot can lift above shelf then translate 135 mm in+x and lower to bath floorz 4.

Stopper rack:65×100×4 base centered(270, -150). Four 18 mm-high guide blocks bottomz 4/topz 22:two 12×50×18 centered(x, y, z)=(±17.5, 0, 13), two 23×12×18 centered(0, ±17.5, 13). Clear opening 23×23 mm around native lower stem, estimated 21.0 mm maximum diameter; nominal 1.0 mm radial clearance. Head bottomz 24.25 clears guides by 2.25 mm. Use free clearance, not a clamp. Native stopper tip settles onto base topz 4; robot grips the head and lifts 90 mm out. The guides stabilize pickup only; they cannot contact the bottle at the distant final rest. Confirm native stem fits freely withcalipers once per pair; adjust stock guide spacing once if needed and record it. No per-trial fabrication.

Inspection pad:110 x 100 x 4 centered(180, -190), rubber-coated stock. Optional final inspection position; not a bubble-clearing or seating aid. Catch tray under whole kit at least 650 x 650 mm, waterproof robot sleeves and absorbent sheets.

Bottle/stopper geometry is not a custom manufactured fixture. Use the named stock product without enlarging its grips. Model internals are explicit uncalibrated estimates described in README; native product photos/CAD are not redistributed.

Final placement rest v 3: center(315, 30), bath floorz 4. Four posts 6 x 12 x 10 centered at radial offsets(±24.4, 0), (0, ±24.4), short dimension radial, post bottomz 4/topz 14. Clear inner faces are 21.4 mm from center. Two-millimetre resilient pad Ø26 sitsz 4–6. Estimated contact stiffness 20 kN/m, damping 40 Ns/m. The open bottle is carried upright from the shelf to this rest; post geometry never clears air or inserts the stopper. Cut stock pieces once; reset uses unmodified bottle/stopper. Include within existing 35 USD fixture/sleeve allowance.

All model-v2 through model-v20 manifests and fixture-v1 through fixture-v11 drawings are superseded historical records. Only model-v21.json and fixture-v12.svg describe this revision.

During the two 40 degree rocks, the bottle is lifted 18 mm above the rest; no post provides the clearing motion. Metered first-fill bubble replication and technician figures are in RESET-PROTOCOL.md.

The current internal collision approximation uses matched conical surfaces with 64 convex sectors. This is an unmeasured profile estimate, not a modified or custom-manufactured bottle; use the stock PH0122B pair. Outer product size, fixtures and real reset are unchanged. See README and model-v21.json.

The 23×23 mm free guide opening gives about 1.0 mm radial clearance to the estimated 21.0 mm stem diameter at guide-top height. It remains a pickup guide only. Confirm clearance on the actual stock pair; no clamp or friction fit is permitted.

Reference finish: after the stopper hand clears, the bottle hand lifts the closed assembly 35 mm before lateral travel, carries it to (180,160) mm and lowers its base to the existing bath floor, z 4 mm. No new fixture is added. The open rest at (315,30) mm remains the top-up and seating support.
