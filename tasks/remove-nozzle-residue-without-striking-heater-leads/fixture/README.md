# Hotend geometry, standard residue coupon and estimates to check

All sizes in millimetres. The printer is a full-size geometric reconstruction of the Original Prusa MK4S (maker
figures: 500 x 550 x 400, 7 kg), not the maker's CAD. Frame, bed, rods, carriage, fans, fan shroud, power supply and
control box are kept in place; motors and hidden fasteners are left out. Nothing is scaled or enlarged.

Estimates to check with a caliper on the real hotend (from maker photographs and the 13 mm nozzle wrench):

| Feature | Value used |
|---|---|
| Heater block | 13 x 20 x 12, turned 35 degrees about the nozzle axis |
| Nozzle | 6 below the block; hex shoulder 8 across and 1 tall; taper to 1.2 at the tip |
| Fan shroud | side walls 25 from the nozzle axis, back wall 27 behind it, lower edge 0.5 below the block underside, open at the front |
| Heater leads | 2.2 across, leaving the block's back left face square to it at mid-height, 4.0 and 6.2 left of the face's centre line |
| Thermistor leads | 1.3 across, 7.8 and 9.0 left of the face's centre line, same height |
| Lead route | 12 out from the face, then 29 up to the carriage; lowest point 4.9 above the block underside |
| Brush (Prusa Brass Nozzle Brush) | 172 long; 12 x 12 wood; 34 long head carrying 27 tufts (9 x 3, 3.5 apart along, 4 apart across) of 14 mm brass bristles; 35 g |
| Catch tray | 120 x 80 x 15, 1 mm stainless sheet, 140 g |

## Standard residue coupon

One coupon per trial, printed as a single 0.2 mm layer of dark PETG (Prusament PETG, nozzle 250 C) on any desktop
FDM printer, 24 to a plate. Outline in the block's own axes (x' square to the lead-exit face, positive toward it;
y' along that face, positive to the robot's left; origin on the nozzle axis): the rectangle x' -6.0 to 3.5,
y' -9.5 to 9.5, less a 4.5 radius hole centred on the origin. That leaves the block's outer 0.5 bare on three sides,
a 3.0 bare strip along the lead-exit face and 0.5 of clearance round the 8 mm nozzle shoulder. Area 120 mm2.

`coupon-cells.csv` lists the 482 cells of 0.5 x 0.5 that the simulation scores: centre in block axes (xb, yb) and in
the printer's axes relative to the block centre (x toward the back, y to the left seen from the front, z up; the
35 degree turn included).

## Coupon transfer stand

A 1.5 mm steel plate faced with 0.1 mm PTFE film, with a 9 mm circular nozzle hole and a shallow
pocket that locates the coupon to the outline above, carried on three M4 steel rods (nuts set the height once) over a
printed base. The base has locators that sit on fixed marks on the build plate surrogate, and the whole stand rides
on a small laboratory jack. The intended transfer is that warmed PETG bonds to brass and releases from the PTFE face when lowered; this has not been verified. The circular hole supports the C-shaped coupon spine; an open-front slot would leave that spine unsupported. Lower the platen below the nozzle tip before withdrawing it. This is a proposed fixture; it has not been built or timed.

`native-scene.svg` draws the hotend in plan view with the lead exit and the coupon, and the transfer stand from
the side.

Licence: MIT for the code; CC BY 4.0 for this text, the drawing and the cell table. Product links identify the real
parts only; no maker mesh or photograph is included.

## Finger covers

`protection/__init__.py` adds 1.5875 mm rigid cover envelopes to distal finger hulls (hulls whose lowest local z is at least -17 mm), leaving sliding carriages clear. The same triangle surfaces enter hand sampling. All four fingers are covered; dimensions use the robot's native hull frames. Material: USA Industrials BULK-RS-S50FDA-226, nominal 1/16 in; supplier tolerance is +/-0.012 in, so measure and match the actual sheet before a physical trial. Thermal protection of the original pads remains unqualified. No cover is a demonstrated physical safety result.
