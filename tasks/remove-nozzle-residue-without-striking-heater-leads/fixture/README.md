# Hotend geometry, residue map and estimates to check

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

`reset-mask.csv` lists each of the 60 residue patches (centre and outward normal) relative to the block centre, in
the printer's axes (x toward the back, y to the left seen from the front, z up); the 35 degree turn is already
included. Patches 0 to 39 are on the block underside, 40 to 59 on the nozzle taper. Each is 1.2 across and 0.5 thick.

`native-scene.svg` is the drawing of the hotend in plan view (looking down through the block) and from the front, with the lead exit and the
patches.

Licence: MIT for the code; CC BY 4.0 for this text, the drawing and the residue map. Product links identify the real
parts only; no maker mesh or photograph is included.
