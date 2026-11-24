# Fixture package

`dimensions.json` and `drawing.svg` specify the native mold. The source STL measures 193.19912 x 167.31536 x 18 mm. Its thickness is on its native Y axis; the simulation rotates that to Z. The base seat radius of 85.05256 mm agrees with the native DXF hexagon after coordinate conversion. The analytic reconstruction preserves the bottom access hole, ledge, casting cavity and circumferential relief. The 0.2 mm radial print allowance is an estimate for a printed fit.

Source hardware: `MMintLab/gelslim_hardware`, `gel_mold/outer_ring.STL` and `gel_mold/hex_acrylic_base.DXF`, master downloaded November 21, 2026. The repository tree contains no license file. Source meshes and CAD files are not redistributed in this package. Geometry here is an independently coded analytic hexagonal construction from dimensions. Code and drawings in this task are MIT and CC BY 4.0 respectively. Redistribution permission would need verification before packaging native meshes.

The maker specifies a printed ring and 1/4 inch cast acrylic. Each sheet uses 70 g silicone base plus 10% activator, degassed and cured for 16–18 h. Material SKU and batch must match the maker's bill of materials. This simulation does not establish physical equivalence of alternative silicones.

The push-out stand is a stock Corning PYREX 3140-90 crystallizing dish (90 mm across, 50 mm tall), upturned. It is narrower than the ring's 117.3 mm bottom opening across flats, so it bears on the exposed back of the acrylic while the ring is pressed down past it. Its 190 g mass is an estimate.

Construction assumptions: PLA, 0.2 mm layers, 0.4 mm nozzle, 80% infill; a dry sealed-laminate counter 1100 x 1300 x 60 mm. No release agent. These are reproducibility choices, not maker requirements. Batch, cure and macro-inspection qualification remain unperformed.

Repeated trials use sheets cast in logged batches the day before and kept in their molds. The batch recipe, acceptance checks, witness peel and per-trial swap are in [SPECIMENS.md](SPECIMENS.md).
