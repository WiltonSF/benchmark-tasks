# Fixture package

`dimensions.json` and `drawing.svg` specify the native mold. Source STL measured193.19912 by167.31536 by18mm. It has its thickness on its nativeY axis; simulation rotates that toZ. The base seat radius85.05256mm is consistent with the native DXF hexagon after its coordinate conversion. The independent geometric reconstruction preserves the bottom access hole, ledge, casting cavity and circumferential relief. The0.2mm radial print allowance is an estimate for a printed fit.

Source hardware: `MMintLab/gelslim_hardware`, `gel_mold/outer_ring.STL` and `gel_mold/hex_acrylic_base.DXF`, master downloaded2026-11-21. The repository tree contains no license file. No source meshes or CAD files are redistributed in this package. Geometry here is an independently coded analytic hexagonal construction from dimensions. Code/drawings in this task are MIT/CC BY4.0 respectively. A future packaged native mesh would need verified redistribution permission.

The maker specifies a3D printed ring and1/4inch cast acrylic. A sheet is made with70g silicone base plus10% activator, degassed and cured16–18h. Material SKU and batch must match the maker's current bill of materials; this simulation does not establish physical equivalence of alternative silicones.

The push-out stand is a stock Corning PYREX 3140-90 crystallizing dish (90 mm across, 50 mm tall), upturned. It is narrower than the ring's 117.3 mm bottom opening (across flats), so it can bear on the exposed back of the acrylic while the ring is pressed down past it. Its 190 g mass is an estimate.

Replica construction assumptions:PLA,0.2mm layers,0.4mm nozzle,80%infill;dry sealed-laminate counter1100×1300×60mm. No release agent. These choices are specified for reproducibility,not claimed as maker requirements. Batch/cure and macro-inspection qualification remain unperformed.
