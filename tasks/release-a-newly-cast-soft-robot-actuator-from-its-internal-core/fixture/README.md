# Reconstruction drawing schedule (mm)

Maker reference: https://softroboticstoolkit.com/book/fr-make-actuator-body . These are reconstructed drawings, not downloaded original maker CAD. No rescaling is allowed.

| Part | Dimensions/material | Datum |
|---|---|---|
| Actuator |170 long;12.7 diameter half-round lumen;2 original wall;1 estimated encapsulation;24 g estimate|Lumen flat planez0; axisy; final arch radius9.35, flat undersidez-3|
| Casting core |220 long;12.7 diameter half-round steel,109 g estimate|Half-round flat planez0;50 protruding beyond body at-y|
| Body/encapsulation molds |170 cavity;34x27x170 outer block; split at x0; two detachable5mm end plates;0.1 radial/base clearance; two4mm crown fill/vent ports|Stage body: radius8.35,base2; stage encapsulation: radius9.35,base3;12.7 half-round rod passes front plate and ends flush at rear|
| Saddles |Two10 wide(x),12 long(y),77 high(z), wood/PLA; bottoms fixed onbench|Centresx320,y-65/+65; flat contacttop|
| Tray |100x200 interior,6 floor;6 walls,12 wallheight|CentreA320,205;B345,195; floor undersideonbench|
| Mat |150x350x2 silicone|Centre560,0|
| Vise |100 base,100x25 jaws,43 open gap|Centre560,-180; open, not gripping actuator|
| Ruler |20x220x2;10 pitchblack marks|Centre440,290|

Geometry acceptance before a real trial: ruler/caliper verify lumen12.7+/-0.2 and length170+/-0.5; no burrs, clean half-round flat, polished core, repeatable cast mass recorded. Native simulation adds0.2 radial collision clearance; printed/steel physical fit must be requalified using the witness cast. Do not enlarge the actuator or shorten its core to make the grasp easier. Molds are fabrication context; core release begins after the outer mold is already removed.

In the CAD, z is the longitudinal axis, x spans the flat width, and y points toward the arch. The simulator rotates that longitudinal axis into world y. `part` selects an individually exportable part; `stage` selects `body` or `encapsulation`. For each stage export `mold_left`, `mold_right`, `front_end` and `rear_end`. For example: `openscad -D 'part="mold_left"' -D 'stage="body"' -o body-left.stl actuator.scad`. The default is an exploded drawing; do not print it as one piece. `part="assembled"` shows the rod at z-50..170, the front plate at z-5..0, and the rear plate at z170..175.

The mold cavity subtracts the filled outside cast profile, leaving the entire removable core volume free. Left/right halves separate along x; front/rear plates detach along z. Two M4 clearance holes below the cavity accept manually installed M4x40 bolts/nuts. Hold the removable end plates against the ends using two ordinary small clamps, with mold release/seam seal as the maker procedure requires; these manual preparation items use the existing fixture/consumables allowances. Core front-plate clearance is0.1mm estimated; the closed rear plate locates its far end at z170. Fill and vent through4mm crown ports after positioning the rod; trim sprues flush before photographing/enrolling a stock cast. The body stage forms the2mm shell, followed by the maker's passive reinforcement and estimated1mm encapsulation stage. These are reconstruction tools requiring a witness casting, fit/leak check and dimensional verification before real evaluation, not qualified maker tooling. No print or casting trial has been performed.
