// Inert surroundings only; not an electrophoresis vessel. Millimetres.
// Complete 110x120x160 external envelope from XCell SureLock user guide.
// 6 mm walls; internal core/wedge are estimated visible envelopes.
difference(){cube([110,120,160]);translate([6,6,6])cube([98,108,148]);}
translate([26,40,6])cube([58,35,148]);
translate([84,25,6])cube([14,70,105]);
