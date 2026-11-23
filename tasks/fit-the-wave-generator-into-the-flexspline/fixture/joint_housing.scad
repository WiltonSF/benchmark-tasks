// Printed robot-joint housing and assembly cradle for a Harmonic Drive CSF-14-50-2A-R component set.
// Units: mm. Print in PETG (housing) and PLA (cradle), 4 walls, 40 % infill.
// The circular spline sits in the 50 mm h6 pilot and is bolted with 8 x M3 on the 44 mm circle (catalogue values);
// the flexspline is bolted to the output flange, which turns in the housing's lower bore on a 6805-2RS bearing
// (25 x 37 x 7 mm). The flexspline's open end is level with the circular spline's top face, as the catalogue
// requires. The wave generator goes in from the top.
$fn = 120;
pilot_d = 50.0;      // circular spline pilot (catalogue øA h6)
cs_width = 8.0;      // circular spline seat depth below the top face (estimate from the outline drawing)
bolt_pcd = 44.0;     // circular spline bolt circle (catalogue Q)
housing_od = 72.0;
housing_h = 54.0;    // top face 60 mm above the bench: base in the cradle's 4 mm pocket (floor at 6 mm)
bearing_od = 37.0;   // output bearing seat (6805-2RS)
bearing_w = 7.0;
cradle = [120, 120, 10];

module housing() {
  difference() {
    cylinder(d = housing_od, h = housing_h);
    translate([0, 0, housing_h - cs_width]) cylinder(d = pilot_d + 0.1, h = cs_width + 1);   // circular spline pilot
    translate([0, 0, -1]) cylinder(d = pilot_d - 1.0, h = housing_h);                         // flexspline clearance
    translate([0, 0, -1]) cylinder(d = bearing_od, h = bearing_w + 1);                         // output bearing
    for (a = [0:45:315]) rotate(a) translate([bolt_pcd / 2, 0, housing_h - cs_width - 12])
      cylinder(d = 2.5, h = 13);                                                               // M3 tapping holes
  }
}

module cradle() {
  difference() {
    translate([-cradle[0] / 2, -cradle[1] / 2, 0]) cube(cradle);
    translate([0, 0, 6]) cylinder(d = housing_od + 0.4, h = 10);                               // 4 mm locating pocket
    translate([0, 0, -1]) cylinder(d = 40, h = 20);                                            // output flange clearance
  }
}

cradle();
translate([0, 0, 6]) housing();   // pocket floor
