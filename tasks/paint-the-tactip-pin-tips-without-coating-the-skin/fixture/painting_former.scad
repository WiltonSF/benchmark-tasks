// TacTip skin painting former: the robot's stand-in for the maker's thumb inside the everted skin.
// Print in PLA, 0.2 mm layers, 20 % infill, dome down on a brim or with tree supports under the dome.
// All dimensions in mm. Origin: the centre of the dome. +z runs up the handle.
$fn = 96;
R_FORMER = 19.0;   // dome and body: the everted cast skin (40 mm outside, 1 mm wall) is pulled over it
BODY_TOP = 25.0;   // the skin's cylindrical skirt covers the body from the dome's equator to here
SHANK_R = 12.0;  SHANK_TOP = 31.0;
FLANGE_R = 40.0; FLANGE_TOP = 35.0;  // 80 mm flange: hangs the former on a 70 mm beaker's rim
HANDLE_W = 20.0; HANDLE_TOP = 75.0;  // 20 mm square handle, 40 mm long (flat faces for a parallel gripper)
union() {
  sphere(r = R_FORMER);
  cylinder(r = R_FORMER, h = BODY_TOP);
  translate([0, 0, BODY_TOP]) cylinder(r = SHANK_R, h = SHANK_TOP - BODY_TOP);
  translate([0, 0, SHANK_TOP]) cylinder(r = FLANGE_R, h = FLANGE_TOP - SHANK_TOP);
  translate([-HANDLE_W / 2, -HANDLE_W / 2, FLANGE_TOP]) cube([HANDLE_W, HANDLE_W, HANDLE_TOP - FLANGE_TOP]);
}
