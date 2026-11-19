// Captured gantry manufacturing source G1. Units mm. Original code MIT.
// Set PART then render/export. Holes/hollows are manufacturing details;
// native contact simulation retains their enclosing solids as disclosed.
PART="assembly";
// Fit-only machining input for the internal nut-block's lower longitudinal edges.
// Default1 mm is an authored allowance, NOT a vendor tube-corner measurement.
NUT_CORNER_RELIEF=1.0;
$fn=64;
eps=0.01;
fix=[25,125,225];
reg=[75,175];

module rail_bores() {
  for(x=fix) translate([x,0,-eps]) cylinder(d=3.4,h=8+2*eps);
  for(x=reg) translate([x,0,-eps]) cylinder(d=2,h=8+2*eps);
}
module track_foot() { difference() {
  translate([0,-6,0]) cube([250,12,3]); rail_bores();
} }
module track_neck() { difference() {
  translate([0,-4,3]) cube([250,8,2]); rail_bores();
} }
module track_head() { difference() {
  translate([0,-6,5]) cube([250,12,3]); rail_bores();
  for(x=fix) translate([x,0,8+eps]) rotate([180,0,0])
    cylinder(d1=6.8,d2=3.4,h=1.7+eps);
} }
module track() { track_foot();track_neck();track_head(); }

// One handed cheek; rotate180 aboutZ for its matching opposite.
// Stock bounding box45.4 x9.25 x10. Toe is integral, not glued on.
module cheek() { difference() {
  union() {
    translate([-22.7,6.25,3]) cube([45.4,7.25,10]);
    translate([-22.7,4.25,3.25]) cube([45.4,2,1.5]);
  }
  for(x=[-16,16]) {
    translate([x,13.5+eps,10.625]) rotate([90,0,0]) cylinder(d=2.2,h=7.25+2*eps);
    translate([x,13.5+eps,10.625]) rotate([90,0,0]) cylinder(d1=3.9,d2=2.2,h=.85+eps);
  }
} }
module cap() { difference() {
  translate([-22.7,-6.25,8.25]) cube([45.4,12.5,4.75]);
  for(x=[-5,5]) {
    translate([x,0,8.25-eps]) cylinder(d=3.4,h=4.75+2*eps);
    translate([x,0,8.25-eps]) cylinder(d1=6.8,d2=3.4,h=1.7+eps);
  }
  for(x=[-16,16],side=[-1,1])
    translate([x,side*(6.25+eps),10.625]) rotate([side*90,0,0]) cylinder(d=1.6,h=5+eps);
} }
module carriage() { cheek();rotate([0,0,180]) cheek();cap(); }

module nut_block() { difference() {
  translate([-7.75,-6,0]) cube([15.5,12,6]);
  for(side=[-1,1]) translate([0,6,0]) rotate([90,0,0]) linear_extrude(height=12)
    polygon([[side*7.75,0],[side*(7.75-NUT_CORNER_RELIEF),0],[side*7.75,NUT_CORNER_RELIEF]]);
  for(x=[-5,5]) translate([x,0,-eps]) cylinder(d=2.5,h=6+2*eps);
} }
module beam() { difference() {
  translate([-10,-200,13]) cube([20,400,20]);
  translate([-8,-201,15]) cube([16,402,16]);
  for(y=[-170,170],x=[-5,5]) translate([x,y,12]) cylinder(d=3.4,h=4);
  for(y=[-15,15]) {
    translate([-11,y,23]) rotate([0,90,0]) cylinder(d=4.5,h=22);
    translate([-10-eps,y,23]) rotate([0,90,0]) cylinder(d1=9.2,d2=4.5,h=2.35+eps);
  }
} }
module ballast() { difference() {
  translate([10,-25,13]) cube([40,50,40]);
  for(y=[-15,15]) translate([10-eps,y,23]) rotate([0,90,0]) cylinder(d=3.3,h=12+eps);
} }
module crush_spacer() { difference() {
  rotate([0,90,0]) cylinder(d=8,h=16);
  translate([-eps,0,0]) rotate([0,90,0]) cylinder(d=4.5,h=16+2*eps);
  // This entry faces the beam's countersunk wall at X=-8 in assembly.
  // The head cone continues0.35 past its2 mm wall; this relief clears it.
  translate([-eps,0,0]) rotate([0,90,0]) cylinder(d1=5.3,d2=4.5,h=.40+eps);
} }
module stop() { difference() {
  translate([0,-15,0]) cube([10,30,20]);
  for(y=[-8,8]) {
    hull() for(dx=[-.5,.5]) translate([5+dx,y,-eps]) cylinder(d=3.4,h=20+2*eps);
    hull() for(dx=[-.5,.5]) translate([5+dx,y,20+eps]) rotate([180,0,0])
      cylinder(d1=6.8,d2=3.4,h=1.7+eps);
  }
} }
module moving_assembly() {
  for(y=[-170,170]) { translate([0,y,0]) carriage();translate([0,y,15]) nut_block(); }
  beam();ballast();
  for(y=[-15,15]) translate([-8,y,23]) crush_spacer();
}
module assembly() {
  for(y=[-170,170]) { color("grey") translate([30,y,0]) track();color("red") translate([280,y,0]) stop(); }
  color("silver") translate([216,0,0]) rotate([0,0,-.45]) moving_assembly();
}

if(PART=="assembly") assembly();
if(PART=="track_foot") track_foot();
if(PART=="track_neck") translate([0,0,-3]) track_neck();
if(PART=="track_head") translate([0,0,-5]) track_head();
if(PART=="cheek") translate([22.7,-4.25,-3]) cheek();
if(PART=="cap") translate([22.7,6.25,-8.25]) cap();
if(PART=="beam") translate([10,200,-13]) beam();
if(PART=="nut_block") translate([7.75,6,0]) nut_block();
if(PART=="ballast") translate([-10,25,-13]) ballast();
if(PART=="stop") translate([0,15,0]) stop();
if(PART=="crush_spacer") crush_spacer();
