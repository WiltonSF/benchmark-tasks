// mm. Dimensioned reconstruction, not recovered maker CAD. Axis z is actuator length.
// Export each selector separately; stage=body or encapsulation. No rescaling.
$fn=96;
part="exploded";
stage="encapsulation";
r=stage=="body" ? 8.35 : 9.35;
base=stage=="body" ? 2 : 3;
module halfround(rad,h){intersection(){cylinder(r=rad,h=h);translate([-rad,0,0])cube([2*rad,rad,h]);}}
module outer(rad=r,b=base,h=170){union(){halfround(rad,h);translate([-rad,-b,0])cube([2*rad,b,h]);}}
module body(){difference(){outer();translate([0,0,-0.01])halfround(6.35,170.02);}}
module core(){halfround(6.35,220);}
// Subtract the FILLED outside profile: the rod's entire volume is free, not mold material.
// 0.1 mm radial/base mold clearance is an estimated fabrication allowance.
module cavity(){translate([0,0,-0.01])outer(r+0.1,base+0.1,170.02);}
module mold_block(){
 difference(){
  translate([-17,-10,0])cube([34,27,170]);
  cavity();
  // Split 4 mm fill/vent ports at the crown; trim sprues flush before enrollment.
  for(z=[15,155])translate([0,r-1,z])rotate([-90,0,0])cylinder(r=2,h=20);
  // Two M4 clearance holes through x, below the cast; manual assembly only.
  for(z=[25,145])translate([-18,-6,z])rotate([0,90,0])cylinder(r=2.2,h=36);
 }
}
module mold_left(){intersection(){mold_block();translate([-17,-10,-1])cube([17,27,172]);}}
module mold_right(){intersection(){mold_block();translate([0,-10,-1])cube([17,27,172]);}}
// Detachable front plate supports the protruding rod; rear plate closes flush at z170.
module front_end(){difference(){translate([-17,-10,-5])cube([34,27,5]);translate([0,0,-5.01])halfround(6.45,5.02);}}
module rear_end(){translate([-17,-10,170])cube([34,27,5]);}
module saddle(){cube([10,12,77]);}
module tray(){difference(){cube([112,212,18]);translate([6,6,6])cube([100,200,13]);}}
if(part=="body")body();
else if(part=="core")core();
else if(part=="mold_left")mold_left();
else if(part=="mold_right")mold_right();
else if(part=="front_end")front_end();
else if(part=="rear_end")rear_end();
else if(part=="saddle")saddle();
else if(part=="tray")tray();
else if(part=="assembled"){mold_left();mold_right();front_end();rear_end();translate([0,0,-50])core();}
else {body();translate([45,0,0])core();translate([90,0,0])mold_left();translate([110,0,0])mold_right();translate([165,0,0])front_end();translate([210,0,0])rear_end();}
