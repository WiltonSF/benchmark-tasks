// Full-size fabrication supplements, revision12; dimensions mm.
// Select one PART and orient its longest bore vertically for printing.
use <geometry.scad>
use <fabrication.scad>
$fn=72;
PART="spring_bridge";
module bore_y(d,h){rotate([90,0,0]) cylinder(d=d,h=h,center=true);}
module spring_bridge(){difference(){union(){
 translate([0,0,46.96]) cube([33,34,4],center=true);
 for(y=[-15,15])translate([0,y,23])cube([33,4,46],center=true);
 translate([0,21,3])cube([32,16,6],center=true);
 // Integral third X/Z coupling: open -Y face clears the bridge by1mm.
 translate([0,28.5,37.8])cube([34,21,32],center=true);
 for(x=[-12,12])translate([x,23,20.4])cube([4,8,34.8],center=true);
 }
 for(y=[-6,6])translate([0,y,46.96])cylinder(d=6.4,h=6,center=true);
 // Stock-tube clearance includes its12.7mm OD; bores also relieve the cheek/foot locally.
 translate([0,23,26.9])cylinder(d=13.7,h=60,center=true);
 translate([0,23,37.8])rotate([0,90,0])cylinder(d=13.7,h=40,center=true);
 translate([9,26,37.8])cylinder(d=4.3,h=36,center=true);
 translate([0,21,26.3])rotate([0,90,0])cylinder(d=4.3,h=40,center=true);
}}
module floating_spacer(){difference(){cylinder(d=8,h=1.2);translate([0,0,-1])cylinder(d=6.2,h=4);}}
module plunger(){difference(){union(){cube([24,20,5],center=true);for(y=[-8,8])translate([0,y,-5])cube([12,4,8],center=true);}for(y=[-6,6]){translate([0,y,0])cylinder(d=6.0,h=9,center=true);translate([0,y,.5])rotate([0,90,0])cylinder(d=2.5,h=30,center=true);}translate([0,0,-6.5])bore_y(4.2,26);}}
module roller(){difference(){bore_y(8,12);bore_y(4.2,16);}}
module bearing(){difference(){cube([24,20,24],center=true);bore_y(8.3,24);for(x=[-8,8])translate([x,0,-8])cylinder(d=4.3,h=30,center=true);}}
module pivot_ear(){difference(){union(){cube([18,8,24],center=true);translate([0,0,-10])cube([32,16,4],center=true);}bore_y(5.0,12);for(x=[-12,12])translate([x,0,-10])cylinder(d=4.3,h=8,center=true);}}
module clamp_print(){difference(){union(){specimen_guide_clamp();bore_y(14,12);}translate([0,-100,0])bore_y(4.85,300);translate([5.8,-100,0])cube([6.6,213,6.6],center=true);for(y=[-195,0]){translate([5.8,y,0])cylinder(d=3.3,h=30,center=true);translate([5.8,y,(y==0?3.15:2.85)])cylinder(d1=3.3,d2=6.6,h=1.65);translate([5.8,y,-9.5])cylinder(d=7,h=10,center=true);}translate([5,8,0])rotate([0,90,0])cylinder(d=2.5,h=8,center=true);}}
module knob_print(){difference(){bore_y(24,80);translate([0,32,0])bore_y(8.1,18);}}
module shaft_collar(d=8.1){difference(){bore_y(24,12);bore_y(d,18);rotate([0,90,0])cylinder(d=3.3,h=30,center=true);}}
module wheel_print(){difference(){union(){bore_y(200,12);bore_y(24,24);}bore_y(8.1,30);translate([-73.612,0,42.5])bore_y(8.4,30);rotate([0,90,0])cylinder(d=3.3,h=30,center=true);}}
module crank_print(){difference(){union(){bore_y(76,12);bore_y(24,24);}bore_y(8.1,30);translate([-25.548,0,14.75])bore_y(9.9,30);rotate([0,90,0])cylinder(d=3.3,h=30,center=true);}}
module guard_print(){difference(){union(){knife_guard_guard();for(y=[-49,49])translate([0,y,0])bore_y(14,8);}bore_y(5.0,112);}}
// Keep +x toward housing: axis at global x339; outer face x348 leaves12mm shell clearance.
// Local x-extent -16..9; y-extent -12..16 clears the fixed upper brace.
// Countersunk M4x30 heads flush at +X9; washers/nuts on the -X face.
module corner_socket(){difference(){translate([-3.5,2,0])cube([25,28,32],center=true);bore_y(13.1,36);cylinder(d=13.1,h=36,center=true);for(p=[[0,-7.5,0],[0,0,-9]])translate(p){rotate([0,90,0])cylinder(d=4.3,h=36,center=true);translate([6.95,0,0])rotate([0,90,0])cylinder(d1=4.3,d2=8.4,h=2.05);}}}
// The second corner joins X/Y tubes without rotating its housing-facing relief.
module corner_socket_xy(){difference(){translate([-3.5,0,0])cube([25,32,32],center=true);bore_y(13.1,36);rotate([0,90,0])cylinder(d=13.1,h=36,center=true);for(p=[[-9,0,0],[0,9,0]])translate(p)cylinder(d=4.3,h=36,center=true);}}
module tube_socket(){difference(){cube([32,32,24],center=true);bore_y(13.1,36);rotate([0,90,0])cylinder(d=3.3,h=36,center=true);for(x=[-12,12])translate([x,0,0])cylinder(d=4.3,h=30,center=true);}}
if(PART=="spring_bridge")spring_bridge();
if(PART=="spacer")floating_spacer();
if(PART=="plunger")plunger();
if(PART=="roller")roller();
if(PART=="bearing")bearing();
if(PART=="pivot_ear")pivot_ear();
if(PART=="clamp")clamp_print();
if(PART=="knob")knob_print();
if(PART=="collar")shaft_collar();
module pivot_collar_small(){difference(){bore_y(10,4);bore_y(4.85,8);translate([3.5,0,0])rotate([0,90,0])cylinder(d=2.5,h=5,center=true);}}
if(PART=="pivot_collar")pivot_collar_small();
if(PART=="wheel")wheel_print();
if(PART=="crank")crank_print();
if(PART=="guard")guard_print();
if(PART=="carriage")carriage_print();
if(PART=="tube_socket")tube_socket();
if(PART=="corner_socket")corner_socket();
if(PART=="corner_socket_xy")corner_socket_xy();


// Remove the metal stand-in from the printed body and leave its seating slot.
module holder_print(){union(){difference(){knife_holder_e_base();
 translate([33,0,101])cube([0.6,80.2,8.2],center=true);
 translate([34.35,0,93.475])cube([3.3,82.2,19.25],center=true);
 for(y=[-30,30]){translate([28.7,y,92.5])rotate([0,90,0])cylinder(d=4.3,h=14,center=true);translate([29.7,y,92.5])rotate([0,90,0])cylinder(d=5.5,h=6.0,center=true);}
 for(x=[-40,40],y=[-40,40]){translate([x,y,15])cylinder(d=4.3,h=32,center=true);translate([x,y,27.95])cylinder(d1=4.3,d2=8.4,h=2.05);}
}translate([32.9,0,96.7])cube([0.6,80,0.4],center=true);for(y=[-30,30])translate([32.975,y,88.5])cube([0.75,8,2],center=true);}}

module blade_pressure_bar(){difference(){translate([294.9375,55,93.475])cube([3.175,82,19.05],center=true);for(y=[25,85]){translate([294.9375,y,92.5])rotate([0,90,0])cylinder(d=4.3,h=6,center=true);translate([294.475,y,92.5])rotate([0,90,0])cylinder(d1=4.3,d2=8.4,h=2.05);}}}
if(PART=="blade_pressure_bar")blade_pressure_bar();
if(PART=="holder")holder_print();
