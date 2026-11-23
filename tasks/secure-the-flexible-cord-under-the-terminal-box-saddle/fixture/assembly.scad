// Original custom-fixture revision 3. Millimetres. Maker enclosure CAD is not redistributed.
$fn=64;
module saddle(){difference(){translate([-26,-8,4])cube([52,16,18]);for(x=[-18,18])translate([x,0,3])cylinder(d=3.3,h=20);for(x=[-22,22])for(y=[-5,5])translate([x,y,3])cylinder(d=2.5,h=20);}}
module bar(){difference(){translate([-25,-7,32.9])cube([50,14,3]);for(x=[-18,18])translate([x,0,32])cylinder(d=4.6,h=5);}}
color([.7,.7,.7])saddle();color([.5,.5,.6])bar();
color([.45,.45,.45])translate([0,-231,27.45])rotate([-90,0,0])cylinder(d=10.9,h=230);
for(x=[-18,18])color([.1,.1,.1])translate([x,0,36.3])cylinder(d=7/cos(30),h=3.2,$fn=6);

module return_tray(){difference(){cube([200,200,15]);translate([4,4,3])cube([192,192,13]);}}
translate([80,0,0])color([.12,.12,.12])return_tray();
