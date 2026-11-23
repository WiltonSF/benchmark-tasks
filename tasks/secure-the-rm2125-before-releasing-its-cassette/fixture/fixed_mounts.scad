// Fabrication-only fixed mounts, mm above bench; no runtime edits.
$fn=72;
PART="guard_pedestal";
SIDE=1; // +1 at y113, -1 at y-3
module guard_pedestal(side=1){
 y=side==1?113:-3; fy=side==1?107:3; by=side==1?104:6;
 difference(){union(){
 translate([250.5,y,75.25])cube([12,8,90.5],center=true);
 translate([250.5,fy,32])cube([24,20,4],center=true);
 }
 translate([250.5,y,117])rotate([90,0,0])cylinder(d=5,h=12,center=true);
 for(x=[242.5,258.5])translate([x,by,32])cylinder(d=4.3,h=8,center=true);
 }
}
module guide_base(){difference(){
 translate([371.5,55,41.5])cube([57,146,8],center=true);
 for(y=[-5,115]){
 translate([390,y,41.5])cylinder(d=4.3,h=12,center=true);
 translate([390,y,43.4])cylinder(d1=4.3,d2=8.4,h=2.1);
 translate([351,y,41.5])cylinder(d=8.3,h=12,center=true);
 for(dy=[-8.5,8.5])translate([356,y+dy,41.5])cylinder(d=3.3,h=12,center=true);
 }
}}
module guide_floor_foot(y=115){difference(){
 translate([390,y,27.75])cube([20,24,19.5],center=true);
 translate([390,y,27.75])cylinder(d=4.3,h=24,center=true);
 for(z=[21,34.5])translate([390,y,z])cylinder(d=5.5,h=6,center=true);
}}
module rod_retainer(y=115){difference(){
 translate([351,y,55.5])cube([16,24,20],center=true);
 translate([351,y,55.5])cylinder(d=8.1,h=24,center=true);
 for(dy=[-8.5,8.5]){
 translate([356,y+dy,55.5])cylinder(d=3.3,h=24,center=true);
 translate([356,y+dy,63.85])cylinder(d1=3.3,d2=6.6,h=1.65);
 }
 for(z=[51,60]){
 translate([345,y,z])rotate([0,90,0])cylinder(d=2.5,h=8,center=true);
 translate([344.5,y,z])rotate([0,90,0])cylinder(d=4,h=3,center=true);
 }
}}
if(PART=="guard_pedestal")guard_pedestal(SIDE);
if(PART=="guide_base")guide_base();
if(PART=="guide_floor_foot")guide_floor_foot(115);
if(PART=="rod_retainer")rod_retainer(115);
if(PART=="assembly"){
 for(side=[-1,1])guard_pedestal(side);
 guide_base();for(y=[-5,115]){rod_retainer(y);guide_floor_foot(y);}
}
