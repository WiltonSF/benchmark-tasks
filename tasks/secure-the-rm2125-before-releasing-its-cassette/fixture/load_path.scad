// Load-path revision11; millimetres above bench at initial head pose.
$fn=72;
PART="outboard_hanger";
ROD_Y=115;
module bore_y(d,h){rotate([90,0,0])cylinder(d=d,h=h,center=true);}
module outboard_hanger(){difference(){union(){
 translate([339,-166,243.25])cube([25,24,20],center=true);
 translate([334.5,-166,221.25])cube([16,12,34],center=true);
 translate([334.5,-166,201.45])bore_y(16,8);
 }
 translate([339,-166,243.25])bore_y(13.1,28);
 translate([334.5,-166,201.45])bore_y(4.85,16);
 translate([339,-166,243.25])rotate([0,90,0])cylinder(d=4.3,h=40,center=true);
 translate([349.45,-166,243.25])rotate([0,90,0])cylinder(d1=4.3,d2=8.4,h=2.05);
}}
module upper_guide_clamp(y=115){difference(){union(){
 translate([351,y,282.5])cube([13,13,10],center=true);
 translate([358.5,y,272])cube([8,8,14],center=true);
 translate([377.5,y,270])cube([43,8,10],center=true);
 for(x=[380,394])translate([x,y,270])cylinder(d=10,h=10,center=true);
 }
 translate([351,y,277.5])cylinder(d=8.1,h=30,center=true);
 translate([345.5,y,282.5])rotate([0,90,0])cylinder(d=2.5,h=10,center=true);
 for(x=[380,394]){translate([x,y,270])cylinder(d=3.3,h=30,center=true);translate([x,y,273.35])cylinder(d1=3.3,d2=6.6,h=1.65);}
}}
module steel_spine(){difference(){translate([340.3,-45,201.45])cube([6.35,211,6.35],center=true);for(y=[-140,55])translate([340.3,y,201.45])cylinder(d=3.3,h=10,center=true);}}
if(PART=="outboard_hanger")outboard_hanger();
if(PART=="upper_guide_clamp")upper_guide_clamp(ROD_Y);
if(PART=="steel_spine")steel_spine();
