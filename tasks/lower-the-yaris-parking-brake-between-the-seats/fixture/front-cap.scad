// Front tunnel cap and passive shutter, mm. Fabrication geometry, not OEM CAD.
// Model X=local x; model Y=local z+10; model Z=depth behind y=-78.
// Complete receipt sweep and retention checks: stock-load-design.md.
view="assembly"; shutter_local_z=-10; head_angle=82; // nominal illustrative angle; match the received screw and leave >=0.3mm land.
module cap(){difference(){translate([-58,-58,0])cube([116,116,18]);
 translate([-31,-50.5,-1])cube([62,101,6]);
 translate([-6,-15,-1])cube([12,30,20]);
 for(x=[-33,33])translate([x-8,-38,-1])cube([16,76,4]);
 for(x=[-36,36],y=[-25,0,25])translate([x,y,-1])cylinder(h=20,d=3,$fn=32);
}}
module retainer(side=1){difference(){translate([33*side-8,-38,0])cube([16,76,3]);
 for(y=[-25,0,25]){translate([36*side,y,-1])cylinder(h=5,d=4.5,$fn=32);
 translate([36*side,y,-.01])cylinder(h=(9-4.5)/2/tan(head_angle/2),d1=9,d2=4.5,$fn=48);}
}}
module shutter(){difference(){linear_extrude(1)offset(r=1)square([58,78],center=true);translate([0,0,-1])cylinder(h=3,d=10,$fn=64);}}
if(view=="cap")cap();
if(view=="retainer")retainer();
if(view=="shutter")shutter();
if(view=="assembly"){color([.65,.52,.32,.65])cap();for(s=[-1,1])color("silver")retainer(s);translate([0,shutter_local_z+10,3.5])color("grey")shutter();}
