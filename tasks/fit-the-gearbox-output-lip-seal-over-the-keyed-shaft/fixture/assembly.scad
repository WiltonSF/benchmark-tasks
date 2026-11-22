// mm. Full PLE080 envelope COPY; custom carrier/cage; not OEM CAD.
// PART: assembly,housing,carrier,cage,shaft,collar,sleeve,driver,vise.
PART="assembly"; $fn=96;
module annulus(ri,ro,h){difference(){cylinder(r=ro,h=h);translate([0,0,-1])cylinder(r=ri,h=h+2);}}
module axial_holes(h,d=4.5){for(a=[45:90:315])rotate([0,0,a])translate([26,0,-1])cylinder(d=d,h=h+2);}
module housing(){difference(){union(){difference(){union(){translate([-45,-45,0])cube([90,90,12]);translate([0,0,12])cylinder(d=80,h=69.5);}translate([0,0,-1])cylinder(d=60,h=84);}for(a=[45:90:315])rotate([0,0,a]){translate([26,0,65.5])cylinder(d=7,h=16);intersection(){translate([26,-3,73.5])cube([14,6,8]);cylinder(d=80,h=82);}}}translate([0,0,73.5])axial_holes(9,3.3);}}
// Replaces upper12mm spigot. 35H8 finished bore,0.5 radial x1 axial entry lead.
module carrier(){difference(){cylinder(d=60,h=12);translate([0,0,1])cylinder(d=35,h=12);translate([0,0,11])cylinder(d1=35,d2=36,h=1.01);translate([0,0,-1])cylinder(d=23,h=14);axial_holes(12);}}
// Independently supported steel bearing cup; no carrier/body contact.
module cage(){difference(){union(){cylinder(d=59,h=2);translate([0,0,2])cylinder(d=44,h=18);}translate([0,0,-1])cylinder(d=38,h=22);translate([0,0,8])cylinder(d=42,h=13);for(a=[0:90:270])rotate([0,0,a])translate([25.5,0,-1])cylinder(d=3.5,h=4);for(a=[0:90:270])rotate([0,0,a])translate([25.5,0,0]){translate([0,0,.4])cylinder(d1=3.5,d2=6.2,h=1.35);translate([0,0,1.75])cylinder(d=6.2,h=1);}}}
module shaft(){difference(){union(){cylinder(d=20,h=75.5);translate([0,0,23.5])cylinder(d=22,h=4);}translate([7,-3,50.5])cube([5,6,26]);}}
// Steel20ID x32OD x8 split collar:0.8 slit; M3 clearance/tap across slit.
module collar(){difference(){annulus(10,16,8);translate([9,-.4,-1])cube([9,.8,10]);translate([13,0,4])rotate([-90,0,0])cylinder(d=3.4,h=12);translate([13,2,4])rotate([-90,0,0])cylinder(d=5.8,h=12);translate([13,-12,4])rotate([-90,0,0])cylinder(d=2.5,h=12);}}
module collar_screw(){translate([13,-10,4])rotate([-90,0,0])cylinder(d=3,h=12);translate([13,2,4])rotate([-90,0,0])cylinder(d=5.5,h=3);}
module sleeve(){difference(){cylinder(r=10.6,h=51);translate([0,0,-1])cylinder(r=10.2,h=53);cylinder(r1=10.5,r2=10.2,h=2);}}
module guide_insert(){difference(){cylinder(d=29,h=7);translate([0,0,-1])cylinder(d=21.3,h=9);cylinder(d1=24,d2=21.3,h=3);}}
module driver(){difference(){annulus(14.5,19,50);for(z=[12,36])for(a=[0:120:359])rotate([0,0,a])translate([13,0,z])rotate([0,90,0])cylinder(d=2.5,h=8);}for(z=[7,31])translate([0,0,z])color("dimgray")guide_insert();}
module vise(){difference(){translate([-70,-70,0])cube([140,140,4]);for(a=[0:90:270])rotate([0,0,a])translate([25.5,0,-1])cylinder(d=14,h=6);for(x=[-60,60])for(y=[-60,60])translate([x,y,-1])cylinder(d=4.5,h=6);}for(y=[-53,53])translate([-45,y-8,4])cube([90,16,45]);translate([0,-118,25])rotate([-90,0,0])cylinder(d=8,h=60);translate([-40,-118,25])rotate([0,90,0])cylinder(d=6,h=80);}
if(PART=="housing")housing();if(PART=="carrier")carrier();if(PART=="cage")cage();if(PART=="shaft")shaft();if(PART=="collar")collar();if(PART=="sleeve")sleeve();if(PART=="driver")driver();if(PART=="vise")vise();
if(PART=="assembly"){
 color("dimgray")vise();color("gray")translate([0,0,4])housing();
 color("silver")translate([0,0,85.5])carrier();color("darkgray")translate([0,0,64])cage();
 color("gray")translate([0,0,72])annulus(10,21,12);
 color("silver")translate([0,0,62])shaft();color("gray")translate([0,0,64])collar();color("black")translate([0,0,64])collar_screw();
 color("gray")translate([0,0,84])annulus(10,11,1.5);
 color("blue",.5)translate([0,0,89.5])sleeve();translate([80,0,4])color("silver")driver();
}

// Instrument stack. A is45deg; B is-45deg, clearing cardinal shaft posts.
module cell(){difference(){translate([-37.5,-6.35,0])cube([75,12.7,12.7]);for(x=[-32,-22,22,32])translate([x,0,-1])cylinder(d=4.5,h=15);}}
module post_holes(d,z){for(a=[0:90:270])rotate([0,0,a])translate([25.5,0,z-1])cylinder(d=d,h=6);}
module cell_holes(angle,positions,z){rotate([0,0,angle])for(x=positions)translate([x,0,z-1])cylinder(d=4.5,h=6);}
module pad(){difference(){translate([-4,-6.35,0])cube([8,12.7,3.3]);translate([0,0,-1])cylinder(d=4.5,h=6);}}
module mounts(){
 color("silver")difference(){translate([-70,-70,-4])cube([140,140,4]);post_holes(14,-4);cell_holes(45,[22,32],-4);for(x=[-60,60])for(y=[-60,60])translate([x,y,-5])cylinder(d=4.5,h=6);}
 color("gray")translate([0,0,-20])rotate([0,0,45])cell();
 color("silver")rotate([0,0,45])for(x=[22,32])translate([x,0,-7.3])pad();
 color("silver")difference(){translate([-90,-90,-24])cube([180,180,4]);post_holes(3.5,-24);cell_holes(45,[-32,-22],-24);cell_holes(-45,[22,32],-24);rotate([0,0,45])translate([-17,-10,-25])cube([57,20,6]);}
 color("gray")translate([0,0,-40])rotate([0,0,-45])cell();
 color("silver")rotate([0,0,-45])for(x=[22,32])translate([x,0,-27.3])pad();
 color("silver")difference(){translate([-110,-110,-44])cube([220,220,4]);cell_holes(-45,[-32,-22],-44);rotate([0,0,-45])translate([-17,-10,-45])cube([57,20,6]);for(x=[-100,100])for(y=[-100,100])translate([x,y,-45])cylinder(d=6.5,h=6);}
 for(a=[0:90:270])rotate([0,0,a])translate([25.5,0,-20])color("silver")annulus(1.75,3,84);
}
module post_fasteners(){for(a=[0:90:270])rotate([0,0,a])translate([25.5,0,0]){translate([0,0,-34])cylinder(d=3,h=98.3);translate([0,0,64.3])cylinder(d1=3,d2=6,h=1.5);translate([0,0,65.8])cylinder(d=6,h=.2);translate([0,0,-26.4])annulus(1.5,3.2,2.4);}}
if(PART=="mounts"){mounts();color("black")post_fasteners();}
if(PART=="assembly"){mounts();color("black")post_fasteners();}
