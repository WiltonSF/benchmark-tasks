// Monolithic A36 shoe, A36 contoured spreader,1144 backing plate and uniform bushes; mm.
view="assembly"; // shoe,inner_plate,outer_wedge,bush,assembly
plate_t=2.5; dw=.5; di=.58; bush_od=4.4; socket_d=4.41; bush_id=3.015;
inner_face=-.381082; bush_L=7.618918-plate_t+dw+di; holes=[16.9,23.7];
function nylon(z)=-4.580218-tan(10)*(z-17);
module shoe(){difference(){union(){translate([-11,-11,0])cube([27,22,4]);translate([-12,-11,0])cube([4,22,28]);}for(z=holes){translate([-13,0,z])rotate([0,90,0])cylinder(h=6,d=2.5,$fn=64);translate([-7.999,0,z])rotate([0,-90,0])cylinder(h=dw+.001,d=socket_d,$fn=64);}}}
module plate_blank(){rotate([90,0,0])linear_extrude(height=13,center=true)polygon(points=[[inner_face-plate_t+.4,12.8],[inner_face,12.8],[inner_face,28.5],[inner_face-plate_t,28.5],[inner_face-plate_t,13.2]]);}
module inner_plate(){difference(){plate_blank();for(z=holes){translate([inner_face-plate_t-.001,0,z])rotate([0,90,0])cylinder(h=di+.001,d=socket_d,$fn=64);translate([inner_face+1,0,z])rotate([0,-90,0])cylinder(h=4,d=3.3,$fn=64);translate([inner_face+.001,0,z])rotate([0,-90,0])cylinder(h=1.73,d1=6.76,d2=3.3,$fn=64);}}}
module outer_wedge(){difference(){polyhedron(points=[[-8,-6.5,12.8],[-8,6.5,12.8],[-8,-6.5,28.5],[-8,6.5,28.5],[nylon(12.8)-.05,-6.5,12.8],[nylon(12.8)-.05,6.5,12.8],[nylon(28.5)-.05,-6.5,28.5],[nylon(28.5)-.05,6.5,28.5]],faces=[[0,2,3,1],[4,5,7,6],[0,1,5,4],[2,6,7,3],[0,4,6,2],[1,3,7,5]]);for(z=holes)translate([-9,0,z])rotate([0,90,0])cylinder(h=10,d=4.5,$fn=64);}}
module bush(){difference(){cylinder(h=bush_L,d=bush_od,$fn=64);translate([0,0,-1])cylinder(h=bush_L+2,d=bush_id,$fn=64);}}
if(view=="shoe")shoe();if(view=="inner_plate")inner_plate();if(view=="outer_wedge")outer_wedge();if(view=="bush")bush();
if(view=="assembly"){shoe();inner_plate();outer_wedge();for(z=holes)translate([-8-dw,0,z])rotate([0,90,0])bush();}
