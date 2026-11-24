// Complete OEM mount and rear cradle, mm; manufactured sizes, not OEM CAD.
view="oem_base"; shim_t=1.65; side=1; // mirror rear-cradle foot with side=-1
module slot(length,width,thickness){hull(){translate([-(length-width)/2,0,-1])cylinder(h=thickness+2,d=width,$fn=48);translate([(length-width)/2,0,-1])cylinder(h=thickness+2,d=width,$fn=48);}}
if(view=="oem_base")translate([-109,-25,0])cube([218,50,6.35]); // drill received key/stud positions after registration
if(view=="oem_shim")translate([-25,-40,0])cube([50,80,shim_t]);
if(view=="oem_capture")difference(){translate([-106,-25,0])cube([212,50,12.7]);translate([-10,-26,-1])cube([20,31,15]);for(x=[-90,90])translate([x,0,-1])cylinder(h=15,d=10,$fn=48);}
if(view=="oem_key_layer")difference(){translate([-18,-12.5,0])cube([36,25,6.35]);for(x=[-9,9])translate([x,0,-1])cylinder(h=9,d=10,$fn=48);}
if(view=="oem_key_spreader")difference(){translate([-18,-20,0])cube([36,40,6.35]);for(x=[-9,9])translate([x,0,-1])cylinder(h=9,d=10,$fn=48);}
if(view=="cartridge_post")translate([-75,-25,0])cube([150,50,6.35]);
if(view=="cartridge_foot")difference(){translate([-100,-25,0])cube([200,50,6.35]);for(x=[-85,85])translate([x,17*side,0])slot(20,11,6.35);}

// Clamp straps bear on the fixed OEM bracket flange; never clamp the moving lever or button linkage.

// Guard rails are400x25x6.35 edge-on;4hangers85x25x6.35 and2cross-supports220x25x6.35.
// Each100x50x6.35 bench mounting foot has2 diameter10 holes50mm apart; underside80x40x6 spreader.
if(view=="bench_foot")difference(){translate([-50,-25,0])cube([100,50,6.35]);for(x=[-25,25])translate([x,0,-1])cylinder(h=9,d=10,$fn=48);}

if(view=="oem_key_stack")for(i=[0:3])translate([0,0,6.35*i])difference(){translate([-18,-12.5,0])cube([36,25,6.35]);for(x=[-9,9])translate([x,0,-1])cylinder(h=9,d=10,$fn=48);}
