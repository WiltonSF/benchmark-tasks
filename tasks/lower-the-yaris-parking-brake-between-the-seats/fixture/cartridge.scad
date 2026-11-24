// Guarded new-draw-rod cartridge; mm. Fabricated dimensions, not OEM CAD. See stock-load-design.md.
// Fixed front plate u126..132. The coupled new draw rod pulls the follower toward +u, compressing springs.
// Moving follower u27..33 at zero total compression, u49.1..55.1 at22.1mm stop.
// OEM nut/rod lines below are illustrative acceptance-envelope geometry only.
view="assembly"; compression=0; L1=45; L2=45; front_stop=8; total_stop=22.1; stop_length=L1+L2+3-total_stop; front_comp=min(compression/2,front_stop); rear_comp=compression-front_comp; f0=126-(L1+L2+3)-6;
module plate(front=false){difference(){translate([-30,-30,0])cube([60,60,6]);
 if(front){translate([0,0,-1])cylinder(h=8,d=11,$fn=64);translate([0,0,-.1])cylinder(h=3.1,d=23,$fn=96);}
 else translate([0,0,-1])cylinder(h=8,d=16,$fn=64);
 for(x=[-22,22],y=[-22,22])translate([x,y,-1])cylinder(h=8,d=10,$fn=48);
 }}
module ring(od,id,length){difference(){cylinder(h=length,d=od,$fn=96);translate([0,0,-1])cylinder(h=length+2,d=id,$fn=96);}}
module follower(){ring(30,10.5,6);}
if(view=="rear_plate")plate();
if(view=="front_plate")plate(true);
if(view=="follower")follower();
if(view=="separator")ring(24,10.5,3);
if(view=="staged_stop")ring(23,21,L2-front_stop+3);
if(view=="front_seat")ring(20.8,10.5,3);
if(view=="stop")ring(33.4,26.64,stop_length);
if(view=="cage")ring(42.164,35.052,120);
if(view=="assembly"){
 color("silver")plate();translate([0,0,126])color("silver")plate(true);
 // Four180mm ties,24mm each outside132mm frame;2mm washer+two8.5mm fullnuts fit with5mm spare thread.
 for(x=[-22,22],y=[-22,22]){
 translate([x,y,-24])color("grey")cylinder(h=180,d=9.525,$fn=32);
 for(u=[-19,-10.5,134,142.5])translate([x,y,u])color("silver")ring(14.29/cos(30),9.6,8.5);
 for(u=[-2,132])translate([x,y,u])color("silver")ring(25.4,10.5,2);
 }
 translate([0,0,6])color([.4,.5,.6,.18])ring(42.164,35.052,120);
 translate([0,0,126-stop_length])color([.4,.5,.6,.3])ring(33.4,26.64,stop_length);
 translate([0,0,f0+compression])color("silver")follower();
 // Total compression includes preload. Matched-rate illustration; verify actual spring deflections.
 translate([0,0,f0+6+compression])color([.8,.1,.1,.4])ring(20,10,L1-rear_comp);
 translate([0,0,126-L2+front_comp-3])color("silver")ring(24,10.5,3);
 translate([0,0,126-L2+front_comp])color([.8,.1,.1,.4])ring(20,10,L2-front_comp);
 translate([0,0,126-(L2-front_stop)])color([.3,.5,.6,.45])ring(23,21,L2-front_stop+3);
 translate([0,0,126])color("silver")ring(20.8,10.5,3);
 // New 3/8 draw rod and two follower retaining nuts; original OEM rod is in the separate yoke drawing.
 translate([0,0,-14+compression])color("grey")cylinder(h=270,d=9.525,$fn=48);
 for(u=[f0-19,f0-10.5])translate([0,0,u+compression])color("grey")cylinder(h=8.5,d=14.29/cos(30),$fn=6);
}

// Yoke coordinates: a rearwards along +Z from the front of the 3 mm plate, X across, Y up.
module yoke(){difference(){union(){translate([-20,-12,0])cube([40,24,3]);for(x=[-20,14])translate([x,-12,3])cube([6,24,55]);}
 hull(){for(y=[-3.5,3.5])translate([0,y,-1])cylinder(h=5,d=10,$fn=64);}
 translate([-21,0,50])rotate([0,90,0])cylinder(h=42,d=10.2,$fn=64);
}}
module eye(){difference(){translate([-6,-13,40])cube([12,26,50]);translate([-7,0,50])rotate([0,90,0])cylinder(h=14,d=10.2,$fn=64);translate([0,0,62])cylinder(h=29,d=7.9375,$fn=64);translate([0,0,59])cylinder(h=3,d1=0,d2=7.9375,$fn=64);}} // 5/16 pilot:28mm full diameter, tip<=31mm;bottoming-tap3/8-16 full20mm. Not a major-diameter clearance hole.
module retained_pin(){difference(){cylinder(h=40,d=10,$fn=64);translate([0,0,-.1])cylinder(h=8.1,d=2.5,$fn=32);translate([0,0,32])cylinder(h=8.1,d=2.5,$fn=32);translate([0,0,-.01])cylinder(h=.41,d1=3.8,d2=3,$fn=48);translate([0,0,39.6])cylinder(h=.41,d1=3,d2=3.8,$fn=48);}} // tapM3>=6.5mm aftercountersink; pilot8mm fullcyl plus drillpoint<=9mm.
if(view=="pin")retained_pin();
if(view=="yoke")yoke();if(view=="eye")eye();
if(view=="coupling") {color("silver")yoke();color("grey")eye();for(x=[-14,6])translate([x,0,50])rotate([0,90,0])ring(16,10.2,8);translate([-20,0,50])rotate([0,90,0])retained_pin();translate([0,0,3])ring(26,8.5,3);}
// The two M3 pin-end bores are8mm deep;16OD/3.3ID/1.5 retainers/M3x8, in the written cut list.
