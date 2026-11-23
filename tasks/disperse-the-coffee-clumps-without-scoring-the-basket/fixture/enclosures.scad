// Generated from task.py MACHINE/GRINDER; units mm, world x/y, counter z=0.
// 3mm closed shells preserve every external dimension; passive, no functional internals.
module panelbox(s){difference(){cube(s,center=true);cube(s-[6,6,6],center=true);}}
module tubecyl(r,h){difference(){cylinder(r=r,h=h,center=true,$fn=64);if(r>6 && h>6)cylinder(r=r-3,h=h-6,center=true,$fn=64);}}
// espresso_machine
translate([697.5,100,200]) panelbox([235,330,320]);
translate([660,100,12.5]) panelbox([310,330,25]);
translate([542.5,100,250]) panelbox([75,275,35]);
translate([540,100,225]) tubecyl(32,25);
translate([555,100,33]) panelbox([90,260,14]);
translate([715,180,367.5]) tubecyl(65,65);
translate([545,-45,145]) tubecyl(6,190);
// coffee_grinder
translate([650,-235,115]) panelbox([160,150,230]);
translate([650,-235,305]) tubecyl(75,150);
translate([572,-235,155]) panelbox([65,85,40]);
translate([630,-235,6]) panelbox([200,160,12]);
