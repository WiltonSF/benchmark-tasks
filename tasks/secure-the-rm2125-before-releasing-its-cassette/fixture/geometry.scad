// Millimetres. Original geometry; edit PART for printable link selection.
$fn=72;
PART="assembly";
module instrument_housing_base(){union(){
translate([0.0, 0.0, 9.0]) cube([472.0, 438.0, 18.0],center=true);
translate([230.0, 0.0, 141.5]) cube([12.0, 438.0, 247.0],center=true);
translate([0.0, 213.0, 141.5]) cube([448.0, 12.0, 247.0],center=true);
translate([21.0, -213.0, 141.5]) cube([430.0, 12.0, 247.0],center=true);
translate([-223.0, -213.0, 71.0]) cube([26.0, 12.0, 106.0],center=true);
translate([-223.0, -213.0, 210.5]) cube([26.0, 12.0, 109.0],center=true);
translate([0.0, 0.0, 259.0]) cube([472.0, 438.0, 12.0],center=true);
translate([-230.0, -123.0, 135.5]) cube([12.0, 180.0, 235.0],center=true);
translate([-230.0, 150.0, 135.5]) cube([12.0, 126.0, 235.0],center=true);
}}
if(PART=="instrument_housing_base") instrument_housing_base();
if(PART=="assembly") translate([596.0, -16.0, 0.0]) instrument_housing_base();
module lock_receiver_base(){union(){
translate([-58.0, -2.0, 4.999999999999997]) cube([84.0, 12.0, 210.00000000000003],center=true);
translate([58.0, -2.0, 4.999999999999997]) cube([84.0, 12.0, 210.00000000000003],center=true);
translate([0.0, -2.0, -58.0]) cube([32.0, 12.0, 84.0],center=true);
translate([0.0, -2.0, 47.25]) cube([32.0, 12.0, 62.5],center=true);
translate([0.0, -2.0, 100.75]) cube([32.0, 12.0, 18.500000000000004],center=true);
translate([-11.25, -2.0, 84.99999999999999]) cube([9.500000000000002, 12.0, 12.999999999999998],center=true);
translate([11.25, -2.0, 84.99999999999999]) cube([9.500000000000002, 12.0, 12.999999999999998],center=true);
}}
if(PART=="lock_receiver_base") lock_receiver_base();
if(PART=="assembly") translate([380.0, -255.0, 140.0]) lock_receiver_base();
module handwheel_base(){union(){
translate([0.0, 0.0, 0.0]) rotate([90,0,0]) cylinder(r=15.00000,h=16.00000,center=true);
translate([0.0, 0.0, -77.5]) cube([40.0, 16.0, 125.0],center=true);
translate([0.0, 10.0, -137.0]) cube([80.0, 60.0, 6.0],center=true);
}}
if(PART=="handwheel_base") handwheel_base();
if(PART=="assembly") translate([380.0, -255.0, 140.0]) handwheel_base();
module handwheel_wheel(){union(){
translate([0.0, -18.0, 0.0]) rotate([90,0,0]) cylinder(r=100.00000,h=12.00000,center=true);
translate([0.0, 130.0, 0.0]) rotate([90,0,0]) cylinder(r=4.00000,h=310.00000,center=true);
translate([0.0, 280.0, 0.0]) rotate([90,0,0]) cylinder(r=38.00000,h=12.00000,center=true);
translate([-25.54774941164094, 294.0, 14.750000000000004]) rotate([90,0,0]) cylinder(r=5.00000,h=28.00000,center=true);
}}
if(PART=="handwheel_wheel") handwheel_wheel();
if(PART=="assembly") translate([380.0, -255.0, 140.0]) handwheel_wheel();
module handwheel_handle(){union(){
translate([0.0, -15.5, 0.0]) rotate([90,0,0]) cylinder(r=12.00000,h=80.00000,center=true);
translate([0.0, 15.5, 0.0]) rotate([90,0,0]) cylinder(r=4.00000,h=64.00000,center=true);
}}
if(PART=="handwheel_handle") handwheel_handle();
if(PART=="assembly") translate([306.3878406783227, -314.5, 182.5]) handwheel_handle();
module specimen_guide_base(){union(){
translate([-29.0, -60.0, 162.5]) cylinder(r=3.97000,h=250.00000,center=true);
translate([-29.0, 60.0, 162.5]) cylinder(r=3.97000,h=250.00000,center=true);
translate([-15.0, 0.0, 249.0]) cube([22.0, 138.0, 8.0],center=true);
translate([-15.0, 0.0, 41.5]) cube([22.0, 138.0, 8.0],center=true);
translate([-29.0, -60.0, 282.5]) cube([14.0, 14.0, 10.0],center=true);
translate([-21.5, -60.0, 272.0]) cube([8.0, 8.0, 14.0],center=true);
translate([-2.5, -60.0, 270.0]) cube([43.0, 8.0, 10.0],center=true);
translate([0.0, -60.0, 270.0]) cylinder(r=5.00000,h=10.00000,center=true);
translate([14.0, -60.0, 270.0]) cylinder(r=5.00000,h=10.00000,center=true);
translate([-29.0, 60.0, 282.5]) cube([14.0, 14.0, 10.0],center=true);
translate([-21.5, 60.0, 272.0]) cube([8.0, 8.0, 14.0],center=true);
translate([-2.5, 60.0, 270.0]) cube([43.0, 8.0, 10.0],center=true);
translate([0.0, 60.0, 270.0]) cylinder(r=5.00000,h=10.00000,center=true);
translate([14.0, 60.0, 270.0]) cylinder(r=5.00000,h=10.00000,center=true);
}}
if(PART=="specimen_guide_base") specimen_guide_base();
if(PART=="assembly") translate([380.0, 55.0, 0.0]) specimen_guide_base();
module specimen_guide_head(){union(){
translate([-7.249999999999999, 73.34000000000002, -27.838400000000043]) cube([24.0, 50.0, 46.0],center=true);
translate([-7.249999999999999, 52.34000000000001, 11.161599999999966]) cube([24.0, 8.0, 32.0],center=true);
translate([-7.249999999999999, 94.34000000000002, 11.161599999999966]) cube([24.0, 8.0, 32.0],center=true);
translate([-14.249999999999998, 73.34000000000002, -11.838400000000027]) cube([22.0, 30.0, 26.0],center=true);
translate([-29.25, 73.34000000000002, -29.03840000000002]) cube([20.0, 30.0, 6.0],center=true);
translate([50.75, 63.340000000000025, -42.83840000000003]) cube([84.0, 20.0, 8.0],center=true);
translate([50.75, 63.340000000000025, -23.33840000000001]) cube([84.0, 20.0, 8.0],center=true);
translate([3.7500000000000036, 63.340000000000025, -33.08840000000002]) cube([10.0, 20.0, 11.5],center=true);
translate([97.75, 63.340000000000025, -33.08840000000002]) cube([10.0, 20.0, 11.5],center=true);
translate([5.750000000000005, 128.34, -27.838400000000043]) cube([40.0, 80.0, 12.0],center=true);
translate([9.750000000000002, 112.34000000000002, 9.161599999999964]) cube([12.0, 12.0, 86.0],center=true);
translate([9.750000000000002, -18.159999999999982, 56.16159999999998]) cube([12.0, 277.0, 8.0],center=true);
translate([-12.499999999999996, -103.65999999999997, 56.16159999999998]) cube([44.5, 12.0, 8.0],center=true);
translate([-34.75, -103.65999999999997, 39.36159999999997]) cube([12.0, 12.0, 36.0],center=true);
translate([-34.75, -105.65999999999998, 21.36159999999998]) cube([32.0, 16.0, 6.0],center=true);
translate([-34.75, -126.66, 65.32159999999998]) cube([33.0, 34.0, 4.0],center=true);
translate([-9.249999999999995, 73.34000000000002, -59.838400000000014]) cube([20.0, 138.0, 12.0],center=true);
translate([-9.249999999999995, 73.34000000000002, -48.83840000000003]) cube([20.0, 50.0, 14.0],center=true);
translate([10.750000000000002, 13.340000000000018, -59.838400000000014]) cube([38.0, 18.0, 12.0],center=true);
translate([10.750000000000002, 133.34, -59.838400000000014]) cube([38.0, 18.0, 12.0],center=true);
translate([-34.75, -141.66, 41.36159999999997]) cube([33.0, 4.0, 46.0],center=true);
translate([-34.75, -111.65999999999998, 41.36159999999997]) cube([33.0, 4.0, 46.0],center=true);
translate([9.750000000000002, -147.66, 56.16159999999998]) cube([25.0, 24.0, 20.0],center=true);
translate([5.250000000000004, -147.66, 34.16159999999996]) cube([16.0, 12.0, 34.0],center=true);
translate([5.250000000000004, -147.66, 14.361599999999974]) rotate([90,0,0]) cylinder(r=8.00000,h=8.00000,center=true);
}}
if(PART=="specimen_guide_head") specimen_guide_head();
if(PART=="assembly") translate([329.25, -18.340000000000018, 187.08840000000004]) specimen_guide_head();
module specimen_guide_clamp(){union(){
translate([-19.000000000000004, 0.0, -9.000000000000007]) cube([48.0, 30.0, 6.0],center=true);
translate([-40.00000000000001, 0.0, -11.199999999999989]) cube([3.0, 30.0, 6.0],center=true);
translate([5.499999999999998, -100.0, 0.0]) cube([10.0, 211.0, 9.0],center=true);
translate([0.0, 0.0, -2.2000000000000073]) cube([15.0, 22.0, 14.0],center=true);
translate([-21.000000000000004, -200.0, 0.0]) cube([49.0, 24.0, 8.0],center=true);
translate([-40.00000000000001, -200.0, -50.499999999999986]) cube([14.0, 24.0, 101.0],center=true);
}}
if(PART=="specimen_guide_clamp") specimen_guide_clamp();
if(PART=="assembly") translate([334.5, 55.0, 201.45000000000002]) specimen_guide_clamp();
module knife_holder_e_base(){union(){
translate([0.0, 0.0, 15.0]) cube([100.0, 110.0, 30.0],center=true);
translate([0.0, 0.0, 65.0]) cube([40.0, 100.0, 70.0],center=true);
translate([27.0, 0.0, 95.0]) cube([14.0, 100.0, 16.0],center=true);
translate([33.0, 0.0, 101.0]) cube([0.5, 80.0, 8.0],center=true);
}}
if(PART=="knife_holder_e_base") knife_holder_e_base();
if(PART=="assembly") translate([260.0, 55.0, 0.0]) knife_holder_e_base();
module knife_guard_base(){union(){
translate([0.0, -58.0, -4.0]) cube([12.0, 8.0, 15.0],center=true);
translate([0.0, 58.0, -4.0]) cube([12.0, 8.0, 15.0],center=true);
}}
if(PART=="knife_guard_base") knife_guard_base();
if(PART=="assembly") translate([250.5, 55.0, 117.0]) knife_guard_base();
module knife_guard_guard(){union(){
translate([-23.0, 0.0, 0.0]) cube([46.0, 100.0, 4.0],center=true);
translate([-44.0, 0.0, -6.0]) cube([4.0, 100.0, 10.0],center=true);
translate([-34.0, 90.0, 2.0]) cube([24.0, 94.0, 8.0],center=true);
translate([-34.0, 110.0, 16.0]) rotate([90,0,0]) cylinder(r=12.00000,h=30.00000,center=true);
}}
if(PART=="knife_guard_guard") knife_guard_guard();
if(PART=="assembly") translate([250.5, 55.0, 117.0]) knife_guard_guard();
module cassette_base(){union(){
translate([0.0, 0.0, 0.0]) cube([6.5, 40.4, 28.400000000000002],center=true);
translate([-6.25, 0.0, 0.0]) cube([6.0, 30.0, 22.0],center=true);
}}
if(PART=="cassette_base") cassette_base();
if(PART=="assembly") translate([300.0, 55.0, 175.25000000000003]) cassette_base();
module grey_tray_base(){union(){
translate([0.0, 0.0, 1.5]) cube([148.0, 108.0, 3.0],center=true);
translate([-72.0, 0.0, 11.0]) cube([4.0, 108.0, 22.0],center=true);
translate([72.0, 0.0, 11.0]) cube([4.0, 108.0, 22.0],center=true);
translate([0.0, -52.0, 11.0]) cube([140.0, 4.0, 22.0],center=true);
translate([0.0, 52.0, 11.0]) cube([140.0, 4.0, 22.0],center=true);
}}
if(PART=="grey_tray_base") grey_tray_base();
if(PART=="assembly") translate([215.0, 215.0, 0.0]) grey_tray_base();
