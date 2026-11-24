// mm. Real-trial bench drawing for the core-withdrawal task. Axis y is the core axis; z is up; bench top z=0.
// Bought parts are drawn as envelopes only (steel-jaw vise drawn as a 25.4 x 50.8 mm jaw-contact envelope).
// part = "assembled" | "core" | "wall" | "jaw_gauge"
$fn=96;
part="assembled";
core_d=13.0; core_l=220; exposed=50;           // 304 stainless rod, ends chamfered 0.5 x 45 deg
wall_id=12.7; wall_od=15.875; wall_l=170;      // NewAge Silcon 1/2" x 5/8" Shore A50 tube, cut square
axis_z=80;                                    // core axis height above the bench
jaw_y=-115; jaw_w=25.4; jaw_h=50.8; jaw_t=12;  // jaw centre on the core axis frame (y), jaw size
core_y=-25;                                    // core centre; wall centre at y=0, far ends flush at y=+85
module core(){translate([0,core_y-core_l/2,axis_z])rotate([-90,0,0])cylinder(d=core_d,h=core_l);}
module wall(){translate([0,-wall_l/2,axis_z])rotate([-90,0,0])difference(){cylinder(d=wall_od,h=wall_l);translate([0,0,-1])cylinder(d=wall_id,h=wall_l+2);}}
module jaws(){
  translate([-jaw_h/2,jaw_y-jaw_w/2,axis_z-core_d/2-jaw_t])cube([jaw_h,jaw_w,jaw_t]);
  translate([-jaw_h/2,jaw_y-jaw_w/2,axis_z+core_d/2])cube([jaw_h,jaw_w,jaw_t]);
  translate([-62.5,jaw_y-62.5,0])cube([125,125,12]);
  translate([-20,jaw_y-20,12])cube([40,40,axis_z-core_d/2-jaw_t-12]);
}
// Reset gauge: a 3 mm plate with a 50.0 mm slot spanning rear core end y=-135 to wall start y=-85.
// It does NOT measure from a jaw face: the forward jaw edge is y=-102.3, 17.3 mm behind the wall start.
module jaw_gauge(){difference(){cube([60,20,3]);translate([5,5,-1])cube([exposed,10,5]);}}
if(part=="assembled"){color("silver")core();color("pink")wall();color("dimgray")jaws();}
if(part=="core")core();
if(part=="wall")wall();
if(part=="jaw_gauge")jaw_gauge();
