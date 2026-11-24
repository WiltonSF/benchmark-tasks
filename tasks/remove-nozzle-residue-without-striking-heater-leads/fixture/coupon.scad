// Millimetres. Proposed coupon, unqualified transfer/bond. One layer, 0.2 mm PETG.
// openscad -o coupon.stl coupon.scad; for a 24-coupon plate: -D batch=true
batch = false;
module coupon() {
  linear_extrude(height=0.2) difference() {
    translate([-6,-9.5]) square([9.5,19]);
    circle(r=4.5,$fn=180);
  }
}
if(batch) for(i=[0:5],j=[0:3]) translate([12*i,22*j,0]) coupon();
else coupon();
