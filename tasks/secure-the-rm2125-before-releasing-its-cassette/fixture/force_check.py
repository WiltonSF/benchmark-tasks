"""Independent travel/force calculation for the vertical roller follower; not a hardware measurement."""
import math,json
FREE=15.88;SOLID=3.10;RATE=.736;PRELOAD=1.0;SPACER=1.2
ARM=40.;FOOT_DOWN=101.;ARM_TOP=4.;ROLLER_R=4.;MODEL_K=1.4;MODEL_PRE=.06

def check():
 theta=math.radians(15);sec=1/math.cos(theta)
 # The plunger is constrained vertically at x=-40mm. A roller follows the rotating
 # arm's upper face,4mm above its pivot. Virtual work gives its resisting torque.
 travel=ARM*math.tan(theta)+(ARM_TOP+ROLLER_R)*(sec-1)
 slope=ARM*sec*sec+(ARM_TOP+ROLLER_R)*sec*math.tan(theta)
 k=2*(RATE/2);spring_force=k*(PRELOAD+travel)
 closed=2*FREE+SPACER-PRELOAD;opened=closed-travel;solid=2*SOLID+SPACER
 torque=spring_force*slope/1000
 foot_arm=(FOOT_DOWN*math.cos(theta)-ARM*math.sin(theta))/1000
 ideal=torque/foot_arm;model=(MODEL_PRE+MODEL_K*theta)/foot_arm
 lip_rise=ARM*math.sin(theta)+14.2*(1-math.cos(theta))
 result=dict(angle_deg=15,lever40mm_point_rise_mm=ARM*math.sin(theta),lip_bottom_rise_mm=lip_rise,
  spring_follower_travel_mm=travel,each_stack_free_mm=2*FREE+SPACER,
  seat_gap_closed_mm=closed,seat_gap_open_mm=opened,solid_stack_mm=solid,
  remaining_to_solid_mm=opened-solid,equivalent_N_per_mm=k,force_at_foot_ideal_N=ideal,
  force_at_foot_simulated_N=model,gravity_allowance_N=.30,model_gravity_allowance_N=.55,force_at_foot_model_total_N=model+.55,
  spring_load_each_N=spring_force/2,start_height_mm=175.25,top_height_mm=190.,yoke_slot_centre_initial_mm=154.)
 assert 10<lip_rise<11 and 10<travel<11.1 and opened-solid>12
 assert 3.5<=ideal<=5.5 and 3.5<=model<=5.5 and model>=ideal
 assert ideal*.9>3.5 and ideal*1.1+.30<5.5
 assert spring_force/2<9.27
 assert model+.55<5.5
 # The model's torque is conservative over the nominal spring's entire opening path.
 for i in range(101):
  a=theta*i/100;c=1/math.cos(a)
  displacement=ARM*math.tan(a)+(ARM_TOP+ROLLER_R)*(c-1)
  derivative=ARM*c*c+(ARM_TOP+ROLLER_R)*c*math.tan(a)
  real_torque=k*(PRELOAD+displacement)*derivative/1000
  assert MODEL_PRE+MODEL_K*a>=real_torque-1e-10
 return result
if __name__=='__main__':print(json.dumps(check(),indent=2))
