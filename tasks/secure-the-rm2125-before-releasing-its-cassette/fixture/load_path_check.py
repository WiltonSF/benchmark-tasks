"""Reproducible elastic bounds, N/mm units. Analytical, not measured hardware.
Steel E=200GPa/G=77GPa engineering assumptions; printed PETG E=2050MPa
from maker bending data, https://store.bblcdn.com/ce12d65176a94f1086e6aefa238e62e2.pdf.
Joint seating and loaded loss must be checked on an assembled fixture.
"""
import json, math, pathlib, sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]));import task as t
E=200000.;G=77000.;EP=2050.
d=4.7625;L=242.;a=21.;b=L-a;P=math.hypot(5.5,10.)
I=math.pi*d**4/64;Ie=.7*I # 30% stiffness reduction for local dog-point dimple
shaft=P*a*a*b*b/(3*E*Ie*L)
stress=(P*a*b/L)*(d/2)/Ie
side=6.35;J=.1406*side**4
spine_twist=550*195/(G*J)*1.2 # holes/joint allowance; no-slip proof still required
spine_lip=45*spine_twist
OD=12.7;wall=.0478*25.4;ID=OD-2*wall
It=math.pi*(OD**4-ID**4)/64;Jt=2*It
reaction=5.5*b/L
rail_bend=reaction*260**3/(3*E*It)
upright_twist=reaction*260*90/(G*Jt)
frame=rail_bend+260*upright_twist+.04 # .04mm vertical/self-weight allowance
web=reaction*42**3/(3*500*(12*16**3/12)) # effective quarter-modulus printed hanger
# Simply supported 8mm guides (conservative relative to clamped ends): conservative mid-span 13N total on one rod.
guide_elastic=13*212**3/(48*E*(math.pi*8**4/64))
guide_bound=guide_elastic+.06 # bracket/roof seating allowance, enrolled not measured
radial=2*(4.85-d)/2 # worst independent bearing and rocker relief motion
axis_motion=frame+web+shaft+.02+radial+guide_bound
ratio=45/math.hypot(40,88.2)
foot=5.5*101**3/(3*EP*(24*14**3/12))
# 40mm arm: subtract full5mm groove thickness for a conservative section.
arm=10*40**3/(3*EP*(24*(8**3-5**3)/12))
loss=axis_motion*ratio+foot*ratio+arm+spine_lip
nominal=45*math.sin(math.radians(15)) # scoring lip is at x45mm; actual CAM accounts rotation
# Fixture lip offsets make actual nominal vertical lift10.8366mm (force_check geometry).
nominal=10.8366
wheel_max=(.58+.542*9.81*.0295*math.sin(math.pi/3)+.055*9.81*.085*math.sin(math.pi/3))/.085
wheel_min=.58/.085
# Geometry-based native COM at15deg; mass assignment follows current runtime.
vol=np.array([np.prod(sh.size) for sh in t.clamp.shapes]);centres=np.array([np.array(sh.at)-t.clamp.origin for sh in t.clamp.shapes])
com=(centres*vol[:,None]).sum(0)/vol.sum();R=t.parts._rot((0,1,0),math.pi/12)
lever=.101*math.cos(math.pi/12)-.040*math.sin(math.pi/12)
native_gravity=t.clamp.mass*9.81*abs((R@com)[0])/lever
# Solid PETG1260kg/m3 minus spine channel and estimated1.5cm3 axle/fastener relief.
pmass=vol.sum()*1260;moment=(centres*vol[:,None]*1260).sum(0)
for v,c in [( .0066**2*.211,np.array([.0058,-.100,0.])),(1.5e-6,np.array([-.001,0,0]))]:pmass-=v*1260;moment-=c*v*1260
for mass,c in [(.00635**2*.211*7850,[.0058,-.100,0]),(math.pi*(.0047625/2)**2*.264*7850,[0,-.101,0]),(.004,[.0058,-.100,-.005])]:pmass+=mass;moment+=mass*np.array(c)
physical_gravity=9.81*abs((R@moment)[0])/lever
assert native_gravity<.55 and physical_gravity<.30
assert stress<200 and frame<.55 and web<.07 and guide_bound<.13
assert loss<1.,loss
assert 6<=wheel_min<=wheel_max<=9
out=dict(runtime_sha256=t.SOURCE_SHA256,native_gravity_at15_N=native_gravity,physical_mass_estimate_kg=float(pmass),physical_gravity_estimate_at15_N=physical_gravity,physical_gravity_allowance_N=.30,steel_E_MPa=E,steel_G_MPa=G,PETG_bending_E_MPa=EP,shaft_effective_I_factor=.7,shaft_deflection_mm=shaft,shaft_bending_stress_MPa=stress,steel_spine_lip_loss_mm=spine_lip,frame_tip_bound_mm=frame,hanger_web_deflection_mm=web,guide_elastic_mm=guide_elastic,guide_with_seating_mm=guide_bound,bearing_and_relief_play_mm=radial,axis_motion_bound_mm=axis_motion,foot_elastic_mm=foot,arm_elastic_mm=arm,loaded_lip_loss_conditional_estimate_mm=loss,loaded_lip_lift_min_mm=nominal-loss,nominal_lip_lift_mm=nominal,wheel_force_60_to_0_deg_N=[wheel_min,wheel_max],requirements='Before enrollment: finish bores4.80–4.85mm; spine proof0.55Nm without slip; actual15degree opening and10.84+/-1mm lip lift simultaneously at3.5–5.3N foot force;10N horizontal workholding proof; reject motion or loss outside tolerance. Unbuilt, unmeasured; simplified linear-elastic conditional estimates do not validate joints or hardware.')
print(json.dumps(out,indent=2))
