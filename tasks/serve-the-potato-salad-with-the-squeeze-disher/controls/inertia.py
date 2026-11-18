"""Prescribed constitutive trajectories, not robot/food calibration evidence."""
import pathlib,sys,numpy as np,math
from dataclasses import replace
P=pathlib.Path(__file__).parent;T=P.parent;sys.path.insert(0,str(T));import task as m
b,p=m.BOWL.state(),m.PLATE.state()
def run(hz,kind):
 dt=1/hz;f=m.Food();position=np.array((3.,3.,3.));time=0.
 def step(v):
  nonlocal position,time
  time+=dt;position=position+np.array((v*dt,0,0));tool=replace(m.DISHER.state(),pos=position.copy(),lin_vel=np.array((v,0,0)))
  f.advance(dt,b,tool,p,math.pi,0.)
 for _ in range(round(.1/dt)):step(0.)
 amount=.5*m.CAPACITY;f.cap=m.VW*.5;f.source*=1-amount/f.source.sum();onset=time
 for i in range(round(.5/dt)):
  t=(i+1)*dt
  if kind=='small_pulse':v=.25 if i==0 else 0.
  elif kind=='large_pulse':v=3. if i==0 else 0.
  elif kind=='accel30':v=30*t
  elif kind=='accel50':v=50*t
  else:v=3*math.sin(2*math.pi*(2 if kind=='shake2' else 20)*t)
  step(v)
  if f.inertial_releases:break
 released=bool(f.inertial_releases);expected=kind not in ('small_pulse','accel30');assert released==expected,(hz,kind,released,f.response_accel);assert not f.invalid and abs(f.total()-m.INITIAL.sum())<1e-10
 print(hz,kind,'release',released,'after_ms',round((time-onset)*1000,3),'response',f.response_accel)
for hz in (120,240,480):
 for kind in ('small_pulse','large_pulse','accel30','accel50','shake2','shake20'):run(hz,kind)
f=m.Food();f.cap=m.VW*.5;f.source*=1-f.cap.sum()/f.source.sum();tool=replace(m.DISHER.state(),pos=np.array((3.,3.,3.)));f.advance(1/240,b,tool,p,math.pi,0.);assert f.invalid and f.error=='unknown50ms inertial history'
f=m.Food();f.advance(1/240,b,replace(tool,lin_vel=np.array((np.nan,0,0))),p,math.pi,0.);assert f.invalid
f=m.Food();f.advance(.2,b,tool,p,math.pi,0.);assert f.invalid
print('All18 dynamic classifications and cold/NaN/gap fail-closed controls PASS. Tool-level50ms response is uncalibrated; not material-cohort-resolved.')
