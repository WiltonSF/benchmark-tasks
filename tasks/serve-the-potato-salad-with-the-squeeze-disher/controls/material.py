"""Prescribed trajectory; native friction application explicitly stubbed."""
from yam_arena.task_file import load
from dataclasses import replace
import numpy as np,math
import pathlib
m=load(str(pathlib.Path(__file__).resolve().parents[1]/'task.py')).module
b,p=m.BOWL.state(),m.PLATE.state();f=m.Food();original_advance=f.advance
def native_stub(*a,**kw):
 result=original_advance(*a,**kw);f.applied_mu=.04*min(1.,f.cap.sum()/m.CAPACITY);return result
f.advance=native_stub;
r=m.parts._rot((0,1,0),-math.radians(35))@m.parts._rot((0,0,1),math.pi)
def tool(head,q,vel=(0,0,0)):
 return replace(m.DISHER.state(joints={'lever':q/m.RATIO,'wiper':q}),pos=np.asarray(head)-r@m.HEAD,quat_xyzw=m.parts._matrix_to_quat(r),lin_vel=np.asarray(vel))
t=tool(b.pos+(-.015,-.025,.020),math.pi)
for _ in range(20):f.advance(.05,b,t,p,math.pi,0.)
assert f.cap.sum()==0,'stationary overlap cannot load food'
for dy in (-.025,.025):
 for z in np.linspace(.045,.020,37):
  t=tool(b.pos+(-.015,dy,z),math.pi,(0,0,-.021/1.8));f.advance(.05,b,t,p,math.pi,0.)
 for x in np.linspace(-.015,.005,25):
  t=tool(b.pos+(x,dy,.020),math.pi,(.015/1.2,0,0));f.advance(.05,b,t,p,math.pi,0.)
 for yy in np.linspace(dy,dy-math.copysign(.010,dy),21):
  t=tool(b.pos+(.005,yy,.020),math.pi,(0,-math.copysign(.010,dy),0));f.advance(.05,b,t,p,math.pi,0.)
 print('capture ml',f.cap.sum()*1e6,'work',f.input_work)
 held=f.cap.sum();t=tool(p.pos+(0,math.copysign(.040,dy),.045),math.pi)
 for _ in range(15):f.advance(.05,b,t,p,math.pi,0.)
 assert abs(f.cap.sum()-held)<1e-12,'ordinary carry/pour cannot unlock food'
 for q in np.linspace(math.pi,.45,37):
  t=tool(p.pos+(0,math.copysign(.040,dy),.045),float(q));f.advance(.05,b,t,p,float(q),-1.5)
 for _ in range(15):f.advance(.05,b,t,p,.45,0.)
 for q in np.linspace(.45,math.pi,17):
  t=tool(p.pos+(0,math.copysign(.040,dy),.045),float(q));f.advance(.05,b,t,p,float(q),3.)
 for _ in range(15):f.advance(.05,b,t,p,math.pi,0.)
 print('served ml',f.served()*1e6,'retained',f.cap.sum()*1e6,'flying',len(f.flying),'spill',f.spill*1e6,'conserved',f.total()-m.INITIAL.sum())
 assert abs(f.total()-m.INITIAL.sum())<1e-10 and not f.invalid
m._food=f;heaps=m.portions();print('components ml',[v*1e6 for v in heaps])
assert len(heaps)>=2 and heaps[1]>=14e-6 and f.served()>=34e-6 and f.spill<=3e-6
print('Prescribed material control passes; this is not a robot run.')
