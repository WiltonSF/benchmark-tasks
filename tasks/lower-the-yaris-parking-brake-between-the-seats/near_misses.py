"""Constructed sample histories check the judge and its ratchet rules; none is physics evidence."""
import math,types
import numpy as np
import parts
shared=next((fn.__globals__ for fn in parts._ON_STEP if fn.__name__=='update'),None)
if shared is None:
 from yam_arena.task_file import load
 import pathlib
 task=load(pathlib.Path(__file__).with_name('task.py')).module
else:task=types.SimpleNamespace(**shared)
from mechanism import Sample,D

# Actuator integrity includes external gravity through the spawn-time performance envelope.
# A raised, missing or nonfinite native maximum is rejected, independently of policy targets.
import motors
_caps_names=list(motors.CAPS)
_caps_values=[motors.CAPS[n] for n in _caps_names]
assert motors.limits_valid(_caps_values,_caps_names)
for _bad in (float('nan'),28.):
 _v=list(_caps_values);_v[1]=_bad
 assert not motors.limits_valid(_v,_caps_names)
assert not motors.limits_valid(_caps_values[:-1],_caps_names[:-1])

def run(final=0.,lift=1.,press=.008,clear=.05,hold=1.1,how='lowered',bad=None,contact=True):
 """Feed the judge a 240 Hz history obeying its own stops; return the final brake state."""
 j=task._judge;j.reset();t=[0.];pose=[task.START_DEG,0.]
 def step(a,b=None,clearance=0.,integrity=True):
  b=pose[1] if b is None else b
  a=max(a,math.degrees(j.lower)) if j.previous is not None else a   # the pawl stop
  b=min(b,j.button_hi)                                             # the button stop
  t[0]+=1/240;pose[:]=[a,b];j.step(Sample(t[0],a*D,b,clearance,integrity,contact))
 step(task.START_DEG)
 for a in np.linspace(task.START_DEG,task.START_DEG+lift,25)[1:]:step(a)
 for b in np.linspace(0,press,13)[1:]:step(pose[0],b)
 if how=='dropped':
  for a in np.linspace(pose[0],final,13)[1:]:step(a)             # 0.05 s fall
 else:
  for a in np.linspace(pose[0],final,481)[1:]:step(a)            # 2 s controlled descent
 for b in np.linspace(pose[1],0,13)[1:]:step(pose[0],b)
 for a in np.linspace(pose[0],pose[0]-task.PITCH_DEG,49)[1:]:step(a)   # the cable settles the lever onto its tooth
 if how=='held':clear=0.
 for i in range(round(hold*240)):step(pose[0],clearance=clear)
 if bad=='integrity':step(pose[0],clearance=clear,integrity=False)
 state=task.BRAKE.state(joints={'lever':pose[0]*D-task.ANGLE,'button':pose[1]})
 task._unit_certificate['value']=task.signature(state)
 return state

def cases():
 specs=[('lowered fully, button out, hands clear',dict(),1.),
        ('stopped on the 5 degree tooth',dict(final=6.),.75),
        ('stopped on the 10 degree tooth',dict(final=10.5),.5),
        ('stopped on the 10 degree tooth from above',dict(final=13.),.5),
        ('stopped on the 15 degree tooth',dict(final=17.),.5),
        ('stopped on the 20 degree tooth',dict(final=21.),.25),
        ('button pressed without lifting the lever (blocked by the loaded pawl)',dict(lift=0.),0.),
        ('lifted 0.3 degree only, pawl not freed',dict(lift=.3),0.),
        ('lifted but button pushed 4 mm only',dict(press=.004),0.),
        ('lifted a full tooth higher, never pressed',dict(lift=5.2,press=0.),0.),
        ('button in, lever let fall to the stop',dict(how='dropped'),0.),
        ('lowered slowly with no hand on grip',dict(contact=False),0.),
        ('lowered fully but a hand still on the lever',dict(how='held'),0.),
        ('hands 29 mm clear',dict(clear=.029),0.),
        ('hands 30 mm clear',dict(clear=.030),1.),
        ('still for 0.9 s only',dict(hold=.9),0.),
        ('lever or mount knocked out of place',dict(bad='integrity'),0.)]
 for label,kw,want in specs:
  state=run(**kw)
  yield label,{task.BRAKE.name:state},want,want==1.
 task._judge.reset();task._unit_certificate['value']=None
CASES=cases()
if __name__=='__main__':
 for label,states,want,done in CASES:
  got=task.score(states);assert got==want and task.success(states)==done,(label,got,want)
  print('PASS',label,got)
