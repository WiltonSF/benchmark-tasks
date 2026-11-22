"""Constructed histories check the judge; none constitutes physics evidence."""
import math,types
import numpy as np
import parts
shared=next((fn.__globals__ for fn in parts._ON_STEP if fn.__name__=='update'),None)
if shared is None:
 from yam_arena.task_file import load
 import pathlib
 task=load(pathlib.Path(__file__).with_name('task.py')).module
else:task=types.SimpleNamespace(**shared)
from mechanism import Sample,Judge,D

def prepare(angle=0.,holder=1,clear=.05,hold=1.1,bad=None):
 j=task._judge;j.reset();time=0.
 def step(a,b=0.,press=0.,moment=None,clearance=0.,integrity=True,other=0.,cable=True):
  nonlocal time
  time+=1/240;forces=np.zeros((2,2,3));moments=np.zeros((2,2));pr=np.zeros(2)
  if moment is not None:forces[holder,0]=[0,0,-moment/.22];moments[holder,0]=moment
  if press:pr[1-holder]=press;forces[1-holder,1]=[0,press,0]
  j.step(Sample(time,a*D,b,forces,moments,pr,clearance,integrity,-Judge.load(a*D) if cable else 0.,other))
 step(24)
 for a in np.linspace(24,26,49)[1:]:step(a,moment=60)
 step(26,.006,6,60)
 for a in np.linspace(26,angle,241)[1:]:step(a,.006,6,max(5,Judge.load(a*D)))
 step(angle)
 for i in range(round(hold*240)):step(angle,clearance=clear)
 if bad=='integrity':step(angle,clearance=clear,integrity=False)
 if bad=='other_link':step(angle,clearance=clear,other=3.)
 if bad=='cable':step(angle,clearance=clear,cable=False)
 if bad=='observation':j.fail('missing observation')
 state=task.BRAKE.state(joints={'lever':angle*D-task.ANGLE,'button':0.})
 task._unit_certificate['value']=task.signature(state)
 return state

def cases():
 specs=[('full',0,1,.05,1.1,None,1.),('swapped hands',0,0,.05,1.1,None,1.),
        ('partial4',4,1,.05,1.1,None,.75),('partial8',8,1,.05,1.1,None,.75),
        ('partial12',12,1,.05,1.1,None,.5),('partial16',16,1,.05,1.1,None,.5),
        ('partial20',20,1,.05,1.1,None,.25),('no progress',24,1,.05,1.1,None,0.),
        ('clearance below',0,1,.0299,1.1,None,0.),('clearance exact',0,1,.035,1.1,None,1.),
        ('hold short',0,1,.05,.99,None,0.),('integrity violation',0,1,.05,1.1,'integrity',0.),
        ('nonhand support',0,1,.05,1.1,'other_link',0.),('force-free cable',0,1,.05,1.1,'cable',0.),
        ('missing observation',0,1,.05,1.1,'observation',0.)]
 for label,a,h,c,t,b,want in specs:
  state=prepare(a,h,c,t,b)
  yield label,{task.BRAKE.name:state},want,want==1.
 task._judge.reset();task._unit_certificate['value']=None
CASES=cases()
if __name__=='__main__':
 for label,states,want,done in CASES:
  got=task.score(states);assert got==want and task.success(states)==done,(label,got,want)
  print('PASS',label,got)
