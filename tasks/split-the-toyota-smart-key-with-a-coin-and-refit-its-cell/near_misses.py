"""Scoring counterexamples; fabricated states test the grader, never count as a simulated trial."""
import copy, math
import numpy as np
import task
CASES=[('do nothing',{},0.,False),('swap cell before coin opening',{task.NEW.name:task.NEW.state(pos=tuple(np.asarray(task.FOB.pos)+task.SEAT))},0.,False)]
def run():
 saved=task.S;checks=[]
 def grade(*,split=True,new=True,up=True,oldout=True,reclosed=True,q=0.,tray=True,known=True,clear=True,hold=1.,zero=False,offset=(0,0,0)):
  task.S=task.fresh();o={s.name:s.state() for s in task.OBJECTS}
  o[task.FOB.name]=task.FOB.state(joints={'cover':q})
  if new:
   orient=(1.,0.,0.,0.) if not up else (0.,0.,0.,1.)
   o[task.NEW.name]=task.NEW.state(pos=tuple(np.asarray(task.FOB.pos)+task.SEAT+np.asarray(offset)),quat_xyzw=orient)
  if tray:o[task.OLD.name]=task.OLD.state(pos=(task.OLD_TRAY.pos[0],task.OLD_TRAY.pos[1],task.TOP+.0056))
  else:o[task.OLD.name]=task.OLD.state(pos=(.43,.15,task.TOP+.004))
  task.S.update(split=split,old_out=oldout,reclosed=reclosed,known=known,clear=clear,hold=hold,zero=zero)
  task.S['cert']=task.signature(o)
  return task.score(o),o
 def check(name,expected,**kw):
  value,_=grade(**kw);checks.append((name,abs(value-expected)<1e-9,value,expected))
 try:
  check('full released endpoint',1.)
  check('no loaded coin split: exchange alone earns zero',0.,split=False)
  check('coin split only',.25,new=False,q=.5,reclosed=False)
  check('opened and correctly seated',.5,q=.8,reclosed=False)
  check('new cell inverted',.25,up=False)
  check('new cell still in presentation tray',.25,new=False)
  check('old cell not removed',.25,oldout=False)
  check('closed but spent cell outside tray',.75,tray=False)
  check('cover gap below 0.5mm',1.,q=.0045)
  check('cover gap above 0.5mm',.5,q=.0046)
  check('pose near closed without contact latch is partial',.5,reclosed=False)
  check('new cell outside seat tolerance',.25,offset=(.00201,0,0))
  check('new cell at seat tolerance',1.,offset=(.002,0,0))
  check('missing observation fails closed',0.,known=False)
  check('hands have not released',0.,clear=False)
  check('short stillness',0.,hold=.99)
  check('whole-trial violation cannot recover',0.,zero=True)
  _,o=grade();o[task.OLD.name]=task.OLD.state();task.S['cert']=task.signature(o);checks.append(('old cell returned inside case cannot earn exchange tier',task.score(o)==.25,task.score(o),.25))
  _,o=grade();task.S['cert']=();checks.append(('stale observation fails closed',task.score(o)==0.,task.score(o),0.))
  _,o=grade();checks.append(('empty hand sample fails closed',not task.clear_hands([],o),None,None))
  # Face centre 0.5mm above the case, even when no hull vertex is near it.
  cloud=np.array([[task.X,task.Y,task.START_Z+.0086]])
  checks.append(('near hand face centre is not released',not task.clear_hands([cloud],o),None,None))
 finally:task.S=saved
 for name,ok,value,want in checks:print(('ok' if ok else 'FAIL'),name,'' if value is None else f'{value} expected {want}')
 return sum(not ok for _,ok,_,_ in checks)
if __name__=='__main__':raise SystemExit(run())
