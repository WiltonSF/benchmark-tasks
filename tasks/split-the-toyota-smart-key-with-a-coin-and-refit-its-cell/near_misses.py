"""Scoring counterexamples; fabricated states test the grader, never count as a simulated trial."""
import math
import numpy as np
import task
SEATW=np.array((task.CX,task.CY,task.SEAM_Z))+task.RCASE@task.SEAT
CASES=[('do nothing',{},0.,False),('swap cell without opening',{task.NEW.name:(tuple(SEATW),None,None)},0.,False)]
def run():
 saved=task.S;checks=[]
 def grade(*,split=True,off=True,new=True,up=True,oldout=True,latched=(True,True),relatched=(True,True),gap=0.,tray=True,
  known=True,clear=True,hold=1.,zero=False,offset=(0,0,0),cover_tilt=0.):
  task.S=task.fresh();o={s.name:s.state() for s in task.OBJECTS}
  if gap or cover_tilt:
   r=task.RCASE@task.parts._rot((0,1,0),cover_tilt)
   o[task.COVER.name]=task.COVER.state(pos=tuple(np.array(task.COVER.pos)+task.RCASE@np.array((0,0,gap))),quat_xyzw=tuple(task.parts._matrix_to_quat(r)))
  if new:
   r=task.RCASE if up else task.RCASE@task.parts._rot((1,0,0),math.pi)
   o[task.NEW.name]=task.NEW.state(pos=tuple(SEATW+task.RCASE@np.asarray(offset)),quat_xyzw=tuple(task.parts._matrix_to_quat(r)))
  if tray:o[task.OLD.name]=task.OLD.state(pos=(task.TRAY_C[0],task.TRAY_C[1],task.TOP+.003+.0016))
  else:o[task.OLD.name]=task.OLD.state(pos=(.43,.15,task.TOP+.0026))
  task.S.update(split=split,cover_off=off,old_out=oldout,latched=list(latched),relatched=list(relatched),known=known,clear=clear,hold=hold,zero=zero)
  task.S['cert']=task.signature(o)
  return task.score(o),o
 def check(name,expected,**kw):
  value,_=grade(**kw);checks.append((name,abs(value-expected)<1e-9,value,expected))
 try:
  check('full released endpoint',1.)
  check('no tool split: exchange alone earns zero',0.,split=False)
  check('split but cover never taken off',0.,off=False)
  check('split only, cell not exchanged',.25,new=False,latched=(False,False),relatched=(False,False),gap=.03)
  check('opened and correctly seated, cover left off',.5,latched=(False,False),relatched=(False,False),gap=.03)
  check('cover set on but not snapped',.5,latched=(False,False),relatched=(False,False))
  check('only one snap re-engaged',.5,latched=(True,False),relatched=(True,False))
  check('snapped flag without re-engagement record',.5,relatched=(False,False))
  check('new cell minus up under closed cover',.25,up=False)
  check('new cell still in presentation tray',.25,new=False)
  check('old cell not removed',.25,oldout=False)
  check('closed but spent cell outside tray',.75,tray=False)
  check('seam gap 0.5 mm',1.,gap=.0005)
  check('seam gap above 0.5 mm',.5,gap=.00051)
  check('cover cocked on one end',.5,cover_tilt=math.radians(1.))
  check('new cell outside seat tolerance',.25,offset=(.00201,0,0))
  check('new cell at seat tolerance',1.,offset=(.002,0,0))
  check('missing observation fails closed',0.,known=False)
  check('hands have not released',0.,clear=False)
  check('short stillness',0.,hold=.99)
  check('whole-trial violation cannot recover',0.,zero=True)
  _,o=grade();o[task.OLD.name]=task.OLD.state();task.S['cert']=task.signature(o)
  checks.append(('old cell returned to its pocket cannot earn exchange tier',task.score(o)==.25,task.score(o),.25))
  _,o=grade();task.S['cert']=();checks.append(('stale observation fails closed',task.score(o)==0.,task.score(o),0.))
  _,o=grade();checks.append(('empty hand sample fails closed',not task.clear_hands([],o),None,None))
  cloud=np.array([[task.CX,task.CY,task.SEAM_Z+task.H+.004]])
  checks.append(('hand 4 mm above the cover is not released',not task.clear_hands([cloud],o),None,None))
 finally:task.S=saved
 for name,ok,value,want in checks:print(('ok' if ok else 'FAIL'),name,'' if value is None else f'{value} expected {want}')
 return sum(not ok for _,ok,_,_ in checks)
def gate_controls():
 """Fabricated contact inputs exercise state gating only; never native physics evidence."""
 from types import SimpleNamespace as NS
 saved=(task.S,task.read_contacts,task.set_snap,task.parts._np,task.event)
 failures=0
 task.event=lambda _text:None
 class Scene(dict):
  env_origins=np.zeros((1,3))
 def trial(*,supported=False,left=True,right=True,jaws=True,travel=None):
  task.S=task.fresh();calls=[]
  z=task.SEAM_Z if supported else task.PRY_Z
  travel=task.SNAP_TRAVEL+.0001 if travel is None else travel
  scene=Scene()
  for name,zz in [(task.BASE.name,z),(task.COVER.name,z+travel),(task.DRIVER.name,z)]:
   scene[name]=NS(data=NS(root_link_pose_w=np.array([[task.CX,task.CY,zz,*task.QCASE]])))
  sim=NS(env=NS(unwrapped=NS(scene=scene)),t=1.)
  def contacts(*_):
   task.S.update(left_base=10. if left else 0.,right_tool=10. if right else 0.,
    right_cover=0.,jaw_load=[20.,20. if jaws else 0.],raw_tool_torque=0.,current_torque=0.,
    blade_cover=20.,blade_base=20.,press=0.,press_x=0.)
  task.read_contacts=contacts;task.set_snap=lambda k,on:calls.append((k,on));task.parts._np=np.asarray
  task.snap_physics(sim)
  assert not task.S['zero'],task.S['why']
  return task.S['split'],calls
 try:
  for label,want,args in [
   ('supported key cannot release even with synthetic loaded blade',False,dict(supported=True)),
   ('free held key with synthetic valid load releases notch snap',True,{}),
   ('absent left holding contact blocks release',False,dict(left=False)),
   ('absent right tool contact blocks release',False,dict(right=False)),
   ('one loaded jaw cannot release',False,dict(jaws=False)),
   ('travel below lip does not release',False,dict(travel=task.SNAP_TRAVEL-.00001))]:
   got,calls=trial(**args);ok=(got==want and ((0,False) in calls)==want)
   print('ok' if ok else 'FAIL',label,'[synthetic gate control]');failures+=not ok
 finally:
  task.S,task.read_contacts,task.set_snap,task.parts._np,task.event=saved
 return failures
if __name__=='__main__':raise SystemExit(run()+gate_controls())
