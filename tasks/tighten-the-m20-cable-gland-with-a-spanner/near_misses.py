"""Endpoint injections without a matching observed force/history certificate fail closed.
These are scorer controls, not simulated physical trials.
"""
import math
import numpy as np
import task
AT_MARK=task.AXIS[0]+task.CAP_X-task.HEX_HALF_LEN-task.MARK_LOCAL_X   # cable origin x with the mark on the cap face
CASES=[
 ('do nothing',{},0.,False),
 ('cable fed to the mark but no observed hand/contact state',{'cable':task.CABLE.state(pos=(AT_MARK,.015,task.AXIS[2]))},0.,False),
 ('cable above hole',{'cable':task.CABLE.state(pos=(AT_MARK,.015,task.TOP+.080))},0.,False),
 ('cable beside hole',{'cable':task.CABLE.state(pos=(AT_MARK,.035,task.AXIS[2]))},0.,False),
 ('cap turned into the green sector without any observed tool history',
  {'gland':task.GLAND.state(joints={'cap':math.radians(210.)})},0.,False),
 ('feed outside wall without bore alignment',{'cable':task.CABLE.state(pos=(.30,.060,task.TOP+.055))},0.,False),
 ('held moving cable cannot score',{'cable':task.CABLE.state(pos=(AT_MARK,.015,task.AXIS[2]),lin_vel=(.1,0,0))},0.,False),
]
def _run():
 import importlib.util
 from pathlib import Path
 root=Path(__file__).resolve().parents[4]
 spec=importlib.util.spec_from_file_location('factory_check',root/'factory/check.py')
 checker=importlib.util.module_from_spec(spec);spec.loader.exec_module(checker)
 from yam_arena.task_file import load
 loaded=load(Path(__file__).with_name('task.py'))
 checker.check_scoring(loaded,Path(__file__).parent)
 return len(checker.failures)


def contract_controls():
 """Explicit synthetic observed histories; these do not prove physical feasibility."""
 names=list(task._STATE)
 saved={n:getattr(task,n) for n in names}
 snug=task.SNUG_LOW_DEG+3.
 try:
  objects={o.name:o.state() for o in task.OBJECTS}
  def history(q,x=AT_MARK,tool_peak=None,tool_work=None,pull=True,park=True,clear=True,held=1.1,damage=False,
              known=True,q_peak=None,load_hold=1.1,peak_torque=0.,radial_offset=0.):
   objects['cable']=task.CABLE.state(pos=(x,.015+radial_offset,task.AXIS[2]))
   objects['gland']=task.GLAND.state(joints={'cap':math.radians(q)})
   task._known=known;task._invalid=False;task._damage=damage;task._held=held;task._clear=clear
   task._peak_torque=peak_torque
   task._q=q;task._q_peak=q if q_peak is None else q_peak
   task._tool_peak=task._q_peak if tool_peak is None else tool_peak
   task._tool_work=task.seal_work(task.FIRST_CONTACT_DEG,max(task.FIRST_CONTACT_DEG,task._tool_peak)) if tool_work is None else tool_work
   task._pull_ok=pull;task._parked=park;task._max_loaded_hold=load_hold
   task._cert=task.snapshot(objects)
   return task.score(objects)
  for label,args,want in [
    ('fed to the mark, cap short of first contact',dict(q=0.),.25),
    ('first contact, short of snug',dict(q=task.FIRST_CONTACT_DEG+30.),.5),
    ('just below the 4.5 Nm line',dict(q=task.SNUG_LOW_DEG-.5),.5),
    ('brief torque peak without sustained load',dict(q=snug,load_hold=.99),.5),
    ('snug but pull unproven',dict(q=snug,pull=False),.75),
    ('complete observed endpoint',dict(q=snug),1.),
    ('snug after unloading spring-back',dict(q=snug-task.WINDUP_DEG,q_peak=snug),1.),
    ('loosened after reaching snug',dict(q=snug-10.,q_peak=snug),.5),
    ('mark 4 mm too deep',dict(q=snug,x=AT_MARK+.004),0.),
    ('mark 4 mm short',dict(q=snug,x=AT_MARK-.004),0.),
    ('mark 2.5 mm deep is within tolerance',dict(q=snug,x=AT_MARK+.0025),1.),
    ('whole cable fits bore at 1.49 mm centre offset',dict(q=snug,radial_offset=.00149),1.),
    ('cable centre in bore but sheath intersects wall at 1.51 mm',dict(q=snug,radial_offset=.00151),0.),
    ('held endpoint',dict(q=snug,clear=False),0.),
    ('less than one second still',dict(q=snug,held=.99),0.),
    ('snug angle reached without the seated spanner',dict(q=snug,tool_peak=task.FIRST_CONTACT_DEG+30.,tool_work=.5),.5),
    ('untouched spanner returned to the rest',dict(q=snug,tool_work=0.),.5),
    ('unparked spanner',dict(q=snug,park=False),.75),
    ('over-torque line crossed earlier, then loosened',dict(q=snug,damage=True),0.),
    ('exact red line rejected even without prior latch',dict(q=task.RED_DEG),0.),
    ('measured overtorque despite below-red final angle',dict(q=snug,peak_torque=5.01),0.),
    ('unknown hand/force history',dict(q=snug,known=False),0.)]:
   got=history(**args)
   assert got==want,(label,got,want)
  s=objects['cable'];c=np.asarray(s.pos)+np.array((.04,.0055,0))
  assert not task.released([np.array([c])],objects),'face-contact release bypass'
  task._pull_start=np.asarray(s.pos).copy();task._pull_cert_pos=np.asarray(s.pos).copy();task._pull_quat=np.asarray(s.quat_xyzw).copy();task._pull_q=task._q
  assert task.pull_certificate_valid(s),'fresh pull rejected'
  shifted=task.CABLE.state(pos=np.asarray(s.pos)+(.0021,0,0))
  assert not task.pull_certificate_valid(shifted),'translation kept stale pull'
  rotated=task.CABLE.state(pos=s.pos,quat_xyzw=(math.sin(math.radians(1.1)/2),0,0,math.cos(math.radians(1.1)/2)))
  assert not task.pull_certificate_valid(rotated),'rotation kept stale pull'
  task._q+=1.1
  assert not task.pull_certificate_valid(s),'cap movement kept stale pull'
  # Qualify the endpoint AFTER the test, while retaining original residual bounds.
  task._q=task._pull_q
  start=np.asarray(s.pos).copy();end=start+(.0019,0,0)
  qend=(math.sin(.02),0,0,math.cos(.02))
  task._pull_start=start;task._pull_cert_pos=end;task._pull_quat=np.asarray(qend)
  certified=task.CABLE.state(pos=end,quat_xyzw=qend)
  assert task.pull_certificate_valid(certified),'certified rotation during pull treated as post-test motion'
  excess=task.CABLE.state(pos=start+(.0021,0,0),quat_xyzw=qend)
  assert not task.pull_certificate_valid(excess),'original2mm residual limit lost'
  moved=task.CABLE.state(pos=end-(.0021,0,0),quat_xyzw=qend)
  assert not task.pull_certificate_valid(moved),'post-test translation within original radius kept stale certificate'
  print('30 synthetic history/release controls PASS (not physics evidence)')
 finally:
  for n,v in saved.items():setattr(task,n,v)

contract_controls()

if __name__=='__main__':raise SystemExit(_run())
