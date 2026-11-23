"""Endpoint injections without a matching observed force/history certificate fail closed.
These are scorer controls, not simulated physical trials.
"""
import numpy as np
import task
CASES=[
 ('do nothing',{},0.,False),
 ('cable through but no observed hand/contact state',{'cable':task.CABLE.state(pos=(.22,.015,task.TOP+.055))},0.,False),
 ('cable above hole',{'cable':task.CABLE.state(pos=(.22,.015,task.TOP+.080))},0.,False),
 ('cable beside hole',{'cable':task.CABLE.state(pos=(.22,.035,task.TOP+.055))},0.,False),
 ('nut turned without any physical force certificate',{'gland':task.GLAND.state(joints={'cap':9.})},0.,False),
 ('feed outside wall without bore alignment',{'cable':task.CABLE.state(pos=(.30,.060,task.TOP+.055))},0.,False),
 ('held moving cable cannot score',{'cable':task.CABLE.state(pos=(.22,.015,task.TOP+.055),lin_vel=(.1,0,0))},0.,False),
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
 names=['_known','_invalid','_damage','_held','_q','_work','_clear','_pull_ok','_parked','_cert']
 saved={n:getattr(task,n) for n in names}
 try:
  objects={o.name:o.state() for o in task.OBJECTS}
  objects['cable']=task.CABLE.state(pos=(.220,.015,task.AXIS[2]))
  def history(q,work=100.,pull=True,park=True,clear=True,held=1.1,damage=False,known=True):
   objects['gland']=task.GLAND.state(joints={'cap':q})
   task._known=known;task._invalid=False;task._damage=damage;task._held=held
   task._q=q;task._work=work;task._clear=clear;task._pull_ok=pull;task._parked=park
   task._cert=task.snapshot(objects)
   return task.score(objects)
  for label,args,want in [
    ('fed only, released',dict(q=0.),.25),
    ('first contact, short of snug',dict(q=6.),.5),
    ('snug but pull unproven',dict(q=8.5,pull=False),.75),
    ('complete observed endpoint',dict(q=8.5),1.),
    ('held endpoint',dict(q=8.5,clear=False),0.),
    ('less than one second still',dict(q=8.5,held=.99),0.),
    ('unverified resisting work',dict(q=8.5,work=0.),0.),
    ('unparked spanner',dict(q=8.5,park=False),.75),
    ('previous damage despite loosening',dict(q=8.5,damage=True),0.),
    ('unknown hand/force history',dict(q=8.5,known=False),0.)]:
   got=history(**args)
   assert got==want,(label,got,want)
  # One sample on a broad face is 0.5 mm away. Corner-only geometry would miss
  # it; the production release predicate must reject it.
  s=objects['cable'];c=np.asarray(s.pos)+np.array((.04,.0055,0))
  assert not task.released([np.array([c])],objects),'face-contact release bypass'
  print('11 synthetic history/release controls PASS (not physics evidence)')
 finally:
  for n,v in saved.items():setattr(task,n,v)

contract_controls()

if __name__=='__main__':raise SystemExit(_run())
