"""Physical rail engagement and stop-gap counterexamples."""
import copy,numpy as np
import task
CASES=[('unobserved reset',{},0.,False)]
def run():
 keys=('_known','_invalid','_certificate','_slid','_initial','_held','_offrail_travel')
 old={k:copy.deepcopy(getattr(task,k)) for k in keys};results=[]
 def grade(x=task.HOME,y=0.,z=task.BEAM_Z,yaw=0.,held=1.,offrail=0.,known=True,slide=.0413):
  s=task.BEAM.state(pos=(x,y,z),quat_xyzw=task.parts._matrix_to_quat(task.parts._rot((0,0,1),yaw)))
  task._known=known;task._invalid=False;task._certificate=task.stamp(s);task._slid=np.full(2,slide);task._initial=task.measure(s)['fronts']-slide
  task._held=held;task._offrail_travel=offrail
  return task.score({task.BEAM.name:s}),s
 try:
  results.append(('both stops and released',grade()[0]==1.))
  results.append(('unobserved state',grade(known=False)[0]==0.))
  results.append(('no net sliding',grade(slide=0.)[0]==0.))
  results.append(('hovering off rail',grade(z=task.BEAM_Z+.005)[0]<=.25))
  results.append(('below rail height',grade(z=task.BEAM_Z-.005)[0]<=.25))
  results.append(('laterally off rail',grade(y=.002)[0]<=.25))
  results.append(('well beyond stop',grade(x=task.HOME+.003)[0]<=.25))
  results.append(('0.11mm past stop cannot be full',grade(x=task.HOME+.00011)[0]<1.))
  results.append(('0.049mm gap within feeler bound',grade(x=task.HOME-.000049)[0]==1.))
  results.append(('0.051mm gap outside feeler bound',grade(x=task.HOME-.000051)[0]<1.))
  results.append(('one end on stop at rack angle',grade(yaw=.003)[0]<.75))
  results.append(('both ends still5mm away',grade(x=task.HOME-.0051)[0]==.25))
  results.append(('movement off rail earns zero',grade(offrail=.004)[0]==0.))
  results.append(('short release',grade(held=.95)[0]==.75))
  _,s=grade();task._certificate=();results.append(('stale state',task.score({task.BEAM.name:s})==0.))
  _,s=grade();results.append(('NaN hands fail closed',not task.released([np.full((4,3),np.nan)],s)))
  results.append(('empty hands fail closed',not task.released([],s)))
 finally:
  for k,v in old.items():setattr(task,k,v)
 for name,ok in results:print(('ok' if ok else 'FAIL'),name)
 return sum(not ok for _,ok in results)
if __name__=='__main__':raise SystemExit(run())
