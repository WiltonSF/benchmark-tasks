"""Synthetic adversarial states, not robot trials. Every tier, boundaries and latched zeros."""
import numpy as np
from dataclasses import replace
import parts
class Cases:
 def __iter__(self):
  m=parts._softcore_namespace
  start={a.name:a.state() for a in m['OBJECTS']}
  body=m['BODY'].state(pos=(.32,.205,m['TOP']+.009));core=m['CORE'].state(pos=(.22,-.27,m['TOP']))
  full={'actuator':body,'core':core}
  def rec(**kw):
   m['S'].clear();m['S'].update(m['fresh']());m['S'].update(known=True,steps=10,physics=120,observed_physics=120,t=3.,clear=0.,still=0.);m['S'].update(kw)
  def case(label,change,want,**kw):rec(**kw);return(label,change,want,want==1.)
  yield case('do nothing',{},0.)
  yield case('intact extracted centred released2s',full,1.)
  yield case('exact1s interval',full,.5,t=1.)
  yield case('0.999s interval',full,0.,t=.999)
  yield case('1.999s interval',full,.5,t=1.999)
  yield case('exact2s interval',full,1.,t=2.)
  yield case('held at target',full,0.,clear=None)
  yield case('unknown arm state',full,0.,known=False)
  yield case('missing force history',full,0.,physics=0)
  yield case('offcentre inside tray',full|{'actuator':m['BODY'].state(pos=(.337,.205,m['TOP']+.009))},.75)
  yield case('released bench outside tray',full|{'actuator':m['BODY'].state(pos=(.32,0,m['TOP']+.003))},.25)
  yield case('partial extraction',full|{'core':m['CORE'].state(pos=(.32,.180,m['TOP']+.009))},0.)
  yield case('floating inside tray',full|{'actuator':m['BODY'].state(pos=(.32,.205,m['TOP']+.04))},0.)
  yield case('body moving',full|{'actuator':m['BODY'].state(pos=(.32,.205,m['TOP']+.009),lin_vel=(.02,0,0))},0.)
  for reason in ['rim tear','offaxis tear','overforce','offbench','unknown pressure integrity']:
   yield case(reason+' stayszero',full,0.,zero=True,why=reason)
  paths=['/World/RightArm/gripper/tip_left','/World/RightArm/gripper/tip_right']
  rec();m['rim_damage']({paths[0]:2.01,paths[1]:2.01})
  yield('opposed thin-rim pinch above2N each',full,0.,False)
  rec();m['rim_damage']({paths[0]:8.01})
  yield('single rim press above8N',full,0.,False)
  rec();m['rim_damage']({paths[0]:3.,paths[1]:0.})
  yield('one-sided3N brush is not a rim grip',full,1.,True)
  inverted=m['TRAY'].state(pos=(.32,.205,m['TOP']+.009),quat_xyzw=(1.,0.,0.,0.))
  exterior=m['BODY'].state(pos=(.32,.205,m['TOP']+.015))
  yield case('inverted tray exterior floor is not its cavity',full|{'actuator':exterior,'parts_tray':inverted},0.)
  floating_tray=m['TRAY'].state(pos=(.32,.205,m['TOP']+.10))
  floating_body=m['BODY'].state(pos=(.32,.205,m['TOP']+.106))
  yield case('tray must have bench support',full|{'actuator':floating_body,'parts_tray':floating_tray},0.)
  yield case('body over tray edge at floor height has no support',full|{'actuator':m['BODY'].state(pos=(.395,.205,m['TOP']+.009))},0.)
  yield case('centre exactly10mm passes',full|{'actuator':m['BODY'].state(pos=(.330,.205,m['TOP']+.009))},1.)
  yield case('centre10.002mm is lower tier',full|{'actuator':m['BODY'].state(pos=(.330002,.205,m['TOP']+.009))},.75)
  rec();far=dict(start)|full
  far['actuator']=m['BODY'].state(pos=(.32,.205,m['TOP']+.009),quat_xyzw=(0.,0.,2**-.5,2**-.5))
  force,engaged,_=m['resistance'](far)
  assert engaged==0 and np.linalg.norm(force)==0 and not m['S']['zero']
  yield('rotated released body far from core creates no force or damage',full,1.,True)
  rec();m['resistance'](start)
  escaped=dict(start);escaped['core']=m['CORE'].state(pos=(m['X']+.050,-.025,m['Z']))
  m['resistance'](escaped)
  yield('lateral escape from engaged core stays a tear',full,0.,False)
  rec();m['resistance'](start)
  freed=dict(start);freed['core']=m['CORE'].state(pos=(m['X'],-.250,m['Z']))
  f0,ov,_=m['resistance'](freed);assert ov==0 and np.linalg.norm(f0)==0
  m['resistance'](far);assert not m['S']['zero']
  inserted=dict(start);inserted['core']=m['CORE'].state(lin_vel=(0.,.03,0.))
  f1,ov,_=m['resistance'](inserted)
  assert ov>0 and f1[1]<0 and m['S']['interface_engaged']
  yield('real aligned reinsertion restores resistance, not an irreversible unlock',full,1.,True)
  rec();m['resistance'](start);m['resistance'](freed)
  reversed_core=dict(start);reversed_core['core']=m['CORE'].state(quat_xyzw=(0.,0.,1.,0.),lin_vel=(0.,.03,0.))
  force,ov,_=m['resistance'](reversed_core)
  assert ov>0 and force[1]<0 and not m['S']['zero']
  yield('end-for-end half-round reinsertion also restores resistance',full,1.,True)
  # Exercise the real contact consumer, then recover valid observations. A data
  # fault must remain zero rather than merely throw or clear on the next step.
  class Contact:
   def __init__(self,field=None,value=0.,part=0):self.field=field;self.value=value;self.part=part
   def get_contact_data(self,dt):
    arrays=[np.array([3.]),np.array([[.32,.245,m['TOP']+.017]]),np.array([[1.,0.,0.]]),np.array([0.]),np.array([[1.],[0.]]),np.zeros((2,1))]
    if self.part:arrays[4]=arrays[4][::-1].copy()
    if self.field is not None:arrays[self.field].flat[0]=self.value
    return arrays
  for field,name in enumerate(('force','position','normal','separation','count','start')):
   for value,label in ((np.nan,'NaN'),(np.inf,'+infinity'),(-np.inf,'-infinity')):
    for part in (0,1):
     rec();m['S'].update(view=Contact(field,value,part),org=np.zeros(3),contact_paths=['/World/RightArm/link5'])
     m['damage_contacts'](dict(start)|full,1/240)
     assert m['S']['zero'] and not m['S']['known'],(name,label,part)
     m['S']['view']=Contact();m['damage_contacts'](dict(start)|full,1/240)
     for t in (0.,1.,2.1):m['observe'](dict(start)|full,np.array([[2.,2.,2.]]),t)
     yield(f'{label} contact {name} part{part} stayszero after valid recovery',full,0.,False)
  rec();m['S'].update(view=Contact(),org=np.zeros(3),contact_paths=['/World/RightArm/link5'])
  m['damage_contacts'](dict(start)|full,1/240)
  assert not m['S']['zero']
  yield('finite3N interior contact preserves full score',full,1.,True)
  for field,value,label in ((4,-1.,'negative count'),(4,.5,'fractional count'),(5,.5,'fractional start'),(5,-1.,'negative active start'),(5,2.,'out-of-range start')):
   rec();m['S'].update(view=Contact(field,value),org=np.zeros(3),contact_paths=['/World/RightArm/link5'])
   m['damage_contacts'](dict(start)|full,1/240)
   yield(label+' invalidates contact trial',full,0.,False)
  for value,label in ((np.nan,'NaN'),(np.inf,'infinite')):
   rec();m['observe'](dict(start)|full,np.array([[2.,2.,2.]]),value)
   for t in (0.,1.,2.1):m['observe'](dict(start)|full,np.array([[2.,2.,2.]]),t)
   yield(label+' observation time stayszero after recovery',full,0.,False)
  class TruncatedContact(Contact):
   def __init__(self,width):self.width=width
   def get_contact_data(self,dt):
    return np.zeros((1,1)),np.zeros((1,3)),np.zeros((1,3)),np.zeros((1,1)),np.zeros((2,self.width)),np.zeros((2,self.width))
  for width in (0,1,3):
   rec();m['S'].update(view=TruncatedContact(width),org=np.zeros(3),contact_paths=['/World/LeftArm/link5','/World/RightArm/link5'])
   m['damage_contacts'](dict(start)|full,1/240)
   assert m['S']['zero'] and not m['S']['known']
   m['S'].update(view=Contact(),contact_paths=['/World/RightArm/link5'])
   m['damage_contacts'](dict(start)|full,1/240)
   for t in (0.,1.,2.1):m['observe'](dict(start)|full,np.array([[2.,2.,2.]]),t)
   yield(f'contact table width{width} mismatches2 paths and stayszero after recovery',full,0.,False)
  valid=dict(start)|full
  for name in ('actuator','core','parts_tray'):
   for field in ('low','high'):
    for value,label in ((np.nan,'NaN'),(np.inf,'+infinity'),(-np.inf,'-infinity')):
     vector=getattr(valid[name],field).copy();vector[0]=value
     bad=valid|{name:replace(valid[name],**{field:vector})}
     for route in ('observe','score'):
      rec()
      if route=='observe':m['observe'](bad,np.array([[2.,2.,2.]]),0.)
      else:assert m['score'](bad)==0.
      assert m['S']['zero'] and not m['S']['known'],(name,field,label,route)
      for t in (0.,1.,2.1):m['observe'](valid,np.array([[2.,2.,2.]]),t)
      yield(f'{label} {name} {field} via{route} stayszero after valid recovery',full,0.,False)
   rec();bad=valid|{name:replace(valid[name],low=valid[name].high+1.)}
   assert m['score'](bad)==0. and m['S']['zero']
   yield(name+' inverted bounds permanently invalidate trial',full,0.,False)
   rec();bad={n:st for n,st in valid.items() if n!=name}
   assert m['score'](bad)==0. and m['S']['zero']
   yield(name+' missing from score input permanently invalidates trial',full,0.,False)
  rec();m['S'].update(m['fresh']())
  # Transport-fault cases exercise the real consumer, including recovery.
  class MalformedContact(Contact):
   def __init__(self,kind):self.kind=kind
   def get_contact_data(self,dt):
    a=Contact().get_contact_data(dt)
    if self.kind in ('zero normal','short normal'):
     a[2][:]=0. if self.kind=='zero normal' else .1
    elif self.kind=='shared active range':a[4][:]=1.
    elif self.kind=='distant contact point':a[1][:]=10.
    return a
  for kind in ('zero normal','short normal','shared active range','distant contact point'):
   rec();m['S'].update(view=MalformedContact(kind),org=np.zeros(3),contact_paths=['/World/RightArm/link5'])
   m['damage_contacts'](valid,1/240)
   assert m['S']['zero'] and not m['S']['known']
   m['S']['view']=Contact();m['damage_contacts'](valid,1/240)
   for t in (0.,1.,2.1):
    m['S']['physics']+=12;m['observe'](valid,np.array([[2.,2.,2.]]),t)
   yield(kind+' contact stayszero after valid recovery',full,0.,False)
  rec();m['observe'](valid,np.array([[2.,2.,2.]]),4.)
  assert m['S']['zero']
  m['S']['physics']+=12;m['observe'](valid,np.array([[2.,2.,2.]]),6.)
  yield('stopped physics callback remainszero after restart',full,0.,False)
  from types import SimpleNamespace
  rec(runtime=SimpleNamespace(_t=3.05))
  assert m['score'](valid)==0. and m['S']['zero']
  yield('stale observer clock remainszero',full,0.,False)
  rec(physics=132)
  yield('new physics without observer invalidates score',full,0.,False)
  outside=valid|{'actuator':m['BODY'].state(pos=(1.2,.205,m['TOP']+.009)), 'parts_tray':m['TRAY'].state(pos=(1.2,.205,m['TOP']+.003))}
  yield case('tray and body at floor height outside real tabletop',outside,0.)
  yield case('floating released core has no bench support',full|{'core':m['CORE'].state(pos=(.22,-.27,m['TOP']+.05))},0.)
  for axis in ((1.,-1.,2**.5),(-2.,1.,-1.),(1.,1.,-1.)):
   a=np.asarray(axis);a/=np.linalg.norm(a)
   for angle in (np.pi,np.pi-1e-7,np.pi+1e-7):
    q=np.r_[a*np.sin(angle/2),np.cos(angle/2)];expected=m['parts']._quat_to_matrix(q)
    state=m['BODY'].state(quat_xyzw=q)
    assert np.allclose(m['parts']._quat_to_matrix(state.quat_xyzw),expected,atol=1e-12)
  yield case('mixed-axis half-turn state conversions preserve orientation',full,1.)
  rec();m['S'].update(m['fresh']())
CASES=Cases()
