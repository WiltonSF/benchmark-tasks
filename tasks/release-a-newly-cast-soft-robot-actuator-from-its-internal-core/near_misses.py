"""Synthetic adversarial states, not robot trials: every tier, its boundaries, and latched zeros."""
import numpy as np
from dataclasses import replace
import parts
class Cases:
 def __iter__(self):
  m=parts._softcore_namespace; TOP=m['TOP']; B=m['BODY']; C=m['CORE']; T=m['TRAY']
  zc=TOP+.006+m['RO']
  start={a.name:a.state() for a in m['OBJECTS']}
  body=B.state(pos=(.32,.205,zc)); core=C.state()
  full={'actuator':body,'core':core}
  def rec(**kw):
   m['S'].clear();m['S'].update(m['fresh']());m['S'].update(known=True,jaw_valid=True,steps=10,physics=120,observed_physics=120,t=3.,clear=0.,still=0.,bond='turn',turned=True,max_twist=88.,core0=(np.array(C.pos,float),np.array([0.,0.,0.,1.])));m['S'].update(kw)
  def case(label,change,want,**kw):rec(**kw);return(label,change,want,want==1.)
  yield case('do nothing: wall still on clamped core',{},0.)
  yield case('wall in tray but bond never released by a turn',full,0.,bond='bonded',turned=False)
  yield case('bond released but turn short of a quarter turn',full,0.,turned=False,max_twist=60.)
  yield case('bond still pending (never armed)',full,0.,bond='pending')
  yield case('wall withdrawn, centred in tray, released 2 s',full,1.)
  yield case('exactly 1 s released and still',full,.5,t=1.)
  yield case('0.999 s released',full,0.,t=.999)
  yield case('1.999 s released',full,.5,t=1.999)
  yield case('exactly 2 s released',full,1.,t=2.)
  yield case('held over target (hands not clear)',full,0.,clear=None)
  yield case('unknown arm state',full,0.,known=False)
  yield case('no physics history',full,0.,physics=0)
  yield case('off-centre 17 mm inside tray',full|{'actuator':B.state(pos=(.337,.205,zc))},.75)
  yield case('centre exactly 10 mm passes',full|{'actuator':B.state(pos=(.330,.205,zc))},1.)
  yield case('centre 10.002 mm is lower tier',full|{'actuator':B.state(pos=(.330002,.205,zc))},.75)
  yield case('released on bench outside tray',full|{'actuator':B.state(pos=(.50,.10,TOP+m['RO']))},.25)
  yield case('partly withdrawn, still on core',full|{'actuator':B.state(pos=(.32,.150,m['Z']-m['CLEAR']))},0.)
  yield case('withdrawn only 3 mm past core tip (needs 5 mm)',full|{'actuator':B.state(pos=(.32,.085+.085+.003,m['Z']-m['CLEAR']))},0.)
  yield case('floating above tray',full|{'actuator':B.state(pos=(.32,.205,zc+.04))},0.)
  yield case('wall still moving in tray',full|{'actuator':B.state(pos=(.32,.205,zc),lin_vel=(.02,0,0))},0.)
  # The core must stay clamped: pulling it out of the vise instead of the wall off the core scores 0.
  yield case('core pulled 1.5 mm out of the vise',full|{'core':C.state(pos=(m['X'],m['CORE_Y']+.0015,m['Z']))},0.)
  yield case('jaw contacts missing',full,0.,jaw_valid=False)
  yield case('no recorded clamped start pose',full,0.,core0=None)
  yield case('core slipped 0.9 mm (within 1 mm) still counts as clamped',full|{'core':C.state(pos=(m['X'],m['CORE_Y']+.0009,m['Z']))},1.)
  q=np.r_[np.array([1.,0,0])*np.sin(np.radians(3)/2),np.cos(np.radians(3)/2)]
  yield case('core tilted 3 degrees in the vise',full|{'core':C.state(quat_xyzw=q)},0.)
  yield case('core spun3degrees around its own axis in vise',full|{'core':C.state(quat_xyzw=(0.,np.sin(np.radians(1.5)),0.,np.cos(np.radians(1.5))))},0.)
  yield case('core removed from vise and laid on bench, wall in tray',full|{'core':C.state(pos=(.22,-.27,TOP+m['R']))},0.)
  for reason in ['rim pinch','off-axis tear','wall squeeze','dropped off bench','invalid contact data']:
   yield case(reason+' stays zero',full,0.,zero=True,why=reason)
  paths=['/World/RightArm/gripper/tip_left','/World/RightArm/gripper/tip_right']
  rec();m['rim_damage']({paths[0]:m['RIM_N']+.01,paths[1]:0.})
  yield('cut-end press above 16.7 N',full,0.,False)
  rec();m['rim_damage']({paths[0]:m['RIM_N']-.01,paths[1]:m['RIM_N']-.01})
  yield('cut-end contacts below 16.7 N each are not a tear',full,1.,True)
  # Bond: pulled or levered while bonded is a tear; it lets go only after BOND_TURN degrees of twist.
  bonded=dict(rel=np.eye(3),off=np.array(B.pos)-np.array(C.pos))
  def twisted(deg):
   h=np.radians(deg)/2;return B.state(quat_xyzw=(0.,np.sin(h),0.,np.cos(h)))
  rec(bond='bonded',bond_rel=bonded,turned=False);m['bond_physics']({'actuator':B.state(pos=(B.pos[0],B.pos[1]+.0006,B.pos[2])),'core':core})
  yield('bonded wall pulled 0.6 mm along the core is a tear',full,0.,False)
  rec(bond='bonded',bond_rel=bonded,turned=False);m['bond_physics']({'actuator':twisted(m['BOND_TURN']-1.),'core':core})
  assert not m['S']['release_due']
  yield('bond still holds 1 degree short of its release twist',full,0.,False)
  rec(bond='bonded',bond_rel=bonded,turned=False);m['bond_physics']({'actuator':twisted(m['BOND_TURN']+.5),'core':core})
  assert m['S']['release_due'] and m['S']['turned'] and not m['S']['zero']
  m['S']['bond']='turn'
  yield('twist past the release angle frees the bond (then withdrawal scores)',full,1.,True)
  rec(bond='bonded',bond_rel=bonded,turned=False);q=np.r_[np.array([1.,0,0])*np.sin(np.radians(2)/2),np.cos(np.radians(2)/2)]
  m['bond_physics']({'actuator':B.state(quat_xyzw=q),'core':core})
  yield('bonded wall levered 2 degrees is a tear',full,0.,False)
  rec();sqz={'/World/LeftArm/a/tip_left':30.,'/World/LeftArm/a/tip_right':30.,'/World/LeftArm/a/gripper':10.}
  class Many:
   def get_contact_data(self,dt):
    n=len(sqz);return [np.array(list(sqz.values())),np.tile([[m['X'],0.,m['Z']+m['RO']]],(n,1)),np.tile([[0.,0.,1.]],(n,1)),np.zeros(n),np.array([[1.]*n,[0.]*n]),np.array([list(range(n)),[0.]*n])]
  for _ in range(30):
   m['S'].update(view=Many(),org=np.zeros(3),contact_paths=list(sqz));m['damage_contacts'](start,1/240)
  yield('one hand pressing 70 N in total over three links for 0.125 s is a tear',full,0.,False)
  inverted=T.state(pos=(.32,.205,TOP+.009),quat_xyzw=(1.,0.,0.,0.))
  yield case('inverted tray exterior is not its cavity',full|{'actuator':B.state(pos=(.32,.205,TOP+.015+m['RO'])),'parts_tray':inverted},0.)
  yield case('tray lifted off bench',full|{'actuator':B.state(pos=(.32,.205,zc+.10)),'parts_tray':T.state(pos=(.32,.205,TOP+.103))},0.)
  yield case('wall over tray rim has no floor support',full|{'actuator':B.state(pos=(.395,.205,zc))},0.)
  # No task-generated load exists: the interference and friction are native collisions.
  contact=[(np.array([m['X'],0.,m['Z']+m['R']]),m['CUT_N'],0.)]
  rec()
  for contacts in ((),contact,[(contact[0][0],.14,.003)]):
   f,tau,_,_=m['interface'](start,contacts)
   assert np.linalg.norm(f)==0 and np.linalg.norm(tau)==0
  yield('no task-generated wrench on proximity or contact; native interface tested on GPU',full,1.,True)
  rec();m['interface'](start,contact)
  esc=dict(start);esc['actuator']=B.state(pos=(m['X']+.006,0.,m['Z']))
  rec();m['interface'](start,contact);m['interface'](esc,())
  yield('previous loaded contact cannot tear an off-axis wall without current contact',full,1.,True)
  rec();m['interface'](esc,[(contact[0][0],m['CUT_N']*.99,0.)])
  yield('off-axis contact below the 33.3 N derived load cannot latch damage',full,1.,True)
  rec();m['interface'](esc,[(contact[0][0],m['CUT_N'],m['CONTACT_SEP']+.00001)])
  yield('separated contact cannot latch off-axis damage',full,1.,True)
  rec();m['interface'](esc,[(np.array([2.,2.,2.]),m['CUT_N'],0.)])
  yield('loaded contact outside both bodies cannot latch damage',full,1.,True)
  rec();m['interface'](esc,[(contact[0][0],m['CUT_N'],0.)]);m['interface'](start,())
  yield('threshold-qualified current contact latches off-axis damage after recovery',full,0.,False)
  rec();m['interface'](start,contact)
  m['interface'](esc,contact)
  yield('wall forced6mm sideways while in loaded core contact is a tear',full,0.,False)
  rec();m['interface'](start,contact)
  tw=dict(start);tw['actuator']=B.state(quat_xyzw=(0.,0.,np.sin(np.radians(10)),np.cos(np.radians(10))))
  m['interface'](tw,contact)
  yield('wall levered20degrees off axis while in core contact is a tear',full,0.,False)
  rec();sq=0.
  class Contact:
   def __init__(self,f):self.f=f
   def get_contact_data(self,dt):
    return [np.array([self.f]),np.array([[m['X'],0.,m['Z']+m['RO']]]),np.array([[0.,0.,1.]]),np.array([0.]),np.array([[1.],[0.]]),np.zeros((2,1))]
  for _ in range(30):
   m['S'].update(view=Contact(m['CUT_N']+.1),org=np.zeros(3),contact_paths=['/World/LeftArm/tip_left']);m['damage_contacts'](start,1/240)
  yield('one finger pad above 33.3 N for 0.125 s is a tear',full,0.,False)
  rec()
  for _ in range(30):
   m['S'].update(view=Contact(30.),org=np.zeros(3),contact_paths=['/World/LeftArm/tip_left']);m['damage_contacts'](start,1/240)
  yield('a 30 N pad grip is not a tear',full,1.,True)
  rec();m['S'].update(view=Contact(np.nan),org=np.zeros(3),contact_paths=['/World/LeftArm/tip_left']);m['damage_contacts'](start,1/240)
  m['S'].update(view=Contact(3.));m['damage_contacts'](start,1/240)
  yield('NaN contact force stays zero after valid recovery',full,0.,False)
  valid=dict(start)|full
  for name in ('actuator','core','parts_tray'):
   rec();vec=valid[name].low.copy();vec[0]=np.nan
   assert m['score'](valid|{name:replace(valid[name],low=vec)})==0.
   yield(f'NaN {name} bounds permanently invalidate the trial',full,0.,False)
   rec();assert m['score']({n:st for n,st in valid.items() if n!=name})==0.
   yield(f'{name} missing from score input invalidates the trial',full,0.,False)
  rec();m['observe'](valid,np.array([[2.,2.,2.]]),4.)
  yield('stopped physics callback stays zero',full,0.,False)
  rec(physics=132)
  yield('new physics without a current observation scores 0',full,0.,False)
  rec();m['S'].update(m['fresh']())
CASES=Cases()
