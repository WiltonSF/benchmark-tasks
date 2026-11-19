"""Review-only CPU controls; native dynamics, support and robot sensors are stubbed.
These checks test fail-closed scoring logic, not physical feasibility.
"""
from yam_arena.task_file import load
from types import SimpleNamespace as NS
from dataclasses import replace
import numpy as np
import hashlib,json,pathlib
P=pathlib.Path(__file__).resolve().parents[1]/'task.py'
m=load(str(P)).module
finger_effort=np.zeros((1,2))
robot=NS(joint_names=['joint7','joint8'],root_view=NS(get_dof_projected_joint_forces=lambda:finger_effort))
asset=NS(body_names=['base','lever','wiper'],joint_names=['wiper'],data=NS(joint_vel=np.zeros((1,1)),body_com_pos_w=np.zeros((1,3,3))))
class Scene(dict):
 __getattr__=dict.__getitem__
base=NS(scene=Scene(disher=asset,env_origins=np.zeros((1,3)),left_robot=robot,right_robot=robot))
objects={o.name:o.state(pos=m.lots.current().pos(o.name)) for o in m.OBJECTS}
sim=NS(env=NS(unwrapped=base),t=0.,states=lambda:objects)
m.force=lambda *a,**kw:None
m.visual=lambda *a,**kw:None
m.other_links_clear=lambda *a:True
m.supported=lambda *a:True
far=np.array([[3.,3.,3.]])
m.release.hand_geometry=lambda *a:([far],far.copy())
def reset():
 global objects,finger_effort
 finger_effort=np.zeros((1,2))
 objects={o.name:o.state(pos=m.lots.current().pos(o.name)) for o in m.OBJECTS}
 m._history=m.native_history.History(expected={n:m.lots.current().pos(n) for n in ('bowl','plate')});m._safety=m.native_history.Safety()
 m._food=m.Food();m._food.deposits=[[np.array((-.020,-.025,.006)),17e-6],[np.array((.020,.025,.006)),17e-6]]
 m._food.source*=1-m._food.served()/m._food.source.sum()
 m._state=dict(known=False,invalid=False,held=0.,t=None,hands=None,corners=None,stamp=None)
 asset.data.body_com_pos_w[0,0]=objects['disher'].pos
 m._state.update(tier=0.,finger_force=None,peak_finger_force=0.)
 sim.t=0.
def native_tick(contact=None,dt=1/240,food_clear=True):
 m._food.ticks+=1;m._history.observe(dt,{n:(objects[n].pos,m.rotation(objects[n])) for n in ('bowl','plate')},food_clear)
 m._safety.observe(np.tile(finger_effort[0],2),np.zeros((4,3)) if contact is None else contact)
def tick(n=1):
 for i in range(n):
  sim.t+=.05
  for _ in range(12):native_tick()
  m.step(sim)
def row(label):print(json.dumps(dict(check=label,score=m.score(objects),invalid=m._state.get('invalid'),held=m._state.get('held'),error=m._food.error)))
print('task.py sha256',hashlib.sha256(P.read_bytes()).hexdigest())
reset();row('missing runtime');tick(45);row('synthetic complete goal with all sensor/support stubs true');assert m.score(objects)==1
m._food.deposits[0][1]+=1e-8;row('material changed after certificate');assert m.score(objects)==0
reset();tick(45);objects['plate']=replace(objects['plate'],pos=objects['plate'].pos+(.001,0,0));row('pose changed after certificate');assert m.score(objects)==0
reset();tick(5);m._food.cap[0]=m.VW[0];m._food.source*=1-m.VW[0]/m._food.source.sum();tick();row('material geometry count changed');assert not m._state['invalid'] and m._state['held']==0
reset();tick(5);sim.t+=.2;m.step(sim);row('step gap');assert m._state['invalid'] and m.score(objects)==0
reset();objects['plate']=replace(objects['plate'],pos=objects['plate'].pos+(.031,0,0));tick();objects['plate']=m.PLATE.state();tick(45);row('plate moved beyond limit then restored');assert m._state['invalid'] and m.score(objects)==0
reset();m.supported=lambda *a:False;tick(45);row('unknown/no table support');assert m.score(objects)==0
m.supported=lambda *a:True
reset();near=np.array([objects['bowl'].pos+(0,0,.0025)]);m.release.hand_geometry=lambda *a:([near],near.copy());tick(45);row('hand geometry intersects bowl');assert m.score(objects)==0
m.release.hand_geometry=lambda *a:([far],far.copy())
reset();m._food.deposits[0][1]=20e-6;m._food.deposits[1][1]=13.998e-6;m._food.source=m.INITIAL*(1-m._food.served()/m.INITIAL.sum());tick(45);row('safe partial tier');assert m.score(objects)==.5
m._food.deposits[1][1]+=.004e-6;m._food.source*=1-.004e-6/m._food.source.sum();tick();row('partial to full restarts hold');assert m._state['held']==0 and m.score(objects)==0
tick(40);row('full tier after fresh 2s');assert m.score(objects)==1
reset();finger_effort=np.full((1,2),40.01);tick(45);row('exact force limit');assert m.score(objects)==1
reset();finger_effort=np.full((1,2),40.02);tick();finger_effort[:]=0;tick(45);row('force alarm is independent of primary score');assert not m._state['invalid'] and m.score(objects)==1 and m._safety.overload
reset();finger_effort=np.full((1,2),np.nan);tick();tick(45);row('unknown force is independent of primary score');assert not m._state['invalid'] and m.score(objects)==1 and m._safety.error
reset();sim.t=.05;m.step(sim);row('missing physics substeps');assert m._state['invalid'] and m.score(objects)==0
print('CPU scorer controls passed; explicit native physics/contact/sensor stubs, no robot evidence.')

# Native-history branches are exercised between control observations.
for lot in m.LOTS:
 m.lots.use(lot.name)
 for sign in (-1,1):
  reset();objects['plate']=replace(objects['plate'],pos=objects['plate'].pos+(sign*.030,0,0));tick();assert not m._history.invalid
  objects['plate']=m.PLATE.state(pos=m.lots.current().pos('plate'));tick(45);assert m.score(objects)==1
  reset();objects['plate']=replace(objects['plate'],pos=objects['plate'].pos+(sign*.030002,0,0));native_tick();objects['plate']=m.PLATE.state(pos=m.lots.current().pos('plate'));tick(45);assert m._history.invalid and m.score(objects)==0
 reset();pose=m.parts._matrix_to_quat(m.parts._rot((1,0,0),np.deg2rad(10)));objects['bowl']=replace(objects['bowl'],quat_xyzw=pose);native_tick();assert not m._history.invalid
 pose=m.parts._matrix_to_quat(m.parts._rot((1,0,0),np.deg2rad(10.002)));objects['bowl']=replace(objects['bowl'],quat_xyzw=pose);native_tick();objects['bowl']=m.BOWL.state(pos=m.lots.current().pos('bowl'));tick(45);assert m._history.invalid and m.score(objects)==0
 reset();native_tick(contact=np.array([[0.,0.,40.01]]*4));tick(45);assert m.score(objects)==1
 reset();native_tick(contact=np.array([[0.,0.,40.02]]*4));tick(45);assert not m._history.invalid and m.score(objects)==1 and m._safety.overload
 reset();native_tick(contact=np.full((4,3),np.nan));tick(45);assert not m._history.invalid and m.score(objects)==1 and m._safety.error
 reset();native_tick(dt=.11);assert m._history.invalid
 reset();native_tick(food_clear=False);tick(45);assert m._history.hand_food_contact and m._history.invalid and m.score(objects)==0
 reset();native_tick(food_clear=None);tick(45);assert m._history.invalid and m.score(objects)==0
 reset();tick(45);m._history.ticks-=1;assert m.score(objects)==0
 print('Native motion/contact history and independent force/unknown-monitor scoring controls PASS',lot.name)
print('All runtime/control/contact data in this CPU script are explicit stubs. These results are scorer logic only.')

# Volume/spill boundaries and full-face separation, with the same explicit stubs.
m.lots.use('A')
reset();m._food.spill=3e-6;m._food.source=m.INITIAL*(1-(m._food.served()+m._food.spill)/m.INITIAL.sum());tick(45);assert m.score(objects)==1
reset();m._food.spill=3.002e-6;m._food.source=m.INITIAL*(1-(m._food.served()+m._food.spill)/m.INITIAL.sum());tick(45);assert m.score(objects)==0
reset();m._food.deposits=[[np.array((-.004,-.004,.006)),17e-6],[np.array((.004,.004,.006)),17e-6]];tick(45);assert m.progress()==.5
# Flat face/centre counterexample: all box vertices are distant, face intersects.
v=np.array(list(__import__('itertools').product((-.05,.05),(-.05,.05),(-.001,.001))));box=('test',np.array((0,0,.004)),np.eye(3),np.array((.002,.002,.002)))
assert not m.release.convex_clear(v,box[1],box[2],box[3])
assert not m.release.sampled_clear([np.array([[0.,0.,.001]])],[box])
box=('test',np.array((0,0,.030)),np.eye(3),np.array((.002,.002,.002)))
assert m.release.convex_clear(v,box[1],box[2],box[3])
print('Volume/spill/touching-mound/flat-face controlsPASS; all native inputs remain stubs.')
