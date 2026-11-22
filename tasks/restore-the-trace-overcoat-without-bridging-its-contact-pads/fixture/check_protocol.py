"""Stateful scorer regressions with explicit mocked robot/render interfaces; not robot demonstrations."""
import sys,pathlib,dataclasses,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import task as t
class Sim:
 def __init__(self,states):self.t=0.;self.data=states
 def states(self):return self.data

def states():
 out={o.name:o.state() for o in t.OBJECTS}
 # The laminate rests level on the four standoffs at its declared start.
 out[t.BOARD.name]=t.BOARD.state(pos=np.asarray(t.BOARD_A))
 return out

def start():
 sim=Sim(states());t.reset(sim);return sim

def tick(sim,count=1):
 for _ in range(count):sim.t+=.05;t.step(sim)

def full_film():t.M['volume'][t.TARGET]=t.AREA*.00005
# Robot/render interfaces are deliberately explicit in these scorer-only probes.
t.visual=lambda *args:None
t.hands=lambda sim,need_clouds=True:([np.array([[10.,10.,10.]])] if need_clouds else [],8.)
sim=start();full_film();tick(sim,43);assert t.score(sim.data)==1.,(t.H,t.coverage());print('PASS supported released full film requires stateful 2 s hold')
# Changing any scored observation invalidates its certificate immediately.
bad=dict(sim.data);bad[t.CUP.name]=dataclasses.replace(bad[t.CUP.name],pos=np.array([math.nan,0,0]));assert t.score(bad)==0
bad=dict(sim.data);del bad[t.CUP.name];assert t.score(bad)==0;print('PASS NaN or missing cup cannot inherit full credit')
sim=start();full_film();sim.data[t.CUP.name]=dataclasses.replace(sim.data[t.CUP.name],ang_vel=np.array([0.,0.,1.]));tick(sim,43);assert t.score(sim.data)==.75;print('PASS spinning cup prevents full stillness hold')
sim=start();full_film();b=sim.data[t.BOARD.name];sim.data[t.BOARD.name]=t.BOARD.state(pos=b.pos+(0,0,.003),quat_xyzw=b.quat_xyzw);tick(sim,43);assert t.score(sim.data)==.75;print('PASS controller lifted 3 mm off its standoffs lacks support')
sim=start();full_film();b=sim.data[t.BRUSH.name];sim.data[t.BRUSH.name]=t.BRUSH.state(pos=b.pos+(0,0,.003));tick(sim,43);assert t.score(sim.data)==.75;print('PASS brush centre near rack is insufficient without support')
sim=start();b=sim.data[t.BOARD.name];sim.data[t.BRUSH.name]=t.BRUSH.state(pos=b.pos+(0,0,-.003),quat_xyzw=(1,0,0,0));tick(sim);assert t.H['bad'];print('PASS inverted dry wooden backing intrusion latches zero')
sim=start();full_film();sim.data[t.CUP.name]=t.CUP.state(quat_xyzw=t.parts._matrix_to_quat(t.parts._rot((1,0,0),math.pi/2)));tick(sim);assert t.M['bad'] and t.M['cup']==0 and abs(t.M['lost']-5e-6)<1e-15;print('PASS tipped cup spills finite liquid and latches zero')
# Loading uses real local immersion and remaining liquid height.
sim=start();cup=sim.data[t.CUP.name];tips=np.tile(np.asarray(cup.pos)+(0,0,.006),(21,1));before=t.M['cup'];t.load_brush(tips,cup,.05);assert t.M['wet'].sum()>0 and abs(before-t.M['cup']-t.M['wet'].sum())<1e-18
print('PASS upright loading conserves finite cup supply')
t.M=t.new_material();t.M['cup']=1e-9;t.load_brush(tips,cup,.05);assert t.M['wet'].sum()==0;print('PASS almost empty cup has a lower liquid surface')
# A 3D airborne approach may coat its actual landing region, never its airborne projected path.
t.M=t.new_material();t.M['wet'][:]=2e-9
a=np.tile([0.,0.,.08],(21,1));a[0]=[-.0285,t.TRACE_Y,.010]
b=a.copy();b[0]=[-.0155,t.TRACE_Y,.0003]
t.deposit(a,.05);t.deposit(b,.05)
assert not np.any(t.M['volume'][(t.GRID[:,0]<-.018)&t.TARGET]);print('PASS airborne sweep projection cannot paint uncontacted copper')
# Spatial scar length is not repeated travel; native contact-pad copper is included.
t.M=t.new_material()
for k in range(40):
 p=np.tile([0.,0.,.08],(21,1));p[0]=[-.026+(k%2)*.0002,t.TRACE_Y,-.003];t.deposit(p,.05)
assert t.M['scratch']<.0004 and not t.M['bad'];print('PASS repeated 0.2 mm rubbing remains a short scar')
t.M=t.new_material();pad=next(p for p in t.DATA['pads'] if p['name']=='GPIO1:22')
# A 270-degree arc on the actual annulus has more than 1 mm unique scar length.
for angle in np.linspace(0,1.5*math.pi,60):
 p=np.tile([0.,0.,.08],(21,1));p[0]=[pad['x']/1000+.0007*math.cos(angle),pad['y']/1000+.0007*math.sin(angle),-.003];t.deposit(p,.05)
assert t.M['bad'] and t.M['scratch']>=.001;print('PASS long dry hard scar on native pad copper latches zero')
# A fast move well above the board is harmless; a 100 mm jump that could touch the board in one step fails closed.
t.M=t.new_material();t.deposit(np.tile([0.,0.,.08],(21,1)),.05);t.deposit(np.tile([.3,.0,.08],(21,1)),.05);assert not t.M['bad']
print('PASS a fast airborne move above the board is not penalised')
t.M=t.new_material();t.M['wet'][:]=2e-9;t.deposit(np.tile([-.06,t.TRACE_Y,.0003],(21,1)),.05);t.deposit(np.tile([.06,t.TRACE_Y,.0003],(21,1)),.05);assert t.M['bad']
print('PASS a 120 mm in-contact jump in one step fails closed')
