"""Independent model and endpoint controls; no hardware calibration claim."""
import sys,pathlib
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import task as t

def reset():t.M=t.new_material()
def sweep(wet=True,height=.0003,y=t.TRACE_Y):
 reset()
 if wet:t.M['wet'][:]=5e-10
 start=t.M['wet'].sum()
 for x in np.linspace(-.0285,-.0155,160):
  # Only one leading bundle touches the path; other bundles stay in air.
  pts=np.tile([0.,0.,.08],(21,1));pts[0]=[x,y,height]
  t.deposit(pts,.05)
 return start
reset();sweep(False);assert t.M['volume'].sum()==0;print('PASS dry stroke = no transfer')
sweep(True,.005);assert t.M['volume'].sum()==0;print('PASS hover = no transfer')
start=sweep();assert t.M['volume'].sum()>0
assert abs(start-t.M['wet'].sum()-t.M['volume'].sum())<1e-18
print('PASS finite supply conservation',t.M['volume'].sum(),'m3; coverage',t.coverage())
assert not t.M['bad'];snapshot=t.M['volume'].copy()
for _ in range(20):t.deposit(np.tile([0.,0.,.08],(21,1)),.05)
assert np.all(t.M['volume']>=snapshot);print('PASS persistent deposit after lift')
# Wet bundle contacting actual GPIO pad face must contaminate, not ignore the keepout.
reset();t.M['wet'][:]=5e-10
p=next(p for p in t.DATA['pads'] if p['name']=='GPIO1:22')
for x in np.linspace(p['x']/1000-.0003,p['x']/1000+.0003,80):
 pts=np.tile([0.,0.,.08],(21,1));pts[0]=[x,p['y']/1000,.0003];t.deposit(pts,.05)
assert t.M['bad'];print('PASS actual native pad fouling latches zero')
t.M['volume'][:]=0;t.deposit(np.tile([0.,0.,.08],(21,1)),.05);assert t.M['bad'];print('PASS returning/cleaning cannot erase violation')
# Independent endpoint tests: film masks are generated outcomes, history is explicit.
st={o.name:o.state() for o in t.OBJECTS};b=st[t.BOARD.name];tool=st[t.BRUSH.name]
ids=np.flatnonzero(t.TARGET);assert len(ids)==28
for n,want in [(0,0.),(6,0.),(7,.25),(13,.25),(14,.5),(22,.5),(23,.75),(26,.75),(27,1.),(28,1.)]:
 reset();t.M['volume'][ids[:n]]=t.AREA*.00005
 t.H=dict(known=True,bad=False,held=2.,certificate=t.certificate(st))
 got=t.score(st);assert got==want,(n,got,want)
print('PASS tier thresholds both sides, exact full score and success')
t.H['held']=1.99;assert t.score(st)==.75;print('PASS held tool / incomplete release hold capped')
t.H['known']=False;assert t.score(st)==0;print('PASS unknown hand state fail closed')
reset();t.M['volume'][ids]=t.AREA*.00005;t.M['volume'][ids[10]]=0
t.H=dict(known=True,bad=False,held=2.,certificate=t.certificate(st))
assert t.score(st)==0;print('PASS internal 0.5 mm gap zero')
t.M['volume'][ids[10]]=t.AREA*.001;assert t.score(st)==0;print('PASS 1 mm pool zero')
# A dry overcompressed stroke on the native copper must not escape the damage gate.
reset()
for x in np.linspace(-.026,-.024,40):
 pts=np.tile([0.,0.,.08],(21,1));pts[0]=[x,t.TRACE_Y,-.003];t.deposit(pts,.05)
assert t.M['bad'] and t.M['scratch']>=.001;print('PASS dry hard-contact copper damage latches zero')
# Whole-brush strokes: the three leading bundles of the actual 40-degree raked face, swept along the trace.
import math
def brush_stroke(heading,x0,x1,comp,seconds,fresh=True):
 if fresh:reset()
 t.M['wet'][18:21]=3e-9  # one full dip of the leading row
 r=t.parts._rot((0,0,1),math.radians(heading))@t.parts._rot((0,1,0),math.radians(40.))
 local=t.TIPS@r.T;local=local-local[19];n=int(seconds/.05)
 for k in range(-10,n+1):
  s=min(max(k/n,0),1);s=s*s*(3-2*s);p=np.array([x0+(x1-x0)*s,t.TRACE_Y,.0008-comp]);t.deposit(local+p,.05)
 for _ in range(5):t.deposit(local+p+(0,0,.01),.05)
brush_stroke(30,-.0285,-.013,.001,4.);brush_stroke(30,-.013,-.0285,.001,4.,fresh=False)
assert t.coverage()[0]==1. and t.coverage()[1]==0 and not t.M['bad'];print('PASS two light reloaded passes cover the trace and keep the pads clean')
brush_stroke(30,-.0285,-.013,.001,14.);assert t.coverage()[0]<.5 and not t.M['bad'];print('PASS one load stretched over a slow 14 s stroke runs dry: coverage',round(t.coverage()[0],3))
brush_stroke(-30,-.0285,-.013,.001,4.);assert t.M['bad'];print('PASS mirrored brush heading started at the trace end fouls GPIO1:24')
brush_stroke(30,-.0285,-.013,.003,4.);assert t.M['bad'];print('PASS pressing 3 mm splays the footprint onto GPIO1:24')
