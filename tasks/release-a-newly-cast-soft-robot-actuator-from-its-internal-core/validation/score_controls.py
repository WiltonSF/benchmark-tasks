import importlib.util,pathlib,sys
import numpy as np
p=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(p))
spec=importlib.util.spec_from_file_location('task_under_test',p/'task.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def reset():m.S=m.fresh()
def states(body=(.32,.205,None),core=(.22,-.27,None)):
 b=(*body[:2],m.TOP+.009) if body[2] is None else body
 c=(*core[:2],m.TOP) if core[2] is None else core
 return {o.name:o.state(pos=b if o.name=='actuator' else c if o.name=='core' else o.pos) for o in m.OBJECTS}
def ready(t=3,clear=0,still=0):m.S.update(known=True,steps=5,physics=12,observed_physics=12,t=t,clear=clear,still=still)
def check(label,want,o):
 got=m.score(o);assert got==want,(label,got,want);assert m.success(o)==(got==1);print(label,got)
reset();o=states();check('unknown histories',0,o)
ready();check('intact centred released',1,o)
m.S['clear']=None;check('held / hand unknown',0,o)
ready(t=1.5);check('one second release tier',.5,o)
ready();check('offcentre inside tray',.75,states(body=(.337,.205,None)))
check('released on table',.25,states(body=(.32,0,m.TOP+.003)))
o2=states();o2['core']=m.CORE.state(pos=(.32,.18,m.TOP+.009));check('still partly on core',0,o2)
m.fail('tear');check('latched tear',0,o);ready();check('return to start cannot erase tear',0,o)
reset();o={a.name:a.state() for a in m.OBJECTS};o['core']=m.CORE.state(lin_vel=(0,-.03,0))
for _ in range(240):f,overlap,va=m.resistance(o)
assert 5.9<=np.linalg.norm(f)<=6.01;assert f@o['core'].lin_vel<0
print('resistance reaches6N from actual relative velocity',np.linalg.norm(f))
o['core']=m.CORE.state(pos=(m.X+.0041,-.025,m.Z));m.resistance(o);assert m.S['zero'];print('offaxis permanentzero',m.S['why'])
reset();o={a.name:a.state() for a in m.OBJECTS};h=np.array([[m.X,m.L/2,m.Z+.01]])
assert not m.hand_clear(h,o);print('face-adjacent hand blocks release')
print('PASS scorer/material controls')
