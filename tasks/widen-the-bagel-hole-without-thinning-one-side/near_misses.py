"""Offline public scorer states fail closed without runtime hand evidence."""
import numpy as np
import task,model
# Full-looking geometry cannot earn credit without observed runtime release.
correct={s.name:s.state(pos=task.C+model.P0[i]*1.4) for i,s in enumerate(task.SECTORS)}
neck={s.name:s.state(pos=task.C+model.P0[i]*np.array([1.,1.6,1.])) for i,s in enumerate(task.SECTORS)}
CASES=[('no-op',{},0.,False),('full-looking but unverified held ring',correct,0.,False),('one-axis pull without release evidence',neck,0.,False),('missing hand record',{},0.,False)]
if __name__=='__main__':
 import model,numpy as np
 for scale,want in [(1.,0.),(1.10,0.),(1.12,.25),(1.15,.25),(1.25,.25),(1.26,.25),(1.28,.5),(1.35,1.),(1.36,1.),(1.45,1.)]:
  p=model.P0*scale;rest=np.full(model.N,model.L0*scale)
  g=model.geometry(p,rest);got=model.tier(g)
  assert got==want,(scale,got,want)
  print(scale,got,{k:round(v*1000,2) for k,v in g.items() if np.isscalar(v)})
 # Numerical threshold tests use independent measured endpoint values.
 base=dict(d=.045,D=.055,w_min=.012,w_max=.0162,h_min=.012,outer=.130)
 assert model.tier(base)==1.
 for key,v in [('d',.029),('D',.066),('w_min',.011),('h_min',.011)]:
  g=base|{key:v};assert model.tier(g)==0.,(key,g)
 for d,want in [(.0299,0),(.030,.25),(.0399,.25),(.040,.5),(.0449,.5),(.045,1.)]:
  assert model.tier(base|{'d':d})==want
 for key,v in [('w_max',.0163),('outer',.131)]:assert model.tier(base|{key:v})==.75
 assert model.tier(base|{'D':.056})==.5
 r=model.Record(valid_reset=True);p=model.P0*1.45;g=model.geometry(p,np.full(16,model.L0*1.45))
 for t in np.arange(.05,2.2,.05):r.update(t,p,True,True,True,False)
 assert r.score(g)==1.
 r.update(2.25,p,True,False,True,False);assert r.score(g)==0.
 r.update(2.30,p,True,True,True,True)
 for t in np.arange(2.35,5.,.05):r.update(t,p,True,True,True,False)
 assert r.score(g)==0.
 # A face at0.5mm must fail even when distant hull vertices would pass.
 objects={spec.name:spec.state() for spec in task.OBJECTS}
 st=objects[task.SECTORS[0].name]
 face=np.array([(st.high[0]+.0005,st.pos[1],st.pos[2])])
 assert task.hand_distance([face]*6,objects)<=.000501
 try:task.hand_distance([np.empty((0,3))]*6,objects)
 except ValueError:pass
 else:raise AssertionError('unknown hand accepted')
 print('threshold, flattening, neck, hold, recontact, face proximity, unknown hand and permanent damage cases PASS')
