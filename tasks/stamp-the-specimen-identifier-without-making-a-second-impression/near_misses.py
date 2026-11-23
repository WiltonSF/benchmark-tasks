"""Synthetic scorer boundaries plus physically distinct no-progress controls."""
import copy
import numpy as np
CASES=[('no-op',{},0.,False),('punch touching brass is not an impression',{'punch_1':((.278,0,.833),None,None)},0.,False)]
def checks(m):
 s=m.COUPON.state();r=np.eye(3)
 marks=[]
 for i,d in enumerate((1,2,3)):
  mark=m.imprint(d,.08,(m.CX+(d-2)*.012,0,m.Z+m.L/2),r);marks.append(mark)
  assert m.tiers(marks,s,True)==[.25,.5,1.][i]
 assert m.tiers(marks,s,False)==.75
 assert m.tiers(marks+[marks[-1]],s,True)==0.
 assert m.tiers(list(reversed(marks)),s,True)==1.
 wrong=copy.deepcopy(marks);wrong[0]['center']=wrong[1]['center'];assert m.tiers(wrong,s,True)==0.
 weak=m.imprint(1,.00001,(m.CX-.012,0,m.Z+m.L/2),r)
 assert weak['coverage']<.9 and m.tiers([weak],s,True)==0.
 tilted=m.imprint(1,.08,(m.CX-.012,0,m.Z+m.L/2),m.parts._rot((1,0,0),.15))
 assert tilted['coverage']<.9
 for coverage,expected in [(.9,.25),(.899,0.)]:
  q=copy.deepcopy(marks[:1]);q[0]['coverage']=coverage;assert m.tiers(q,s,True)==expected
 for angle,expected in [(10,1.),(10.01,.75),(20,.75),(20.01,0.)]:
  q=copy.deepcopy(marks);q[0]['yaw']=np.radians(angle);assert m.tiers(q,s,True)==expected
 st={p.name:p.state() for p in m.OBJECTS};original=m.REC
 m.REC=m.fresh();m.REC.update(known=True,marks=marks,hold=1.,clear=True,parked=True,certificate=m.certificate(st))
 assert m.score(st)==1. and m.success(st)
 for key,value in [('known',False),('hold',.999),('clear',False),('fail','historical second impression')]:
  old=m.REC[key];m.REC[key]=value;assert m.score(st)==0.;m.REC[key]=old
 missing=dict(st);del missing['hammer'];assert m.score(missing)==0.
 drift=dict(st);drift['punch_1']=m.PUNCHES[1].state();assert m.score(drift)==0.
 m.REC=original
 for e in [0.,.000001]:assert m.imprint(1,e,(m.CX-.012,0,m.Z+m.L/2),r)['coverage']==0.
 two=copy.deepcopy(marks);two[0]['coverage']=.1;assert m.tiers(two,s,True)==.5
 for pitch,expected in [(11.,1.),(10.99,.75),(13.,1.),(13.01,.75)]:
  q=copy.deepcopy(marks);q[0]['center'][0]=-pitch/1000;assert m.tiers(q,s,True)==expected
 for baseline,expected in [(1.,1.),(1.01,.75)]:
  q=copy.deepcopy(marks);q[0]['center'][1]=baseline/1000;assert m.tiers(q,s,True)==expected
 for error,expected in [(4.,.25),(4.01,0.)]:
  q=copy.deepcopy(marks[:1]);q[0]['center'][1]=error/1000;assert m.tiers(q,s,True)==expected
 for gap,expected in [(.5,.25),(.51,0.)]:
  q=copy.deepcopy(marks[:1]);q[0]['max_gap']=gap/1000;assert m.tiers(q,s,True)==expected
 for angle in [90,180,-180]:
  q=copy.deepcopy(marks);q[0]['yaw']=np.radians(angle);assert m.tiers(q,s,True)==0.
 q=copy.deepcopy(marks);q[0]['d']=4;assert m.tiers(q,s,True)==0.
 spec=m.PUNCHES[0];stp=spec.state();rot=m.parts._quat_to_matrix(stp.quat_xyzw)
 for gap in [.0005,.04998,.050,.05002]:
  point=stp.pos+rot@np.array((m.HALF+gap,0,0));actual=m.hand_gap([np.array([point])],spec,stp)
  assert abs(actual-gap)<1e-8
  assert (actual>=.05-m.EPS)==(gap>=.05-m.EPS)
 from dataclasses import replace
 ang=np.radians(20);qr=np.array((0,0,np.sin(ang/2),np.cos(ang/2)));R=m.parts._quat_to_matrix(qr)
 shifted=replace(s,pos=s.pos+np.array((.001,.001,0)),quat_xyzw=qr)
 point=shifted.pos+R@np.array((-.012,0,.0015+m.L/2))
 attached=m.imprint(1,.08,point,R,substrate=shifted)
 assert np.linalg.norm(np.asarray(attached['center'])-(-.012,0))<1e-9
 assert abs(attached['yaw'])<1e-9 and m.tiers([attached],shifted,True)==.25
 assert np.max(abs(np.asarray(attached['points'])-np.asarray(marks[0]['points'])))<1e-9
 print('material law, tier, repeated/wrong character, tilt, coverage and rotation controls PASS')
if __name__=='__main__':
 import pathlib,sys
 sys.path.insert(0,str(pathlib.Path(__file__).parent))
 from yam_arena.task_file import load
 checks(load(pathlib.Path(__file__).with_name('task.py')).module)
