"""Independent CPU scorer/contact-buffer controls. Native force calibration is GPU evidence, not a CPU claim."""
import sys,pathlib,numpy as np
folder=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(folder))
import task,near_misses
start={p.name:p.state() for p in task.OBJECTS};count=0
for label,changes,want,success in near_misses.CASES:
 states=start|changes;got=task.score(states)
 assert abs(got-want)<1e-9 and task.success(states)==success,(label,got,want)
 count+=1
class View:
 def __init__(self,kind):
  self.kind=kind;self.fs=np.array([200.,200.]) if kind=='jaw' else np.array([-5. if kind=='body' else 5.])
  self.ns=np.array([[0.,0.,1.],[0.,0.,-1.]]) if kind=='jaw' else np.array([[1.,0.,0.]])
  self.pts=np.array([[task.X,task.JAW_Y,task.Z-task.R],[task.X,task.JAW_Y,task.Z+task.R]]) if kind=='jaw' else np.array([[task.X+task.R,0.,task.Z]])
  self.sep=np.zeros(len(self.fs));self.counts=np.ones((1,2 if kind=='jaw' else 1));self.first=np.array([[0.,1.]]) if kind=='jaw' else np.zeros((1,1))
 def get_contact_data(self,dt):return self.fs,self.pts,self.ns,self.sep,self.counts,self.first
 def get_friction_data(self,dt):
  sign=-1 if self.kind=='body' else 1
  return np.array([[0.,sign,0.]]),np.array([[task.X+task.R,0.,task.Z]]),np.ones((1,1)),np.zeros((1,1))
def views():return [View('body'),View('jaw'),View('core')]
def read(v):
 task.S.clear();task.S.update(task.fresh());task.S.update(org=np.zeros(3),iview=v)
 return task.native_contacts(1/240)
v=views();read(v)
assert task.S['jaw_valid'];assert np.linalg.norm(task.S['native_wrench']['net_force'])<1e-10
assert np.linalg.norm(task.S['native_wrench']['net_moment_world'])<1e-10
print('PASS signed normal + tangential contact reaction and world-moment parsing')
# Active corruption must fail closed; unused GPU capacity must not be read.
for label,mutate in [
 ('NaN active normal force',lambda v:v[0].fs.__setitem__(0,np.nan)),
 ('NaN separation',lambda v:v[0].sep.__setitem__(0,np.nan)),
 ('nonunit normal',lambda v:v[0].ns.__setitem__((0,0),2.)),
 ('missing filter table',lambda v:setattr(v[0],'counts',np.zeros((1,0)))),
 ('overlapping jaw contact ranges',lambda v:v[1].first.__setitem__((0,1),0.)),
 ('out of bounds active buffer',lambda v:v[0].counts.__setitem__((0,0),3.)),
]:
 v=views();mutate(v)
 try:read(v)
 except (ValueError,IndexError):pass
 else:raise AssertionError(label+' accepted')
 print('PASS reject',label)
v=views()
for w in v:
 w.fs=np.r_[w.fs,np.nan];w.pts=np.vstack([w.pts,[np.nan]*3]);w.ns=np.vstack([w.ns,[np.nan]*3]);w.sep=np.r_[w.sep,np.nan]
read(v);assert task.S['jaw_valid'];print('PASS ignore inactive undefined capacity')
print('PASS',count,'scoring cases plus8 contact-buffer/reaction controls')
