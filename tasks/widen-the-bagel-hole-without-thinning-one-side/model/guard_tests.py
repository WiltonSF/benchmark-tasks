"""Offline adversarial native-state snapshots; these do not execute PhysX."""
import pathlib,sys
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent.parent))
import task,model,parts
from yam_arena import analysis,station
from yam_arena.constants import READY

class Tensor:
 def __init__(self,a):self.a=np.asarray(a)
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):return self.a
class View:
 def __init__(self,pose):self.pose=np.asarray(pose)
 def get_transforms(self):return self.pose
 def get_link_transforms(self):return self.pose
class Scene(dict):pass
scene=Scene();scene.env_origins=[Tensor(np.zeros(3))]
objects={o.name:o.state() for o in task.OBJECTS}
cells=model.cell_points(task.centers(objects),np.full(model.N,model.L0))
for i,spec in enumerate(task.SECTORS):task._mesh_points[spec.name]=cells[i]-spec.pos
for spec in [*task.SECTORS,task.PAN,task.TOOL]:scene[spec.name]=SimpleNamespace(root_view=View([[*spec.pos,0,0,0,1]]))
for arm in ('left','right'):
 poses=analysis.fk_link_poses_world({f'joint{i+1}':float(q) for i,q in enumerate(READY)},station.BASES[arm]);names=list(poses)
 scene[arm+'_robot']=SimpleNamespace(body_names=names,root_view=View([[np.r_[poses[n][:3,3],parts._matrix_to_quat(poses[n][:3,:3])] for n in names]]))
class BundleView:
 def get_transforms(self):return np.vstack([scene[spec.name].root_view.pose[0] for spec in [*task.SECTORS,task.PAN,task.TOOL]])
task._body_view=BundleView();task._body_index={spec.name:i for i,spec in enumerate([*task.SECTORS,task.PAN,task.TOOL])};task._env_origin=np.zeros(3)
task._current_sim=SimpleNamespace(env=SimpleNamespace(unwrapped=SimpleNamespace(scene=scene)))
task._record=model.Record(valid_reset=True);task._guard_previous=None;task._guard_edge=None;task._guard_p=None

def check(label,condition):
 assert condition,label
 print(label,'PASS')

task.native_guard(1/240)
check('native snapshots read without stale data',task._record.substeps==1 and not task._record.invalid)
check('clear ready hands are accepted',task._record.sub_clear)
left=scene['left_robot'];saved=left.root_view.pose.copy()
for name in ('gripper','tip_left','tip_right'):
 left.root_view.pose[0,left.body_names.index(name),:3]=task.SECTORS[0].pos
 task._record.sub_clear=True
 task.native_guard(1/240)
 check('between-control hand recontact interrupts hold '+name,not task._record.sub_clear)
left.root_view.pose=saved
view=scene[task.SECTORS[0].name].root_view;saved=view.pose.copy();view.pose[0,0]+=.10
task.native_guard(1/240)
check('substep overstretch is irreversible',task._record.invalid and task._record.sub_tear)
view.pose=saved;task.native_guard(1/240)
check('returning damaged food does not clear history',task._record.invalid)

p=model.P0*1.4;g=model.geometry(p,np.full(16,model.L0*1.4))
record=model.Record(valid_reset=False)
for t in np.arange(.05,3,.05):record.update(t,p,True,True,True,False)
check('unvalidated reset cannot be repaired into credit',record.score(g)==0.)
record.valid_reset=True
check('known valid released positive control',record.score(g)==1.)

# Folded doubled ring formerly earned.25; it now fails geometric validation.
angles=np.arange(8)*2*np.pi/8
fold=np.vstack([np.c_[.032*np.cos(angles),.032*np.sin(angles),np.zeros(8)],np.c_[.057*np.cos(angles),.057*np.sin(angles),np.zeros(8)]])
try:model.geometry(fold,np.full(16,model.L0*3))
except ValueError:print('overlapping double-loop endpoint PASS')
else:raise AssertionError('folded material earned a tier')

# A global rigid flip does not make the same released ring a different task.
flipped=p*np.array([1,-1,-1]);check('flipped intact ring remains legal',model.tier(model.geometry(flipped,np.full(16,model.L0*1.4)))==1.)
print('All offline guard counterexamples PASS')
