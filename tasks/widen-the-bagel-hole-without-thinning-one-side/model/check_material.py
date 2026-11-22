"""Declared-law ablations. These are synthetic checks, not recipe calibration."""
import numpy as np
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import model
out={}
for label,scale,flatten in [('reset',1.,1.),('uniform free enlargement',1.4,1.),('flattened ring',1.4,.35)]:
 p=model.P0*scale;r=np.full(model.N,model.L0*scale);q=np.full(model.N,flatten)
 a,b=model.sections(p,r,q);length=(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)+np.linalg.norm(p-np.roll(p,1,axis=0),axis=1))/2
 assert np.max(abs(a*b*length-model.A*model.B*model.L0))<1e-12
 try:g=model.geometry(p,r,q);score=model.tier(g);g={k:v for k,v in g.items() if np.isscalar(v)}
 except ValueError:g={};score=0.
 out[label]={'score':score,'geometry':g,'relative_volume_error':float(np.max(abs(a*b*length/(model.A*model.B*model.L0)-1)))}
assert out['reset']['score']==0.;assert out['uniform free enlargement']['score']==1.;assert out['flattened ring']['score']==0.
# One-axis hard pull cannot masquerade as evenly distributed shaping.
p=model.P0.copy();p[:,1]*=1.6;r=np.full(16,model.L0)
g=model.geometry(p,r);assert model.tier(g)<1.
out['one-axis pull']={'score':model.tier(g),'geometry':{k:v for k,v in g.items() if np.isscalar(v)}}
# A ring geometrically correct but not released, held for<2s or damaged earns0.
g=model.geometry(model.P0*1.4,np.full(16,model.L0*1.4))
for tag,clear,damage,stop in [('held',False,False,3.),('short release',True,False,1.9),('damage',True,True,3.)]:
 rec=model.Record(valid_reset=True)
 for t in np.arange(.05,stop,.05):rec.update(t,model.P0*1.4,True,clear,True,damage)
 assert rec.score(g)==0.;out[tag]={'score':rec.score(g)}
print(json.dumps(out,indent=2))
# The provided sharp scraper cannot remove a core and keep shaping credit.
import task
objects={spec.name:spec.state() for spec in task.OBJECTS}
p=task.centers(objects)
edge_at=p[0]-np.array((0.,.0508,-.0175))
objects[task.TOOL.name]=task.TOOL.state(pos=edge_at,lin_vel=(.01,0,0))
assert task.cutting_edge(objects,p)
objects[task.TOOL.name]=task.TOOL.state(pos=edge_at-np.array((0,0,.022)),lin_vel=(.01,0,0))
assert not task.cutting_edge(objects,p)
