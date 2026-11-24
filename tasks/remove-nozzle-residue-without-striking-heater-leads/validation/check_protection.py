"""Offline geometric regression. Does not test thermal insulation or native contacts."""
import pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent.parent))
import protection
from yam_arena import analysis
original={name:[h for h in analysis.link_hulls()[name] if h.vertices[:,2].min() >= -.017]
          for name in ('tip_left','tip_right')}
initial={name:protection.cover_hulls(name) for name in original}
protection.install()
# Spawn enumerates every robot link on both arms. Non-finger calls must not evict
# the unexpanded cover source and cause a second expansion of analysis hulls.
for _ in range(2):
    for name in analysis.link_hulls(): protection.cover_hulls(name)
for name,hulls in initial.items():
    assert protection.cover_hulls(name) is hulls
    assert len(analysis.link_hulls()[name]) >= len(hulls)
    for (points,faces),base in zip(hulls,original[name]):
        assert np.allclose(points.min(0),base.vertices.min(0)-protection.THICKNESS)
        assert np.allclose(points.max(0),base.vertices.max(0)+protection.THICKNESS)
        assert faces.min() >= 0 and faces.max() < len(points)
    print(name,len(hulls),'matching convex cover pieces; single expansion')
print('PASS (geometry only; no thermal qualification)')
