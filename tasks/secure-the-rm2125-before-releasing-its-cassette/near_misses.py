"""Synthetic state/history controls. These test the scorer, not robot execution or contact physics."""
import pathlib,math,types
import numpy as np
import parts

def cases():
 path=pathlib.Path(__file__).with_name('task.py').resolve()
 g=next(fn.__globals__ for fn in parts._ON_STEP if pathlib.Path(fn.__globals__.get('__file__','/')).resolve()==path)
 t=types.SimpleNamespace(**g)
 def make(w=math.pi/3,h=.010,a=math.radians(190),q=0.,cp=None):
  o={p.name:p.state() for p in t.OBJECTS}
  o['handwheel']=t.WHEEL.state(joints={'wheel':w,'handle':h})
  o['specimen_guide']=t.HEAD.state(joints={'head':.01475,'clamp':q})
  o['knife_guard']=t.GUARD.state(joints={'guard':a})
  o['cassette']=t.CASS.state(pos=cp or (t.CX,t.CY,t.TOP+.190))
  return o
 far=[np.array([[-1.,-1.,t.TOP+1.]])]
 def prime(o,n=1,clouds=far):
  t.S.clear();t.S.update(t.fresh())
  for _ in range(n):t.observe(o,clouds,.05)
 def emit(label,o,want):return label,o,want,want==1.
 o=make();t.S.clear();t.S.update(t.fresh());yield emit('unknown gripper state',o,0.)
 o=make(a=0);prime(o);yield emit('pin locked; guard still open',o,.25)
 o=make(h=.0059);prime(o);yield emit('handle short of 6 mm',o,0.)
 o=make(h=.006,a=0);prime(o);yield emit('handle exactly 6 mm',o,.25)
 o=make(w=math.pi/3+math.radians(2.6),a=0);prime(o);yield emit('wheel outside lock tolerance',o,0.)
 o=make();prime(o);yield emit('both safeguards and released cassette still clamped',o,.5)
 o=make(a=math.radians(174.9));prime(o);yield emit('guard short of cover threshold',o,.25)
 o=make(a=math.radians(175));prime(o);yield emit('guard exactly cover threshold',o,.5)
 o=make(cp=(.22,.12,t.TOP+.05));prime(o);yield emit('cassette removed and released',o,.75)
 o=make();o['cassette']=t.CASS.state(pos=(.215,.215,t.TOP+.00625),quat_xyzw=(0,math.sin(math.pi/4),0,math.cos(math.pi/4)))
 prime(o,20);yield emit('less than 1 second since first eligible sample',o,.75)
 prime(o,21);yield emit('one second supported tray placement',o,1.)
 moved=dict(o);moved['grey_tray']=t.TRAY.state(pos=(.4,.25,t.TOP));prime(moved,25);yield emit('cassette at stale tray position',moved,.75)
 moved=dict(o);r=parts._rot((0,0,1),.3);moved['grey_tray']=t.TRAY.state(pos=(.24,.29,t.TOP),quat_xyzw=parts._matrix_to_quat(r))
 moved['cassette']=t.CASS.state(pos=(.24,.29,t.TOP+.00625),quat_xyzw=parts._matrix_to_quat(r@parts._rot((0,1,0),math.pi/2)))
 prime(moved,21);yield emit('moved and turned tray follows current pose',moved,1.)
 held=make();prime(held,clouds=[np.array([[t.CX+.00325+.0005,t.CY,t.TOP+.190]])]);yield emit('broad hand face 0.5 mm from cassette',held,0.)
 unsafe=make(w=0,h=0,a=0,cp=(.2,.1,t.TOP+.05));prime(unsafe);yield emit('unsafe direct removal',unsafe,0.)
 repaired=make();t.observe(repaired,far,.05);yield emit('putting cassette back cannot erase unsafe removal',repaired,0.)
 t.S.clear();t.S.update(t.fresh())
CASES=cases()
