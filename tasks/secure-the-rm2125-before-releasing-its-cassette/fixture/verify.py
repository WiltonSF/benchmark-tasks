"""Independent fabricated-state scorer controls and physical-force arithmetic, without Isaac."""
import sys,pathlib,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import task as t
from force_check import check

def objects(wheel=math.pi/3,handle=.010,guard=math.radians(190),clamp=0.,out=False,tray=False):
 o={p.name:p.state() for p in t.OBJECTS}
 o['handwheel']=t.WHEEL.state(joints={'wheel':wheel,'handle':handle})
 o['specimen_guide']=t.HEAD.state(joints={'head':.01475,'clamp':clamp})
 o['knife_guard']=t.GUARD.state(joints={'guard':guard})
 p=(t.CX,t.CY,t.TOP+.190)
 if out:p=(.22,.12,t.TOP+.05)
 if tray:
  # Cassette lies on its broad face, x axis (its thickness) vertical.
  p=np.array(o['grey_tray'].pos)+np.array((0,0,.003+.0065/2))
  o['cassette']=t.CASS.state(pos=p,quat_xyzw=(0,math.sin(math.pi/4),0,math.cos(math.pi/4)))
 else:o['cassette']=t.CASS.state(pos=p)
 return o
CLEAR=[np.array([[-1.,-1.,t.TOP+1.]])]
def case(label,o,expected,steps=1):
 t.S.clear();t.S.update(t.fresh())
 for k in range(steps):t.observe(o,CLEAR,.05)
 got=t.score(o);assert got==expected,(label,got,expected,t.S)
 assert t.success(o)==(expected==1.)
 print('PASS',label,got)

def run():
 print(check())
 gap=wheel_face= -.018-.006
 knob_face=t.knob0[1]+.040+.010
 assert wheel_face-knob_face>=.001-1e-9
 for shift in (0.,11.00017714):
  pin_lo,pin_hi=9.+shift,51.+shift
  assert min(pin_hi,48.96)-max(pin_lo,44.96)>=3.99
  assert pin_lo-shift>=8.9
 print('PASS physical knob/disc clearance and moving guide-pin engagement')
 bad=objects(tray=True);bad['knife_guard'].joints['guard']=float('inf');case('nonfinite joint rejected',bad,0.,25)
 bad=objects(tray=True);bad['knife_guard'].links.clear();case('missing link observation rejected',bad,0.,25)
 case('unknown hand observation',objects(),0.,steps=0)
 case('wheel only',objects(guard=0),.25)
 case('both safeguards',objects(),.5)
 case('released removed cassette',objects(out=True),.75)
 case('tray before 1 second',objects(tray=True),.75,19)
 case('tray 1 second',objects(tray=True),1.,21)
 o=objects(tray=True);o['grey_tray']=t.TRAY.state(pos=(.25,.2,t.TOP),quat_xyzw=(0,0,math.sin(.2),math.cos(.2)))
 # Carry the final cassette with its moved and rotated tray in both coordinates.
 r=t.parts._quat_to_matrix(o['grey_tray'].quat_xyzw)
 o['cassette']=t.CASS.state(pos=o['grey_tray'].pos+r@np.array((0,0,.00625)),quat_xyzw=t.parts._matrix_to_quat(r@t.parts._rot((0,1,0),math.pi/2)))
 case('moved rotated tray',o,1.,21)
 old=objects(tray=True);old['grey_tray']=t.TRAY.state(pos=(.4,.25,t.TOP));case('old tray coordinates do not count',old,.75,25)
 case('unsafe removal permanent zero',objects(wheel=0,handle=0,guard=0,out=True),0.)
 t.S.clear();t.S.update(t.fresh());t.observe(objects(guard=0),CLEAR,.05)
 # A broad face at the cassette, not just hull vertices, triggers the safety zone.
 t.observe(objects(guard=0),[np.array([[t.CX,t.CY,t.TOP+.19]])],.05)
 t.observe(objects(),CLEAR,.05);assert t.score(objects())==0.
 print('PASS unsafe reach cannot be erased')
 # Exact clearance comparison rejects a gripper face <=5 mm from a cassette face.
 o=objects();point=np.array([[t.CX+.00325+.0049,t.CY,t.TOP+.19]])
 assert not t.released(o,[point])
 print('PASS 4.9 mm face gap cannot count as released')
 # Reuse a primed score with changed current observations, without a callback.
 import copy
 for mode in ('missing','nan','inf','finite-changed'):
  good=objects(tray=True);case('prime link certificate '+mode,good,1.,25)
  bad=copy.deepcopy(good)
  if mode=='missing':bad['knife_guard'].links.clear()
  else:
   pos,quat=bad['knife_guard'].links['guard']
   pos[0]=float(mode) if mode in ('nan','inf') else pos[0]+.01
  assert t.score(bad)==0. and not t.success(bad),mode
  print('PASS current link rejected before callback',mode)

if __name__=='__main__':run()
