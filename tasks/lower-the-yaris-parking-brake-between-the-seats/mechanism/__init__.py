"""Contact-qualified parking-brake ratchet and independent, fail-closed trial judge."""
from dataclasses import dataclass
import math
import numpy as np

D=math.pi/180
@dataclass(frozen=True)
class Sample:
    t: float
    theta: float
    button: float
    forces: np.ndarray                 # forces on each hand: [hand, lever/button, xyz]
    moments: np.ndarray                # forces ON lever, moment about pivot: [hand, lever/button]
    press: np.ndarray                  # inward axial button force, each hand
    clearance: float
    integrity: bool
    cable_effort: float                # actual signed joint effort
    other_link_force: float=0.
    cable_reference_theta: float | None=None  # native effort was applied at the preceding physics pose

class Judge:
    def __init__(self,start=24*D,pitch=4*D):
        self.start,self.pitch=start,pitch
        self.reset()
    def reset(self):
        self.invalid=False;self.reason='';self.known=False;self.previous=None
        self.tooth=self.start;self.released=False;self.lifted_once=False;self.pressed_once=False
        self.hold=0.;self.last=None;self.role=None;self.work=np.zeros(2)
        self.lower=self.start;self.button_hi=.001;self.supported=False;self.previous_eligible=False
        self.peak_moment=np.zeros(2);self.peak_press=np.zeros(2);self.peak_support=np.zeros(2)
        self.unheld_descent=0.;self._moment_history=[]
    def fail(self,reason):
        self.invalid=True;self.released=False;self.lower=self.tooth;self.button_hi=.001;self.supported=False
        if not self.reason:self.reason=reason
    @staticmethod
    def load(theta):return 3.+112.2*theta
    def step(self,s):
        self.last=s;self.known=False
        try:
            f=np.asarray(s.forces,float);m=np.asarray(s.moments,float);press=np.asarray(s.press,float)
            vals=np.r_[s.t,s.theta,s.button,s.clearance,s.cable_effort,s.other_link_force,f.ravel(),m.ravel(),press,[] if s.cable_reference_theta is None else [s.cable_reference_theta]]
            if f.shape!=(2,2,3) or m.shape!=(2,2) or press.shape!=(2,) or not np.isfinite(vals).all():
                self.fail('missing or nonfinite observation');return
            self.known=True
            if self.invalid:return
            if not s.integrity:self.fail('assembly integrity')
            if s.other_link_force>2.+1e-6:self.fail('other robot link touched lever or button')
            if not -.5*D<=s.theta<=36.5*D or not -.0005<=s.button<=.0085:self.fail('outside physical travel')
            reference=s.theta if s.cable_reference_theta is None else s.cable_reference_theta
            if s.cable_reference_theta is not None and abs(reference-(self.previous.theta if self.previous is not None else self.start))>1e-6:self.fail('cable reference is not preceding physics pose')
            if abs(s.cable_effort+self.load(reference))>.5:self.fail('declared cable load absent')
            load=max(self.load(s.theta),-s.cable_effort)
            dt=s.t-self.previous.t if self.previous is not None else 0.
            if self.previous is not None and (dt<=0 or dt>1/200+1e-7):self.fail('physics sample gap')
            if self.invalid:self.previous=s;return
            self.peak_moment=np.maximum(self.peak_moment,m[:,0]);self.peak_press=np.maximum(self.peak_press,press)
            support_contact=np.linalg.norm(f[:,0],axis=1)
            self.peak_support=np.maximum(self.peak_support,support_contact)
            if self.previous is not None and s.theta>self.previous.theta:
                self.work+=np.maximum(0.,m[:,0])*(s.theta-self.previous.theta)
            # Lift must physically counter the complete declared cable moment.
            lifted=s.theta>=self.tooth+1*D-1e-8
            holders=[i for i in (0,1) if m[i,0]>=load-.1 and support_contact[i]>=1.]
            if lifted and holders:self.lifted_once=True
            pairs=[(h,p) for h in holders for p in (0,1) if p!=h and press[p]>=5.-1e-6 and s.button>=.005-1e-6]
            if lifted and pairs:
                self.released=True;self.pressed_once=True;self.role=pairs[0]
            pressers=[i for i in (0,1) if press[i]>=5.-1e-6 and s.button>=.005-1e-6]
            pressed=s.button>=.0045 if self.released else bool(pressers)
            lower=0. if self.released and pressed else self.tooth
            if self.released and not pressed:
                self.tooth=max(0.,self.pitch*math.floor(max(0.,s.theta)/self.pitch+1e-8))
                self.released=False;lower=self.tooth
            if not self.released:
                while s.theta>=self.tooth+self.pitch+1e-8:self.tooth+=self.pitch
                lower=self.tooth
            if s.theta<lower-.5*D:self.fail('ratchet bypass')
            self._moment_history.append(m[:,0].copy());self._moment_history=self._moment_history[-12:]
            mean_moment=np.mean(self._moment_history,axis=0)
            if self.previous is not None and self.released and pressed and s.theta<self.previous.theta:
                partners=[(h,p) for h in (0,1) for p in (0,1) if h!=p and support_contact[h]>=1. and mean_moment[h]>=load-.1 and press[p]>=4.5]
                if not partners:self.unheld_descent+=self.previous.theta-s.theta
                if self.unheld_descent>.25*D:self.fail('lowered without force-qualified second holding hand')
            button_hi=.008 if lifted and holders or self.released else .001
            supported=(not self.released and abs(s.theta-self.tooth)<=.5*D+1e-8 and s.button<=.0011)
            if s.theta<=.5*D+1e-8:supported=supported and self.tooth<=1e-8
            still=self.previous is not None and dt>0 and abs(s.theta-self.previous.theta)/dt<.5*D and abs(s.button-self.previous.button)/dt<.002
            eligible=self.known and not self.invalid and self.pressed_once and self.lifted_once and s.theta<=20*D+1e-8 and supported and s.clearance>=.030 and still
            if eligible and getattr(self,'previous_eligible',False):self.hold+=dt
            else:self.hold=0.
            self.previous_eligible=eligible;self.previous=s
            if not self.invalid:self.lower=lower;self.button_hi=button_hi;self.supported=supported
        except (AttributeError,TypeError,ValueError,IndexError):self.fail('malformed observation')
    def score(self):
        s=self.last
        if self.invalid or not self.known or s is None or self.hold<1.-1e-8 or not self.pressed_once or not self.lifted_once or not getattr(self,'supported',False):return 0.
        if s.theta<=.5*D+1e-8 and self.tooth<=1e-8:return 1.
        if s.theta<=8*D+1e-8:return .75
        if s.theta<=16*D+1e-8:return .5
        if s.theta<=20*D+1e-8:return .25
        return 0.
