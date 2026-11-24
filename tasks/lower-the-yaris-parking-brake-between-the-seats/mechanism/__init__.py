"""Parking-brake ratchet state and the trial judge. The judge reads only lever angle, button travel, hand clearance and time."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np

D=math.pi/180
@dataclass(frozen=True)
class Sample:
    t: float
    theta: float          # lever angle above its bottom stop, rad
    button: float         # button travel, m
    clearance: float      # nearest gripper surface to lever or button, m (conservative)
    integrity: bool       # links are where the joints say they are
    hand_on_grip: bool=True  # binary physical contact, no continuous force requirement

class Judge:
    def __init__(self,start=24*D,pitch=4*D,clear=1.*D,release=.005,reengage=.0045,travel=.008,locked=.001,
                 drop=8*D,drop_s=.1,clear_m=.030,hold_s=1.):
        self.start,self.pitch,self.clear=start,pitch,clear
        self.release,self.reengage,self.travel,self.locked=release,reengage,travel,locked
        self.drop,self.drop_s,self.clear_m,self.hold_s=drop,drop_s,clear_m,hold_s
        self.reset()
    def reset(self):
        self.invalid=False;self.reason='';self.known=False;self.previous=None;self.last=None
        self.tooth=self.start;self.free=False;self.released_once=False;self.dropped=False
        self.hold=0.;self.resting=False;self.previous_eligible=False
        self.lower=self.start;self.button_hi=self.locked;self._history=deque();self.unheld_descent=0.
    def fail(self,reason):
        self.invalid=True
        if not self.reason:self.reason=reason
    def step(self,s):
        """One physics sample. Updates the ratchet (pawl) state and the hold timer."""
        self.last=s;self.known=False
        try:
            vals=np.array((s.t,s.theta,s.button,s.clearance),float)
            if not np.isfinite(vals).all():self.fail('missing or nonfinite observation');return
            self.known=True
            if not s.integrity:self.fail('assembly integrity')
            if not -.5*D<=s.theta<=36.5*D or not -.0005<=s.button<=self.travel+.0005:self.fail('outside physical travel')
            dt=s.t-self.previous.t if self.previous is not None else 0.
            if self.previous is not None and (dt<=0 or dt>1/200+1e-7):self.fail('physics sample gap')
            # Ratchet. The pawl sits on a tooth; the cable load holds it there. Lifting the lever off the tooth
            # frees the button; the button pushed in lifts the pawl; the pawl drops back when the button comes out.
            if self.free:
                if s.button<self.reengage:
                    self.free=False;self.tooth=max(0.,self.pitch*math.floor(max(0.,s.theta)/self.pitch+1e-8))
            else:
                while s.theta>=self.tooth+self.pitch-1e-8 and self.tooth+self.pitch<=36*D:self.tooth+=self.pitch
                if s.button>=self.release-1e-7 and s.theta>=self.tooth+self.clear-1e-8:self.free=True;self.released_once=True
            unloaded=s.theta>=self.tooth+self.clear-1e-8
            self.lower=0. if self.free else self.tooth
            # A loaded pawl blocks the button. The stop is never set below the button's present travel.
            self.button_hi=self.travel if (self.free or unloaded) else max(self.locked,min(self.button_hi,s.button))
            if self.previous is not None and self.free and not s.hand_on_grip:
                self.unheld_descent+=max(0.,self.previous.theta-s.theta)
                if self.unheld_descent>.25*D:self.fail('button-held descent without hand on grip')
            # Dropped lever: fell `drop` within `drop_s`.
            self._history.append((s.t,s.theta))
            while self._history and s.t-self._history[0][0]>self.drop_s+1e-9:self._history.popleft()
            if max(th for _,th in self._history)-s.theta>=self.drop:self.dropped=True
            self.resting=(not self.free and abs(s.theta-self.tooth)<=.5*D+1e-8 and s.button<=self.locked+.0001)
            still=self.previous is not None and dt>0 and abs(s.theta-self.previous.theta)/dt<.5*D and abs(s.button-self.previous.button)/dt<.002
            eligible=(not self.invalid and self.released_once and self.resting and still and s.clearance>=self.clear_m)
            self.hold=self.hold+dt if eligible and self.previous_eligible else 0.
            self.previous_eligible=eligible;self.previous=s
        except (AttributeError,TypeError,ValueError,IndexError):self.fail('malformed observation')
    def score(self):
        s=self.last
        if self.invalid or not self.known or s is None or not self.released_once or not self.resting or self.hold<self.hold_s-1e-8:return 0.
        if s.theta<=.5*D+1e-8 and self.tooth<=1e-8:tier=1.
        elif s.theta<=8*D+1e-8:tier=.75
        elif s.theta<=16*D+1e-8:tier=.5
        elif s.theta<=20*D+1e-8:tier=.25
        else:tier=0.
        return 0. if self.dropped else tier
