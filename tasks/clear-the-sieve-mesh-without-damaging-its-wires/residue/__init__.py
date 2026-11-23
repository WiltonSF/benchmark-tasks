"""Declared local wire-wedge and granular contact model, SI units.
Retained grains move only under local bristle load or inertial load sufficient
against estimated native-wire friction. No coverage clock or efficacy radius.
Released grain centres integrate gravity and sphere/cylinder/plane contacts.
"""
import numpy as np
PITCH=.00181;WIRE=.000315;OPEN=.00118;RADIUS=.0358
SITES=np.array([(i*PITCH,j*PITCH,.00033) for i in (-10,-5,0,5,10) for j in (-9,-3,3,9)])
RADII=np.array([.00063+.00001*(i%6) for i in range(20)])
SITES[:,2]=np.sqrt((RADII+WIRE)**2-(PITCH/2)**2)-.000002
WIRE_POINTS=np.array([(i*PITCH,(j+.5)*PITCH,0.) for i in range(-20,21) for j in range(-20,20) if (i*PITCH)**2+((j+.5)*PITCH)**2<RADIUS**2]+[((j+.5)*PITCH,i*PITCH,0.) for i in range(-20,21) for j in range(-20,20) if (i*PITCH)**2+((j+.5)*PITCH)**2<RADIUS**2])
MASS=3.5e-6
def tip_flex(load):
    # Estimated local bending/splay of26.1mm soft natural bristles, at most7.7% of length.
    return min(.002,max(0.,float(load))/500)

class Residue:
    def __init__(self):
        self.retained=np.ones(20,dtype=bool);self.offset=np.zeros((20,3));self.pos=np.zeros((20,3));self.vel=np.zeros((20,3));self.bad=False;self.damaged=False
        self.previous_brush=None;self.previous_mesh=None;self.previous_velocity=None;self.previous_pan=None;self.previous_panR=None;self.release_count=0;self.peak=0.
        self.widening=np.zeros(20);self.last_brush_load=0.;self.contact_sites=[]
    def initialize(self,mesh,R):
        self.pos=SITES@R.T+mesh
    def advance(self,dt,mesh,R,brush_tip,brush_axis,load,pan,panR,brush_back=None):
        if not np.isfinite(np.r_[dt,mesh,R.ravel(),brush_tip,brush_axis,load,pan,panR.ravel()]).all() or not 0<dt<=.1:self.bad=True;return
        self.last_brush_load=load;self.peak=max(self.peak,load);self.contact_sites=[]
        axis=brush_axis/np.linalg.norm(brush_axis)
        # The far plane of the physical bristle bundle; disk extends .0261m back along its axis.
        delta=brush_tip-self.previous_brush if self.previous_brush is not None else np.zeros(3)
        move=mesh-self.previous_mesh if self.previous_mesh is not None else np.zeros(3)
        velocity=move/dt
        accel=(velocity-self.previous_velocity)/dt if self.previous_velocity is not None else np.zeros(3)
        local_world=SITES@R.T+mesh
        # Whole-trial overload rule applies to every native wire, including after all grains leave.
        wire_delta=WIRE_POINTS@R.T+mesh-brush_tip
        wire_depth=wire_delta@axis
        wire_radial=np.linalg.norm(wire_delta-wire_depth[:,None]*axis,axis=1)
        wire_contact=(wire_depth>=-.0261-WIRE)&(wire_depth<=WIRE+tip_flex(load))&(wire_radial<=.015875+WIRE+tip_flex(load))
        if load>3.+1e-8 and wire_contact.any():
            self.damaged=True
            self.widening[:]=np.maximum(self.widening,.00010)
        for i in np.where(self.retained)[0]:
            point=local_world[i]+R@self.offset[i]
            v=point-brush_tip;depth=float(v@axis);radial=np.linalg.norm(v-depth*axis)
            # Native finite bristle cylinder must physically contain/contact the grain, and be spring-loaded.
            flex=tip_flex(load)
            contact=bool(-.0261-RADII[i]<=depth<=RADII[i]+flex and radial<=.015875+RADII[i]+flex and load>=.1)
            tangent=R.T@(delta-move);tangent[2]=0.
            force=load*(2*RADII[i])**2/(np.pi*.015875**2)*max(0.,float(axis@R[:,2])) if contact else 0.
            inertia=R.T@(-MASS*accel);inertia[2]=0.
            resist=.00045+.00006*(i%5) #N, estimated elastic-wire friction, not calibrated.
            tangent_length=np.linalg.norm(tangent)
            applied=inertia+(force*tangent/tangent_length if tangent_length>1e-9 else np.zeros(3))
            driving=np.linalg.norm(applied)
            if contact:self.contact_sites.append(int(i))
            if driving>resist:
                # Tangential bristle travel plus inertial displacement under F=ma.
                brush_travel=tangent_length if force>0 else 0.
                inertial_travel=max(0.,np.linalg.norm(inertia)-resist)/MASS*dt*dt/2
                travel=min(brush_travel*(1-resist/driving)+inertial_travel,.003)
                self.offset[i]+=applied/driving*travel
            self.pos[i]=local_world[i]+R@self.offset[i]
            if np.linalg.norm(self.offset[i][:2])>=PITCH/2+WIRE:
                self.retained[i]=False;self.release_count+=1
                # No collection credit on release. A small actual sweep velocity and gravity determine where it lands.
                self.vel[i]=velocity+np.clip((delta-move)/dt,-.03,.03)+R[:,2]*.015
        steps=max(1,int(np.ceil(dt/.001)));h=dt/steps
        oldpan=pan.copy() if self.previous_pan is None else self.previous_pan
        oldR=panR.copy() if self.previous_panR is None else self.previous_panR
        pan_velocity=(pan-oldpan)/dt;dr=panR@oldR.T
        omega=np.array((dr[2,1]-dr[1,2],dr[0,2]-dr[2,0],dr[1,0]-dr[0,1]))/(2*dt)
        for substep in range(steps):
            alpha=(substep+1)/steps;ph=oldpan+(pan-oldpan)*alpha
            rawR=oldR*(1-alpha)+panR*alpha
            a=rawR[:,0]/np.linalg.norm(rawR[:,0]);b=rawR[:,1]-a*(rawR[:,1]@a);b/=np.linalg.norm(b)
            pr=np.column_stack((a,b,np.cross(a,b)))
            for i in np.where(~self.retained)[0]:
                r=RADII[i];old=self.pos[i].copy();self.vel[i,2]-=9.81*h;self.pos[i]+=self.vel[i]*h
                # Released grains still collide with the moving finite bristle bundle.
                # The same measured head load gates tool transfer; freed grains can be brushed again.
                along=float((self.pos[i]-brush_tip)@axis)
                radial_vector=self.pos[i]-brush_tip-along*axis
                rho_brush=float(np.linalg.norm(radial_vector))
                flex=tip_flex(load)
                if load>.1 and -.0261-r<along<r+flex and rho_brush<.015875+r+flex:
                    front=r+flex-along;back=along+.0261+r;side=.015875+r+flex-rho_brush
                    if front<=min(back,side):normal=axis;penetration=front
                    elif back<=side:normal=-axis;penetration=back
                    else:normal=radial_vector/max(rho_brush,1e-12);penetration=side
                    self.pos[i]+=normal*penetration
                    tool_velocity=np.clip(delta/dt,-.15,.15)
                    relative=float((self.vel[i]-tool_velocity)@normal)
                    if relative<0:self.vel[i]-=1.05*relative*normal
                    self.vel[i]+=.15*(tool_velocity-self.vel[i])
                # Native spherical-grain contacts with individual crossed wire cylinders in the finite circular mesh.
                q=(self.pos[i]-mesh)@R
                if np.linalg.norm(q[:2])<RADIUS+r and abs(q[2])<r+WIRE+.0003:
                    for a in (0,1):
                        wire=round(q[a]/PITCH-.5)*PITCH+PITCH*.5
                        d=np.array([q[a]-wire,q[2]])
                        n=np.linalg.norm(d)
                        if 1e-12<n<r+WIRE:
                            normal=np.zeros(3);normal[a]=d[0]/n;normal[2]=d[1]/n
                            self.pos[i]+=R@normal*(r+WIRE-n);nw=R@normal;vn=self.vel[i]@nw
                            if vn<0:self.vel[i]-=1.05*vn*nw;self.vel[i]*=.9
                # Full native sieve frame wall, not an open suspended wire patch.
                q=(self.pos[i]-mesh)@R;rho=np.linalg.norm(q[:2]);vlocal=self.vel[i]@R
                if -.007225<=q[2]<=.037225 and .0368-r<rho<.038+r:
                    normal=q[:2]/max(rho,1e-12);q[:2]=normal*(.0368-r);vn=vlocal[:2]@normal
                    if vn>0:vlocal[:2]-=1.1*vn*normal
                    self.pos[i]=mesh+R@q;self.vel[i]=R@vlocal
                # Moving original pan: co-moving contacts and Coulomb tangential impulse.
                q=(self.pos[i]-ph)@pr;surface=pan_velocity+np.cross(omega,self.pos[i]-ph)
                vlocal=(self.vel[i]-surface)@pr;rho=np.linalg.norm(q[:2]);inside=rho<=.0368-r
                if inside and q[2]<.001+r and (old-ph)@pr[:,2]>=.001-r-abs(pan_velocity@pr[:,2])*h:
                    q[2]=.001+r;impulse=max(0.,-vlocal[2])*1.08;vlocal[2]=max(0.,-vlocal[2]*.08)
                    speed=np.linalg.norm(vlocal[:2])
                    if speed>0:vlocal[:2]*=max(0.,1-.45*impulse/speed)
                    self.pos[i]=ph+pr@q;self.vel[i]=surface+pr@vlocal
                elif .001<=q[2]<=.04445 and .0368-r<rho<.038+r:
                    n=q[:2]/rho;q[:2]=n*(.0368-r);vn=vlocal[:2]@n
                    if vn>0:vlocal[:2]-=1.1*vn*n
                    self.pos[i]=ph+pr@q;self.vel[i]=surface+pr@vlocal
                # Counter collision. A grain on the counter remains uncollected, never teleports into the pan.
                if self.pos[i,2]<.7475+r:
                    self.pos[i,2]=.7475+r;self.vel[i,2]=max(0.,-self.vel[i,2]*.08);self.vel[i,:2]*=.9
            # Native grain/grain sphere contacts; no overlap-based hidden particle counting.
            delta_pair=self.pos[:,None,:]-self.pos[None,:,:]
            distance=np.linalg.norm(delta_pair,axis=2)
            overlap=RADII[:,None]+RADII[None,:]-distance
            pairs=np.argwhere(np.triu((overlap>1e-9)&(~self.retained[:,None]|~self.retained[None,:]),1))
            for i,j in pairs:
                normal=delta_pair[i,j]/distance[i,j] if distance[i,j]>1e-12 else np.array((1.,0,0))
                a=0. if self.retained[i] else 1.;b=0. if self.retained[j] else 1.;total=a+b
                self.pos[i]+=normal*overlap[i,j]*a/total;self.pos[j]-=normal*overlap[i,j]*b/total
                closing=float((self.vel[i]-self.vel[j])@normal)
                if closing<0:
                    impulse=-1.1*closing/total
                    self.vel[i]+=impulse*normal*a;self.vel[j]-=impulse*normal*b
        self.previous_brush=brush_tip.copy();self.previous_mesh=mesh.copy();self.previous_velocity=velocity;self.previous_pan=pan.copy();self.previous_panR=panR.copy()
    def classify(self,mesh,R,pan,panR):
        p=(self.pos-pan)@panR
        rim_z=pan[2]+panR[2,2]*.04445-np.linalg.norm(panR[2,:2])*.038
        collected=(self.pos[:,2]+RADII<=rim_z+1e-8)&(~self.retained)&(np.linalg.norm(p[:,:2],axis=1)+RADII<=.0368+1e-8)&(p[:,2]>=.001-1e-8)&(p[:,2]+RADII<=.04445+1e-8)&(np.linalg.norm(self.vel,axis=1)<.01)
        q=(self.pos-mesh)@R
        blocked=(~self.retained)&(np.linalg.norm(q[:,:2],axis=1)<RADIUS)&(np.abs(q[:,2])<RADII+WIRE)
        escaped=(~self.retained)&(~collected)&(~blocked)
        return collected,blocked,escaped
    def counts(self,mesh,R,pan,panR):
        return tuple(int(a.sum()) for a in self.classify(mesh,R,pan,panR))
