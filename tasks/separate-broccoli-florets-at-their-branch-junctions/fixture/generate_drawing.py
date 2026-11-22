"""Generate the dimensioned synthetic anatomy/setup plan from task.py, without simulator or external assets."""
from pathlib import Path
import sys,math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import task as m
out=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1130" viewBox="0 0 1200 1130">','<rect width="1200" height="1130" fill="white"/>','<g font-family="Arial" fill="#17212a"><text x="40" y="35" font-size="24">Broccoli head, paring knife and board: plan view of layouts A and B</text><text x="40" y="60" font-size="15">Dimensions in mm. Synthetic trimmed head (not a scan); catalogue board size; 76 mm blade. Head tipped 3.9 deg onto its crowns.</text>']
scale=1.04
for j,lot in enumerate(m.LOTS):
 ox=65+j*590;oy=120
 def xy(q):return (ox+(q[0]-.10585)*1000*scale,oy+(.26797-q[1])*1000*scale)
 def line(a,b,col,w):
  a,b=xy(a),xy(b);out.append(f'<line x1="{a[0]:.2f}" y1="{a[1]:.2f}" x2="{b[0]:.2f}" y2="{b[1]:.2f}" stroke="{col}" stroke-width="{w}" stroke-linecap="round"/>')
 out.append(f'<text x="{ox}" y="95" font-size="19">Layout {lot.name}</text>')
 x,y=xy((.10585,.26797));out.append(f'<rect x="{x}" y="{y}" width="{368.3*scale}" height="{535.94*scale}" fill="#f2f3ec" stroke="#65747c" stroke-width="2"/>')
 root=np.array(lot.pos('stem'));r=m.rz(lot.yaw_of('stem'))
 line(root+r@np.array([0,.035,0]),root+r@np.array([0,-.065,0]),'#71a548',34*scale)
 for i,n in enumerate(m.NAMES):
  c=np.array(lot.pos(n));rf=m.rz(lot.yaw_of(n));a=root+r@m.ANCHOR
  line(a,c,'#80ad59',13*scale);line(c,c+rf@np.array([.035,0,0]),'#86b553',16*scale)
  bud=c+rf@np.array([.042,0,0]);bx,by=xy(bud)
  out.append(f'<circle cx="{bx:.2f}" cy="{by:.2f}" r="{m.WIDTHS[i]*450*scale:.2f}" fill="#245d29" stroke="#173f20"/>')
  cx,cy=xy(c);out.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="4" fill="#cf4b2c"/><text x="{cx+6:.2f}" y="{cy-8:.2f}" font-size="14">{i+1}</text>')
 kn=np.array(lot.pos('knife'));rk=m.rz(lot.yaw_of('knife'))
 line(kn-rk@np.array([.05,0,0]),kn+rk@np.array([.05,0,0]),'#202528',22*scale)
 line(kn+rk@np.array([.05,0,0]),kn+rk@np.array([.126,0,0]),'#9ca8b0',5)
 out.append(f'<text x="{ox}" y="{oy+580}" font-size="15">Board 368.3 x 535.9 x 16 (thickness estimated)</text>')
 out.append(f'<text x="{ox}" y="{oy+604}" font-size="15">Stem 34 x 100; stalks 16; crowns 34-38 across</text>')
 out.append(f'<text x="{ox}" y="{oy+628}" font-size="15">Red dots: reference planes; side branch 26 mm beyond</text>')
# Side elevation along one floret (schematic, 3 px per mm, drawn untipped).
k=3.0;bx=60;by=1060
def P(x,z):return (bx+x*k,by-z*k)
def box(x0,x1,z0,z1,fill):
    a=P(x0,z1);out.append(f'<rect x="{a[0]:.1f}" y="{a[1]:.1f}" width="{(x1-x0)*k:.1f}" height="{(z1-z0)*k:.1f}" fill="{fill}" stroke="#2f4a20"/>')
out.append('<text x="40" y="845" font-size="19">Side elevation along one floret (schematic, untipped): mm above the board top</text>')
a,b=P(-5,0),P(370,0);out.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#65747c" stroke-width="3"/>')
box(171,191,-1,40,'#f4d7cf')
box(0,100,0.5,34.5,'#71a548');box(100,175,11,24,'#80ad59');box(175,210,9.5,25.5,'#86b553')
c=P(217,22);out.append(f'<ellipse cx="{c[0]:.1f}" cy="{c[1]:.1f}" rx="{19*k:.1f}" ry="{12.5*k:.1f}" fill="#245d29" stroke="#173f20"/>')
f=P(201,17.5);out.append(f'<line x1="{f[0]:.1f}" y1="{f[1]-30:.1f}" x2="{f[0]:.1f}" y2="{f[1]+30:.1f}" stroke="#cf4b2c" stroke-width="2"/>')
t0=P(171,44);out.append(f'<text x="{t0[0]:.1f}" y="{t0[1]:.1f}" font-size="14">shaded: cut band, 10-30 mm below the side branch (red)</text>')
t1=P(0,-8);out.append(f'<text x="{t1[0]:.1f}" y="{t1[1]+6:.1f}" font-size="14">stem 34 dia, axis 17.5 up; shoot 16 dia; crown 34-38 across, top about 35; at rest the head tips 3.9 deg onto its crowns</text>')
hx=320;box(hx-11,hx+11,0,24,'#202528')
t2=P(hx-40,34);out.append(f'<text x="{t2[0]:.1f}" y="{t2[1]:.1f}" font-size="14">knife end view: handle 22 x 24,</text><text x="{t2[0]:.1f}" y="{t2[1]+18:.1f}" font-size="14">edge flush with its underside</text>')
out+=['<text x="40" y="1120" font-size="15">Angles -22,-49,-76,-104,-131,-158 degrees; bough radii 67,72,75,75,72,67 mm. See task.py for exact frames.</text>','</g></svg>']
Path(__file__).with_name('head-layout.svg').write_text('\n'.join(out))
