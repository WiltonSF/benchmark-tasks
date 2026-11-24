"""Dimensioned representative half-head plan, generated from the same constants as the task."""
from pathlib import Path
import sys,math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import task as m
out=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1050" viewBox="0 0 1200 1050">',
'<rect width="1200" height="1050" fill="white"/><g font-family="Arial" fill="#17212a">',
'<text x="40" y="35" font-size="22">Natural half head, paring knife and board: layouts A and B</text>',
'<text x="40" y="62" font-size="16">Dimensions in mm; guide only. Green: produce. Red dots: severing reference planes.</text>']
scale=1.04
for j,lot in enumerate(m.LOTS):
 ox=65+j*590;oy=125
 def xy(q):return (ox+(q[0]-.10585)*1000*scale,oy+(.26797-q[1])*1000*scale)
 def line(a,b,col,w):
  a,b=xy(a),xy(b);out.append(f'<line x1="{a[0]:.2f}" y1="{a[1]:.2f}" x2="{b[0]:.2f}" y2="{b[1]:.2f}" stroke="{col}" stroke-width="{w}" stroke-linecap="round"/>')
 out.append(f'<text x="{ox}" y="101" font-size="19">Layout {lot.name}: holding arm {"left" if j==0 else "right"}</text>')
 x,y=xy((.10585,.26797));out.append(f'<rect x="{x}" y="{y}" width="{368.3*scale}" height="{535.94*scale}" fill="#f2f3ec" stroke="#65747c" stroke-width="2"/>')
 root=np.array(lot.pos('stem'));r=m.rz(lot.yaw_of('stem'))
 for sh in m.STEM.shapes[:1]:
  vertices=[root+r@np.array([x,y,0.]) for x,y in [(-m.STEM_TIP_R,m.STEM_Y[0]),(m.STEM_TIP_R,m.STEM_Y[0]),(m.STEM_TIP_R,m.STEM_TIP_END),(m.STEM_R,m.STEM_TIP_END),(m.STEM_R,m.STEM_Y[1]),(-m.STEM_R,m.STEM_Y[1]),(-m.STEM_R,m.STEM_TIP_END),(-m.STEM_TIP_R,m.STEM_TIP_END)]]
  points=' '.join(f'{a:.2f},{b:.2f}' for a,b in map(xy,vertices));out.append(f'<polygon points="{points}" fill="#85ab65" stroke="#466d37"/>')
 for i,n in enumerate(m.NAMES):
  c=np.array(lot.pos(n));rf=m.rz(lot.yaw_of(n));origin=root+r@m.ORIGINS[i]
  line(origin,c+rf@np.array([.035,0,0]),'#86b553',7*scale)
  bud=c+rf@np.array([.042,0,0]);bx,by=xy(bud)
  out.append(f'<circle cx="{bx:.2f}" cy="{by:.2f}" r="{m.CROWN_WIDTHS[i]*500*scale:.2f}" fill="#245d29" stroke="#173f20"/>')
  cx,cy=xy(c);out.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="3" fill="#cf4b2c"/><text x="{cx+5:.2f}" y="{cy-7:.2f}" font-size="13">{i+1}</text>')
 kn=np.array(lot.pos('knife'));rk=m.rz(lot.yaw_of('knife'))
 line(kn-rk@np.array([.05,0,0]),kn+rk@np.array([.05,0,0]),'#202528',22*scale)
 line(kn+rk@np.array([.05,0,0]),kn+rk@np.array([.126,0,0]),'#9ca8b0',5)
 out.append(f'<text x="{ox}" y="710" font-size="15">Board 368.3 x 535.9; knife blade 76</text>')
 out.append(f'<text x="{ox}" y="735" font-size="15">Stalks 27,32,33,33,32,27 long; thickness 7</text>')
 out.append(f'<text x="{ox}" y="760" font-size="15">Crowns 38,40,44,44,40,38; total mass 125 g</text>')
out += ['<text x="40" y="815" font-size="19">Measured sample and geometric estimates</text>',
'<text x="40" y="845" font-size="16">Two heads, four halves: leaves off, halve through main stem, lay cut-face down. Timed 47 / 53 s per head.</text>',
'<text x="40" y="872" font-size="16">Free stalks 23-39; thickness 5-10 (median 7); largest-six crowns 29-48. Crown acceptance remains 30-50.</text>',
'<text x="40" y="899" font-size="16">Main stem: estimated original diameter 40, distal 25 long narrows to 32; length 145. Branch axes 13 above flat face.</text>',
'<text x="40" y="926" font-size="16">Branch origins, angles, heights and individual mass allocation are estimates; no forced fan or printed holder.</text>',
'<text x="40" y="953" font-size="16">Red reference to first side branch: 26. Cut-face acceptance: every point 10-30 below its first side branch.</text>',
'<text x="40" y="980" font-size="16">This is a representative symmetric model within measured ranges, not a measured natural specimen outline.</text>',
'<text x="40" y="1015" font-size="15">Guide only, not a full-size template. Transfer setup coordinates by ruler. See task.py for exact frames.</text>', '</g></svg>']
Path(__file__).with_name('head-layout.svg').write_text('\n'.join(out))
