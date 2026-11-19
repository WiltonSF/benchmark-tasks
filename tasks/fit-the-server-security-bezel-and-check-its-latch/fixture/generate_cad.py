"""Export manufacturing modules and the exact scene assembly in millimetres."""
from pathlib import Path
import sys,json
import numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root));import task
# Each origin is explicitly carried into the assembly; loose exports use local coordinates.
modules={
 'frame':(task.FRAME_SHAPES,(0,0,0)),
 'tabs':(task.TAB_SHAPES,(.005,-.220,0)),
 'guide':(task.GUIDE_SHAPES,(.003,.190,0)),
 'latch':(task.ls,(.003,.204,0)),
 'latch_nose':(task.ls[:1],(.003,.204,0)),
 'stem':(task.ls[1:2],(.003,.172,0)),
 'collar':(task.ls[2:3],(.003,.18092,0)),
 'cross_pin':(task.ls[3:4],(-.001,.18092,0)),
 'flag':(task.ls[4:],(-.0055,.194,0)),
 'right_receiver':(task.RIGHT_RECEIVER_SHAPES,(.370,-.2299,task.zc)),
 'left_receiver':(task.LEFT_RECEIVER_SHAPES[:-1],(.370,.2299,task.zc)),
 'keeper':(task.LEFT_RECEIVER_SHAPES[-1:],(.3605,.2284,task.zc)),
 'rack':(task.fixed[:task.RACK_BASE_COUNT],(.370,0,task.TOP)),
}
def arr(x):return json.dumps([round(float(v)*1000,8) for v in x])
lines=['// Millimetres. Regenerate from the declared scene; see ASSEMBLY.md.', 'part="assembly"; // frame, frame_left, frame_right, tabs, guide, latch, right_receiver, left_receiver, keeper, rack, spring']
machined={'frame','latch_nose','stem','collar'}
for name,(ss,origin) in modules.items():
 lines.append('module '+name+('_blank' if name in machined else '')+'(){union(){')
 for s in ss:
  line='translate('+arr(np.asarray(s.at)-origin)+') '
  if s.kind=='box':line+='cube('+arr(s.size)+',center=true);'
  elif s.kind=='cylinder':
   if s.axis=='x':line+='rotate([0,90,0]) '
   if s.axis=='y':line+='rotate([90,0,0]) '
   line+='cylinder(r='+str(s.size[0]*1000)+',h='+str(s.size[1]*1000)+',center=true,$fn=48);'
  else:line+='polyhedron(points='+json.dumps([[float(v)*1000 for v in p] for p in s.vertices])+',faces='+json.dumps(s.faces)+',convexity=10);'
  lines.append(line)
 lines.append('}}')
lines += [
 '// Manufacturing holes are excluded from the conservative native contact envelopes.',
 'module axis_x_hole(x,y,z,r,h){translate([x,y,z])rotate([0,90,0])cylinder(r=r,h=h,center=true,$fn=48);}',
 'module frame(){frame_blank();}',
 'module frame_split_machined(){difference(){frame_blank();for(z=[-18.225,18.225]){translate([11.55,0,z])cube([2.1,60.2,6.2],center=true);for(y=[-24,-8,8,24]){axis_x_hole(0,y,z,1.6,26);translate([-12.51,y,z])rotate([0,90,0])cylinder(r1=3.1,r2=1.6,h=1.5,$fn=48);}}}}',
 'module splice_plates(){for(z=[-18.225,18.225])difference(){translate([11.5,0,z])cube([2,60,6],center=true);for(y=[-24,-8,8,24])axis_x_hole(11.5,y,z,1.25,3);}}',
 'module frame_left(){intersection(){frame_split_machined();translate([-100,0,-100])cube([200,300,200]);}}',
 'module frame_right(){intersection(){frame_split_machined();translate([-100,-300,-100])cube([200,300,200]);}}',
 'module latch_nose(){difference(){latch_nose_blank();translate([0,-20.01,0])rotate([-90,0,0])cylinder(r=1.25,h=6.01,$fn=48);axis_x_hole(-3,-10,0,1.55,8.02);}}',
 'module stem(){difference(){stem_blank();axis_x_hole(-4,8.92,0,1,15);}}',
 'module collar(){difference(){collar_blank();rotate([90,0,0])cylinder(r=1.55,h=3,center=true,$fn=48);axis_x_hole(-4,0,0,1,15);}}',
 '// Spring drawing:8 active turns plus closed end coils,0.5mm wire,6mm OD,22mm free.',
 'module spring(){for(i=[0:319])hull(){for(j=[i,i+1])translate([2.75*cos(j*9),2.75*sin(j*9),0.25+21.5*j/320])sphere(r=.25,$fn=8);}for(z=[.25,21.75])translate([0,0,z])rotate_extrude($fn=64)translate([2.75,0])circle(r=.25,$fn=12);}',
 'module assembly(){',
 'translate([370,-2,66.675]){frame();translate([5,-220,0])tabs();translate([3,190,0])guide();translate([3,204,0])latch();}',
 'translate([370,-229.9,66.675])right_receiver();translate([370,229.9,66.675])left_receiver();translate([360.5,228.4,66.675])keeper();translate([370,0,0])rack();}',
 ]
for name in [*modules,'frame_left','frame_right','splice_plates','spring','assembly']:lines.append('if(part=="'+name+'") '+name+'();')
(root/'fixture'/'parts.scad').write_text('\n'.join(lines)+'\n')
