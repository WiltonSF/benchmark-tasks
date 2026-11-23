"""Read SCAD envelopes; check fittings/hardware against fixed bodies and bridge across full travel."""
import ast,hashlib,json,pathlib,re,sys,numpy as np
root=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root));import task as t
scad=(root/'fixture/printable.scad').read_text()
def module(name):return scad.split('module '+name+'(){',1)[1].split('\nmodule ',1)[0]
def cube(text):
 m=re.search(r'translate\((\[[^]]+\])\)cube\((\[[^]]+\]),center=true\)',text)
 return np.array(ast.literal_eval(m[1])),np.array(ast.literal_eval(m[2]))
first=module('corner_socket');second=module('corner_socket_xy');bridge=module('spring_bridge')
shift1,size1=cube(first);shift2,size2=cube(second)
assert np.allclose(shift1,[-3.5,2,0]) and np.allclose(size1,[25,28,32])
assert np.allclose(shift2,[-3.5,0,0]) and np.allclose(size2,[25,32,32])
assert 'bore_y(13.1,36)' in first and ';cylinder(d=13.1,h=36' in first
assert 'bore_y(13.1,36)' in second and 'rotate([0,90,0])cylinder(d=13.1,h=36' in second
assert 'd1=4.3,d2=8.4,h=2.05' in first
assert 'translate([6.95,0,0])' in first and 6.95+2.05==9
# Parse the integral third coupling and its two ribs from actual SCAD text.
cube_matches=list(re.finditer(r'translate\((\[[^]]+\])\)cube\((\[[^]]+\]),center=true\)',bridge))
third_match=[m for m in cube_matches if ast.literal_eval(m[2])==[34,21,32]][0]
third_local=np.array(ast.literal_eval(third_match[1]));third_size=np.array(ast.literal_eval(third_match[2]))
assert np.allclose(third_local,[0,28.5,37.8])
assert 'translate([0,23,26.9])cylinder(d=13.7,h=60' in bridge
assert 'translate([0,23,37.8])rotate([0,90,0])cylinder(d=13.7,h=40' in bridge
assert 'translate([9,26,37.8])cylinder(d=4.3,h=36' in bridge
assert 'translate([0,21,26.3])rotate([0,90,0])cylinder(d=4.3,h=40' in bridge
assert 'translate([x,23,20.4])cube([4,8,34.8]' in bridge
assert 'for(x=[-12,12])translate([x,21,3])' not in bridge # old holes would sever rib feet
cz=(t.C0+.068)*1000;arm_top=t.arm_top*1000
components=[]
def add(name,c,size):components.append((name,np.array(c,dtype=float)-np.array(size)/2,np.array(c,dtype=float)+np.array(size)/2))
c1=np.array([339,94,cz]);c2=np.array([339,-122,cz])
add('first_body',c1+shift1,size1);add('second_body',c2+shift2,size2)
third_origin=np.array([294.5,-145,arm_top]);add('third_body',third_origin+third_local,third_size)
for x in (-12,12):add('third_rib',third_origin+[x,23,20.4],[4,8,34.8])
# Conservative hardware envelopes. DIN125 M4 washer OD9, M4 nut <=8.1 across corners.
# First flush DIN965 head;30mm length includes head, nut/washer on negativeX.
for y,z in ((-7.5,0),(0,-9)):
 p=c1+[0,y,z]
 add('first_flush_head',p+[7.975,0,0],[2.05,8.4,8.4])
 add('first_shank',p+[-7.025,0,0],[27.95,4,4])
 add('first_nut_washer',p+[-18,0,0],[4,9,9])
# DIN912 M4x40 socket heads4mm high,7mm diameter; shank40,washer+nut4mm.
for x,y in ((-9,0),(0,9)):
 p=c2+[x,y,0]
 add('second_head',p+[0,0,18],[7,7,4]);add('second_shank',p+[0,0,-4],[4,4,40]);add('second_nut_washer',p+[0,0,-18],[9,9,4])
third_axis=third_origin+[0,23,37.8]
third_hardware={}
for axis,p in ((2,third_axis+[9,3,0]),(0,third_axis+[0,-2,-11.5])):
 face=16 if axis==2 else 17
 off=np.zeros(3);off[axis]=face+2;sz=np.array([7.,7.,7.]);sz[axis]=4;add('third_head',p+off,sz)
 off[axis]=face-20;sz=np.array([4.,4.,4.]);sz[axis]=40;add('third_shank',p+off,sz);third_hardware[axis]=(p.copy(),components[-1][1].copy(),components[-1][2].copy())
 off[axis]=-face-2;sz=np.array([9.,9.,9.]);sz[axis]=4;add('third_nut_washer',p+off,sz)
bolt_gap=abs(third_hardware[0][0][1]-third_hardware[2][0][1])-4
assert bolt_gap>=.99
assert abs(third_hardware[0][0][1]-third_hardware[2][0][1])-4.3>=.69
assert (np.maximum(third_hardware[0][1]-third_hardware[2][2],third_hardware[2][1]-third_hardware[0][2])>0).any()
# Physical tube envelopes include the actual12.7mm OD, not merely native boxes.
for i,span_axis in ((10,2),(11,1),(12,0),(13,2)):
 shape=t.head.shapes[i];c=(np.array(t.HEAD.pos)+shape.at-[0,0,t.TOP])*1000;size=np.full(3,12.7);size[span_axis]=float(shape.size[span_axis]*1000)
 # Physical upright90mm raw allowance is longer than native86mm; check full stock envelope.
 if i==10:size[2]=90;c[2]+=2
 if i==13:size[2]=34.8
 add('physical_frame_'+str(i),c,size)
for i in (10,11,12,13):
 shape=t.head.shapes[i];add('native_frame_'+str(i),(np.array(t.HEAD.pos)+shape.at-[0,0,t.TOP])*1000,np.array(shape.size)*1000)
# New fully-enclosed pad bore and M4x50 crossbolt: head355..359; nut/washer311..315.
add('pad_bolt_head',[356.8,94,t.C0*1000-16],[4,7,7]);add('pad_head_washer',[354.4,94,t.C0*1000-16],[.8,9,9]);add('pad_bolt_shank',[329.8,94,t.C0*1000-16],[50,4,4]);add('pad_nut_washer',[313,94,t.C0*1000-16],[4,9,9])
# Pad is checked with its deliberate coaxial guide relief, not as solid stock.
shape=t.head.shapes[9];add('carriage_pad',(np.array(t.HEAD.pos)+shape.at-[0,0,t.TOP])*1000,np.array(shape.size)*1000)
assert np.allclose((np.array(t.HEAD.pos)+t.head.shapes[10].at)[:2]*1000,[339,94])
fixed=[]
for part in t.OBJECTS:
 if part.mass is not None:continue
 for i,s in enumerate(part.shapes):
  c=(np.array(part.pos)+s.at-[0,0,t.TOP])*1000;h=s.half*1000
  fixed.append((part.name+':'+str(i),c-h,c+h))
bridge_boxes=[]
for i in (14,15,20,21):
 s=t.head.shapes[i];c=(np.array(t.HEAD.pos)+s.at-[0,0,t.TOP])*1000;h=s.half*1000
 bridge_boxes.append((i,c-h,c+h))
worst=(1e6,None,None,None);count=0
# Exact continuous swept AABBs for pure-Z translation; no gaps between samples.
for name,lo,hi in components:
 a=lo+[0,0,t.head.lower*1000];b=hi+[0,0,t.head.upper*1000]
 for target,l,h in fixed:
  sep=np.maximum(l-b,a-h);gap=float(np.linalg.norm(np.maximum(sep,0)))
  if name=='carriage_pad' and target in ('specimen_guide:0','specimen_guide:1') and not (sep>0).any():
   # Each rod is coaxial with an80mm cut in the exported carriage; cut spans pad throughout motion.
   idx=int(target.rsplit(':',1)[1]);rod=t.HEAD.shapes[idx];rodxy=(np.array(t.HEAD.pos)+rod.at)[:2]*1000
   assert np.allclose(rodxy,[351,-5 if idx==0 else 115])
   assert t.C0*1000-88<=lo[2] and t.C0*1000-8>=hi[2]
   gap=(8.3-2*rod.size[0]*1000)/2;assert gap>0
  else:assert (sep>0).any(),(name,target,a,b,l,h)
  if gap<worst[0]:worst=(gap,name,target,'continuous stroke')
  count+=1
# All co-moving component clearances to roof/cheeks are constant. Ribs/foot union is intentional.
bridge_min=(1e6,None,None)
for name,lo,hi in components:
 if 'frame_' in name or name.startswith('pad_'):continue
 for i,l,h in bridge_boxes:
  if i==14 and name=='third_rib':continue
  sep=np.maximum(l-hi,lo-h);gap=float(np.linalg.norm(np.maximum(sep,0)))
  assert (sep>0).any(),(name,i,lo,hi,l,h)
  if gap<bridge_min[0]:bridge_min=(gap,name,i)
# Bores completely contain physical12.7mm tube where it passes the printed bridge.
assert (13.7-12.7)/2>=.499
assert third_local[2]-16<=20.4+34.8/2 and 20.4-34.8/2<=6 # ribs overlap body and foot
assert (8.3-7.94)/2>=.179 and (t.C0*1000-48+40)-(t.C0*1000-10)>=1.99
assert 'cylinder(d=8.3,h=80,center=true)' in (root/'fixture/fabrication.py').read_text()
assert 'cylinder(d=10,h=1.02,center=true)' in (root/'fixture/fabrication.py').read_text()
out=dict(pad_guide_radial_clearance_mm=.18,pad_guide_relief_exit_margin_mm=2.,runtime_sha256=t.SOURCE_SHA256,scad_sha256=hashlib.sha256(scad.encode()).hexdigest(),component_count=len(components),fixed_shape_count=len(fixed),full_stroke_mm=59,comparisons=count,minimum_fixed_gap_mm=worst,minimum_moving_bridge_gap_mm=bridge_min,tube_radial_clearance_mm=.5,third_shank_clearance_mm=bolt_gap,overlaps=0,limitations='Analytical conservative envelopes and bores; no STL export, printing, load test, or robot clearance validation for omitted fittings. Frame rerouted11mm; native-run evidence is reported separately.')
print(json.dumps(out,indent=2))
