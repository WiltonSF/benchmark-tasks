"""Generate dimensioned drawings and editable full-size geometry from the task declaration."""
import pathlib,sys,json,csv,html
import numpy as np
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
import task as t

def run():
 rows=[];scad=['// Millimetres. Original geometry; edit PART for printable link selection.','$fn=72;','PART="assembly";']
 views=[]
 for part in t.OBJECTS:
  for link,origin,shapes in part.links:
   module=part.name+'_'+link;body=[]
   for k,s in enumerate(shapes):
    pos=(np.array(s.at)-origin)*1000;world=(np.array(part.pos)+np.array(s.at)-[0,0,t.TOP])*1000
    size=np.array(s.size)*1000
    rows.append(dict(part=part.name,link=link,shape=k,kind=s.kind,axis=s.axis,
     x_mm=round(world[0],4),y_mm=round(world[1],4),z_mm=round(world[2],4),
     local_x_mm=round(pos[0],4),local_y_mm=round(pos[1],4),local_z_mm=round(pos[2],4),size_mm=json.dumps(size.tolist())))
    if s.kind=='box':geom='cube('+json.dumps(size.tolist())+',center=true);'
    else:geom=('rotate([90,0,0]) ' if s.axis=='y' else 'rotate([0,90,0]) ' if s.axis=='x' else '')+f'cylinder(r={size[0]:.5f},h={size[1]:.5f},center=true);'
    body.append('translate('+json.dumps(pos.tolist())+') '+geom)
   # The editable union preserves each functional surface. Add fabrication bores as listed in README.
   scad+=['module '+module+'(){union(){',*body,'}}',f'if(PART=="{module}") {module}();']
   shift=(np.array(part.pos)+origin-[0,0,t.TOP])*1000
   scad.append('if(PART=="assembly") translate('+json.dumps(shift.tolist())+') '+module+'();')
   views.append((module,shapes,np.array(part.pos)))
 (HERE/'geometry.scad').write_text('\n'.join(scad)+'\n')
 with (HERE/'geometry.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 # Front and side orthographic views, with numeric coordinates supplied for every primitive.
 for axes,label in [((0,2),'side'),((0,1),'plan')]:
  out=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="850" viewBox="120 -380 780 650">',
   '<rect x="120" y="-380" width="780" height="650" fill="white"/>',
   '<g font-family="monospace" font-size="9" fill="#222">',f'<text x="140" y="-350">RM2125 RTS procedure fixture: {label}; all dimensions mm; scene rev 1</text>']
  for part in t.OBJECTS:
   for link,origin,shapes in part.links:
    for s in shapes:
     p=(np.array(part.pos)+s.at-[0,0,t.TOP])*1000;half=s.half*1000
     a,b=axes;x=p[a]-half[a];y=-(p[b]+half[b]) if b==2 else p[b]-half[b]
     color='#' + ''.join(f'{int(c*255):02x}' for c in s.color)
     out.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{2*half[a]:.3f}" height="{2*half[b]:.3f}" fill="{color}" fill-opacity=".18" stroke="#555" stroke-width=".5"/>')
   for j in part.joints:
    p=(np.array(part.pos)+j.origin-[0,0,t.TOP])*1000
    a,b=axes;y=-p[b] if b==2 else p[b]
    out.append(f'<circle cx="{p[a]}" cy="{y}" r="2" fill="red"/><text x="{p[a]+3}" y="{y-4}">{html.escape(part.name+"."+j.name)}</text>')
  if b==2:
   out+=['<path d="M 290,-175.25 H 180 M 290,-190 H 180" stroke="#0a5" fill="none"/>',
    '<text x="145" y="-178">175.25 start</text><text x="145" y="-193">190.00 top</text>']
  out+=['<text x="140" y="230">Shell: 438 wide x 472 deep x 265 high. Wheel diameter 200; knob radius 85; crank radius 29.5.</text>',
   '<text x="140" y="245">Exact sizes/centres: geometry.csv. Pivots: assembly.json. Captive spring-stack detail: spring-mount.svg.</text>','</g></svg>']
  (HERE/(label+'.svg')).write_text('\n'.join(out))
 assembly=[]
 for part in t.OBJECTS:
  for j in part.joints:
   assembly.append(dict(part=part.name,joint=j.name,type=type(j).__name__,parent=j.parent,
    origin_mm=((np.array(part.pos)+j.origin-[0,0,t.TOP])*1000).tolist(),axis=j.axis,lower=j.lower,upper=j.upper,mass_kg=j.mass,spring=j.spring,target=j.target,max_effort=j.max_effort))
 (HERE/'assembly.json').write_text(json.dumps(dict(version=1,units='mm for origins; SI for joint quantities',cassette_start_height_mm=175.25,cassette_upper_height_mm=190.,joints=assembly),indent=2)+'\n')
 print('wrote geometry.scad, geometry.csv, assembly.json, side.svg, plan.svg')
if __name__=='__main__':run()
