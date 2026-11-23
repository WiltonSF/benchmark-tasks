"""Analytical swept support checks and optional recorded robot hull SAT screening.
No native simulation or hardware claim. Run --recorded with gzipped native telemetry.
"""
import argparse,contextlib,hashlib,io,itertools,json,pathlib,sys
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));import task as t
from scipy.spatial import ConvexHull
D=json.loads((ROOT/'fixture/fixed-mount-envelopes.json').read_text())
boxes=[(b['name'],np.array(b['center_mm']),np.array(b['size_mm'])) for b in D['boxes']]+[(b['name'],np.array(b['center_mm']),np.array(b['size_mm'])) for b in json.loads((ROOT/'fixture/load-path-fixed.json').read_text())['boxes']]
# SAT includes face normals and every hull-edge x box-edge axis; no vertex-only shortcut.
def separation(vertices,planes,c,size):
 hull=ConvexHull(vertices);edges={tuple(sorted((int(a),int(b)))) for face in hull.simplices for a,b in itertools.combinations(face,2)}
 e=np.array([vertices[b]-vertices[a] for a,b in edges]);axes=[np.eye(3),planes]
 for axis in np.eye(3):axes.append(np.cross(e,axis))
 axes=np.concatenate(axes);norm=np.linalg.norm(axes,axis=1);axes=axes[norm>1e-9]/norm[norm>1e-9,None]
 p=vertices@axes.T;q=c@axes.T;r=np.abs(axes)@(size/2)
 return float(np.maximum(q-r-p.max(0),p.min(0)-q-r).max())
def corners(c,size):return c+np.array(list(itertools.product((-1,1),repeat=3)))*size/2
worst=(1e6,None,None);comparisons=0
# Continuous Z sweep of all carriage surfaces (including frame) across59 mm.
for n,c,s in boxes:
 for i,shape in enumerate(t.head.shapes):
  p=(np.array(t.HEAD.pos)+shape.at-[0,0,t.TOP])*1000
  lo=p-shape.half*1000+[0,0,t.head.lower*1000];hi=p+shape.half*1000+[0,0,t.head.upper*1000]
  sep=np.maximum(c-s/2-hi,lo-c-s/2);gap=float(np.linalg.norm(np.maximum(sep,0)))
  assert (sep>0).any(),('carriage',n,i,lo,hi)
  if gap<worst[0]:worst=(gap,n,'carriage:'+str(i))
  comparisons+=1
# Existing physical tube, fitting and bolt envelopes against added fixed supports.
import runpy
with contextlib.redirect_stdout(io.StringIO()):existing=runpy.run_path(str(ROOT/'fixture/clearance_check.py'))
for name,lo,hi in existing['components']:
 a=lo+[0,0,t.head.lower*1000];b=hi+[0,0,t.head.upper*1000]
 for n,c,s in boxes:
  sep=np.maximum(c-s/2-b,a-c-s/2);gap=float(np.linalg.norm(np.maximum(sep,0)))
  assert (sep>0).any(),('fitting',name,n)
  if gap<worst[0]:worst=(gap,n,name)
  comparisons+=1
# Clamp angles0..15.5 deg and full continuous Z stroke, using conservative swept AABBs.
for deg in np.arange(0,15.501,.25):
 a=np.radians(deg);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
 for i,shape in enumerate(t.clamp.shapes):
  pivot=(np.array(t.HEAD.pos)+t.clamp.origin-[0,0,t.TOP])*1000
  p=pivot+R@((np.array(shape.at)-t.clamp.origin)*1000)
  verts=corners(np.zeros(3),shape.half*2000)@R.T+p
  lo=verts.min(0)+[0,0,t.head.lower*1000];hi=verts.max(0)+[0,0,t.head.upper*1000]
  for n,c,s in boxes:
   sep=np.maximum(c-s/2-hi,lo-c-s/2);gap=float(np.linalg.norm(np.maximum(sep,0)))
   assert gap>.6,('clamp',deg,i,n,gap)
   if gap<worst[0]:worst=(gap,n,'clamp:'+str(i))
   comparisons+=1
# Guard sweep: exact oriented-box SAT at0.25 degree; reserve0.5mm for between samples.
for deg in np.arange(0,190.001,.25):
 a=np.radians(deg);R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
 for i,shape in enumerate(t.guard.shapes):
  local=(np.array(shape.at)-t.guard.origin)*1000
  c0=(np.array(t.GUARD.pos)+t.guard.origin-[0,0,t.TOP])*1000
  p=c0+R@local;v=corners(np.zeros(3),shape.half*2000)@R.T+p
  for n,c,s in boxes:
   sep=np.maximum(c-s/2-v.max(0),v.min(0)-c-s/2)
   if sep.max()>.5:gap=float(np.linalg.norm(np.maximum(sep,0)))
   else:gap=separation(v,R.T,c,s)
   assert gap>.5,('guard',deg,i,n,gap)
   if gap<worst[0]:worst=(gap,n,'guard:'+str(i))
   comparisons+=1
out=dict(runtime_sha256=t.SOURCE_SHA256,added_fixed_components=len(boxes),swept_comparisons=comparisons,minimum_swept_gap_mm=worst,guard_sample_deg=.25,guard_between_sample_reserve_mm=.5,limitations='Analytical geometry only; bores and intended base unions excluded. No printing, load test or hardware result.')
ap=argparse.ArgumentParser();ap.add_argument('--recorded');args=ap.parse_args()
if args.recorded:
 import gzip
 from yam_arena import analysis,station
 op=gzip.open if args.recorded.endswith('.gz') else open
 rows=[json.loads(line) for line in op(args.recorded,'rt')]
 assert rows and all(r['source_sha256']==t.SOURCE_SHA256 for r in rows),'recorded source differs from current geometry'
 recorded_bytes=gzip.open(args.recorded,'rb').read() if args.recorded.endswith('.gz') else pathlib.Path(args.recorded).read_bytes()
 minima={};tests=0;cache={};edge_cache={};native_contacts={};native_upper_proxy={}
 pivot=json.loads((ROOT/'fixture/pivot-mount-envelopes.json').read_text())
 upper=[]
 for i,sh in enumerate(t.HEAD.shapes[4:]):
  c=(np.array(t.HEAD.pos)+sh.at-[0,0,t.TOP])*1000;sz=sh.half*2000
  if i%5==0:
   upper.append((f'native_upper_proxy_{i}',c,sz.copy()));sz=sz.copy();sz[:2]=13
  upper.append((f'physical_upper_clamp_{i}',c,sz))
 record_fixed=boxes+upper+[(f'upper_guide_extension_{y}',np.array([351,y,270.]),np.array([7.94,7.94,35.])) for y in (-5,115)]
 record_boxes=record_fixed+[(b['name'],np.array(b['center_mm']),np.array(b['size_mm'])) for b in pivot['boxes']]+[(n,(lo+hi)/2,hi-lo) for n,lo,hi in existing['components']]+[(b['name'],np.array(b['center_mm']),np.array(b['size_mm'])) for b in json.loads((ROOT/'fixture/load-path-moving.json').read_text())['boxes']]
 bc=np.array([c for _,c,_ in record_boxes]);bs=np.array([sz for _,_,sz in record_boxes]);bl=bc-bs/2;bh=bc+bs/2
 for link,hs in analysis.link_hulls().items():
  starts=np.r_[0,np.cumsum([len(h.vertices) for h in hs])[:-1]]
  cache[link]=(np.concatenate([h.vertices for h in hs])*1000,starts,hs)
 def hull_box_sat(vertices,h,R,c,sz):
  key=id(h)
  if key not in edge_cache:
   hh=ConvexHull(h.vertices);edges={tuple(sorted((int(a),int(b)))) for face in hh.simplices for a,b in itertools.combinations(face,2)}
   edge_cache[key]=np.array([h.vertices[b]-h.vertices[a] for a,b in edges])
  e=edge_cache[key]@R.T;axes=np.concatenate([np.eye(3),h.planes[:,:3]@R.T,*[np.cross(e,a) for a in np.eye(3)]])
  length=np.linalg.norm(axes,axis=1);axes=axes[length>1e-10]/length[length>1e-10,None]
  proj=vertices@axes.T;pc=c@axes.T;rad=np.abs(axes)@(sz/2)
  return float(np.maximum(pc-rad-proj.max(0),proj.min(0)-pc-rad).max())
 for row_index,r in enumerate(rows):
  if row_index%600==0:print(f'recorded clearance {row_index}/{len(rows)}',file=sys.stderr,flush=True)
  lot=r['lot'];best=minima.setdefault(lot,[1e6,None,None,None,None])
  row_lo=bl.copy();row_hi=bh.copy();row_bc=bc.copy();dz=r['joints']['specimen_guide']['head']*1000
  row_lo[len(record_fixed):,2]+=dz;row_hi[len(record_fixed):,2]+=dz;row_bc[len(record_fixed):,2]+=dz
  for arm in ('left','right'):
   q={f'joint{i+1}':float(v) for i,v in enumerate(r['arms'][arm+'_robot']['q'])}
   poses=analysis.fk_link_poses_world(q,station.BASES[arm])
   for link,(local,starts,hs) in cache.items():
    pose=poses[link];R=pose[:3,:3];verts=local@R.T+pose[:3,3]*1000-[0,0,t.TOP*1000]
    vl=np.minimum.reduceat(verts,starts);vh=np.maximum.reduceat(verts,starts)
    sep=np.maximum(row_lo[None,:,:]-vh[:,None,:],vl[:,None,:]-row_hi[None,:,:]);gaps=np.linalg.norm(np.maximum(sep,0),axis=2)
    native_pad=np.array([name=='carriage_pad' for name,_,_ in record_boxes])
    hh,bb=np.where((gaps<best[0]) | ((gaps==0)&native_pad[None,:]))
    for hidx,bidx in zip(hh,bb):
     gap=float(gaps[hidx,bidx])
     if gap<5:
      end=starts[hidx+1] if hidx+1<len(starts) else len(verts)
      sat_gap=hull_box_sat(verts[starts[hidx]:end],hs[hidx],R,row_bc[bidx],bs[bidx])
      gap=sat_gap if gap==0 else max(gap,sat_gap)
     if record_boxes[bidx][0].startswith('native_upper_proxy_'):
      prev=native_upper_proxy.get(lot)
      if prev is None or gap<prev[0]:native_upper_proxy[lot]=[gap,r['t'],arm,link,record_boxes[bidx][0]]
      assert gap>0,('native upper proxy interference',lot,r['t'],arm,link,gap)
      continue # Native14mm conservative proxy reported separately; actual13mm print also screened.
     if record_boxes[bidx][0]=='carriage_pad':
      if gap<=0:
       prev=native_contacts.get(lot)
       if prev is None or gap<prev[0]:native_contacts[lot]=[gap,r['t'],arm,link,'carriage_pad']
      continue # Native collidable pad contact is reported separately from omitted hardware.
     assert gap>0,('recorded robot interference',lot,r['t'],arm,link,record_boxes[bidx][0],gap)
     if gap<best[0]:best[:]=[gap,r['t'],arm,link,record_boxes[bidx][0]]
    tests+=len(hs)*len(record_boxes)
 out.update(recorded_rows=len(rows),recorded_components=len(record_boxes),native_pad_contacts=native_contacts,native_upper_proxy_lots=native_upper_proxy,recorded_lots=minima,hull_box_pairs=tests,recorded_sha256=hashlib.sha256(recorded_bytes).hexdigest(),recorded_limit='All logged joint samples screened with conservative AABB lower bounds plus complete hull/box SAT whenever AABB bounds are below5mm. Motion between telemetry samples and dynamic contact response not validated.')
print(json.dumps(out,indent=2))
