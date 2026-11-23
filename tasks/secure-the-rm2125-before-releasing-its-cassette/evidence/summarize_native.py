"""Summarize executed native contact logs; no task scorer or policy imports."""
import json,math,sys,statistics,pathlib,gzip

def summarize(path):
 raw=gzip.decompress(pathlib.Path(path).read_bytes()).decode() if str(path).endswith('.gz') else pathlib.Path(path).read_text();rows=[json.loads(x) for x in raw.splitlines()];groups=[]
 for r in rows:
  if not groups or groups[-1][-1]['lot']!=r['lot'] or groups[-1][-1]['t']>=r['t']:groups.append([])
  groups[-1].append(r)
 out=[]
 for r in groups:
  last=r[-1];forces=[float(v) for a in r for v in a.get('finger_forces',{}).values()]
  hold=[a for a in r if 48<=a['t']<=51 and abs(a['contact_moments']['clamp'][1])>.05]
  load=[]
  for a in hold:
   th=a['joints']['specimen_guide']['clamp'];lever=.101*math.cos(th)-.040*math.sin(th)
   load.append(a['contact_moments']['clamp'][1]/lever)
  out.append(dict(lot=last['lot'],source_sha256=last['source_sha256'],samples=len(r),end_time=last['t'],end_score=last['score'],peak_score=max(a['score'] for a in r),known=last['known'],invalid=last['invalid'],why=last['why'],held_s=last['held'],hands_clear=last['clear'],wheel_turn_final_deg=math.degrees(last['joints']['handwheel']['wheel']),handle_final_mm=1000*last['joints']['handwheel']['handle'],guard_final_deg=math.degrees(last['joints']['knife_guard']['guard']),clamp_peak_deg=max(math.degrees(a['joints']['specimen_guide']['clamp']) for a in r),finger_peak_resultant_N=max(forces),clamp_48_51s_foot_equivalent_N=(dict(n=len(load),median=statistics.median(load),min=min(load),max=max(load)) if load else None),cassette_final_m=last['cassette'],tray_final_m=last['tray']))
 return out
if __name__=='__main__':print(json.dumps(summarize(sys.argv[1]),indent=2))
