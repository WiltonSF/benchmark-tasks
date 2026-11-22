"""Offline geometric regression; neither a native rollout nor dough calibration."""
import sys,json,time,pathlib
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import model

def reviewer_counterexample():
    scale=1.6;p=model.P0*scale;flatten=np.ones(model.N);flatten[:2]=.45
    rest=np.full(model.N,model.L0*scale)
    for k in range(4):
        cells=model.cell_points(p,rest,flatten)
        if k<3:p[:,2]-=cells[:,:,2].min(1)
    g=model.geometry(p,rest,flatten,cells=cells)
    old=float(np.ptp(cells[:,:,2],axis=1).min())
    assert old>.012 and g['h_min']<.010 and model.tier(g)==0.
    assert np.ptp(cells[:,:,2].min(1))<.003
    return {'old_cell_range_mm':old*1000,'corrected_local_profile_mm':g['h_min']*1000,'shared_boundary_mm':np.ptp(cells[0,24:36,2])*1000,'bottom_spread_mm':np.ptp(cells[:,:,2].min(1))*1000,'score':model.tier(g)}

if __name__=='__main__':
    report={'reviewer_counterexample':reviewer_counterexample(),'tiers':{}}
    for scale,want in [(1.,0.),(1.12,.25),(1.28,.5),(1.4,1.)]:
        g=model.geometry(model.P0*scale,np.full(model.N,model.L0*scale));assert model.tier(g)==want
        report['tiers'][str(want)]={'height_mm':g['h_min']*1000,'d_mm':g['d']*1000,'D_mm':g['D']*1000}
    # One plump local wall, with its centre moved outward to preserve the hole:
    # intact and tall enough, but excessive wall variation earns only0.75.
    p=model.P0*1.45;flat=np.ones(model.N);flat[0]=.70;p[0,0]+=.004
    rest=np.full(model.N,model.L0*1.45);g=model.geometry(p,rest,flat)
    assert model.tier(g)==.75,(model.tier(g),g)
    report['tiers']['0.75']={'height_mm':g['h_min']*1000,'d_mm':g['d']*1000,'D_mm':g['D']*1000,'wall_ratio':g['w_max']/g['w_min']}
    # Rotation/translation in the pan plane cannot change the local height.
    cells=model.cell_points(p,rest,flat);angle=.61;q=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1.]])
    shift=np.array([.1,-.15,.2]);g2=model.geometry(p@q.T+shift,rest,flat,cells=cells@q.T+shift)
    assert abs(g2['h_min']-g['h_min'])<2e-7
    report['rotation_height_error_m']=abs(g2['h_min']-g['h_min'])
    print(json.dumps(report,indent=2))
