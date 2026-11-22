"""Uncalibrated MuJoCo contact diagnostics. These are not physical force measurements.

Uses the unchanged factory probe for the short-sample contact test, then a
34-body linked strip for a five-second gravity/contact stability check.
"""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

LENGTH = .8128
PITCH = LENGTH / 34


def short_probe(factory_path, approach='naive'):
    spec = importlib.util.spec_from_file_location('factory_contact_probe', factory_path)
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    # The standard tool permits one primitive per body; add the actual fin and
    # two barb lands to the free short specimen, without changing the factory.
    original = probe.geom_xml
    def compound(o):
        s = original(o)
        if o['name'] == 'sample':
            extra = '<geom type="box" size=".0047625 .01185294 .00075" pos=".0127 0 0" mass=".0002"/>'
            for x in (.0115, .0145):
                extra += f'<geom type="box" size=".0005 .01185294 .0018" pos="{x} 0 0" mass=".0001"/>'
            s = s.replace('</body>', extra + '</body>')
        return s
    probe.geom_xml = compound
    objects = [dict(name='sample',shape='box',size=[.0079375,PITCH/2-.0001,.00635],pos=[.020,0.,.02635],mass=.0013235,friction=.55),
               dict(name='floor_support',shape='box',size=[.06,.05,.01],pos=[.035,0.,.01],free=False),
               dict(name='slot_lower',shape='box',size=[.02,.05,.00238125],pos=[.060,0.,.02238125],free=False),
               dict(name='slot_upper',shape='box',size=[.02,.05,.00238125],pos=[.060,0.,.03031875],free=False),
               dict(name='slot_back',shape='box',size=[.01,.05,.00635],pos=[.059525+.01,0.,.02635],free=False)]
    # Correct opening at x=.040 and 3.175 mm vertically, matching the task.
    objects[2]['size'][2] = (.02635-.003175/2-.020)/2
    objects[2]['pos'][2] = (.02635-.003175/2+.020)/2
    objects[3]['size'][2] = (.0327-.02635-.003175/2)/2
    objects[3]['pos'][2] = (.0327+.02635+.003175/2)/2
    objects[4]['pos'][0] = .040+.009525+.01
    hands=[dict(name='press',start=[-.005,0.,.02635],paddle=[.006,.010,.006],force=[25,25,8],
        path=[[.5,-.005,0.,.02635],[2.,.045,0.,.02635],[3.,.045,0.,.02635]])]
    if approach=='hold_down':
        hands.append(dict(name='hold',start=[.020,0.,.050],paddle=[.007,.010,.004],force=[10,10,8],
            path=[[.5,.020,0.,.0355],[2.,.032,0.,.0355],[3.,.032,0.,.0355]]))
    if approach=='low_paddle':
        hands[0]['paddle']=[.006,.010,.002]
        hands[0]['start'][2]=.023
        hands[0]['path']=[[.5,-.005,0.,.023],[2.,.028,0.,.023],[3.,.028,0.,.023]]
    s = dict(objects=objects,hands=hands,
        goal=dict(object='sample',x_min=.039),seconds=4.)
    model = probe.build(s)
    data = mujoco.MjData(model)
    peaks = dict(hand=0.,kerf=0.)
    contact_force = np.zeros(6)
    for step in range(int(4./model.opt.timestep)):
        t = step*model.opt.timestep
        for h in s['hands']:
            target = probe.target(h['path'], h['start'], t)
            for i,axis in enumerate('xyz'):
                data.ctrl[model.actuator(h['name']+axis).id] = target[i]-h['start'][i]
        mujoco.mj_step(model,data)
        for ci in range(data.ncon):
            c = data.contact[ci]
            names = [model.body(int(model.geom_bodyid[g])).name for g in (c.geom1,c.geom2)]
            if 'sample' not in names:
                continue
            mujoco.mj_contactForce(model,data,ci,contact_force)
            mag = float(np.linalg.norm(contact_force[:3]))
            if 'press' in names: peaks['hand'] = max(peaks['hand'],mag)
            if any(n.startswith('slot_') for n in names): peaks['kerf'] = max(peaks['kerf'],mag)
    pos = data.xpos[model.body('sample').id].copy()
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    return dict(approach=approach,final_center_m=pos.tolist(),peak_contact_N=peaks,
                final_bulb_gap_mm=float((.040-(pos[0]+.0079375))*1000),
                final_height_error_mm=float((pos[2]-.02635)*1000),
                interpretation='Simulator contact diagnostic only; short-sample hands are not robot reach or full-seal evidence.')


def chain_stability():
    root = ET.Element('mujoco')
    ET.SubElement(root,'option',timestep='.001',gravity='0 0 -9.81',iterations='100')
    world = ET.SubElement(root,'worldbody')
    ET.SubElement(world,'geom',type='plane',size='2 2 .1',friction='.55 .005 .0001')
    ET.SubElement(world,'geom',type='box',pos='0 0 .01',size=f'.058 {LENGTH/2+.03} .01')
    parent = world
    for i in range(34):
        body = ET.SubElement(parent,'body',name=f'seal_{i:02}',pos=(f'0 {-LENGTH/2+PITCH/2} .02655' if i==0 else f'0 {PITCH} 0'))
        if i==0: ET.SubElement(body,'freejoint')
        else: ET.SubElement(body,'joint',type='ball',pos=f'0 {-PITCH/2} 0',damping='.00003')
        ET.SubElement(body,'geom',type='box',size=f'.0079375 {PITCH/2-.0001} .00635',mass=str(.045/34),friction='.55 .005 .0001')
        parent=body
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    data=mujoco.MjData(model)
    peak_speed=0.
    for _ in range(5000):
        mujoco.mj_step(model,data)
        peak_speed=max(peak_speed,float(np.max(np.abs(data.qvel))))
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    final_speed=float(np.max(np.abs(data.qvel)))
    assert final_speed < .05, f'Chain did not settle: {final_speed}'
    return dict(duration_s=5.,bodies=34,peak_generalized_speed=peak_speed,final_generalized_speed=final_speed,
                interpretation='Free linked bulb-envelope stability only; fin/barb insertion and robot reach remain unvalidated.')


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--factory-probe',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=dict(status='UNCALIBRATED_CPU_DIAGNOSTICS',
        approaches=[short_probe(args.factory_probe,approach) for approach in ['naive','hold_down','low_paddle']],
        chain=chain_stability())
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
