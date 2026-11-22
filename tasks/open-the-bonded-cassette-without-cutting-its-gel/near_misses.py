"""Synthetic scorer controls, not robot trials."""
import numpy as np
import parts
class Cases:
    def __iter__(self):
        m=parts._cassette_namespace;S=m['S'];f=m['fresh'];TOP=m['TOP'];CX=m['CX']
        def rec(**kw):
            S.clear();S.update(f());S.update(known=True,phys=50,steps=50,t=2.,still=0.,clear=0.,witness=np.ones(3,bool),bond=np.zeros(15,bool));S.update(kw)
        start={p.name:p.state() for p in m['OBJECTS']}
        full={'notched_plate':m['UPPER'].state(pos=(CX,-.2,TOP+.001)), 'gel_knife':m['KNIFE'].state(pos=(.14,-.34,TOP+.007))}
        def emit(label,change,want,win=False,**kw):rec(**kw);return label,change,want,win
        yield emit('untouched cassette',{},0.,bond=np.ones(15,bool))
        yield emit('all seams opened, intact gel retained, empty plate and knife down',full,1.,True)
        yield emit('knife/hand state unavailable',full,0.,known=False)
        yield emit('no native physics history',full,0.,phys=0)
        yield emit('still only 0.999 s',full,0.,t=.999)
        yield emit('hand near plate',full,0.,clear=None)
        yield emit('gel slit remains whole-trial zero',full,0.,zero=True,why='slit')
        for edge in range(3):
            b=np.ones(15,bool);b[edge*5:(edge+1)*5]=False
            yield emit('one full edge released '+str(edge),{},.25,bond=b)
        for edge in range(3):
            b=np.zeros(15,bool);b[edge*5:(edge+1)*5]=True
            yield emit('two edges released '+str(edge),{},.5,bond=b)
        b=np.zeros(15,bool);b[0]=True
        yield emit('one last bond still attached',full,.5,bond=b)
        yield emit('plates not separated',{},.5)
        yield emit('plate held in air',full|{'notched_plate':m['UPPER'].state(pos=(CX,-.2,TOP+.06))},.75)
        yield emit('gel lost from both plates',full|{'hydrated_gel':m['GEL'].state(pos=(.6,0,TOP+.001))},0.)
        # Drive material code: pose-only knife near a bond without load cannot release it.
        rec(bond=np.ones(15,bool));m['material_rule'](start,0.,.01)
        assert S['bond'].all() and not S['zero']
        yield 'force-free blade at start',{},0.,False
        rec(bond=np.ones(15,bool));bad=start|{'notched_plate':m['UPPER'].state(pos=(CX,0,m['UPPER_Z']+.006))};m['material_rule'](bad,0.,.01)
        yield 'pull cover before seam release',full,0.,False
        # Actual movable support, independent of its authored start location.
        moved={
            'inverted_crystallizing_dish':m['DISH'].state(pos=(CX+.012,0.,TOP)),
            'slotted_plate':m['LOWER'].state(pos=(CX+.012,0.,m['BOTTOM_Z'])),
            'hydrated_gel':m['GEL'].state(pos=(CX+.012,0.,m['SEAM']))}
        yield emit('dish and retained original plate moved together 12 mm',full|moved,1.,True)
        moved_only={'inverted_crystallizing_dish':m['DISH'].state(pos=(CX+.020,0.,TOP))}
        yield emit('dish moved away: plate hovering at old start is unsupported',full|moved_only,.75)
        # Local force gate, actual contact neighbourhood, and native blade geometry.
        kpos=(CX-.050-.155+.005,0.,m['SEAM']-m['BLADE_Z'])
        knife=m['KNIFE'].state(pos=kpos)
        open_top=m['UPPER'].state(pos=(CX,0,m['UPPER_Z']+.0006))
        o=start|{'gel_knife':knife,'notched_plate':open_top}
        cp=np.array([[CX-.047,0,m['SEAM']]])
        rec(bond=np.ones(15,bool),witness=np.zeros(3,bool));m['material_rule'](o,5.,.01)
        assert S['bond'].all(), 'aggregate load without local contacts must not open a seam'
        yield 'loaded elsewhere but no local contact witness',{},0.,False
        rec(bond=np.ones(15,bool),witness=np.zeros(3,bool));m['material_rule'](o,.199,.01,cp,[.199])
        assert S['bond'].all()
        yield 'local force just below 0.20 N',{},0.,False
        rec(bond=np.ones(15,bool),witness=np.zeros(3,bool));m['material_rule'](o,.20,.01,cp,[.20])
        assert not S['bond'][:5].any() and S['bond'][5:].all()
        yield 'exact local force/opening gate releases only covered native sites',{},0.,False
        rec(bond=np.ones(15,bool),witness=np.zeros(3,bool));m['material_rule'](o,1.,.01,cp+np.array([[.07,0,0]]),[1.])
        assert S['bond'].all()
        yield 'knife pose right but contact on a different edge',{},0.,False
        rec(bond=np.ones(15,bool),witness=np.zeros(3,bool));m['material_rule'](o,5.,.01,cp+np.array([[0,0,.004]]),[5.])
        assert S['bond'].all()
        yield 'load on top face cannot stand in for load inside seam',{},0.,False
        # A separated edge moves the gap; valid load on its upper inner face remains valid.
        rec(bond=np.ones(15,bool),witness=np.zeros(3,bool))
        up2=m['UPPER'].state(pos=(CX,0,m['UPPER_Z']+.0019))
        k2=m['KNIFE'].state(pos=(kpos[0],0.,kpos[2]+.0019))
        o2=start|{'notched_plate':up2,'gel_knife':k2}
        m['material_rule'](o2,.3,.01,cp+np.array([[0,0,.0024]]),[.3])
        assert not S['bond'][:5].any()
        yield 'load on moved inner face is inside actual opened gap',{},0.,False
        # Actual blade path through the gel trips the permanent cut state.
        rec(bond=np.zeros(15,bool));inside=start|{'gel_knife':m['KNIFE'].state(pos=(CX-.150,0.,m['SEAM']-m['BLADE_Z']))}
        m['material_rule'](inside,0.,.01)
        inside['gel_knife']=m['KNIFE'].state(pos=(CX-.1488,0.,m['SEAM']-m['BLADE_Z']))
        m['material_rule'](inside,.6,.01)
        assert S['zero'] and S['cut_length']>=.001
        yield 'actual 1.2 mm blade motion inside the gel',full,0.,False
        # Returning to start does not rearm a damaged trial.
        m['material_rule'](start,0.,.01)
        yield 'damaged trial returned to untouched pose',{},0.,False
        rec();m['observe'](start,np.array([[np.nan,0,0]]),0.)
        yield 'nonfinite hand state',full,0.,False
        for angle in (.09,.4):
            q=(np.sin(angle/2),0,0,np.cos(angle/2))
            held=m['UPPER'].state(pos=(CX,-.2,TOP+.010),quat_xyzw=q)
            yield emit('tilted plate is not supported '+str(angle),full|{'notched_plate':held},.75)
        S.clear();S.update(f())
CASES=Cases()

if __name__=='__main__':
    import task
    starts={p.name:p.state() for p in task.OBJECTS};n=0
    for name,change,want,win in CASES:
        got=task.score(starts|change);success=task.success(starts|change)
        assert abs(got-want)<1e-7 and success==win,(name,got,want,success,win)
        n+=1
    print(str(n)+' scorer controls PASS')
