"""Scorer and material-rule counterexamples (synthetic states, not robot trials)."""
import numpy as np
import parts
from dataclasses import replace


class Cases:
    def __iter__(self):
        m = parts._gel_namespace; TOP = m['TOP']; BH = m['BH']; H = m['H']; R = m['R']; N = m['N']
        sx, sy = m['STAND_XY']['A']; MX, MY = m['LAY']['A']; SH = m['STAND_H']
        start = {o.name: o.state() for o in m['OBJECTS']}
        ring = m['RING'].state(pos=(sx, sy, TOP))
        base = m['BASE'].state(pos=(sx, sy, TOP + SH + BH / 2))
        gel = m['GEL_SHEET'].state(pos=(MX, MY, TOP + H / 2))
        full = {'outer_ring': ring, 'acrylic_base': base, 'gel_sheet': gel, 'push_out_stand': m['STAND'].state()}
        S = m['S']
        def rec(**kw):
            S.clear(); S.update(m['fresh']()); S.update(known=True, phys=50, steps=50, t=2.1, clear=0., still=0., bond=np.zeros(N, bool), seated=False); S.update(kw)
        def emit(label, change, want, win=False, **kw):
            rec(**kw); return (label, change, want, win)
        yield emit('nested mold at the start', {}, 0., bond=np.ones(N, bool))
        yield emit('ring on counter, base on stand, sheet laid flat, hands clear and still 2 s', full, 1., True)
        yield emit('everything right but still only 1.999 s', full, .75, t=1.999)
        yield emit('unknown arm state', full, 0., known=False)
        yield emit('no physics contact history', full, 0., phys=0)
        yield emit('parts separated without loaded ring release history', full, 0., seated=True)
        yield emit('a hand within 30 mm of a part', full, 0., clear=None)
        yield emit('hands clear only 0.999 s', full, 0., t=.999)
        yield emit('hands clear 1 s, not yet still 2 s', full, .75, t=1.0)
        # Ejection only.
        stack = m['GEL_SHEET'].state(pos=(sx, sy, TOP + SH + BH + H / 2))
        yield emit('acrylic and sheet on the stand, sheet still bonded', full | {'gel_sheet': stack}, .25, bond=np.ones(N, bool))
        yield emit('sheet on the stand, bonds released but lying on the acrylic', full | {'gel_sheet': stack}, .25)
        yield emit('ring pressed off but left tilted against the stand', full | {'outer_ring': m['RING'].state(pos=(sx, sy, TOP + .02), quat_xyzw=(np.sin(.2), 0, 0, np.cos(.2)))}, 0.)
        yield emit('ring tipped up on the counter away from the parts', full | {'outer_ring': m['RING'].state(pos=(.48, .25, TOP + .03), quat_xyzw=(np.sin(.2), 0, 0, np.cos(.2)))}, .75)
        # Sheet peeled out but the acrylic left in the ring: base inside the ring volume.
        nested_base = m['BASE'].state(pos=(sx, sy, TOP + .005 + BH / 2))
        yield emit('sheet peeled and laid flat but acrylic left in the ring', full | {'acrylic_base': nested_base}, 0.)
        yield emit('sheet laid flat inside the ring outline', full | {'gel_sheet': m['GEL_SHEET'].state(pos=(sx, sy, TOP + .005 + H / 2)), 'acrylic_base': m['BASE'].state(pos=(.45, .25, TOP + BH / 2))}, 0.)
        for gap, want in ((.0049, .25), (.0052, 1.)):
            # Sheet flat on the counter beside the stand, its edge `gap` from the acrylic's vertical plane.
            edge = m['GEL_SHEET'].state(pos=(sx - 2 * R - gap, sy, TOP + SH + BH + H / 2))
            yield emit(f'sheet level with the acrylic, {gap * 1000:.1f} mm from it', full | {'gel_sheet': edge}, .5 if want == 1. else .25)
        held = m['GEL_SHEET'].state(pos=(MX, MY, TOP + .06))
        yield emit('sheet free of the acrylic but held up in the air', full | {'gel_sheet': held}, .5)
        moving = m['GEL_SHEET'].state(pos=(MX, MY, TOP + H / 2), lin_vel=(.02, 0, 0))
        yield emit('sheet still sliding on the counter', full | {'gel_sheet': moving}, .5)
        # One band folded over: not laid flat.
        links = dict(gel.links); p6, q6 = links['s6']; links['s6'] = (p6 + np.array([0, 0, .02]), q6)
        yield emit('far band in the air', full | {'gel_sheet': replace(gel, links=links)}, .5)
        folded = m['GEL_SHEET'].state(pos=(MX, MY, TOP + H / 2), joints={'s6': 2.5})
        yield emit('far band folded back', full | {'gel_sheet': folded}, .5)
        yield emit('one band still bonded somewhere', full, .25, bond=np.array([0, 0, 0, 1, 0, 0, 0], bool))
        base_moving = m['BASE'].state(pos=(sx, sy, TOP + SH + BH / 2), lin_vel=(.02, 0, 0))
        yield emit('acrylic sliding off the stand', full | {'acrylic_base': base_moving}, .75)
        base_counter = m['BASE'].state(pos=(.45, .25, TOP + BH / 2))
        yield emit('acrylic laid on the counter instead of the stand', full | {'acrylic_base': base_counter}, 1., True)
        suspended = m['BASE'].state(pos=(.60,-.25,TOP+SH+BH/2))
        yield emit('acrylic suspended away from dish at correct height', full | {'acrylic_base': suspended}, .75)
        outside = m['BASE'].state(pos=(1.20,.0,TOP+BH/2))
        yield emit('acrylic at counter height outside counter bounds', full | {'acrylic_base': outside}, .75)
        tilted_stand = m['STAND'].state(quat_xyzw=(np.sin(.2),0,0,np.cos(.2)))
        yield emit('tilted support cannot support level acrylic', full | {'push_out_stand': tilted_stand}, .75)
        for label in ('tear', 'hole', 'crushed band', 'part fell off'):
            yield emit(label + ' stays 0', full, 0., zero=True, why=label)
        # Material rules, driven through the real rule code.
        st = start['gel_sheet']; o = dict(start)
        def lift(link, dz=0., dx=0.):
            links = dict(st.links)
            if link == 'base':
                return replace(st, pos=st.pos + np.array([dx, 0, dz]))
            p, q = links[link]; links[link] = (p + np.array([dx, 0, dz]), q); return replace(st, links=links)
        T = np.zeros(N, bool)
        def touch(*ks):
            t = T.copy(); t[list(ks)] = True; return t
        def raise_(link,dz,steps,t=None,supplied_N=10.0):
            # Synthetic mechanically connected flap rotates about its next still-bonded hinge.
            # Work is force times actual contacted material-point rise, not force times band-centre gap.
            k=0 if link=='base' else int(link[1:]);W=m['W'];HH=m['H']
            hw=np.asarray(st.pos)+np.array([-R+(k+1)*W,0.,0.])
            cp_local=np.array([-R if k==0 else -R+W/2,0.,-HH/2]);last_cp_z=float(st.pos[2]+cp_local[2])
            for i in range(1,steps+1):
                target=dz*i/steps;lo,hi=0.,1.55
                for _ in range(45):
                    mid=(lo+hi)/2;gap=W/2*np.sin(mid)+HH/2*(1-np.cos(mid))
                    if gap<target:lo=mid
                    else:hi=mid
                angle=(lo+hi)/2;rot=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
                q=np.array([0,np.sin(angle/2),0,np.cos(angle/2)])
                root=hw+rot@(st.pos-hw);links=dict(st.links)
                for j in range(1,k+1):
                    name='s'+str(j);oldpos,_=links[name];links[name]=(hw+rot@(oldpos-hw),q)
                o['gel_sheet']=replace(st,pos=root,quat_xyzw=q,links=links)
                cp_z=float((root+rot@cp_local)[2]);ds=cp_z-last_cp_z;last_cp_z=cp_z
                work=np.zeros(N);resisted=np.zeros(N)
                if t is not None:
                    work[np.flatnonzero(t)]=supplied_N*ds;resisted[k]=m['FRONT_FORCE'][k]*ds
                m['adhesive_step'](o,1/240,t,work,resisted)
        rec(bond=np.ones(N, bool)); o['gel_sheet'] = lift('s3', .003); m['adhesive_step'](o, .01)
        yield ('middle band pulled up 3 mm tears', full, 0., False)
        rec(bond=np.ones(N, bool)); o['gel_sheet'] = replace(st, pos=st.pos + np.array([0, .0031, 0])); m['adhesive_step'](o, .01)
        yield ('bonded sheet dragged 3.1 mm sideways tears', full, 0., False)
        # The corner (an end band, free outer edge) is a peel front from the start, but it only lets go when the
        # robot touches it while it is opened 1 mm against its opening resistance.
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .002, 40)
        assert S['bond'].all() and not S['zero']
        yield ('corner band opened 2 mm with no robot contact stays bonded', full, .25, False)
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N))
        for _ in range(240): o['gel_sheet'] = st; m['adhesive_step'](o, 1 / 240, touch(0))
        assert S['bond'].all() and S['work'][0] == 0.
        yield ('fingertip pressed on the corner for 1 s without opening it: stays bonded', full, .25, False)
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .0009, 40, touch(0))
        assert S['bond'].all()
        yield ('corner opened 0.9 mm by robot contact: stays bonded', full, .25, False)
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .0012, 40, touch(5))
        assert S['bond'].all()
        yield ('corner opened 1.2 mm while the robot touches only a far bonded band: stays bonded', full, .25, False)
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .012, 240, touch(0))
        assert not S['bond'][0] and S['bond'][1:].all() and not S['zero'] and S['work'][0] == 0.
        yield ('corner propagated under area-scaled resisted work lets go, nothing else', full, .25, False)
        for force, label in ((0., 'no supplied work'), (-1., 'top-face downward work'), (.1, '0.1 N equivalent opening load')):
            rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .0012, 40, touch(0), supplied_N=force)
            assert S['bond'].all()
            yield (label + ' cannot release an opened corner', full, .25, False)
        # Opening work done before the contact does not count.
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .0012, 40)
        for _ in range(5): m['adhesive_step'](o, 1 / 240, touch(0))
        assert S['bond'].all()
        yield ('corner opened with no contact, then touched: stays bonded', full, .25, False)
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N)); raise_('base', .0051, 1)
        yield ('corner yanked 5.1 mm in one substep tears', full, 0., False)
        # Next band: the front moves on only through the freed flap or the band itself.
        rec(bond=np.array([0, 1, 1, 1, 1, 1, 1], bool), gap=np.zeros(N), work=np.zeros(N)); raise_('s1', .012, 240, touch(0))
        assert not S['bond'][1] and S['bond'][2:].all() and not S['zero']
        yield ('front propagation requires the full band-area work, no other release', full, .25, False)
        rec(bond=np.array([0, 1, 1, 1, 1, 1, 1], bool), gap=np.zeros(N), work=np.zeros(N)); raise_('s1', .002, 40)
        assert S['bond'][1]
        yield ('front band opened 2 mm with no robot contact stays bonded', full, .25, False)
        rec(bond=np.array([0, 1, 1, 1, 1, 1, 1], bool), gap=np.zeros(N), work=np.zeros(N)); raise_('s1', .0051, 1, touch(0))
        yield ('front band yanked 5.1 mm before letting go tears', full, 0., False)
        # Equal opposing robot work on the same flap/front must cancel before credit.
        rec(bond=np.array([0,1,1,1,1,1,1],bool),gap=np.zeros(N),work=np.zeros(N))
        for i in range(1,41):
            o['gel_sheet']=lift('s1',.0012*i/40)
            rw=np.zeros(N);rw[0]=.0012/40;rw[1]=-.0012/40
            res=np.zeros(N);res[1]=m['FRONT_FORCE'][1]*3*.0012/40
            m['adhesive_step'](o,1/240,touch(0,1),rw,res)
        assert S['bond'][1] and S['work'][1] == 0.
        yield ('opposing signed work on connected flap and front cannot release',full,.25,False)
        # One free flap bounded by two fronts: a single supplied budget cannot be reused on both.
        rec(bond=np.array([1,0,1,1,1,1,1],bool),gap=np.zeros(N),work=np.zeros(N))
        # Both fronts move together, so each must share the only contacted free-band budget.
        for i in range(1,41):
            moved=lift('base',.0012*i/40)
            links=dict(moved.links); p2,q2=links['s2']; links['s2']=(p2+np.array([0,0,.0012*i/40]),q2)
            o['gel_sheet']=replace(moved,links=links)
            rw=np.zeros(N);rw[1]=float(m['FRONT_WORK'][0]) / 40
            res=np.zeros(N);res[0]=m['FRONT_FORCE'][0]*.030/40;res[2]=m['FRONT_FORCE'][2]*.030/40
            m['adhesive_step'](o,1/240,touch(1),rw,res)
        assert S['bond'][0] and S['bond'][2]
        yield ('shared flap work is not duplicated at two fronts',full,.25,False)
        # Supplied work alone cannot replace an actual resisted-load history.
        rec(bond=np.ones(N,bool));o['gel_sheet']=lift('base',.0012)
        rw=np.zeros(N);rw[0]=1.;m['adhesive_step'](o,1/240,touch(0),rw)
        assert S['bond'].all() and S['work'][0]==0.
        yield ('no resisted load history means no fracture credit despite supplied work',full,.25,False)
        # Equal robot/resisted return describes one energy flow, not two separate losses.
        rec(bond=np.ones(N,bool));o=dict(start);S['work'][0]=.010
        rw=np.zeros(N);rw[0]=-.001;res=rw.copy()
        m['adhesive_step'](o,1/240,touch(0),rw,res)
        assert abs(S['work'][0]-.009)<1e-12
        yield ('signed return is debited once while respecting both energy ledgers',full,.25,False)
        rec(bond=np.ones(N,bool));S['bond'][1]=False;o=dict(start)
        rw=np.zeros(N);rw[1]=.001;res=np.zeros(N);res[0]=.001;res[2]=-.001
        m['adhesive_step'](o,1/240,touch(1),rw,res)
        assert S['work'].sum()==0.
        yield ('opposing resisted work at shared fronts cannot create fracture credit',full,.25,False)
        # A single fast yank cannot mark its patch peeled before the damage test.
        rec(bond=np.ones(N,bool));o['gel_sheet']=lift('base',.012)
        rw=np.zeros(N);rw[0]=1.;m['adhesive_step'](o,1/240,touch(0),rw,rw)
        assert S['zero'] and S['bond'].all()
        yield ('one-step high-work12mm yank still damages the prior bonded patch',full,0.,False)
        # Explicit real-load work includes relative translation and rotation, not common motion.
        for common,dz,want in ((False,.001,1.),(False,-.001,-1.),(True,.001,0.)):
            rec(bond=np.ones(N,bool));o=dict(start)
            S['contact_points'][0]=m['gel_points'](o['gel_sheet'])[0];S['contact_weights'][0]=3.;S['touch'][0]=True;S['gap'][0]=.001;S['contact_motion'][0]=np.array([0,0,.00005])
            m['prepare_brakes'](o)
            moved=dict(o);shift=np.array([0,0,dz]);moved['gel_sheet']=replace(o['gel_sheet'],pos=o['gel_sheet'].pos+shift)
            if common:moved['acrylic_base']=replace(o['acrylic_base'],pos=o['acrylic_base'].pos+shift)
            w=m['brake_work'](moved)[0];expected=want*float(np.linalg.norm(S['brake_f'][0]))*abs(dz)
            assert abs(w-expected)<1e-12
        rec();yield ('peel load work is signed and base-relative',full,1.,True)
        rec(bond=np.ones(N,bool));o=dict(start)
        S['contact_points'][0]=m['gel_points'](o['gel_sheet'])[0];S['contact_weights'][0]=3.;S['touch'][0]=True
        m['prepare_brakes'](o);assert np.all(S['brake_f']==0.)
        S['gap'][0]=.001;m['prepare_brakes'](o)
        assert np.linalg.norm(S['brake_f'][0])>=m['FRONT_FORCE'][0]
        assert m['brake_work'](o)[0]==0.
        S['brake_f'][0]*=.5
        moved=dict(o);moved['gel_sheet']=replace(o['gel_sheet'],pos=o['gel_sheet'].pos+np.array([0,0,.001]))
        assert m['brake_work'](moved)[0]==0.
        rec();yield ('opening adhesion load does no stationary work; below-force motion earns no credit',full,1.,True)
        rec(bond=np.ones(N,bool));S['bond'][0]=False;o=dict(start)
        S['contact_points'][0]=m['gel_points'](o['gel_sheet'])[0];S['contact_weights'][0]=3.;S['touch'][0]=True
        m['prepare_brakes'](o)
        assert S['brake_body'][1]==0 and np.all(S['brake_f']==0.)
        rw=np.zeros(N);rw[0]=1.;m['adhesive_step'](o,1/240,touch(0),rw)
        assert S['work'][1]==0. and S['bond'][1]
        yield ('freed flap touching a pick does not preload or debond an unopened next front',full,.25,False)
        # Bond and press-fit forces: equal and opposite on the sheet and the acrylic (zero net force and moment).
        rec(bond=np.ones(N, bool), gap=np.zeros(N), work=np.zeros(N), seated=True); o['gel_sheet'] = lift('s1', .0004); m['adhesive_step'](o, 0.)
        S['contact_points'][0]=m['gel_points'](o['gel_sheet'])[0]+np.array([-.006,0,.0015]);S['contact_weights'][0]=3.;S['touch'][0]=True;S['gap'][0]=.001;S['contact_motion'][0]=np.array([0,0,.00005])
        m['prepare_brakes'](o)
        (fk, pk, gt), (bf, bt), (rf, rp) = m['bond_wrenches'](o); bp = o['acrylic_base'].pos
        net_f = fk.sum(0) + bf + rf; net_m = np.cross(pk, fk).sum(0) + gt.sum(0) + np.cross(bp, bf) + bt + np.cross(rp, rf)
        assert np.abs(net_f).max() < 1e-9 and np.abs(net_m).max() < 1e-9 and fk[1, 2] < -.5
        yield ('bond and fit forces sum to zero force and moment while fit remains seated', full, 0., False)
        # Signed force extraction and contact-point rotation, with synthetic contact buffers.
        class ContactBuffer:
            def __init__(self, point, direction=1., mismatch=False):
                self.point = np.asarray(point, float)[None, :]
                self.normal = np.array([[0., 0., direction]])
                self.counts = np.zeros((N, 1), int); self.counts[0, 0] = 1
                self.starts = np.zeros((N, 1), int); self.mismatch = mismatch
            def get_contact_data(self, dt):
                return np.ones((1, 1)), self.point, self.normal, np.zeros(1), self.counts, self.starts
            def get_contact_force_matrix(self, dt):
                out = np.zeros((N, 1, 3)); out[0, 0] = self.normal[0] * (-1 if self.mismatch else 1)
                return out
            def get_friction_data(self, dt):
                return np.zeros((1, 3)), self.point, np.zeros((N, 1), int), self.starts
        def previous_contact_state():
            prev = m['poses'](start['gel_sheet'])
            b = start['acrylic_base']; prev['acrylic'] = (b.pos, parts._quat_to_matrix(b.quat_xyzw))
            return prev
        for direction, common, expected in ((1., False, .001), (-1., False, -.001), (1., True, 0.)):
            rec(); S['contact_prev'] = previous_contact_state()
            moved = dict(start); dz = np.array([0., 0., .001])
            moved['gel_sheet'] = replace(start['gel_sheet'], pos=start['gel_sheet'].pos + dz)
            if common: moved['acrylic_base'] = replace(start['acrylic_base'], pos=start['acrylic_base'].pos + dz)
            buf = ContactBuffer(moved['gel_sheet'].pos, direction)
            result = m['contact_work'](buf, moved, 1 / 240, np.zeros(3))
            assert abs(result[0] - expected) < 1e-12 and np.all(result[1:] == 0.)
        rec(); S['contact_prev'] = previous_contact_state()
        theta = .2; rotated = dict(start)
        rotated['gel_sheet'] = replace(start['gel_sheet'], quat_xyzw=(0., np.sin(theta / 2), 0., np.cos(theta / 2)))
        rot = parts._quat_to_matrix(rotated['gel_sheet'].quat_xyzw)
        point = rotated['gel_sheet'].pos + rot @ np.array([-.01, 0., 0.])
        result = m['contact_work'](ContactBuffer(point), rotated, 1 / 240, np.zeros(3))
        assert abs(result[0] - .01 * np.sin(theta)) < 1e-12
        rec(); S['contact_prev'] = previous_contact_state()
        try: m['contact_work'](ContactBuffer(point, mismatch=True), rotated, 1 / 240, np.zeros(3))
        except ValueError as e: assert 'differs from PhysX' in str(e)
        else: raise AssertionError('inconsistent signed normal must fail closed')
        rec()
        yield ('synthetic contact work preserves force sign, base-relative motion and rotation; bad normal fails closed', full, 1., True)
        paths=['/World/RightArm/tip_left','/World/RightArm/tip_right']
        assert m['held_pick'](np.array([[1.,0,0],[-1.,0,0]]),paths)
        assert not m['held_pick'](np.array([[1.,0,0],[1.,0,0]]),paths)
        assert not m['held_pick'](np.array([[1.,0,0],[0.,0,0]]),paths)
        assert not m['held_pick'](np.array([[1.,0,0],[-1.,0,0]]),['/World/RightArm/gripper','/World/RightArm/tip_left'])
        yield ('tool work requires opposed contact on both same-arm fingertips',full,1.,True)
        # The ring's fit lets go only after 2 mm of travel made while the robot presses on the ring.
        rec(bond=np.ones(N, bool), seated=True)
        ring_down = dict(o); ring_down['outer_ring'] = replace(o['outer_ring'], pos=o['outer_ring'].pos - np.array([0, 0, .0025]))
        m['fit_step'](ring_down, 0.); assert S['seated']
        m['fit_step'](ring_down, .4); assert S['seated']
        m['fit_step'](ring_down, 7.); assert S['seated']
        # Delayed stationary full-load touch supplies no travel/work.
        assert S['fit_loaded'] == 0. and S['fit_work'] == 0.
        rec(bond=np.ones(N, bool), seated=True)
        for z in np.linspace(0., .0025, 101):
            ring_down['outer_ring'] = replace(o['outer_ring'], pos=o['outer_ring'].pos - np.array([0,0,z]))
            m['fit_step'](ring_down, 7.)
        assert not S['seated'] and S['fit_loaded'] >= .002
        yield ('ring fit released by2mm travel under7N robot load',full,.25,False)
        rec(bond=np.ones(N, bool), seated=True)
        for z in np.linspace(0., .0025, 101):
            ring_down['outer_ring'] = replace(o['outer_ring'], pos=o['outer_ring'].pos - np.array([0,0,z]))
            m['fit_step'](ring_down, 6.9)
        assert S['seated']
        yield ('6.9N ring travel remains unqualified and scores zero', full, 0., False)
        rec(); m['contact_damage']([0, 15.1, 0, 0, 0, 0, 0])
        yield ('15.1 N robot contact on a band crushes it', full, 0., False)
        rec(); m['contact_damage']([0, 14.9, 0, 0, 0, 0, 0])
        yield ('14.9 N robot contact is allowed', full, 1., True)
        # Whole-trial observer, not pre-filled flags.
        observed = dict(start) | full; far = np.array([[-.4, -.5, TOP + .5]])
        rec(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        for j in range(41): m['observe'](observed, far, j * .05)
        yield ('observed 2 s released and still', full, 1., True)
        rec(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        near = np.array([gel.pos + np.array([.05, 0, H / 2 + .02])])
        for j in range(61): m['observe'](observed, near, j * .05)
        yield ('a hand 20 mm above the sheet', full, 0., False)
        m['observe'](observed, None, 3.1)
        yield ('lost arm state stays 0', full, 0., False)
        rec(steps=0, known=False, prev=None, clear=None, still=None, t=None)
        dropped = observed | {'push_out_stand': m['STAND'].state(pos=(sx, -.7, TOP - .03))}
        m['observe'](dropped, far, 0.)
        for j in range(1, 41): m['observe'](observed, far, j * .05)
        yield ('stand knocked off the counter, put back', full, 0., False)
        rec(); S.update(m['fresh']())


CASES = Cases()
