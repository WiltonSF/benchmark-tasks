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
        full = {'outer_ring': ring, 'acrylic_base': base, 'gel_sheet': gel}
        S = m['S']
        def rec(**kw):
            S.clear(); S.update(m['fresh']()); S.update(known=True, phys=50, steps=50, t=2.1, clear=0., still=0., bond=np.zeros(N, bool)); S.update(kw)
        def emit(label, change, want, win=False, **kw):
            rec(**kw); return (label, change, want, win)
        yield emit('nested mold at the start', {}, 0., bond=np.ones(N, bool))
        yield emit('ring on counter, base on stand, sheet laid flat, hands clear and still 2 s', full, 1., True)
        yield emit('everything right but still only 1.999 s', full, .75, t=1.999)
        yield emit('unknown arm state', full, 0., known=False)
        yield emit('no physics contact history', full, 0., phys=0)
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
        for label in ('tear', 'hole', 'crushed band', 'part fell off'):
            yield emit(label + ' stays 0', full, 0., zero=True, why=label)
        # Material rules, driven through the real rule code.
        st = start['gel_sheet']; o = dict(start)
        def lift(link, dz=0., dx=0.):
            links = dict(st.links)
            if link == 'base':
                return replace(st, pos=st.pos + np.array([dx, 0, dz]))
            p, q = links[link]; links[link] = (p + np.array([dx, 0, dz]), q); return replace(st, links=links)
        rec(bond=np.ones(N, bool)); o['gel_sheet'] = lift('s3', .003); m['adhesive_step'](o, .01)
        yield ('middle band pulled up 3 mm tears', full, 0., False)
        rec(bond=np.ones(N, bool)); o['gel_sheet'] = lift('base', .003); m['adhesive_step'](o, .01)
        yield ('corner lifted without a fingertip start tears', full, 0., False)
        rec(bond=np.ones(N, bool)); o['gel_sheet'] = replace(st, pos=st.pos + np.array([0, .0031, 0])); m['adhesive_step'](o, .01)
        yield ('bonded sheet dragged 3.1 mm sideways tears', full, 0., False)
        rec(bond=np.array([0, 1, 1, 1, 1, 1, 1], bool)); o['gel_sheet'] = lift('s1', .002)
        for _ in range(19): m['adhesive_step'](o, .01)
        assert S['bond'][1] and not S['zero']
        m['adhesive_step'](o, .01)
        assert not S['bond'][1] and S['bond'][2:].all() and not S['zero']
        yield ('front band held 2 mm up for 0.2 s lets go, nothing else', full, .25, False)
        rec(bond=np.array([0, 1, 1, 1, 1, 1, 1], bool)); o['gel_sheet'] = lift('s1', .0051); m['adhesive_step'](o, .01)
        yield ('front band yanked 5.1 mm before letting go tears', full, 0., False)
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
