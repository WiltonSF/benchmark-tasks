"""Whole-trial and endpoint regressions. `CASES` is read by the checker; `run()` exercises the cutting rule with
moving knife edges in the Arena Python environment (python near_misses.py)."""
from pathlib import Path
import importlib.util
import math
import numpy as np

CASES = [
 ('unknown hand state fails closed', {}, 0., False),
 ('moving a loose-looking crown earns nothing without a cutting history', {'floret_1': ((.25, -.12, .82), None, None)}, 0., False),
 ('head moved to another place earns nothing', {'stem': ((.3, 0, .8), None, None)}, 0., False),
]


def run():
    spec = importlib.util.spec_from_file_location('broccoli_cases', Path(__file__).with_name('task.py'))
    t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)
    q = lambda yaw: t.parts._matrix_to_quat(t.rz(yaw))
    far = {a: np.array([[0., .5 if a == 'left' else -.5, t.TOP+.3]]) for a in t.ARMS}

    def head(origin=t.HEAD_A, yaw=t.HEAD_YAW_A):
        place, _, quats = t.head_layout(origin, yaw); st = {o.name: o.state() for o in t.OBJECTS}
        for o in (t.STEM, *t.FLORETS): st[o.name] = o.state(pos=place[o.name], quat_xyzw=quats[o.name])
        return st

    def knife_at(point, theta, z):
        origin = np.array([point[0], point[1], z])-t.rz(theta)@np.array([t.X_CUT, 0, -.012])
        return t.KNIFE.state(pos=origin, quat_xyzw=q(theta))

    def hands(st, holding=True, tool=True):
        cc = t.HandClouds(); cc.update({a: v.copy() for a, v in far.items()})
        for arm in t.ARMS:
            cc.fingers[arm] = {'tip_left': far[arm].copy(), 'tip_right': far[arm].copy()}; cc.opening[arm] = .060
        if holding:
            s = st['stem']; r = t.rot(s.quat_xyzw)
            a = s.pos+r@np.array([t.STEM_R, t.GRIP_Y, 0]); b = s.pos+r@np.array([-t.STEM_R, t.GRIP_Y, 0])
            cc['left'] = np.array([a, b]); cc.fingers['left'] = {'tip_left': np.array([a]), 'tip_right': np.array([b])}; cc.opening['left'] = .034
        if tool:
            k = st['knife']; r = t.rot(k.quat_xyzw)
            a = k.pos+r@np.array([t.GRASP[0], .011, 0]); b = k.pos+r@np.array([t.GRASP[0], -.011, 0])
            cc['right'] = np.array([a, b]); cc.fingers['right'] = {'tip_left': np.array([a]), 'tip_right': np.array([b])}; cc.opening['right'] = .022
        return cc

    def cut_point(st, i, aim=t.AIM):
        f = st[t.NAMES[i]]; return np.asarray(f.pos)+t.rot(f.quat_xyzw)[:, 0]*aim

    def stroke(i, loaded=True, holding=True, tool=True, shift=np.zeros(3), theta=None, food_follows=False, reverse=False, steps=41, t0=0.):
        st0 = head(); c = cut_point(st0, i)+shift
        # Florets cut earlier in the demo order already lie elsewhere on the board.
        for j in t.ORDER[:t.ORDER.index(i)]:
            st0[t.NAMES[j]] = t.FLORETS[j].state(pos=(.14+.035*j, .20, t.SURFACE+.0085), quat_xyzw=(0, 0, 0, 1))
            t._REC['cuts'].add(j); t._REC['cut_stalks'][j] = (.02, .02)
        th = t.HEAD_YAW_A+t.ANGLES[i]+math.pi/2+t.OBLIQUE.get(i, 0.) if theta is None else theta
        zs = np.linspace(c[2]+.02, t.SURFACE+.0005, steps)
        if reverse: zs = zs[::-1]
        for k, z in enumerate(zs):
            st = dict(st0)
            if food_follows:
                dz = z-zs[0]
                st['stem'] = t.STEM.state(pos=np.asarray(st0['stem'].pos)+[0, 0, dz], quat_xyzw=st0['stem'].quat_xyzw)
                for n in t.NAMES: st[n] = t.FLORETS[t.NAMES.index(n)].state(pos=np.asarray(st0[n].pos)+[0, 0, dz], quat_xyzw=st0[n].quat_xyzw)
            st['knife'] = knife_at(c, th, z)
            t.update(st, hands(st, holding, tool), t0+.05*k)
            t.mark_applied(loaded)
        return st

    # Every positive tier is tested with the actual endpoint calculation, and no score without release history.
    st = {o.name: o.state() for o in t.OBJECTS}
    for i, o in enumerate(t.FLORETS):
        st[o.name] = o.state(pos=(.19+i*.04, -.15, t.SURFACE+.0085), quat_xyzw=(0, 0, 0, 1))
    t.fresh(); t._REC.update(armed=True, clear=True, still=True, t=2., since=0., stamp=t.stamp(st), cut_stalks={i: (.020, .020) for i in range(6)})
    for n, want in ((0, 0.), (1, .33), (2, .33), (3, .67), (5, .67), (6, 1.)):
        t._REC.update(cuts=set(range(n)), damaged=set(), level=want)
        assert t.score(st) == want, (n, t.score(st), want)
    for n, want in ((1, .67), (2, .67), (3, .33)):
        t._REC.update(cuts=set(range(6)), damaged=set(range(n)), level=want)
        assert t.score(st) == want, (n, t.score(st))
    t._REC.update(damaged=set(), level=1., clear=False); assert t.score(st) == 0
    t._REC.update(clear=True, since=1.05); assert t.score(st) == 0
    t._REC.update(since=1.); assert t.score(st) == 1
    t._REC['zero'] = 'blade clearance'; assert t.score(st) == 0
    t._REC['zero'] = None; t._REC['invalid'] = True; assert t.score(st) == 0
    # A crown hanging off the board edge or a stalk outside 10-30 mm does not count.
    t._REC['invalid'] = False
    off = dict(st); off['floret_6'] = t.FLORETS[5].state(pos=(.46, -.15, t.SURFACE+.0085), quat_xyzw=(0, 0, 0, 1))
    assert t.end_level(off) == .67
    t._REC['cut_stalks'][5] = (.009, .020); assert t.end_level(st) == .67
    # The knife must be left on the board: a knife dropped off the board or resting on the ruler scores 0.
    t._REC['cut_stalks'][5] = (.020, .020); assert t.end_level(st) == 1.
    gone = dict(st); gone['knife'] = t.KNIFE.state(pos=(.49, 0., t.TOP+.0145)); assert t.end_level(gone) == 0.
    lifted = dict(st); lifted['knife'] = t.KNIFE.state(pos=np.asarray(t.KNIFE.pos)+[0, 0, .02]); assert t.end_level(lifted) == 0.
    # Both declared layouts reproduce exactly; a displaced or turned start fails closed as invalid.
    for lot in t.lots.all_lots():
        exact = {n: (t.STEM if n == 'stem' else t.KNIFE if n == 'knife' else t.FLORETS[t.NAMES.index(n)]).state(pos=lot.pos(n), quat_xyzw=t.parts._matrix_to_quat(t.lot_rotation(lot, n))) for n in ('stem', 'knife', *t.NAMES)}
        assert max(t.setup_error(exact, lot)) < 1e-6, lot.name
    t.lots.use('A'); bad = head(origin=t.HEAD_A+[.006, 0, 0]); bad['knife'] = t.KNIFE.state()
    t.fresh(); t._REC['armed'] = True; t.update(bad, hands(bad), 0.)
    assert t._REC['invalid'] and t.score(bad) == 0
    turned = head(yaw=t.HEAD_YAW_A+math.radians(4)); turned['knife'] = t.KNIFE.state()
    t.fresh(); t._REC['armed'] = True; t.update(turned, hands(turned), 0.); assert t._REC['invalid']
    good = head(); good['knife'] = t.KNIFE.state()
    t.fresh(); t._REC['armed'] = True; t.update(good, hands(good), 0.); assert not t._REC['invalid'], t._REC['setup']
    # Fixed knife with distant hands: nothing advances; missing hand state cannot later become valid.
    t.fresh(); t._REC['armed'] = True
    cc = hands(good, False, False)
    for k in range(25): t.update(good, cc, k*.05)
    assert not t._REC['cuts'] and all(not x.any() for x in t._REC['coverage'].values()) and t.score(good) == 0
    t.update(good, None, 1.3); t.update(good, cc, 1.35); assert t._REC['invalid'] and t.score(good) == 0
    # The demo's stroke through each stalk cuts it, retaining 20 mm; every bypass of the rule does not.
    for i in range(6):
        t.fresh(); t._REC['armed'] = True; stroke(i)
        assert i in t._REC['cuts'], (i, t._REC['coverage'][i].sum())
        assert all(.010 <= v <= .030 for v in t._REC['cut_stalks'][i]) and abs(sum(t._REC['cut_stalks'][i])/2-.020) < .0015, t._REC['cut_stalks'][i]
        assert t._REC['cuts'] == set(t.ORDER[:t.ORDER.index(i)+1]) and not t._REC['damaged'] and t._REC['zero'] is None, (i, t._REC['cuts'], t._REC['damaged'], t._REC['zero'])
        # No other floret's section is touched by the demo stroke, so no stray first-crossing is recorded.
        assert all(not t._REC['coverage'][j].any() for j in range(6) if j not in t._REC['cuts']), (i, {j: int(v.sum()) for j, v in t._REC['coverage'].items()})
    for label, kw in (('no declared load', dict(loaded=False)), ('stem not held', dict(holding=False)),
                      ('knife not held by its handle', dict(tool=False)), ('knife and head moving down together', dict(food_follows=True)),
                      ('spine-first upward stroke', dict(reverse=True)), ('stroke 17 mm beside the stalk', dict(shift=t.rz(t.HEAD_YAW_A+t.ANGLES[2]+math.pi/2)@np.array([-.017-.020, 0, 0])))):
        t.fresh(); t._REC['armed'] = True; stroke(2, **kw)
        assert 2 not in t._REC['cuts'], label
    # A complete cut in the wrong place still separates the floret (as a real knife would), but it never qualifies.
    for label, shift, lo, hi in (('cut leaving a 38 mm stalk', -.018, .036, .040), ('cut leaving a 50 mm stalk', -.030, .048, .052), ('cut leaving an 8 mm stalk', .012, .006, .010)):
        t.fresh(); t._REC['armed'] = True; stroke(2, shift=t.rz(t.HEAD_YAW_A+t.ANGLES[2])@np.array([shift, 0, 0]))
        assert 2 in t._REC['cuts'] and all(lo <= v <= hi for v in t._REC['cut_stalks'][2]), (label, t._REC['cut_stalks'].get(2))
        st2 = head(); st2['knife'] = t.KNIFE.state()
        for j in t._REC['cuts']: st2[t.NAMES[j]] = t.FLORETS[j].state(pos=(.14+.035*j, .20, t.SURFACE+.0085), quat_xyzw=(0, 0, 0, 1))
        t._REC['cuts'] = {2}; assert t.end_level(st2) == 0., label
    # A 45-degree oblique stroke still cuts when every section point keeps 10-30 mm.
    t.fresh(); t._REC['armed'] = True; stroke(2, theta=t.HEAD_YAW_A+t.ANGLES[2]+math.pi/2+math.pi/4, steps=61)
    assert 2 in t._REC['cuts'] and all(.01 <= v <= .03 for v in t._REC['cut_stalks'][2]), t._REC['cut_stalks'].get(2)
    # Interrupted grasp: partial incisions persist, but release needs a fresh 0.1 s of loaded contact.
    t.fresh(); t._REC['armed'] = True
    st0 = head(); c = cut_point(st0, 2); th = t.HEAD_YAW_A+t.ANGLES[2]+math.pi/2
    zs = np.linspace(c[2]+.02, c[2], 21)
    for k, z in enumerate(zs):
        s = dict(st0); s['knife'] = knife_at(c, th, z); t.update(s, hands(s), .05*k); t.mark_applied(True)
    partial = t._REC['coverage'][2].copy(); assert partial.any() and not partial.all() and t._REC['loaded_time'][2] > .1
    s = dict(st0); s['knife'] = knife_at(c, th, c[2]); t.update(s, hands(s, holding=False), 1.05)
    assert 2 in t._REC['next_forces'], 'the resistance still acts with the stem let go'
    t.mark_applied(False)
    assert t._REC['loaded_time'][2] == 0 and np.array_equal(partial, t._REC['coverage'][2])
    # A stroke through the rest of the section in a step without the resistance acting, with none in the step after
    # either, is not credited (the one-step entry allowance needs the resistance in the next step).
    s = dict(st0); s['knife'] = knife_at(c, th, c[2]-.012); t.update(s, hands(s), 1.10); t.mark_applied(False)
    s = dict(st0); s['knife'] = knife_at(c, th, c[2]-.012); t.update(s, hands(s), 1.15); t.mark_applied(True)
    assert 2 not in t._REC['cuts'] and np.array_equal(partial, t._REC['coverage'][2]) and not t._REC['pending']
    # Back up 6 mm (incisions kept), then a continuous loaded stroke finishes the section.
    s = dict(st0); s['knife'] = knife_at(c, th, c[2]+.006); t.update(s, hands(s), 1.17); t.mark_applied(True)
    for k, z in enumerate(np.linspace(c[2]+.004, c[2]-.012, 9)):
        s = dict(st0); s['knife'] = knife_at(c, th, z); t.update(s, hands(s), 1.20+.05*k); t.mark_applied(True)
    assert 2 in t._REC['cuts'], (t._REC['coverage'][2].sum(), t._REC['loaded_time'][2])
    # The resistance scales with the edge length inside the 16 mm section and opposes the stroke.
    kn0 = knife_at(c, th, c[2]+.002); kn1 = knife_at(c, th, c[2]-.001); f6 = st0[t.NAMES[2]]
    force, point, length = t.cutting_force(kn0, kn1, f6, f6, .05)
    assert abs(length-.016) < .002 and force[2] > 0 and abs(np.linalg.norm(force)-(t.FEED+t.DRAG)*length) < .5, (force, length)
    kup = knife_at(c, th, c[2]+.002)
    force_up, _, _ = t.cutting_force(kn1, kup, f6, f6, .05)
    assert force_up[2] < 0 and abs(np.linalg.norm(force_up)-t.DRAG*length) < .5, force_up
    far_knife = knife_at(c, th, c[2]+.05); assert t.cutting_force(far_knife, far_knife, f6, f6, .05)[1] is None
    # A knife held still in the cut meets the feed resistance only (no drag); drawn back slower than 5 mm/s it still
    # meets the feed, faster only the drag.
    f_stall, _, _ = t.cutting_force(kn1, kn1, f6, f6, .05)
    assert f_stall[2] > 0 and abs(np.linalg.norm(f_stall)-t.FEED*length) < .3, f_stall
    creep = knife_at(c, th, c[2]-.001+.0002)
    f_creep, _, _ = t.cutting_force(kn1, creep, f6, f6, .05)
    assert f_creep[2] > 0 and abs(np.linalg.norm(f_creep)-t.FEED*length) < .3, f_creep
    # The moment the force would have about the knife's centre of mass, taken where the edge is in the stalk (about
    # 0.85 N m here). It is logged in traces but not applied (see README).
    total, moment = t.knife_wrench(kn1, {2: (force, point, length)})
    com = np.asarray(kn1.pos)+t.rot(kn1.quat_xyzw)@t.KNIFE_COM
    assert np.allclose(total, force) and np.allclose(moment, np.cross(np.asarray(point)-com, force)) and .6 < np.linalg.norm(moment) < 1.1, moment
    assert abs(t.KNIFE_COM[0]-.0064) < .0005 and abs(t.KNIFE_COM[1]) < 1e-9
    # The resistance acts with the stem unheld or the knife not held by its handle; only the credit needs both grips.
    for kw in (dict(holding=False), dict(tool=False)):
        t.fresh(); t._REC['armed'] = True; seen = 0.
        for k, z in enumerate(np.linspace(c[2]+.02, c[2]-.006, 21)):
            s = dict(st0); s['knife'] = knife_at(c, th, z); t.update(s, hands(s, **kw), .05*k); t.mark_applied(True)
            if 2 in t._REC['next_forces']: seen = max(seen, float(np.linalg.norm(t._REC['next_forces'][2][0])))
        assert seen > 5. and 2 not in t._REC['cuts'] and not t._REC['coverage'][2].any(), (kw, seen)
    # Native grasp needs opposed fingertip contact on the stem itself.
    s = head(); cc = hands(s); assert t.native_grasp(cc, 'left', s['stem'])
    cc.fingers['left']['tip_right'] = far['left']; assert not t.native_grasp(cc, 'left', s['stem'])
    r = t.rot(s['stem'].quat_xyzw); corner = s['stem'].pos+r@np.array([.018, t.GRIP_Y, .018])
    cc.fingers['left'] = {'tip_left': np.array([corner]), 'tip_right': np.array([s['stem'].pos+r@np.array([-.018, t.GRIP_Y, .018])])}
    assert not t.native_grasp(cc, 'left', s['stem'])
    # Exact damage threshold inside a real model bud cylinder; discarded crowns use the same identity.
    o = t.FLORETS[0]; state = o.state(); shape = o.shapes[2]; centre = np.asarray(shape.at)
    pts = [centre+np.array([shape.size[0]-d, 0, 0]) for d in (.0029, .003, .0031)]
    flags = [t.bud_interior(np.array([state.pos+t.rot(state.quat_xyzw)@p]), state, o, .003) for p in pts]
    assert flags == [False, False, True], flags
    # Full blade and swept clearance: a spine-near point is outside the old edge-only radius.
    knife = t.KNIFE.state(); r = t.rot(knife.quat_xyzw)
    near = knife.pos+r@np.array([.09, 0, .027])
    cc = {'left': np.array([near]), 'right': far['right']}
    distance, contact = t.blade_clearance(cc, None, 'left', {'knife': knife}, None)
    assert distance < .03 and np.linalg.norm(t.edge(knife)-near, axis=1).min() > .03
    old = t.KNIFE.state(pos=knife.pos-r@np.array([.1, 0, 0])); new = t.KNIFE.state(pos=knife.pos+r@np.array([.1, 0, 0]))
    on = knife.pos+r@np.array([.09, 0, 0]); cc['left'] = np.array([on])
    distance, contact = t.blade_clearance(cc, cc, 'left', {'knife': new}, {'knife': old})
    assert contact and distance < .001
    # The full blade within 30 mm of the holding gripper zeroes the whole trial.
    s = head(); k = s['knife'] = knife_at(cut_point(s, 5), t.HEAD_YAW_A+t.ANGLES[5]+math.pi/2, t.SURFACE+.02)
    cc = hands(s); kr = t.rot(k.quat_xyzw); nearhand = k.pos+kr@np.array([.09, 0, .034])
    cc['left'] = np.vstack([cc['left'], nearhand]); cc.fingers['left']['tip_left'] = np.vstack([cc.fingers['left']['tip_left'], nearhand])
    t.fresh(); t._REC['armed'] = True; t.update(s, cc, 0.); assert t._REC['zero'] and 'within 30 mm' in t._REC['zero'], (t._REC['zero'], t._REC['holding'], t._REC['min_clearance'])
    # Exact primitive support and pose-based hold are independent of noisy instantaneous velocity.
    obj = t.FLORETS[2]; r = t.parts._rot((1, 0, 0), np.pi/4)@t.rz(-np.pi/4); qq = t.parts._matrix_to_quat(r)
    state = obj.state(pos=(.25, 0, 0), quat_xyzw=qq); lo, _ = t.true_bounds(obj, state)
    state = obj.state(pos=(.25, 0, t.SURFACE-lo[2]), quat_xyzw=qq)
    lo, _ = t.true_bounds(obj, state)
    assert abs(lo[2]-t.SURFACE) < 1e-9 and state.low[2] < t.SURFACE-.003
    fixed = {o.name: o.state() for o in t.OBJECTS}; t.fresh()
    assert not t.pose_hold(fixed, True, 0.) and not t.pose_hold(fixed, True, .99) and t.pose_hold(fixed, True, 1.)
    moved = dict(fixed); moved['knife'] = t.KNIFE.state(pos=np.array(t.KNIFE.pos)+(.003, 0, 0))
    assert not t.pose_hold(moved, True, 1.05) and t.pose_hold(moved, True, 2.05) and not t.pose_hold(moved, False, 2.1)
    t.fresh()
    for k in range(61):
        drifting = dict(fixed); drifting['knife'] = t.KNIFE.state(pos=np.array(t.KNIFE.pos)+(.001*k*.05, 0, 0))
        stable = t.pose_hold(drifting, True, k*.05)
        if k >= 20: assert stable
    # A blade back can penetrate a crown while its cutting edge remains outside it.
    spec_ = t.FLORETS[0]; food = spec_.state(); centre = food.pos+t.rot(food.quat_xyzw)@np.asarray(spec_.shapes[2].at)
    kn = t.KNIFE.state(pos=centre-np.array([.088, 0, .005]), quat_xyzw=(0, 0, 0, 1))
    assert not t.bud_interior(t.edge(kn), food, spec_, .003) and t.blade_damage(kn, kn, food, food, spec_)
    budonly = type('BudOnly', (), {'shapes': spec_.shapes[:3]})()
    for depth, want in ((.0029, False), (.003, False), (.0031, True)):
        kk = t.KNIFE.state(pos=centre+np.array([-.088, spec_.shapes[2].size[0]-depth+.001, .003]), quat_xyzw=(0, 0, 0, 1))
        assert t.blade_damage(kk, kk, food, food, budonly) == want
    # Moving produce through a stationary knife between clear endpoints is still damage.
    oldfood = spec_.state(pos=food.pos+np.array([0, .05, 0])); newfood = spec_.state(pos=food.pos-np.array([0, .05, 0]))
    assert not t.blade_damage(kn, kn, oldfood, oldfood, spec_) and not t.blade_damage(kn, kn, newfood, newfood, spec_)
    assert t.blade_damage(kn, kn, oldfood, newfood, spec_)
    point = centre[None, :]
    assert not t.bud_interior(point, oldfood, spec_, .002) and not t.bud_interior(point, newfood, spec_, .002)
    assert t.bud_sweep(point, point, oldfood, newfood, spec_, .002)
    t.fresh(); t.lots.use(None)
    print('PASS: tiers, damage caps, board footprint, stalk band, exact A/B start check and displaced/turned starts invalid, '
          'release, 1 s boundary, permanent zero, missing state, stationary knife, the demo stroke on all six stalks, '
          'no-load / unheld stem / unheld knife / knife-and-head together / spine-first / beside-stalk bypasses, 38 / 50 / 8 mm wrong-length cuts, '
          '45-degree oblique cut, interrupted grasp and unloaded strokes not credited, resistance scaling, logged knife moment, resistance with either grip missing (no credit), 30 mm blade clearance zero, 3 mm bud boundary, full-blade damage, '
          'moving-food damage, exact cylinder support, rolling pose hold')


if __name__ == '__main__': run()
