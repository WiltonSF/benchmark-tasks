"""Material conservation, selective depth and release-certificate regressions."""
import dataclasses
from types import SimpleNamespace
import numpy as np
import task

CASES = [
    ("no runtime measurements", {}, 0., False),
    ("ladle held over stock, unknown hands", {"ladle": task.LADLE.state(
        pos=(0.23, 0.09, task.TOP+0.10))}, 0., False),
    ("ladle on table, unknown history", {"ladle": task.LADLE.state(
        pos=(0.18, 0.36, task.TOP+0.0584))}, 0., False),
]


def checks():
    count = 0
    def require(value, label):
        nonlocal count
        assert value, label
        count += 1

    pan, cup = task.PAN.state(), task.CUP.state()
    interface = task.PAN.pos[2]+task.PAN_BOTTOM+task.INITIAL_BROTH/task.AREA
    surface = interface+task.INITIAL_FAT/task.AREA
    def collect(rim_z):
        liquid = task.Liquid()
        ladle = task.LADLE.state(pos=(pan.pos[0]-task.BOWL[0], pan.pos[1], rim_z-task.BOWL[2]))
        for _ in range(100): liquid.advance(.05, pan, ladle, cup)
        return liquid
    hover = collect(surface+.002)
    require(hover.ladle_fat == hover.ladle_broth == 0., "hover cannot collect")
    deep = collect(interface-.010)
    require(deep.ladle_broth > deep.ladle_fat/4, "deep scoop fails phase purity")
    require(deep.conserved() and not deep.invalid, "deep scoop still conserves both phases")
    shallow = collect(surface-.003)
    require(shallow.ladle_fat > 5e-6 and shallow.ladle_broth == 0., "shallow rim collects fat")
    old = shallow.ladle_fat+shallow.ladle_broth
    ladle = task.LADLE.state(pos=(pan.pos[0]-task.BOWL[0], pan.pos[1], surface-.003-task.BOWL[2]))
    for _ in range(200): shallow.advance(.05, pan, ladle, cup)
    require(shallow.ladle_fat+shallow.ladle_broth <= task.CAPACITY+1e-12, "stationary rim cannot pump beyond cavity")
    require(shallow.conserved(), "long submersion conserves mass")
    inside=task.Liquid()
    amount=10e-6
    inside.fat-=amount/len(inside.fat)
    inside.receive(amount,0.,np.asarray(cup.pos)+(0,0,.030),pan,cup)
    require(inside.cup_fat==amount and inside.spill_fat==0. and inside.conserved(),
            "a stream already inside the cup is retained below its rim")
    below=task.Liquid(); below.fat-=amount/len(below.fat)
    below.receive(amount,0.,np.asarray(cup.pos)+(0,0,-.002),pan,cup)
    require(below.cup_fat==0. and below.spill_fat==amount and below.conserved(),
            "a stream below the cup floor cannot enter upward")
    nested_cup=task.CUP.state(pos=tuple(pan.pos+(0,0,task.PAN_BOTTOM)))
    nested=task.Liquid(); nested.broth-=amount
    nested.receive(0.,amount,np.asarray(nested_cup.pos)+(0,0,.030),pan,nested_cup)
    require(nested.cup_broth==amount and nested.conserved(),
            "nested cup cannot erase actual broth contamination")

    # A125mm-wide face sits0.5mm above the box, with no hull vertex close.
    vertices = np.array([(x,y,z) for x in (-.0625,.0625) for y in (-.0625,.0625) for z in (0.,.02)])
    axis = np.linspace(-.0625,.0625,43)
    face = np.array([(x,y,0.) for x in axis for y in axis])
    center, size = np.array((0.,0.,-.0055)), np.array((.010,.010,.010))
    vertex_gap = np.linalg.norm(np.maximum(np.abs(vertices-center)-size/2,0.),axis=1).min()
    require(vertex_gap > .005, "vertex-only test misses broad face")
    require(not task.sampled_clear([face], [("face",center,np.eye(3),size)]), "face0.5mm away fails release")
    require(not task.sampled_clear([], []), "missing geometry fails closed")
    require(not task.sampled_clear([face], [("bad",center*np.nan,np.eye(3),size)]), "nonfinite box fails closed")

    objects = {o.name:o.state() for o in task.OBJECTS}
    hands = np.array([(0.12,.32,task.TOP+.18)]*3+[(0.12,-.30,task.TOP+.18)]*3)
    trial = task.Trial()
    trial.liquid.cup_fat = .7*task.INITIAL_FAT
    trial.liquid.fat *= .3
    def score(tr):
        original=task._trial
        try:
            task._trial=tr
            return task.score(objects)
        finally: task._trial=original
    for k in range(44): trial.advance(k*.05,objects,True,hands,support=True)
    require(score(trial)==1., "conserved successful end state needs completed2s release")
    unknown_support=task.Trial()
    unknown_support.liquid.cup_fat=.7*task.INITIAL_FAT
    unknown_support.liquid.fat*=.3
    for k in range(44): unknown_support.advance(k*.05,objects,True,hands,support=None)
    require(score(unknown_support)==0., "unknown contact load cannot certify support")
    saved=(task._support_view,task._support_force,task._support_error,task._support_detail)
    mock_sim=SimpleNamespace(env=SimpleNamespace(unwrapped=SimpleNamespace(
        sim=SimpleNamespace(get_physics_dt=lambda:1/240))))
    class Buffer:
        def __init__(self,data):self.data=data
        def numpy(self):return self.data
    class View:
        def __init__(self,data):self.data=data
        def get_contact_force_matrix(self,dt):return Buffer(self.data)
    try:
        for loads,expected,label in (([.40,0.,.40],True,"both supports"),
                ([.80,0.,0.],False,"handle-only support"),
                ([0.,0.,.80],False,"bowl-only support"),
                ([.10,0.,.10],False,"insufficient load")):
            forces=np.zeros((1,3,3));forces[0,:,2]=loads
            task._support_view=View(forces)
            require(task.support_contact(mock_sim) is expected,label)
        task._support_view=View(np.full((1,3,3),np.nan))
        require(task.support_contact(mock_sim) is None,"unknown contact buffer fails closed")
    finally:
        task._support_view,task._support_force,task._support_error,task._support_detail=saved
    for name,spec in (("saucepan",task.PAN),("waste_cup",task.CUP)):
        for shift,expected in ((.005,False),(.005001,True)):
            shifted=dict(objects)
            shifted[name]=spec.state(pos=np.asarray(spec.pos)+(shift,0,0))
            boundary=task.Trial();boundary.advance(0.,shifted,True,hands,support=True)
            require(boundary.violation is expected,f"{name} 5mm inclusive boundary {shift}")
    for shift,expected in ((.002,True),(.002001,False)):
        raised=task.LADLE.state(pos=np.asarray(task.LADLE.pos)+(0,0,shift))
        require(task.on_rest(raised) is expected,f"rest height 2mm boundary {shift}")
        raised_cup=dict(objects,waste_cup=task.CUP.state(pos=np.asarray(task.CUP.pos)+(0,0,shift)))
        boundary=task.Trial()
        for k in range(44):boundary.advance(k*.05,raised_cup,True,hands,support=True)
        require((boundary.held>=2.) is expected,f"cup bottom 2mm boundary {shift}")
    floating=task.Trial()
    floating.liquid.cup_fat=.7*task.INITIAL_FAT;floating.liquid.fat*=.3
    elevated=dict(objects,ladle=task.LADLE.state(pos=tuple(np.asarray(task.LADLE.pos)+(0,0,.0019))))
    for k in range(44): floating.advance(k*.05,elevated,True,hands,support=False)
    old_task=task._trial
    try:
        task._trial=floating
        require(task.score(elevated)==0., "1.9mm support gap with no measured load fails")
    finally: task._trial=old_task
    require(score(dataclasses.replace(trial,known=False))==0., "unknown state cannot reuse credit")
    require(score(dataclasses.replace(trial,violation=True))==0., "past pot movement cannot be erased")
    phantom=task.LADLE.state(pos=(.12,.399,task.TOP+.07115),
        quat_xyzw=(0,0,np.sin(np.pi/4),np.cos(np.pi/4)))
    require(not task.on_rest(phantom), "yawed airborne ladle cannot borrow rest height")
    require(not task.on_rest(task.LADLE.state(pos=(.18,.36,task.TOP+.01))), "below-table ladle is not supported")
    trial.advance(2.2,objects,False,hands)
    require(score(trial)<1., "held tool cannot finish")
    trial.advance(2.25,objects,None,None)
    require(score(trial)==0. and trial.invalid, "missing interval invalidates trial")
    require(score(task.Trial())==0., "no-op zero")
    same_time=task.Trial()
    same_time.advance(0.,objects,True,hands)
    same_time.advance(0.,objects,True,hands)
    require(same_time.invalid,"duplicate noninitial time cannot alter history")
    moved=task.Trial(); nested_objects=dict(objects,waste_cup=nested_cup)
    moved.advance(0.,nested_objects,True,hands)
    require(moved.violation,"moving or nesting receiving vessel irreversibly fails")
    print(f"{count} material and release assertions passed")


if __name__ == "__main__":
    checks()
