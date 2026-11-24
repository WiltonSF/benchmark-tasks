"""Task-local silicone cover geometry. No robot gains, assets or harness files are changed.

The distal finger hulls are expanded by the 1/16-inch cover thickness. The same
convex geometry is used for PhysX contact and hand clearance sampling. Thermal
performance is not simulated; the material service rating is not an insulation test.
"""
from functools import lru_cache
import itertools
import sys
import numpy as np
import parts

THICKNESS = .0015875  # 1/16 inch; one sheet on each of the four fingers
FRICTION = (.9, .8)  # silicone/wood estimate, not a measured coefficient


@lru_cache(maxsize=None)
def cover_hulls(name):
    from yam_arena import analysis
    from scipy.spatial import ConvexHull
    if name not in ('tip_left', 'tip_right'):
        return []
    offsets = np.array(list(itertools.product((-THICKNESS, THICKNESS), repeat=3)))
    result = []
    for hull in analysis.link_hulls()[name]:
        # Distal pad and tip only; the sliding carriage and joint remain uncovered.
        if hull.vertices[:, 2].min() < -.017:
            continue
        points = (hull.vertices[:, None, :] + offsets).reshape(-1, 3)
        hull3 = ConvexHull(points)
        points = points[hull3.vertices]
        hull3 = ConvexHull(points)
        result.append((points, hull3.simplices))
    return result


def install():
    # Make offline collision checks and sampled hand geometry use the same covers.
    # Cache the covers before extending the analysis hull dictionary, avoiding recursion.
    from yam_arena import analysis
    if not getattr(analysis, '_nozzle_covers', False):
        covers = {name: cover_hulls(name) for name in ('tip_left', 'tip_right')}
        for name, pieces in covers.items():
            analysis.link_hulls()[name].extend(analysis._hull_from_triangles(points[faces]) for points, faces in pieces)
        analysis._nozzle_covers = True
    previous = parts._PATCHES.get('yam_arena.embodiment')

    def patch(module):
        if previous:
            previous(module)
        if getattr(module, '_nozzle_covers', False):
            return
        original_cfg = module.yam_arm_cfg

        def cfg(*args, **kwargs):
            config = original_cfg(*args, **kwargs)
            original_spawn = config.spawn.func
            config.spawn.make_uninstanceable = True

            def spawn(prim_path, spawn_cfg, *args, **kwargs):
                prim = original_spawn(prim_path, spawn_cfg, *args, **kwargs)
                from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Vt
                import isaaclab.sim as sim_utils
                from yam_arena.robot import asset_link_paths
                stage = prim.GetStage(); root = str(prim.GetPath())
                material_path = root + '/NozzleCoverMaterial'
                material = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path, 'Material'))
                material.CreateStaticFrictionAttr(FRICTION[0]); material.CreateDynamicFrictionAttr(FRICTION[1])
                material.CreateRestitutionAttr(0.)
                count = 0
                for name, relative in asset_link_paths().items():
                    for i, (points, faces) in enumerate(cover_hulls(name)):
                        path = root + relative + '/nozzle_cover_' + str(i)
                        mesh = UsdGeom.Mesh.Define(stage, path)
                        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
                        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
                        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.flatten().astype(np.int32)))
                        mesh.CreateSubdivisionSchemeAttr('none')
                        mesh.CreateDoubleSidedAttr(True)
                        mesh.CreateDisplayColorAttr([Gf.Vec3f(.65, .12, .035)])
                        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
                        pc = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
                        pc.CreateContactOffsetAttr(.0002); pc.CreateRestOffsetAttr(0.)
                        sim_utils.bind_physics_material(path, material_path, stage=stage)
                        count += 1
                print('[nozzle covers] spawned', count, 'convex covers on', root, 'thickness', THICKNESS, flush=True)
                if count == 0:
                    raise ValueError('missing finger cover colliders')
                return prim
            config.spawn.func = spawn
            return config
        module.yam_arm_cfg = cfg; module._nozzle_covers = True
    parts._PATCHES['yam_arena.embodiment'] = patch
    if 'yam_arena.embodiment' in sys.modules:
        patch(sys.modules['yam_arena.embodiment'])
