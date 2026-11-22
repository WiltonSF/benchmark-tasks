"""Face samples of the gripper and both pads, with unknown data failing closed."""
from functools import lru_cache
import numpy as np


def sample_faces(hull, spacing=.003):
    vertices = np.asarray(hull.vertices)
    points = [vertices]
    for plane in np.unique(hull.planes.round(7), axis=0):
        face = vertices[np.abs(vertices @ plane[:3] - plane[3]) < 2e-6]
        if len(face) < 3:
            raise ValueError('Incomplete hull face')
        a = face.mean(0)
        u = face[0] - a
        u /= np.linalg.norm(u)
        v = np.cross(plane[:3],u)
        face = face[np.argsort(np.arctan2((face-a) @ v, (face-a) @ u))]
        for b, c in zip(face, np.roll(face, -1, axis=0)):
            count = max(1, int(np.ceil(max(np.linalg.norm(a-b), np.linalg.norm(a-c), np.linalg.norm(b-c)) / spacing)))
            points.append(np.array([a + (b-a)*i/count + (c-a)*j/count
                                    for i in range(count+1) for j in range(count+1-i)]))
    cloud = np.concatenate(points)
    _, keep = np.unique(np.floor(cloud/.0015).astype(int), axis=0, return_index=True)
    return cloud[keep]


@lru_cache(maxsize=1)
def local_hand_samples():
    from yam_arena.analysis import link_hulls
    return {name: np.concatenate([sample_faces(hull) for hull in link_hulls()[name]])
            for name in ('gripper','tip_left','tip_right')}


def hand_clouds(sim):
    import parts
    scene = sim.env.unwrapped.scene
    origin = parts._np(scene.env_origins)[0]
    clouds = []
    for arm in ['left_robot', 'right_robot']:
        robot = scene[arm]
        poses = parts._np(robot.data.body_link_pose_w)[0]
        if not np.isfinite(poses).all():
            raise ValueError('Invalid robot poses')
        cloud = []
        for name, points in local_hand_samples().items():
            pose = poses[robot.body_names.index(name)]
            cloud.append(points @ parts._quat_to_matrix(pose[3:]).T + pose[:3] - origin)
        if not cloud:
            raise ValueError('Missing gripper hulls')
        clouds.append(np.concatenate(cloud))
    return clouds
