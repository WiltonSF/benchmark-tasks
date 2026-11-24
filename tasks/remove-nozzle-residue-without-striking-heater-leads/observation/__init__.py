"""Native sampled hand loads and full face-sampled hand geometry."""
import math
from functools import lru_cache
import numpy as np
import parts
import protection
_tip_contact_view=None

@lru_cache(maxsize=1)
def hand_mesh():
    from yam_arena import analysis
    out={}
    for name in ('gripper','tip_left','tip_right'):
        cloud=[]
        for h in analysis.link_hulls().get(name,[]):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7),axis=0):
                face=h.vertices[np.abs(h.vertices@pl[:3]-pl[3])<2e-6]
                if len(face)<3:raise ValueError('unknown hull face')
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(pl[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for a,b in zip(face,np.roll(face,-1,axis=0)):
                    n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                    for i in range(n+1):
                        for j in range(n-i+1):cloud.append(c+(a-c)*(i/n)+(b-c)*(j/n))
        if not cloud:raise ValueError('missing hand geometry')
        cloud=np.asarray(cloud)
        _,keep=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True)
        out[name]=cloud[keep]
    return out

def hand_state(sim):
    base=sim.env.unwrapped
    origin=parts._np(base.scene.env_origins[0]);clouds=[]
    if origin.shape!=(3,) or not np.isfinite(origin).all():raise ValueError('unknown origin')
    for arm in ('left_robot','right_robot'):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        if not np.isfinite(poses).all():raise ValueError('unknown hand poses')
        for name,p in hand_mesh().items():
            pose=poses[asset.body_names.index(name)]
            if np.linalg.norm(pose[3:])<1e-6:raise ValueError('unknown hand rotation')
            clouds.append(p@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin)
    return clouds


@lru_cache(maxsize=1)
def mesh_bounds():return {n:(p.min(0),p.max(0)) for n,p in hand_mesh().items()}

def near_hot_clouds(sim,centre,extent=.040):
    """Prune using transformed complete mesh AABBs before sampling hot-metal contacts."""
    base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);out=[]
    for arm in ('left_robot','right_robot'):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        if not np.isfinite(poses).all():raise ValueError('unknown native hand poses')
        for name,cloud in hand_mesh().items():
            pose=poses[asset.body_names.index(name)];r=parts._quat_to_matrix(pose[3:]);p=pose[:3]-origin
            low,high=mesh_bounds()[name];mid=p+r@((low+high)/2);half=np.abs(r)@((high-low)/2)
            if np.any(np.abs(mid-centre)>half+extent):continue
            out.append(cloud@r.T+p)
    return out

# Optional fixed inspection cameras supplement the native policy views;never alter them.
def install_views():
    import os,sys,pathlib
    outdir=os.environ.get('NOZZLE_VIEWS_DIR')
    if not outdir:return
    from dataclasses import fields
    def look(eye,target):
        eye,target=np.asarray(eye),np.asarray(target);z=(eye-target)/np.linalg.norm(eye-target)
        x=np.cross((0.,0.,1.),z);x/=np.linalg.norm(x);y=np.cross(z,x)
        return tuple(parts._matrix_to_quat(np.column_stack((x,y,z))))
    def patch(module):
        cls=module.BimanualYamEmbodiment
        if getattr(cls,'_nozzle_macro',False):return
        original=cls._camera_cfg
        def cfg(self,height,width,overview):
            from isaaclab.sensors import CameraCfg
            from isaaclab_arena.utils.cameras import ArenaCameraCfg
            from isaaclab_arena.utils.configclass import make_configclass
            from yam_arena import embodiment
            from yam_arena.rig import Camera,Lens
            from yam_arena.task_file import TABLE_TOP
            rig=original(self,height,width,overview)
            have=[(f.name,CameraCfg,getattr(rig,f.name)) for f in fields(rig) if isinstance(getattr(rig,f.name),CameraCfg)]
            z=TABLE_TOP['station']+.182;extra=[]
            for name,eye in [('hotend_front_cam',(.10,-.080,z-.045)),('hotend_back_cam',(.38,.080,z-.045))]:
                target=(.24,0.,z-.006);cam=Camera(eye,look(eye,target),'opengl',Lens(1500.,1500.,320.,180.,640,360))
                c=CameraCfg(prim_path='{ENV_REGEX_NS}/'+name.title().replace('_',''),height=720,width=1280,data_types=['rgb'],spawn=embodiment._pinhole(cam,720,1280,near=.01),offset=embodiment._offset(cam),renderer_cfg=embodiment.RENDERER)
                extra.append((name,CameraCfg,c))
            return make_configclass('NozzleInspectionCameras',have+extra,bases=(ArenaCameraCfg,))()
        cls._camera_cfg=cfg;cls._nozzle_macro=True
    parts._PATCHES['yam_arena.embodiment']=patch
    if 'yam_arena.embodiment' in sys.modules:patch(sys.modules['yam_arena.embodiment'])
    record={'trial':-1,'k':0}
    def save(sim):
        import cv2
        for name in ('hotend_front_cam','hotend_back_cam'):
            raw=sim.env.unwrapped.scene[name].data.output['rgb'];raw=raw.torch if hasattr(raw,'torch') else raw
            rgb=raw[0,...,:3].cpu().numpy();d=pathlib.Path(outdir)/('trial'+str(record['trial']))/name;d.mkdir(parents=True,exist_ok=True)
            cv2.imwrite(str(d/f'{record["k"]:05d}.jpg'),np.ascontiguousarray(rgb[...,::-1]),[cv2.IMWRITE_JPEG_QUALITY,90])
        record['k']+=1
    @parts.on_reset
    def new_trial(sim):record.update(trial=record['trial']+1,k=0)
    @parts.on_step
    def view_step(sim):save(sim)
