"""Original smooth native-envelope visuals over the unchanged physical collision surfaces."""
import math
import numpy as np

def glass(stage,root):
 from pxr import UsdGeom,UsdShade,Sdf,Gf
 path=root+'/glass_visual_material';material=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/surface');shader.CreateIdAttr('UsdPreviewSurface')
 shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set((.72,.87,.92));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.15);shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.20);shader.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.0)
 material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 # Render only the smooth lathe rather than the many overlapping physical rib surfaces.
 for name in ('density_bottle','perforated_stopper'):
  prim=stage.GetPrimAtPath(root+'/'+name)
  for p in list(prim.GetChildren()):
   if p.GetName().startswith('s') and p.IsA(UsdGeom.Gprim):UsdGeom.Imageable(p).MakeInvisible()
  if name=='density_bottle':
   profile=[(0.,.012),(.003,.012)]
   profile += [(float(z),float(math.sqrt(max(0.,.0254**2-(z-.0254)**2)))) for z in np.linspace(.004,.047,18)]
   profile += [(.049,.011),(.06985,.0117)]
  else:profile=[(0.,.0067),(.006,.0073),(.012,.0079),(.018,.0085),(.024,.0091),(.025,.0117),(.05175,.0117)]
  n=48;vertices=[(r*math.cos(a),r*math.sin(a),z) for z,r in profile for a in np.arange(n)*2*math.pi/n]
  counts=[];indices=[]
  for k in range(len(profile)-1):
   for j in range(n):counts.append(4);indices.extend((k*n+j,k*n+(j+1)%n,(k+1)*n+(j+1)%n,(k+1)*n+j))
  mesh=UsdGeom.Mesh.Define(stage,root+'/'+name+'/smooth_glass_visual');mesh.CreatePointsAttr(vertices);mesh.CreateFaceVertexCountsAttr(counts);mesh.CreateFaceVertexIndicesAttr(indices);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
  UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
