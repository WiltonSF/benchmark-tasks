"""Exact oriented-box overlap audit, independent of native PhysX contacts (SI units).
Returns minimum translation distance separating two penetrating boxes, zero otherwise.
This guards native solver tunnelling; it does not generate forces or earn lint credit.
"""
import numpy as np

def box_penetrations(centres_a,rotations_a,halves_a,centres_b,halves_b):
 """Box B axes are identity; all inputs already in that common frame. Output shape (A,B)."""
 ca,ra,ha,cb,hb=map(np.asarray,(centres_a,rotations_a,halves_a,centres_b,halves_b))
 n=len(ca)
 axes_a=np.swapaxes(ra,-1,-2)
 axes_b=np.broadcast_to(np.eye(3),(n,3,3))
 cross=np.cross(axes_a[:,:,None,:],axes_b[:,None,:,:]).reshape(n,9,3)
 axes=np.concatenate([axes_a,axes_b,cross],axis=1)
 lengths=np.linalg.norm(axes,axis=-1)
 valid=lengths>1e-8
 axes=axes/np.maximum(lengths[:,:,None],1e-20)
 projected_a=np.einsum('nki,nij->nkj',axes,ra)
 radius_a=np.einsum('nkj,nj->nk',np.abs(projected_a),ha)
 radius_b=np.einsum('nki,bi->nbk',np.abs(axes),hb)
 distance=np.abs(np.einsum('nbi,nki->nbk',ca[:,None,:]-cb[None,:,:],axes))
 overlaps=radius_a[:,None,:]+radius_b-distance
 overlaps=np.where(valid[:,None,:],overlaps,np.inf)
 return np.maximum(0,np.min(overlaps,axis=-1))


def shaft_phone_penetrations(shaft_specs,phone_shapes,brush_state,phone_pose):
 """Native shaft boxes against every colliding phone box, expressed in the phone frame."""
 import parts
 pp,pr=phone_pose
 centres=[];rotations=[];halves=[];names=[]
 for j in shaft_specs:
  lp,lq=brush_state.links[j.name];lr=parts._quat_to_matrix(lq)
  for sh in j.shapes:
   if not sh.collide or sh.kind!='box':continue
   centres.append((np.asarray(lp)+lr@(np.asarray(sh.at)-j.origin)-pp)@pr)
   rotations.append(pr.T@lr@sh.R)
   halves.append(sh.half);names.append(j.name)
 boxes=[sh for sh in phone_shapes if sh.collide]
 if any(sh.kind!='box' or not np.allclose(sh.R,np.eye(3)) for sh in boxes):
  raise ValueError('phone penetration audit needs its axis-aligned frame boxes')
 hits=box_penetrations(np.array(centres),np.array(rotations),np.array(halves),np.array([x.at for x in boxes]),np.array([x.half for x in boxes]))
 return hits,names
