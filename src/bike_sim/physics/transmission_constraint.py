"""Full chain-geometry coordinate for a power-consistent ideal reduction.

The normal chain model and this reduction use the same selected tangent branch,
absolute sprocket rotations and moving sprocket centres. No fitted anti-squat
force is added. Scalar planar joints are required; the boundary is in metres.
"""
from math import atan2, pi
import numpy as np
import mujoco
from bike_sim.physics.chain import chain_geometry, chain_center_gradient


def linearized_upper_bound(phi_m, jacobian_m, q, boundary_m):
    j,q=np.asarray(jacobian_m,float),np.asarray(q,float)
    if j.ndim!=1 or j.shape!=q.shape:
        raise ValueError('transmission Jacobian requires scalar planar coordinates')
    if not (np.isfinite(j).all() and np.isfinite(q).all() and np.isfinite([phi_m,boundary_m]).all()):
        raise ValueError('non-finite transmission linearization')
    return j.copy(),float(boundary_m-phi_m+j@q)


def constraint_reaction(tension_n,jacobian_m):
    j=np.asarray(jacobian_m,float)
    if j.ndim!=1 or not np.isfinite(j).all() or not np.isfinite(tension_n) or tension_n<0:
        raise ValueError('invalid tensile transmission reaction')
    return -float(tension_n)*j


def _body_kinematics(model,data,name):
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
    if body<1:
        raise ValueError('transmission requires body '+name)
    jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
    mujoco.mj_jac(model,data,jp,jr,data.xpos[body],body)
    rotation=data.xmat[body].reshape(3,3)
    if not np.allclose(rotation[:,1],[0.,1.,0.],atol=1e-9,rtol=0):
        raise ValueError('geometric transmission requires planar sprocket frames')
    raw=atan2(-rotation[2,0],rotation[0,0])
    # For scalar planar joints the rotational Jacobian is constant. Its
    # accumulated coordinate fixes the unwrap branch for arbitrarily many
    # crank turns without temporal state or finite-difference side effects.
    reference=float(jr[1]@(data.qpos-model.qpos0))
    angle=reference+atan2(np.sin(raw-reference),np.cos(raw-reference))
    return body,jp,jr,float(angle)


def transmission_geometry(model,data,gearing):
    if model.nq!=model.nv or np.any(~np.isin(model.jnt_type,[mujoco.mjtJoint.mjJNT_SLIDE,mujoco.mjtJoint.mjJNT_HINGE])):
        raise ValueError('transmission geometry needs scalar planar coordinates')
    front,jp_f,jr_f,tf=_body_kinematics(model,data,'crank')
    rear,jp_r,jr_r,tr=_body_kinematics(model,data,'rear_wheel')
    frame,_,_,theta_frame=_body_kinematics(model,data,'frame')
    cf,cr=data.xpos[front][[0,2]],data.xpos[rear][[0,2]]
    rf,rr=gearing.front_radius_m,gearing.rear_radius_m
    up=data.xmat[frame].reshape(3,3)[[0,2],2]
    psi_reference=pi/2.-theta_frame
    length,psi=chain_geometry(cf,cr,rf,rr,up_xz=up,psi_reference=psi_reference)
    gradient=chain_center_gradient(cf,cr,rf,rr,up_xz=up,psi_reference=psi)
    jacobian=gradient@(jp_r[[0,2]]-jp_f[[0,2]])+rf*jr_f[1]-rr*jr_r[1]
    return float(length+rf*tf-rr*tr),jacobian
