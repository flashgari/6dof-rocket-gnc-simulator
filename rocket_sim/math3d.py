"""Small vector and quaternion helpers."""

from __future__ import annotations

import math
from typing import Iterable

Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

def v_add(a: Vector, b: Vector) -> Vector: return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def v_sub(a: Vector, b: Vector) -> Vector: return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def v_scale(s: float, a: Vector) -> Vector: return (s*a[0], s*a[1], s*a[2])
def v_neg(a: Vector) -> Vector: return (-a[0], -a[1], -a[2])
def v_clamp_norm(a: Vector, max_norm: float) -> Vector:
    mag=norm(a)
    if max_norm <= 0.0: return (0.0,0.0,0.0)
    return a if mag <= max_norm or mag == 0.0 else v_scale(max_norm/mag,a)
def dot(a: Vector, b: Vector) -> float: return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def cross(a: Vector, b: Vector) -> Vector:
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def norm(a: Vector) -> float: return math.sqrt(dot(a,a))
def unit(a: Vector) -> Vector:
    mag=norm(a)
    return (0.0,0.0,0.0) if mag==0.0 else v_scale(1.0/mag,a)
def q_mul(q: Quaternion, r: Quaternion) -> Quaternion:
    w0,x0,y0,z0=q; w1,x1,y1,z1=r
    return (w0*w1-x0*x1-y0*y1-z0*z1, w0*x1+x0*w1+y0*z1-z0*y1, w0*y1-x0*z1+y0*w1+z0*x1, w0*z1+x0*y1-y0*x1+z0*w1)
def q_conj(q: Quaternion) -> Quaternion: return (q[0],-q[1],-q[2],-q[3])
def q_norm(q: Quaternion) -> float: return math.sqrt(sum(c*c for c in q))
def q_normalize(q: Quaternion) -> Quaternion:
    mag=q_norm(q)
    if mag==0.0: raise ValueError('Cannot normalize a zero quaternion.')
    return tuple(c/mag for c in q)  # type: ignore[return-value]
def q_derivative(q_body_to_inertial: Quaternion, omega_body: Vector) -> Quaternion:
    return tuple(0.5*x for x in q_mul(q_body_to_inertial,(0.0,*omega_body)))  # type: ignore[return-value]
def rotate_body_to_inertial(q_body_to_inertial: Quaternion, vector_body: Vector) -> Vector:
    r=q_mul(q_mul(q_body_to_inertial,(0.0,*vector_body)),q_conj(q_body_to_inertial)); return (r[1],r[2],r[3])
def rotate_inertial_to_body(q_body_to_inertial: Quaternion, vector_inertial: Vector) -> Vector:
    r=q_mul(q_mul(q_conj(q_body_to_inertial),(0.0,*vector_inertial)),q_body_to_inertial); return (r[1],r[2],r[3])
def weighted_sum(weights: Iterable[float], vectors: Iterable[tuple[float,...]]) -> tuple[float,...]:
    vs=list(vectors); ws=list(weights)
    return tuple(sum(w*v[i] for w,v in zip(ws,vs)) for i in range(len(vs[0])))
