"""Scalar and small 3D helpers (spec sections 2 and 3). No dependencies."""

from __future__ import annotations

import math

TAU = 2.0 * math.pi

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


def wrap(a: float) -> float:
    """Map an angle to (-pi, pi]."""
    return a - TAU * math.ceil((a - math.pi) / TAU)


def clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def softplus(z: float) -> float:
    return z + math.log1p(math.exp(-z)) if z > 0 else math.log1p(math.exp(z))


def sign(x: float) -> float:
    return 1.0 if x >= 0 else -1.0


IDENTITY: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
ZERO: Vec3 = (0.0, 0.0, 0.0)


def rot_x(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rot_z(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def mat_vec(a: Mat3, v: Vec3) -> Vec3:
    return (
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    )


def mat_t_vec(a: Mat3, v: Vec3) -> Vec3:
    """transpose(a) @ v, i.e. express a world vector in the frame whose rotation is a."""
    return (
        a[0][0] * v[0] + a[1][0] * v[1] + a[2][0] * v[2],
        a[0][1] * v[0] + a[1][1] * v[1] + a[2][1] * v[2],
        a[0][2] * v[0] + a[1][2] * v[1] + a[2][2] * v[2],
    )


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def angle_between(a: Vec3, b: Vec3) -> float:
    """atan2(|a x b|, a . b): accurate for small and large angles alike."""
    cx = a[1] * b[2] - a[2] * b[1]
    cy = a[2] * b[0] - a[0] * b[2]
    cz = a[0] * b[1] - a[1] * b[0]
    return math.atan2(math.sqrt(cx * cx + cy * cy + cz * cz),
                      a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def rpy_matrix(rpy: Vec3) -> Mat3:
    """URDF fixed-axis roll-pitch-yaw: Rz(y) Ry(p) Rx(r)."""
    return mat_mul(rot_z(rpy[2]), mat_mul(rot_y(rpy[1]), rot_x(rpy[0])))


def joint_matrix(yaw: float, pitch: float) -> Mat3:
    """Rz(yaw) Ry(-pitch): positive pitch tilts +x up (spec 3.2)."""
    return mat_mul(rot_z(yaw), rot_y(-pitch))


def direction_angles(d: Vec3) -> tuple[float, float]:
    """(yaw, pitch) of a direction (spec 3.5)."""
    return math.atan2(d[1], d[0]), math.atan2(d[2], math.hypot(d[0], d[1]))


class Frame:
    """A rigid transform: rotation r and origin p, both expressed in M."""

    __slots__ = ("p", "r")

    def __init__(self, r: Mat3 = IDENTITY, p: Vec3 = ZERO) -> None:
        self.r = r
        self.p = p

    def then(self, r: Mat3, p: Vec3 = ZERO) -> Frame:
        """Compose with a local transform (r, p) expressed in this frame."""
        return Frame(mat_mul(self.r, r), add(self.p, mat_vec(self.r, p)))

    def to_local(self, v: Vec3) -> Vec3:
        """Express a direction given in M in this frame."""
        return mat_t_vec(self.r, v)

    def facing(self) -> Vec3:
        """This frame's +x axis in M."""
        return (self.r[0][0], self.r[1][0], self.r[2][0])
