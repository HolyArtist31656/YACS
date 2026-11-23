"""Zero-dependency SE(3) 4x4 rigid-body transform math for CAD-to-AdvantageScope alignment.

All 4x4 matrices are stored as 16-element row-major lists matching Onshape's
``rootAssembly.occurrences[].transform`` convention.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

Mat4 = List[float]
Mat3 = List[List[float]]


def mat4_identity() -> Mat4:
    return [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]


def mat4_mul(a: Sequence[float], b: Sequence[float]) -> Mat4:
    out = [0.0] * 16
    for r in range(4):
        r4 = r * 4
        for c in range(4):
            out[r4 + c] = (
                a[r4 + 0] * b[0 + c]
                + a[r4 + 1] * b[4 + c]
                + a[r4 + 2] * b[8 + c]
                + a[r4 + 3] * b[12 + c]
            )
    return out


def mat4_invert_rigid(t: Sequence[float]) -> Mat4:
    """Exact inverse of a rigid-body SE(3) matrix [R t; 0 1] -> [R^T -R^T t; 0 1]."""
    r00, r01, r02, tx = t[0], t[1], t[2], t[3]
    r10, r11, r12, ty = t[4], t[5], t[6], t[7]
    r20, r21, r22, tz = t[8], t[9], t[10], t[11]
    itx = -(r00 * tx + r10 * ty + r20 * tz)
    ity = -(r01 * tx + r11 * ty + r21 * tz)
    itz = -(r02 * tx + r12 * ty + r22 * tz)
    return [
        r00, r10, r20, itx,
        r01, r11, r21, ity,
        r02, r12, r22, itz,
        0.0, 0.0, 0.0, 1.0,
    ]


def _normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def mat4_from_mated_cs(mated_cs: Dict[str, Any]) -> Mat4:
    """Builds a 4x4 SE(3) matrix from an Onshape matedCS dict (origin, xAxis, [yAxis], zAxis)."""
    orig = mated_cs.get("origin") or [0.0, 0.0, 0.0]
    x_raw = mated_cs.get("xAxis") or [1.0, 0.0, 0.0]
    z_raw = mated_cs.get("zAxis") or [0.0, 0.0, 1.0]
    z = _normalize((float(z_raw[0]), float(z_raw[1]), float(z_raw[2])))
    x = _normalize((float(x_raw[0]), float(x_raw[1]), float(x_raw[2])))
    y_raw = mated_cs.get("yAxis")
    if y_raw and len(y_raw) >= 3:
        y = _normalize((float(y_raw[0]), float(y_raw[1]), float(y_raw[2])))
    else:
        y = _normalize(_cross(z, x))
        x = _normalize(_cross(y, z))
    return [
        x[0], y[0], z[0], float(orig[0]),
        x[1], y[1], z[1], float(orig[1]),
        x[2], y[2], z[2], float(orig[2]),
        0.0,  0.0,  0.0,  1.0,
    ]


def mat4_translation(t: Sequence[float]) -> List[float]:
    return [round(float(t[3]), 6), round(float(t[7]), 6), round(float(t[11]), 6)]


def mat3_from_mat4(t: Sequence[float]) -> Mat3:
    return [
        [float(t[0]), float(t[1]), float(t[2])],
        [float(t[4]), float(t[5]), float(t[6])],
        [float(t[8]), float(t[9]), float(t[10])],
    ]


def mat3_mul(a: Mat3, b: Mat3) -> Mat3:
    return [
        [sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)]
        for r in range(3)
    ]


def rot_axis_deg(axis: str, degrees: float) -> Mat3:
    rad = math.radians(degrees)
    c, s = math.cos(rad), math.sin(rad)
    ax = axis.lower()
    if ax == "x":
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    if ax == "y":
        return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    if ax == "z":
        return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    raise ValueError(f"Invalid axis: {axis}")


def compose_advantagescope_rotations(rotations: Sequence[Dict[str, Any]]) -> Mat3:
    """Composes AdvantageScope ordered rotations (applied sequentially: R = R_k * ... * R_1)."""
    r: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for step in rotations:
        r = mat3_mul(rot_axis_deg(str(step["axis"]), float(step["degrees"])), r)
    return r


def mat3_to_advantagescope_rotations(r: Mat3, eps_deg: float = 1e-4) -> List[Dict[str, Any]]:
    """Decomposes a 3x3 rotation matrix R = Rz(rz) * Ry(ry) * Rx(rx) into AdvantageScope [x, y, z] degrees."""
    # For R = Rz(rz) * Ry(ry) * Rx(rx):
    #   R[2][0] = -sin(ry)
    #   R[2][1] = cos(ry) * sin(rx),  R[2][2] = cos(ry) * cos(rx)
    #   R[1][0] = cos(ry) * sin(rz),  R[0][0] = cos(ry) * cos(rz)
    sin_ry = max(-1.0, min(1.0, -r[2][0]))
    ry = math.asin(sin_ry)
    cos_ry = math.cos(ry)
    if abs(cos_ry) > 1e-6:
        rx = math.atan2(r[2][1], r[2][2])
        rz = math.atan2(r[1][0], r[0][0])
    else:
        # Gimbal lock: set rx = 0 and solve rz from R[0][1], R[1][1]
        rx = 0.0
        rz = math.atan2(-r[0][1], r[1][1])

    out: List[Dict[str, Any]] = []
    for axis, rad in (("x", rx), ("y", ry), ("z", rz)):
        deg = round(math.degrees(rad), 4)
        if abs(deg) > eps_deg:
            out.append({"axis": axis, "degrees": deg})
    return out


# Standard glTF Y-up -> WPILib NWU Z-up rotation: Rz(+90) * Rx(+90)
GLTF_YUP_TO_NWU_R: Mat3 = mat3_mul(rot_axis_deg("z", 90.0), rot_axis_deg("x", 90.0))


def compute_zeroed_transform(
    t_robot_to_joint: Sequence[float],
    t_robot_to_link: Sequence[float],
    include_gltf_yup_to_nwu: bool = True,
) -> Tuple[List[float], List[Dict[str, Any]]]:
    """Solves T_zero = T_robot_to_joint^-1 * T_robot_to_link for AdvantageScope config.json.

    Returns ``(zeroedPosition, zeroedRotations)`` such that at CAD rest:
        T_robot_to_joint * T_zero == T_robot_to_link
    """
    t_zero = mat4_mul(mat4_invert_rigid(t_robot_to_joint), t_robot_to_link)
    zeroed_pos = mat4_translation(t_zero)
    r_zero = mat3_from_mat4(t_zero)
    if include_gltf_yup_to_nwu:
        r_zero = mat3_mul(r_zero, GLTF_YUP_TO_NWU_R)
    zeroed_rots = mat3_to_advantagescope_rotations(r_zero)
    return (zeroed_pos, zeroed_rots)
