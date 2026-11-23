"""Pure-Python GLB 2.0 generator for visually striped spinning roller/wheel meshes.

Generates lightweight, AdvantageScope/Three.js-compatible binary glTF 2.0 (.glb)
cylinders with exact POSITION + NORMAL vertex attributes and high-contrast alternating
stripes so rotation is unmistakably visible in AdvantageScope 3D Field.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import List, Tuple


def generate_striped_roller_glb(
    output_path: Path | str,
    radius_meters: float,
    width_meters: float = 0.28,
    axis: str = "Y",
    segments: int = 24,
    primary_rgba: Tuple[float, float, float, float] = (0.12, 0.72, 0.95, 1.0),
    stripe_rgba: Tuple[float, float, float, float] = (0.98, 0.82, 0.12, 1.0),
) -> Path:
    """Writes a valid GLB 2.0 binary file with POSITION and NORMAL attributes."""
    if radius_meters <= 0.0:
        raise ValueError(f"radius_meters must be positive, got {radius_meters}")
    if width_meters <= 0.0:
        raise ValueError(f"width_meters must be positive, got {width_meters}")
    segments = max(8, int(segments))
    if segments % 2 != 0:
        segments += 1

    axis_u = axis.strip().upper()
    if axis_u not in ("X", "Y", "Z"):
        raise ValueError(f"axis must be X, Y, or Z, got {axis}")

    def to_axis_point(radial_a: float, radial_b: float, axial: float) -> Tuple[float, float, float]:
        if axis_u == "Y":
            return (radial_a, axial, radial_b)
        if axis_u == "X":
            return (axial, radial_a, radial_b)
        return (radial_a, radial_b, axial)

    half_w = width_meters * 0.5
    verts: List[List[Tuple[float, float, float]]] = [[], []]
    norms: List[List[Tuple[float, float, float]]] = [[], []]
    indices: List[List[int]] = [[], []]

    def add_tri(
        mat_idx: int,
        p0: Tuple[float, float, float],
        p1: Tuple[float, float, float],
        p2: Tuple[float, float, float],
        n0: Tuple[float, float, float],
        n1: Tuple[float, float, float],
        n2: Tuple[float, float, float],
    ) -> None:
        base = len(verts[mat_idx])
        verts[mat_idx].extend([p0, p1, p2])
        norms[mat_idx].extend([n0, n1, n2])
        indices[mat_idx].extend([base, base + 1, base + 2])

    c_neg = to_axis_point(0.0, 0.0, -half_w)
    c_pos = to_axis_point(0.0, 0.0, half_w)
    n_cap_pos = to_axis_point(0.0, 0.0, 1.0)
    n_cap_neg = to_axis_point(0.0, 0.0, -1.0)

    for i in range(segments):
        a0 = (2.0 * math.pi * i) / segments
        a1 = (2.0 * math.pi * (i + 1)) / segments
        ca0, sa0 = math.cos(a0), math.sin(a0)
        ca1, sa1 = math.cos(a1), math.sin(a1)
        r0a, r0b = radius_meters * ca0, radius_meters * sa0
        r1a, r1b = radius_meters * ca1, radius_meters * sa1

        p0_neg = to_axis_point(r0a, r0b, -half_w)
        p1_neg = to_axis_point(r1a, r1b, -half_w)
        p0_pos = to_axis_point(r0a, r0b, half_w)
        p1_pos = to_axis_point(r1a, r1b, half_w)
        n0 = to_axis_point(ca0, sa0, 0.0)
        n1 = to_axis_point(ca1, sa1, 0.0)

        mat_idx = (i // max(1, segments // 8)) % 2

        # Outer barrel quad (2 triangles with outward radial normals)
        add_tri(mat_idx, p0_neg, p1_neg, p1_pos, n0, n1, n1)
        add_tri(mat_idx, p0_neg, p1_pos, p0_pos, n0, n1, n0)
        # End caps with outward axial normals
        add_tri(mat_idx, c_pos, p0_pos, p1_pos, n_cap_pos, n_cap_pos, n_cap_pos)
        add_tri(mat_idx, c_neg, p1_neg, p0_neg, n_cap_neg, n_cap_neg, n_cap_neg)

    bin_chunks = bytearray()
    buffer_views = []
    accessors = []
    primitives = []

    for mat_idx in (0, 1):
        v_list = verts[mat_idx]
        n_list = norms[mat_idx]
        i_list = indices[mat_idx]

        # POSITION
        pos_offset = len(bin_chunks)
        xs = [p[0] for p in v_list]
        ys = [p[1] for p in v_list]
        zs = [p[2] for p in v_list]
        for x, y, z in v_list:
            bin_chunks.extend(struct.pack("<fff", x, y, z))
        pos_bv_idx = len(buffer_views)
        buffer_views.append({
            "buffer": 0,
            "byteOffset": pos_offset,
            "byteLength": len(bin_chunks) - pos_offset,
            "target": 34962,
        })
        pos_acc_idx = len(accessors)
        accessors.append({
            "bufferView": pos_bv_idx,
            "componentType": 5126,
            "count": len(v_list),
            "type": "VEC3",
            "min": [min(xs), min(ys), min(zs)],
            "max": [max(xs), max(ys), max(zs)],
        })

        # NORMAL
        norm_offset = len(bin_chunks)
        for nx, ny, nz in n_list:
            bin_chunks.extend(struct.pack("<fff", nx, ny, nz))
        norm_bv_idx = len(buffer_views)
        buffer_views.append({
            "buffer": 0,
            "byteOffset": norm_offset,
            "byteLength": len(bin_chunks) - norm_offset,
            "target": 34962,
        })
        norm_acc_idx = len(accessors)
        accessors.append({
            "bufferView": norm_bv_idx,
            "componentType": 5126,
            "count": len(n_list),
            "type": "VEC3",
        })

        # INDICES
        idx_offset = len(bin_chunks)
        for idx in i_list:
            bin_chunks.extend(struct.pack("<H", idx))
        idx_len = len(bin_chunks) - idx_offset
        while len(bin_chunks) % 4 != 0:
            bin_chunks.append(0)
        idx_bv_idx = len(buffer_views)
        buffer_views.append({
            "buffer": 0,
            "byteOffset": idx_offset,
            "byteLength": idx_len,
            "target": 34963,
        })
        idx_acc_idx = len(accessors)
        accessors.append({
            "bufferView": idx_bv_idx,
            "componentType": 5123,
            "count": len(i_list),
            "type": "SCALAR",
            "min": [0],
            "max": [len(v_list) - 1],
        })

        primitives.append({
            "attributes": {"POSITION": pos_acc_idx, "NORMAL": norm_acc_idx},
            "indices": idx_acc_idx,
            "material": mat_idx,
        })

    gltf = {
        "asset": {"version": "2.0", "generator": "YACSim Onshape Exporter"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": Path(output_path).stem}],
        "meshes": [{"name": Path(output_path).stem, "primitives": primitives}],
        "materials": [
            {
                "name": "RollerPrimary",
                "doubleSided": True,
                "emissiveFactor": [primary_rgba[0] * 0.35, primary_rgba[1] * 0.35, primary_rgba[2] * 0.35],
                "pbrMetallicRoughness": {
                    "baseColorFactor": list(primary_rgba),
                    "metallicFactor": 0.1,
                    "roughnessFactor": 0.5,
                },
            },
            {
                "name": "RollerStripe",
                "doubleSided": True,
                "emissiveFactor": [stripe_rgba[0] * 0.35, stripe_rgba[1] * 0.35, stripe_rgba[2] * 0.35],
                "pbrMetallicRoughness": {
                    "baseColorFactor": list(stripe_rgba),
                    "metallicFactor": 0.1,
                    "roughnessFactor": 0.45,
                },
            },
        ],
        "buffers": [{"byteLength": len(bin_chunks)}],
        "bufferViews": buffer_views,
        "accessors": accessors,
    }

    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    while len(json_bytes) % 4 != 0:
        json_bytes += b" "

    total_length = 12 + 8 + len(json_bytes) + 8 + len(bin_chunks)
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total_length))
        f.write(struct.pack("<II", len(json_bytes), 0x4E4F534A))
        f.write(json_bytes)
        f.write(struct.pack("<II", len(bin_chunks), 0x004E4942))
        f.write(bin_chunks)
    return out
