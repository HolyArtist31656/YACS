#!/usr/bin/env python3
"""Onshape / CAD-to-Field Exporter for YACSim, YAECE, and PathPlanner.

Single CAD source of truth producing:
  1. YAECE SeasonConfig JSON (both ANDYMARK and WELDED field dimension variants)
  2. PathPlanner deploy/pathplanner/navgrid.json (2D inflated occupancy grid)
  3. YACSim 3D Field OBB manifest (yacsim-field.json) for ProjectilePhysicsSim

Zero third-party Python dependencies required (pure Python 3.10+ stdlib).
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Verified 2026 REBUILT Andymark vs. Welded field dimensions (in meters)
FIELD_DIMENSIONS_METERS: dict[str, tuple[float, float]] = {
    "ANDYMARK": (16.513048, 8.042656),  # 54 ft + 2.12 in x 26 ft + 4.64 in
    "WELDED": (16.540480, 8.069580),    # 54 ft + 3.20 in x 26 ft + 5.70 in
}


@dataclass(frozen=True)
class FieldBox3d:
    name: str
    category: str  # OBSTACLE | NO_ENTRY | TRENCH | SCORING_TARGET
    min_xyz: tuple[float, float, float]
    max_xyz: tuple[float, float, float]
    scoring_target: bool = False


def inflate_rect_2d(
    min_x: float, min_y: float, max_x: float, max_y: float, margin: float
) -> list[list[float]]:
    """Returns a CCW 2D rectangle polygon inflated by `margin` meters on each side."""
    x0 = min_x - margin
    y0 = min_y - margin
    x1 = max_x + margin
    y1 = max_y + margin
    return [[round(x0, 4), round(y0, 4)], [round(x1, 4), round(y0, 4)],
            [round(x1, 4), round(y1, 4)], [round(x0, 4), round(y1, 4)]]


def intersects_height_band(box: FieldBox3d, z_min: float, z_max: float) -> bool:
    return box.max_xyz[2] >= z_min and box.min_xyz[2] <= z_max


def classify_part_name(name: str) -> tuple[str, bool]:
    upper = name.upper()
    if upper.startswith("SCORING_SITE_") or "TARGET" in upper or "HUB_OPENING" in upper:
        return ("SCORING_TARGET", True)
    if upper.startswith("NO_ENTRY_") or "RESTRICTED" in upper:
        return ("NO_ENTRY", False)
    if upper.startswith("TRENCH_") or "OVERHEAD" in upper:
        return ("TRENCH", False)
    return ("OBSTACLE", False)


def load_field_spec(spec_path: Path) -> tuple[str, list[FieldBox3d]]:
    """Loads a field geometry JSON spec (exported from Onshape or authored manually)."""
    raw = json.loads(spec_path.read_text(encoding="utf-8"))
    season_name = str(raw.get("seasonName", spec_path.stem))
    boxes: list[FieldBox3d] = []
    for item in raw.get("parts", []):
        name = str(item["name"])
        default_cat, default_target = classify_part_name(name)
        cat = str(item.get("category", default_cat)).upper()
        is_target = bool(item.get("scoringTarget", default_target or cat == "SCORING_TARGET"))
        mn = tuple(float(v) for v in item["minMeters"])
        mx = tuple(float(v) for v in item["maxMeters"])
        boxes.append(FieldBox3d(name=name, category=cat, min_xyz=mn, max_xyz=mx, scoring_target=is_target))
    return season_name, boxes


def build_yaece_season_config(
    season_name: str,
    field_type: str,
    boxes: list[FieldBox3d],
    chassis_z_band: tuple[float, float] = (0.05, 1.20),
    trench_z_band: tuple[float, float] = (0.55, 1.50),
    robot_radius_meters: float = 0.0,
) -> dict[str, Any]:
    length_m, width_m = FIELD_DIMENSIONS_METERS[field_type]
    static_obstacles = []
    no_entry_zones = []
    trench_zones = []
    scoring_sites = []

    for b in boxes:
        if b.scoring_target:
            cx = (b.min_xyz[0] + b.max_xyz[0]) * 0.5
            cy = (b.min_xyz[1] + b.max_xyz[1]) * 0.5
            cz = (b.min_xyz[2] + b.max_xyz[2]) * 0.5
            scoring_sites.append({
                "id": b.name,
                "targetMeters": [round(cx, 4), round(cy, 4), round(cz, 4)],
                "boundsMinMeters": [round(v, 4) for v in b.min_xyz],
                "boundsMaxMeters": [round(v, 4) for v in b.max_xyz],
            })
        elif b.category == "NO_ENTRY":
            no_entry_zones.append({
                "id": b.name,
                "polygonMeters": inflate_rect_2d(
                    b.min_xyz[0], b.min_xyz[1], b.max_xyz[0], b.max_xyz[1], robot_radius_meters
                ),
            })
        elif b.category == "TRENCH" and intersects_height_band(b, trench_z_band[0], trench_z_band[1]):
            trench_zones.append({
                "id": b.name,
                "clearanceHeightMeters": round(b.min_xyz[2], 4),
                "polygonMeters": inflate_rect_2d(
                    b.min_xyz[0], b.min_xyz[1], b.max_xyz[0], b.max_xyz[1], 0.0
                ),
            })
        elif intersects_height_band(b, chassis_z_band[0], chassis_z_band[1]):
            static_obstacles.append({
                "id": b.name,
                "polygonMeters": inflate_rect_2d(
                    b.min_xyz[0], b.min_xyz[1], b.max_xyz[0], b.max_xyz[1], robot_radius_meters
                ),
                "minZMeters": round(b.min_xyz[2], 4),
                "maxZMeters": round(b.max_xyz[2], 4),
            })

    return {
        "schemaVersion": "1.0",
        "seasonName": season_name,
        "fieldType": field_type,
        "fieldLengthMeters": round(length_m, 6),
        "fieldWidthMeters": round(width_m, 6),
        "staticObstacles": static_obstacles,
        "noEntryZones": no_entry_zones,
        "trenchZones": trench_zones,
        "scoringSites": scoring_sites,
    }


def build_pathplanner_navgrid(
    field_type: str,
    boxes: list[FieldBox3d],
    node_size_meters: float = 0.25,
    robot_radius_meters: float = 0.45,
    chassis_z_band: tuple[float, float] = (0.05, 1.20),
) -> dict[str, Any]:
    length_m, width_m = FIELD_DIMENSIONS_METERS[field_type]
    cols = int(math.ceil(length_m / node_size_meters))
    rows = int(math.ceil(width_m / node_size_meters))

    blocking = [
        b for b in boxes
        if not b.scoring_target
        and b.category in {"OBSTACLE", "NO_ENTRY"}
        and intersects_height_band(b, chassis_z_band[0], chassis_z_band[1])
    ]

    grid: list[list[bool]] = []
    for r in range(rows):
        cy = (r + 0.5) * node_size_meters
        row_cells: list[bool] = []
        for c in range(cols):
            cx = (c + 0.5) * node_size_meters
            blocked = False
            for b in blocking:
                if (
                    b.min_xyz[0] - robot_radius_meters <= cx <= b.max_xyz[0] + robot_radius_meters
                    and b.min_xyz[1] - robot_radius_meters <= cy <= b.max_xyz[1] + robot_radius_meters
                ):
                    blocked = True
                    break
            row_cells.append(blocked)
        grid.append(row_cells)

    return {
        "field_size": {"x": round(length_m, 4), "y": round(width_m, 4)},
        "nodeSizeMeters": node_size_meters,
        "grid": grid,
    }


def build_yacsim_field_manifest(season_name: str, boxes: list[FieldBox3d]) -> dict[str, Any]:
    return {
        "schemaVersion": "1.0",
        "seasonName": season_name,
        "obstacles": [
            {
                "name": b.name,
                "category": b.category,
                "scoringTarget": b.scoring_target,
                "minMeters": [round(v, 4) for v in b.min_xyz],
                "maxMeters": [round(v, 4) for v in b.max_xyz],
                "poseMeters": [0.0, 0.0, 0.0],
                "rpyRadians": [0.0, 0.0, 0.0],
            }
            for b in boxes
        ],
    }


def export_all(
    spec_path: Path,
    out_dir: Path,
    node_size_meters: float = 0.25,
    robot_radius_meters: float = 0.45,
) -> dict[str, Path]:
    season_name, boxes = load_field_spec(spec_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Path] = {}

    for field_type in ("ANDYMARK", "WELDED"):
        season_cfg = build_yaece_season_config(
            season_name, field_type, boxes, robot_radius_meters=0.0
        )
        p_season = out_dir / f"yaece-season-{field_type.lower()}.json"
        p_season.write_text(json.dumps(season_cfg, indent=2) + "\n", encoding="utf-8")
        outputs[f"yaece_{field_type.lower()}"] = p_season

    navgrid = build_pathplanner_navgrid(
        "ANDYMARK", boxes, node_size_meters=node_size_meters, robot_radius_meters=robot_radius_meters
    )
    p_nav = out_dir / "navgrid.json"
    p_nav.write_text(json.dumps(navgrid, indent=2) + "\n", encoding="utf-8")
    outputs["navgrid"] = p_nav

    yacsim_field = build_yacsim_field_manifest(season_name, boxes)
    p_field = out_dir / "yacsim-field.json"
    p_field.write_text(json.dumps(yacsim_field, indent=2) + "\n", encoding="utf-8")
    outputs["yacsim_field"] = p_field
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export Onshape/CAD field geometry to YAECE SeasonConfig, PathPlanner navgrid.json, and YACSim 3D field OBBs."
    )
    parser.add_argument("--spec", required=True, type=Path, help="Input field geometry JSON spec.")
    parser.add_argument("--out", required=True, type=Path, help="Output directory.")
    parser.add_argument("--node-size", type=float, default=0.25, help="PathPlanner navgrid cell size (m).")
    parser.add_argument("--robot-radius", type=float, default=0.45, help="Robot inflation radius for navgrid (m).")
    args = parser.parse_args()
    outputs = export_all(args.spec, args.out, args.node_size, args.robot_radius)
    for k, v in outputs.items():
        print(f"Wrote {k}: {v}")


if __name__ == "__main__":
    main()
