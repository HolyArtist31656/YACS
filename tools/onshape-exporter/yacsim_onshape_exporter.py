#!/usr/bin/env python3
"""YACSim Phase 2: Zero-dependency Onshape-to-YACSim CAD Exporter.

Pulls assembly hierarchy, 4x4 occurrence transforms, mate definitions, exact CAD
mass/inertia tensors, bounding boxes, roller/wheel radii, and per-subassembly
glTF/GLB meshes from an Onshape assembly URL via Onshape REST API v6.

Automatically computes AdvantageScope ``zeroedPosition`` and ``zeroedRotations``
for every articulated link using:
    T_zero = (T_robot_to_joint)^-1 * T_robot_to_link
with zero hand-measured anchor points.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from roller_glb_generator import generate_striped_roller_glb
from se3_math import (
    compute_zeroed_transform,
    mat3_from_mat4,
    mat3_to_advantagescope_rotations,
    mat4_from_mated_cs,
    mat4_identity,
    mat4_invert_rigid,
    mat4_mul,
    mat4_translation,
    GLTF_YUP_TO_NWU_R,
    mat3_mul,
)

ONSHAPE_URL_RE = re.compile(
    r"https?://(?P<host>[^/]+)/documents/(?P<did>[0-9a-fA-F]+)/(?P<wvm>[wvm])/(?P<wvmid>[0-9a-fA-F]+)/e/(?P<eid>[0-9a-fA-F]+)"
)

ROLLER_KEYWORDS = ("roller", "wheel", "flywheel", "kicker", "spinner", "intake_bar", "shooter")
ORIGIN_CONNECTOR_NAMES = ("yacsim_origin", "origin_cube", "robot_origin")

ROLLER_PALETTE: List[Tuple[Tuple[float, float, float, float], Tuple[float, float, float, float]]] = [
    ((0.92, 0.25, 0.20, 1.0), (0.98, 0.92, 0.20, 1.0)),  # Red / Yellow (Flywheel)
    ((0.15, 0.65, 0.95, 1.0), (0.95, 0.95, 0.95, 1.0)),  # Blue / White (KickerFront)
    ((0.20, 0.80, 0.45, 1.0), (0.95, 0.95, 0.95, 1.0)),  # Green / White (KickerBack)
    ((0.95, 0.55, 0.12, 1.0), (0.15, 0.15, 0.18, 1.0)),  # Orange / Charcoal (HopperRoller)
    ((0.68, 0.28, 0.92, 1.0), (0.98, 0.88, 0.22, 1.0)),  # Purple / Gold (IntakeRoller)
]


@dataclass(frozen=True)
class OnshapeDocumentRef:
    host: str
    did: str
    wvm: str
    wvmid: str
    eid: str

    @property
    def base_url(self) -> str:
        return f"https://{self.host}"

    @property
    def web_url(self) -> str:
        return f"https://{self.host}/documents/{self.did}/{self.wvm}/{self.wvmid}/e/{self.eid}"


def parse_onshape_url(url: str) -> OnshapeDocumentRef:
    m = ONSHAPE_URL_RE.search(url.strip())
    if not m:
        raise ValueError(
            f"Unrecognized Onshape URL: {url!r}. Expected "
            "https://cad.onshape.com/documents/<did>/<w|v|m>/<wvmid>/e/<eid>"
        )
    return OnshapeDocumentRef(
        host=m.group("host"),
        did=m.group("did"),
        wvm=m.group("wvm"),
        wvmid=m.group("wvmid"),
        eid=m.group("eid"),
    )


class OnshapeRestClient:
    """Minimal stdlib-only Onshape REST API v6 client using HTTP Basic Auth."""

    def __init__(self, ref: OnshapeDocumentRef, access_key: str, secret_key: str, timeout: float = 60.0):
        self.ref = ref
        self.timeout = timeout
        token = base64.b64encode(f"{access_key}:{secret_key}".encode("utf-8")).decode("ascii")
        self._auth_header = f"Basic {token}"

    def _request(
        self,
        method: str,
        path: str,
        query: Optional[Dict[str, str]] = None,
        body: Optional[Dict[str, Any]] = None,
        accept: str = "application/json",
    ) -> bytes:
        url = f"{self.ref.base_url}{path}"
        if query:
            url += "?" + urllib.parse.urlencode(query)
        headers = {"Authorization": self._auth_header, "Accept": accept}
        data = None
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.read()
        except urllib.error.HTTPError as e:
            detail = e.read().decode("utf-8", errors="replace")[:500]
            raise RuntimeError(f"Onshape API {method} {path} failed ({e.code}): {detail}") from e

    def get_assembly_definition(self) -> Dict[str, Any]:
        path = f"/api/v6/assemblies/d/{self.ref.did}/{self.ref.wvm}/{self.ref.wvmid}/e/{self.ref.eid}"
        raw = self._request(
            "GET",
            path,
            query={
                "includeMateFeatures": "true",
                "includeMateConnectors": "true",
                "includeNonSolids": "false",
            },
        )
        return json.loads(raw.decode("utf-8"))

    def get_assembly_mass_properties(self) -> Dict[str, Any]:
        path = f"/api/v6/assemblies/d/{self.ref.did}/{self.ref.wvm}/{self.ref.wvmid}/e/{self.ref.eid}/massproperties"
        return json.loads(self._request("GET", path).decode("utf-8"))

    def get_assembly_bounding_boxes(self) -> Dict[str, Any]:
        path = f"/api/v6/assemblies/d/{self.ref.did}/{self.ref.wvm}/{self.ref.wvmid}/e/{self.ref.eid}/boundingboxes"
        return json.loads(self._request("GET", path).decode("utf-8"))

    def export_assembly_gltf(
        self,
        output_path: Path,
        did: Optional[str] = None,
        wvm: Optional[str] = None,
        wvmid: Optional[str] = None,
        eid: Optional[str] = None,
        resolution: str = "medium",
        poll_interval_sec: float = 2.0,
        max_wait_sec: float = 300.0,
    ) -> Path:
        """Runs Onshape async GLTF/GLB translation for the root or any subassembly."""
        d = did or self.ref.did
        t_wvm = wvm or self.ref.wvm
        t_wvmid = wvmid or self.ref.wvmid
        e = eid or self.ref.eid
        trans_path = f"/api/v6/assemblies/d/{d}/{t_wvm}/{t_wvmid}/e/{e}/translations"
        payload = {
            "formatName": "GLTF",
            "storeInDocument": False,
            "resolution": resolution,
            "flattenAssemblies": False,
            "includeExportIds": True,
        }
        job = json.loads(self._request("POST", trans_path, body=payload).decode("utf-8"))
        job_id = job["id"]

        deadline = time.monotonic() + max_wait_sec
        status = job
        while status.get("requestState") in ("ACTIVE", "QUEUED", None):
            if time.monotonic() > deadline:
                raise TimeoutError(f"Onshape GLTF translation {job_id} timed out after {max_wait_sec}s.")
            time.sleep(poll_interval_sec)
            status = json.loads(self._request("GET", f"/api/v6/translations/{job_id}").decode("utf-8"))

        if status.get("requestState") != "DONE":
            raise RuntimeError(f"Onshape GLTF translation failed: {status.get('failureReason', status)}")

        ext_ids = status.get("resultExternalDataIds") or []
        if not ext_ids:
            raise RuntimeError("Onshape translation completed with no resultExternalDataIds.")

        blob = self._request(
            "GET",
            f"/api/v6/documents/d/{d}/externaldata/{ext_ids[0]}",
            accept="application/octet-stream",
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(blob)
        return output_path


def _dominant_axis(vec: List[float]) -> Tuple[str, bool]:
    ax, ay, az = abs(vec[0]), abs(vec[1]), abs(vec[2])
    if ax >= ay and ax >= az:
        return ("X", vec[0] < 0.0)
    if ay >= ax and ay >= az:
        return ("Y", vec[1] < 0.0)
    return ("Z", vec[2] < 0.0)


def _is_roller_name(name: str) -> bool:
    low = name.lower()
    return any(kw in low for kw in ROLLER_KEYWORDS)


def _estimate_roller_radius_from_bbox(bbox: Dict[str, Any], axis: str) -> Optional[float]:
    low = bbox.get("lowX"), bbox.get("lowY"), bbox.get("lowZ")
    high = bbox.get("highX"), bbox.get("highY"), bbox.get("highZ")
    if any(v is None for v in (*low, *high)):
        return None
    dx = abs(float(high[0]) - float(low[0]))
    dy = abs(float(high[1]) - float(low[1]))
    dz = abs(float(high[2]) - float(low[2]))
    if axis == "X":
        diam = 0.5 * (dy + dz)
    elif axis == "Y":
        diam = 0.5 * (dx + dz)
    else:
        diam = 0.5 * (dx + dy)
    return round(diam * 0.5, 6) if diam > 1e-4 else None


def _extract_mass_props(raw_mp: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not raw_mp:
        return None
    masses = raw_mp.get("mass") or []
    mass_kg = float(masses[0]) if masses else 0.0
    if mass_kg <= 0.0:
        return None
    volumes = raw_mp.get("volume") or [0.0]
    centroid = raw_mp.get("centroid") or [0.0, 0.0, 0.0]
    inertia = raw_mp.get("inertia") or [0.0] * 9
    ixx = abs(float(inertia[0])) if len(inertia) >= 9 else 0.0
    iyy = abs(float(inertia[4])) if len(inertia) >= 9 else 0.0
    izz = abs(float(inertia[8])) if len(inertia) >= 9 else 0.0
    return {
        "volumeCubicMeters": round(float(volumes[0]), 6),
        "massKilograms": round(mass_kg, 6),
        "centerOfMassMeters": [round(float(c), 6) for c in centroid[:3]],
        "ixxKgM2": round(ixx, 6),
        "iyyKgM2": round(iyy, 6),
        "izzKgM2": round(izz, 6),
    }


def _find_robot_origin_transform(root_asm: Dict[str, Any]) -> List[float]:
    """Finds optional YACSIM_ORIGIN / ORIGIN_CUBE mate connector and returns T_cad_to_robot."""
    for feat in root_asm.get("features", []):
        if feat.get("featureType") != "mateConnector":
            continue
        fd = feat.get("featureData", {})
        name = (fd.get("name") or "").strip().lower()
        if any(tag in name for tag in ORIGIN_CONNECTOR_NAMES):
            cs = fd.get("mateConnectorCS") or fd.get("matedCS")
            if cs:
                return mat4_invert_rigid(mat4_from_mated_cs(cs))
    return mat4_identity()


def build_manifest_from_onshape_payloads(
    robot_name: str,
    source_url: str,
    assembly_def: Dict[str, Any],
    mass_props: Optional[Dict[str, Any]] = None,
    bboxes: Optional[Dict[str, Any]] = None,
    generate_roller_meshes: bool = True,
    output_dir: Optional[Path] = None,
    subassembly_export_plan: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Transforms Onshape Assembly Definition + Mass Properties + Bounding Boxes into yacsim-robot.json.

    Automatically solves each link's exact ``zeroedPosition`` and ``zeroedRotations`` via
    ``T_zero = (T_robot_to_joint)^-1 * T_robot_to_link`` from Onshape's 4x4 occurrence
    transforms and mate coordinate systems—requiring zero manual anchor measurements.
    """
    root_asm = assembly_def.get("rootAssembly", {})
    instances = {inst["id"]: inst for inst in root_asm.get("instances", []) if "id" in inst}
    occurrences: Dict[Tuple[str, ...], List[float]] = {}
    occ_by_leaf: Dict[str, List[float]] = {}
    for occ in root_asm.get("occurrences", []):
        path = tuple(occ.get("path") or [])
        tf = occ.get("transform")
        if path and tf and len(tf) == 16:
            tf_f = [float(v) for v in tf]
            occurrences[path] = tf_f
            occ_by_leaf[path[-1]] = tf_f

    t_cad_to_robot = _find_robot_origin_transform(root_asm)

    # Automatic chassis Z-grounding from assembly bounding box if available
    chassis_zero_pos = mat4_translation(t_cad_to_robot)
    if bboxes and bboxes.get("lowZ") is not None:
        chassis_zero_pos[2] = round(chassis_zero_pos[2] - float(bboxes["lowZ"]), 6)
    chassis_zero_rots = mat3_to_advantagescope_rotations(
        mat3_mul(mat3_from_mat4(t_cad_to_robot), GLTF_YUP_TO_NWU_R)
    )

    chassis_mp = _extract_mass_props(mass_props)
    chassis_entry: Dict[str, Any] = {
        "name": "Chassis",
        "meshFile": "model.glb",
        "zeroedPosition": chassis_zero_pos,
        "zeroedRotations": chassis_zero_rots,
        "collisionEnabled": True,
    }
    if chassis_mp:
        chassis_entry["massProperties"] = chassis_mp

    links: List[Dict[str, Any]] = []
    joints: List[Dict[str, Any]] = []
    seen_links = {"Chassis"}
    next_model_idx = 0
    roller_color_idx = 0

    for feat in root_asm.get("features", []):
        if feat.get("featureType") != "mate":
            continue
        fd = feat.get("featureData", {})
        mate_name = (fd.get("name") or feat.get("id") or f"Joint_{next_model_idx}").strip()
        mate_type = (fd.get("mateType") or "FASTENED").upper()
        if mate_type == "FASTENED":
            continue

        mated = fd.get("matedEntities") or []
        if len(mated) < 2:
            continue

        parent_occ_path = tuple(mated[0].get("matedOccurrence") or [])
        child_occ_path = tuple(mated[1].get("matedOccurrence") or [])
        parent_inst = instances.get(parent_occ_path[-1], {}) if parent_occ_path else {}
        child_inst = instances.get(child_occ_path[-1], {}) if child_occ_path else {}

        parent_name = parent_inst.get("name", "Chassis")
        if parent_name not in seen_links:
            parent_name = "Chassis"
        child_name = re.sub(r"[^A-Za-z0-9_]+", "_", mate_name).strip("_") or f"Link_{next_model_idx}"

        # Resolve 4x4 occurrence transforms in robot frame
        t_parent_occ = occurrences.get(
            parent_occ_path, occ_by_leaf.get(parent_occ_path[-1], mat4_identity()) if parent_occ_path else mat4_identity()
        )
        t_child_occ = occurrences.get(
            child_occ_path, occ_by_leaf.get(child_occ_path[-1], mat4_identity()) if child_occ_path else mat4_identity()
        )
        t_robot_to_parent = mat4_mul(t_cad_to_robot, t_parent_occ)
        t_robot_to_link = mat4_mul(t_cad_to_robot, t_child_occ)

        # Mate CS is defined in parent occurrence frame -> transform into robot frame
        mated_cs = mated[0].get("matedCS") or {}
        t_parent_to_joint = mat4_from_mated_cs(mated_cs)
        t_robot_to_joint = mat4_mul(t_robot_to_parent, t_parent_to_joint)

        origin = mat4_translation(t_robot_to_joint)
        z_axis_robot = [t_robot_to_joint[2], t_robot_to_joint[6], t_robot_to_joint[10]]
        axis_name, inverted = _dominant_axis(z_axis_robot)

        min_lim = fd.get("minLimit")
        max_lim = fd.get("maxLimit")

        is_roller = _is_roller_name(mate_name) or _is_roller_name(child_inst.get("name", ""))
        if mate_type == "SLIDER":
            j_type = "prismatic"
        elif is_roller or (mate_type in ("REVOLUTE", "CYLINDRICAL") and min_lim is None and max_lim is None):
            j_type = "continuous"
        else:
            j_type = "revolute"

        mesh_file = f"model_{next_model_idx}.glb"
        next_model_idx += 1

        wheel_radius = None
        if j_type == "continuous":
            if bboxes:
                wheel_radius = _estimate_roller_radius_from_bbox(bboxes, axis_name)
            if wheel_radius is None:
                wheel_radius = 0.0254
            if generate_roller_meshes and output_dir is not None:
                c1, c2 = ROLLER_PALETTE[roller_color_idx % len(ROLLER_PALETTE)]
                roller_color_idx += 1
                generate_striped_roller_glb(
                    output_dir / mesh_file,
                    radius_meters=wheel_radius,
                    width_meters=max(0.12, wheel_radius * 4.0),
                    axis=axis_name,
                    primary_rgba=c1,
                    stripe_rgba=c2,
                )
            zeroed_pos = [0.0, 0.0, 0.0]
            zeroed_rots: List[Dict[str, Any]] = []
        else:
            # Automatically solve exact AdvantageScope anchor transform from Onshape 4x4 matrices!
            # Joint frame at runtime uses translation=origin and identity orientation (pure axis motion),
            # so T_joint_runtime = [I, origin; 0, 1], giving T_zero = T_joint_runtime^-1 * T_robot_to_link.
            t_joint_runtime = [
                1.0, 0.0, 0.0, origin[0],
                0.0, 1.0, 0.0, origin[1],
                0.0, 0.0, 1.0, origin[2],
                0.0, 0.0, 0.0, 1.0,
            ]
            zeroed_pos, zeroed_rots = compute_zeroed_transform(
                t_joint_runtime, t_robot_to_link, include_gltf_yup_to_nwu=True
            )
            if (
                subassembly_export_plan is not None
                and child_inst.get("type") == "Assembly"
                and child_inst.get("documentId")
                and child_inst.get("elementId")
            ):
                subassembly_export_plan.append({
                    "meshFile": mesh_file,
                    "documentId": child_inst["documentId"],
                    "elementId": child_inst["elementId"],
                    "documentVersion": child_inst.get("documentVersion"),
                })

        link_entry: Dict[str, Any] = {
            "name": child_name,
            "meshFile": mesh_file,
            "zeroedPosition": zeroed_pos,
            "zeroedRotations": zeroed_rots,
            "collisionEnabled": j_type != "continuous",
        }
        links.append(link_entry)
        seen_links.add(child_name)

        joint_entry: Dict[str, Any] = {
            "name": child_name,
            "type": j_type,
            "parentLink": parent_name,
            "childLink": child_name,
            "originMeters": origin,
            "originRpyRadians": [0.0, 0.0, 0.0],
            "axis": axis_name,
        }
        if inverted:
            joint_entry["inverted"] = True
        if min_lim is not None:
            joint_entry["minLimit"] = round(float(min_lim), 8)
        if max_lim is not None:
            joint_entry["maxLimit"] = round(float(max_lim), 8)
        if wheel_radius is not None:
            joint_entry["wheelRadiusMeters"] = wheel_radius
        joints.append(joint_entry)

    return {
        "schemaVersion": "1.0",
        "robotName": robot_name,
        "sourceUrl": source_url,
        "extensionXyMarginMeters": 0.35,
        "extensionMaxHeightMeters": 1.85,
        "ignoreRestingOverlaps": True,
        "autoGroundChassis": False,
        "disableAdvantageScopeSimplification": False,
        "chassis": chassis_entry,
        "links": links,
        "joints": joints,
        "cameras": [],
    }


def augment_manifest_with_roller_meshes(
    manifest_path: Path,
    default_width_meters: float = 0.28,
) -> Dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    out_dir = manifest_path.parent
    links: List[Dict[str, Any]] = manifest.setdefault("links", [])
    existing_link_by_name = {l["name"]: l for l in links}

    used_indices = set()
    for l in links:
        m = re.match(r"model_(\d+)\.glb$", l.get("meshFile", ""))
        if m:
            used_indices.add(int(m.group(1)))
    next_idx = (max(used_indices) + 1) if used_indices else 0

    roller_color_idx = 0
    for joint in manifest.get("joints", []):
        if str(joint.get("type", "")).lower() != "continuous":
            continue
        radius = float(joint.get("wheelRadiusMeters") or 0.0254)
        axis = str(joint.get("axis") or "Y").upper()
        joint_name = joint["name"]
        link_name = f"{joint_name}Mesh"

        if link_name in existing_link_by_name:
            mesh_file = existing_link_by_name[link_name]["meshFile"]
        else:
            mesh_file = f"model_{next_idx}.glb"
            next_idx += 1
            link_entry = {
                "name": link_name,
                "meshFile": mesh_file,
                "zeroedPosition": [0.0, 0.0, 0.0],
                "zeroedRotations": [],
                "collisionEnabled": False,
            }
            links.append(link_entry)
            existing_link_by_name[link_name] = link_entry

        joint["childLink"] = link_name
        c1, c2 = ROLLER_PALETTE[roller_color_idx % len(ROLLER_PALETTE)]
        roller_color_idx += 1
        width = 0.38 if "intake" in joint_name.lower() or "hopper" in joint_name.lower() else default_width_meters
        generate_striped_roller_glb(
            out_dir / mesh_file,
            radius_meters=radius,
            width_meters=width,
            axis=axis,
            primary_rgba=c1,
            stripe_rgba=c2,
        )

    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="YACSim Phase 2 Onshape-to-yacsim-robot.json Exporter & Automatic Anchor Solver"
    )
    parser.add_argument("--url", help="Onshape assembly URL (https://cad.onshape.com/documents/.../e/...)")
    parser.add_argument("--robot-name", default="OnshapeRobot", help="Robot name for yacsim-robot.json")
    parser.add_argument("--out", type=Path, help="Output directory for yacsim-robot.json and model*.glb")
    parser.add_argument("--resolution", default="medium", choices=("coarse", "medium", "fine"))
    parser.add_argument("--skip-glb-export", action="store_true", help="Export manifest JSON only (skip Onshape GLTF download)")
    parser.add_argument(
        "--augment-rollers",
        type=Path,
        help="Offline mode: generate striped 3D spinning roller .glb meshes for all continuous joints in an existing yacsim-robot.json",
    )
    args = parser.parse_args(argv)

    if args.augment_rollers:
        manifest = augment_manifest_with_roller_meshes(args.augment_rollers)
        roller_count = sum(1 for j in manifest.get("joints", []) if j.get("type") == "continuous")
        print(f"Augmented {args.augment_rollers} with {roller_count} spinning 3D roller .glb meshes.")
        return 0

    if not args.url or not args.out:
        parser.error("Provide either --augment-rollers <manifest.json> OR both --url <onshape_url> and --out <dir>.")

    access_key = os.environ.get("ONSHAPE_ACCESS_KEY", "").strip()
    secret_key = os.environ.get("ONSHAPE_SECRET_KEY", "").strip()
    if not access_key or not secret_key:
        print(
            "Error: Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY environment variables first.\n"
            "Create keys in Onshape under My Account -> Developer -> API Keys.",
            file=sys.stderr,
        )
        return 2

    ref = parse_onshape_url(args.url)
    client = OnshapeRestClient(ref, access_key, secret_key)
    print(f"Fetching assembly definition from {ref.web_url} ...")
    asm_def = client.get_assembly_definition()
    print("Fetching exact CAD mass properties & bounding boxes ...")
    mass_props = client.get_assembly_mass_properties()
    bboxes = client.get_assembly_bounding_boxes()

    args.out.mkdir(parents=True, exist_ok=True)
    sub_plan: List[Dict[str, Any]] = []
    manifest = build_manifest_from_onshape_payloads(
        robot_name=args.robot_name,
        source_url=ref.web_url,
        assembly_def=asm_def,
        mass_props=mass_props,
        bboxes=bboxes,
        generate_roller_meshes=True,
        output_dir=args.out,
        subassembly_export_plan=sub_plan,
    )

    if not args.skip_glb_export:
        print(f"Exporting root assembly model.glb (resolution={args.resolution}) ...")
        client.export_assembly_gltf(args.out / "model.glb", resolution=args.resolution)
        for item in sub_plan:
            wvm_type = "v" if item.get("documentVersion") else ref.wvm
            wvm_id = item.get("documentVersion") or ref.wvmid
            print(f"Exporting subassembly {item['meshFile']} ...")
            client.export_assembly_gltf(
                args.out / item["meshFile"],
                did=item["documentId"],
                wvm=wvm_type,
                wvmid=wvm_id,
                eid=item["elementId"],
                resolution=args.resolution,
            )

    manifest_file = args.out / "yacsim-robot.json"
    manifest_file.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(
        f"Wrote {manifest_file} ({len(manifest['links'])} links, {len(manifest['joints'])} joints, "
        f"{len(sub_plan)} subassembly GLBs)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
