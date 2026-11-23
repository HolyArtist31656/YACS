"""Unit tests for YACSim Phase 2 Onshape Exporter, SE(3) Anchor Solver & Roller GLB Generator."""

from __future__ import annotations

import json
import math
import struct
import tempfile
import unittest
from pathlib import Path

from se3_math import (
    GLTF_YUP_TO_NWU_R,
    compose_advantagescope_rotations,
    compute_zeroed_transform,
    mat3_from_mat4,
    mat3_mul,
    mat4_mul,
    rot_axis_deg,
)
from yacsim_onshape_exporter import (
    augment_manifest_with_roller_meshes,
    build_manifest_from_onshape_payloads,
    parse_onshape_url,
)

DARWIN_URL = (
    "https://cad.onshape.com/documents/2da573df9d139d7a1e0f08a8/"
    "w/5266e52966df9f56fcb472f0/e/15a6a42133f059b818ad5264"
)


class OnshapeExporterTest(unittest.TestCase):
    def test_parses_darwin_onshape_url(self) -> None:
        ref = parse_onshape_url(DARWIN_URL)
        self.assertEqual(ref.host, "cad.onshape.com")
        self.assertEqual(ref.did, "2da573df9d139d7a1e0f08a8")
        self.assertEqual(ref.wvm, "w")
        self.assertEqual(ref.wvmid, "5266e52966df9f56fcb472f0")
        self.assertEqual(ref.eid, "15a6a42133f059b818ad5264")

    def test_rejects_invalid_onshape_url(self) -> None:
        with self.assertRaises(ValueError):
            parse_onshape_url("https://example.com/not-onshape")

    def test_se3_zeroed_transform_round_trips_arbitrary_3d_pose(self) -> None:
        # Arbitrary joint pivot at (-0.273, 0.05, 0.486)
        t_joint = [
            1.0, 0.0, 0.0, -0.273,
            0.0, 1.0, 0.0,  0.050,
            0.0, 0.0, 1.0,  0.486,
            0.0, 0.0, 0.0,  1.0,
        ]
        # Arbitrary subassembly occurrence rotated 25 deg around Y and 180 deg around Z, placed at (-0.15, 0.02, 0.52)
        r_link = mat3_mul(rot_axis_deg("z", 180.0), rot_axis_deg("y", 25.0))
        t_link = [
            r_link[0][0], r_link[0][1], r_link[0][2], -0.150,
            r_link[1][0], r_link[1][1], r_link[1][2],  0.020,
            r_link[2][0], r_link[2][1], r_link[2][2],  0.520,
            0.0,          0.0,          0.0,           1.0,
        ]

        zero_pos, zero_rots = compute_zeroed_transform(t_joint, t_link, include_gltf_yup_to_nwu=True)
        # Reconstruct R_zero * R_gltf_to_nwu from AdvantageScope ordered rotations
        r_reconstructed = compose_advantagescope_rotations(zero_rots)
        r_expected = mat3_mul(r_link, GLTF_YUP_TO_NWU_R)
        for row in range(3):
            for col in range(3):
                self.assertAlmostEqual(r_reconstructed[row][col], r_expected[row][col], places=4)

        # Verify T_joint * T_zero_pos == T_link translation
        self.assertAlmostEqual(-0.273 + zero_pos[0], -0.150, places=5)
        self.assertAlmostEqual(0.050 + zero_pos[1], 0.020, places=5)
        self.assertAlmostEqual(0.486 + zero_pos[2], 0.520, places=5)

    def test_auto_computes_anchors_grounding_and_subassembly_export_plan(self) -> None:
        r_hood = rot_axis_deg("z", 180.0)
        hood_tf = [
            r_hood[0][0], r_hood[0][1], r_hood[0][2], -0.151,
            r_hood[1][0], r_hood[1][1], r_hood[1][2],  0.0,
            r_hood[2][0], r_hood[2][1], r_hood[2][2],  0.486,
            0.0,          0.0,          0.0,           1.0,
        ]
        asm_def = {
            "rootAssembly": {
                "instances": [
                    {"id": "inst_chassis", "name": "Chassis", "type": "Assembly"},
                    {
                        "id": "inst_hood",
                        "name": "Hood",
                        "type": "Assembly",
                        "documentId": "doc123",
                        "elementId": "elem_hood",
                    },
                ],
                "occurrences": [
                    {"path": ["inst_chassis"], "transform": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]},
                    {"path": ["inst_hood"], "transform": hood_tf},
                ],
                "features": [
                    {
                        "id": "m1",
                        "featureType": "mate",
                        "featureData": {
                            "name": "Hood",
                            "mateType": "REVOLUTE",
                            "minLimit": 0.157,
                            "maxLimit": 0.733,
                            "matedEntities": [
                                {
                                    "matedOccurrence": ["inst_chassis"],
                                    "matedCS": {"origin": [-0.273, 0.0, 0.486], "zAxis": [0.0, -1.0, 0.0]},
                                },
                                {"matedOccurrence": ["inst_hood"]},
                            ],
                        },
                    }
                ],
            }
        }
        bboxes = {"lowX": -0.35, "highX": 0.35, "lowY": -0.35, "highY": 0.35, "lowZ": -0.05, "highZ": 0.65}
        sub_plan = []
        manifest = build_manifest_from_onshape_payloads(
            robot_name="AutoAnchorBot",
            source_url=DARWIN_URL,
            assembly_def=asm_def,
            bboxes=bboxes,
            generate_roller_meshes=False,
            subassembly_export_plan=sub_plan,
        )
        # Chassis grounded by -lowZ = +0.05 m
        self.assertAlmostEqual(manifest["chassis"]["zeroedPosition"][2], 0.05, places=5)
        # Hood anchor solved automatically: -0.151 - (-0.273) = +0.122 m!
        hood_link = manifest["links"][0]
        self.assertAlmostEqual(hood_link["zeroedPosition"][0], 0.122, places=5)
        self.assertAlmostEqual(hood_link["zeroedPosition"][2], 0.0, places=5)
        self.assertTrue(manifest["joints"][0]["inverted"])
        # Subassembly export plan captured Hood's own elementId
        self.assertEqual(len(sub_plan), 1)
        self.assertEqual(sub_plan[0]["elementId"], "elem_hood")

    def test_yacsim_origin_mate_connector_reorients_robot_frame(self) -> None:
        # Origin connector shifted by +1.0 m in CAD X -> robot origin shifts by -1.0 m
        asm_def = {
            "rootAssembly": {
                "instances": [{"id": "c", "name": "Chassis"}],
                "occurrences": [{"path": ["c"], "transform": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}],
                "features": [
                    {
                        "id": "orig",
                        "featureType": "mateConnector",
                        "featureData": {
                            "name": "YACSIM_ORIGIN",
                            "mateConnectorCS": {
                                "origin": [1.0, 0.0, 0.0],
                                "xAxis": [1.0, 0.0, 0.0],
                                "zAxis": [0.0, 0.0, 1.0],
                            },
                        },
                    }
                ],
            }
        }
        manifest = build_manifest_from_onshape_payloads(
            robot_name="OriginBot",
            source_url=DARWIN_URL,
            assembly_def=asm_def,
            generate_roller_meshes=False,
        )
        self.assertAlmostEqual(manifest["chassis"]["zeroedPosition"][0], -1.0, places=5)

    def test_builds_manifest_from_mock_onshape_payloads_with_roller(self) -> None:
        asm_def = {
            "rootAssembly": {
                "instances": [
                    {"id": "inst_chassis", "name": "Chassis"},
                    {"id": "inst_flywheel", "name": "LauncherFlywheel"},
                ],
                "occurrences": [],
                "features": [
                    {
                        "id": "m3",
                        "featureType": "mate",
                        "featureData": {
                            "name": "Flywheel",
                            "mateType": "REVOLUTE",
                            "matedEntities": [
                                {
                                    "matedOccurrence": ["inst_chassis"],
                                    "matedCS": {"origin": [-0.151, 0.0, 0.486], "zAxis": [0.0, 1.0, 0.0]},
                                },
                                {"matedOccurrence": ["inst_flywheel"]},
                            ],
                        },
                    }
                ],
            }
        }
        mass_props = {
            "volume": [0.022],
            "mass": [52.0],
            "centroid": [0.0, 0.0, 0.22],
            "inertia": [3.5, 0, 0, 0, 4.2, 0, 0, 0, 5.8],
        }
        bboxes = {"lowX": -0.0508, "highX": 0.0508, "lowY": -0.15, "highY": 0.15, "lowZ": -0.0508, "highZ": 0.0508}

        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td)
            manifest = build_manifest_from_onshape_payloads(
                robot_name="DarwinOnshape",
                source_url=DARWIN_URL,
                assembly_def=asm_def,
                mass_props=mass_props,
                bboxes=bboxes,
                generate_roller_meshes=True,
                output_dir=out_dir,
            )
            self.assertEqual(manifest["chassis"]["massProperties"]["massKilograms"], 52.0)
            glb_path = out_dir / "model_0.glb"
            self.assertTrue(glb_path.exists())
            magic, version, length = struct.unpack("<III", glb_path.read_bytes()[:12])
            self.assertEqual((magic, version, length), (0x46546C67, 2, glb_path.stat().st_size))

    def test_augment_manifest_generates_valid_glb_with_normals(self) -> None:
        sample = {
            "schemaVersion": "1.0",
            "robotName": "RollerTest",
            "links": [{"name": "Slamtake", "meshFile": "model_0.glb"}],
            "joints": [
                {
                    "name": "IntakeRoller",
                    "type": "continuous",
                    "parentLink": "Slamtake",
                    "childLink": "IntakeRollerVirtual",
                    "originMeters": [0.32, 0.0, 0.0],
                    "axis": "Y",
                    "wheelRadiusMeters": 0.0254,
                }
            ],
        }
        with tempfile.TemporaryDirectory() as td:
            mp = Path(td) / "yacsim-robot.json"
            mp.write_text(json.dumps(sample), encoding="utf-8")
            updated = augment_manifest_with_roller_meshes(mp)
            self.assertEqual(updated["joints"][0]["childLink"], "IntakeRollerMesh")
            glb = Path(td) / "model_1.glb"
            raw = glb.read_bytes()
            json_len = struct.unpack("<I", raw[12:16])[0]
            gltf = json.loads(raw[20:20 + json_len].decode("utf-8"))
            self.assertIn("NORMAL", gltf["meshes"][0]["primitives"][0]["attributes"])


if __name__ == "__main__":
    unittest.main()
