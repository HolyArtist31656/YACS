"""Unit tests for tools/onshape-field-exporter/field_exporter.py."""

import json
import tempfile
import unittest
from pathlib import Path

from field_exporter import (
    FIELD_DIMENSIONS_METERS,
    export_all,
)


class FieldExporterTest(unittest.TestCase):
    def test_exports_andymark_welded_navgrid_and_yacsim_field(self) -> None:
        spec = {
            "seasonName": "REBUILT_2026",
            "parts": [
                {
                    "name": "OBSTACLE_blue_hub_base",
                    "minMeters": [4.0, 3.5, 0.0],
                    "maxMeters": [5.2, 4.7, 1.1],
                },
                {
                    "name": "SCORING_SITE_blue_hub_opening",
                    "minMeters": [4.2, 3.7, 2.0],
                    "maxMeters": [5.0, 4.5, 2.4],
                },
                {
                    "name": "TRENCH_overhead_bar",
                    "minMeters": [7.5, 0.0, 0.70],
                    "maxMeters": [8.5, 1.5, 0.90],
                },
                {
                    "name": "NO_ENTRY_opponent_zone",
                    "minMeters": [14.0, 0.0, 0.0],
                    "maxMeters": [16.5, 3.0, 1.0],
                },
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            spec_path = tmp_path / "field_spec.json"
            spec_path.write_text(json.dumps(spec), encoding="utf-8")
            out_dir = tmp_path / "out"

            outputs = export_all(spec_path, out_dir, node_size_meters=0.5, robot_radius_meters=0.4)
            self.assertEqual(
                set(outputs.keys()),
                {"yaece_andymark", "yaece_welded", "navgrid", "yacsim_field"},
            )

            am = json.loads(outputs["yaece_andymark"].read_text(encoding="utf-8"))
            wd = json.loads(outputs["yaece_welded"].read_text(encoding="utf-8"))
            self.assertAlmostEqual(am["fieldLengthMeters"], FIELD_DIMENSIONS_METERS["ANDYMARK"][0], places=4)
            self.assertAlmostEqual(wd["fieldLengthMeters"], FIELD_DIMENSIONS_METERS["WELDED"][0], places=4)
            self.assertEqual(len(am["staticObstacles"]), 1)
            self.assertEqual(len(am["scoringSites"]), 1)
            self.assertEqual(len(am["trenchZones"]), 1)
            self.assertEqual(len(am["noEntryZones"]), 1)

            nav = json.loads(outputs["navgrid"].read_text(encoding="utf-8"))
            self.assertEqual(nav["nodeSizeMeters"], 0.5)
            # Cell covering blue_hub_base center (4.6, 4.1) must be blocked
            self.assertTrue(nav["grid"][8][9])
            # Open midfield cell (1.0, 1.0) must be clear
            self.assertFalse(nav["grid"][2][2])

            yf = json.loads(outputs["yacsim_field"].read_text(encoding="utf-8"))
            self.assertEqual(len(yf["obstacles"]), 4)
            targets = [o for o in yf["obstacles"] if o["scoringTarget"]]
            self.assertEqual(len(targets), 1)
            self.assertEqual(targets[0]["name"], "SCORING_SITE_blue_hub_opening")


if __name__ == "__main__":
    unittest.main()
