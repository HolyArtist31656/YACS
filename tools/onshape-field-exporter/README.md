# YACSim Onshape / CAD Field Exporter (`tools/onshape-field-exporter/`)

Pure-Python (zero third-party dependencies) CLI tool that turns a single 3D field geometry spec into all three downstream FRC simulation and autonomous files:

1. **YAECE `SeasonConfig` JSON** (`yaece-season-andymark.json` & `yaece-season-welded.json`) with verified Andymark and Welded field dimensions, static obstacles, no-entry zones, overhead trench height bands, and 3D scoring targets.
2. **PathPlanner `navgrid.json`** rasterized from the exact same inflated 2D obstacle polygons.
3. **YACSim 3D Field OBB Manifest (`yacsim-field.json`)** loadable directly via `FieldObstacle3d.loadFieldManifest(Path)` into `ProjectilePhysicsSim`.

## Usage

```powershell
python tools/onshape-field-exporter/field_exporter.py --spec field_spec.json --out build/field-out --node-size 0.25 --robot-radius 0.45
```

## Running Unit Tests

```powershell
python -m unittest tools/onshape-field-exporter/test_field_exporter.py -v
```
