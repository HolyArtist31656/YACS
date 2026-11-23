# YACSim Onshape Exporter (`tools/onshape-exporter/`)

Zero-dependency Python 3 CLI (standard library only — **no `pip install` required**) that pulls assembly definitions, mates, exact mass/inertia tensors, bounding boxes, and glTF/GLB meshes from Onshape's REST API v6 into a ready-to-run `yacsim-robot.json` manifest.

> **Prerequisites**: Requires **Python 3.10+** (no `pip` packages needed) and an **Onshape API Access Key + Secret Key**. See [`docs/ONSHAPE_EXPORTER_GUIDE.md`](../../docs/ONSHAPE_EXPORTER_GUIDE.md) for step-by-step Python installation and Onshape API key setup.

## 1. Export Live From Onshape

Set your Onshape API keys (from **My Account → Developer → API Keys**):

```powershell
$env:ONSHAPE_ACCESS_KEY = "<your-access-key>"
$env:ONSHAPE_SECRET_KEY = "<your-secret-key>"

python tools/onshape-exporter/yacsim_onshape_exporter.py `
  --url "https://cad.onshape.com/documents/2da573df9d139d7a1e0f08a8/w/5266e52966df9f56fcb472f0/e/15a6a42133f059b818ad5264" `
  --robot-name Darwin `
  --out examples/commands3/darwin_6328_manifest_2027/src/main/deploy/yacsim/darwin_onshape
```

## 2. Offline 3D Spinning Roller Mesh Augmentation (No API Keys Needed)

If you already have an AdvantageScope `.glb` assembly where rollers are merged into the chassis or intake meshes (like 6328's `Robot_Darwin`), generate high-contrast, color-striped 3D spinning roller `.glb` meshes for every `continuous` joint in your `yacsim-robot.json`:

```powershell
python tools/onshape-exporter/yacsim_onshape_exporter.py `
  --augment-rollers examples/commands3/darwin_6328_manifest_2027/src/main/deploy/yacsim/darwin/yacsim-robot.json
```

## 3. Run Exporter Unit Tests

```powershell
python -m unittest discover -s tools/onshape-exporter -p "test_*.py" -v
```
