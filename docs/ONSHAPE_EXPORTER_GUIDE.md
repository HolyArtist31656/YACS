# Onshape-to-YACSim Automated Exporter Guide (Phase 2)

YACSim `v2027.2.0-alpha1` includes **`tools/onshape-exporter/`**, a zero-dependency Python 3 CLI (standard library only — **no `pip install` needed**) that connects directly to Onshape's REST API v6 ([Import & Export API](https://onshape-public.github.io/docs/api-adv/translation/), [Onshape API Changelog](https://onshape-public.github.io/docs/changelog/)) to extract:

- Assembly link hierarchy, 4x4 occurrence transforms, and mate graph (`REVOLUTE`, `CYLINDRICAL`, `SLIDER`)
- Exact AdvantageScope `zeroedPosition` and `zeroedRotations` solved automatically via $T_{\text{zero}} = T_{\text{robot}\to\text{joint}}^{-1} \cdot T_{\text{robot}\to\text{link}}$
- Exact CAD mass, center of mass, and 3x3 principal inertia tensor (`Ixx`, `Iyy`, `Izz`)
- Bounding-box roller/wheel radii for `continuous` joints (flywheels, kickers, intake rollers) and automatic chassis floor grounding
- Asynchronous glTF/GLB mesh export (`POST /api/v6/assemblies/.../translations`) for the root assembly and moving subassemblies
- High-contrast, color-striped 3D spinning roller `.glb` meshes so wheel/roller rotation is immediately visible in AdvantageScope 3D Field

> **When is Python 3 required?**
> Python 3 is **only** needed on your development laptop when you want to run `tools/onshape-exporter/yacsim_onshape_exporter.py` to auto-generate a `yacsim-robot.json` manifest from Onshape (or run `--augment-rollers`). Your robot code, WPILib desktop simulation, and SystemCore runtime remain **100% pure Java 25**—and teams using manual Java setup, URDF import, or a pre-generated `yacsim-robot.json` do not need Python installed.

---

## 1. Installing Python 3 (Required for the Onshape Exporter CLI)

The exporter uses Python 3.10+ standard library modules only (`urllib`, `json`, `math`, `struct`). No virtual environments or `pip install` steps are required.

### Windows 10 / 11 (PowerShell)
```powershell
# Install Python 3.12 via winget
winget install --id Python.Python.3.12 -e --source winget --accept-package-agreements --accept-source-agreements

# Refresh PATH in the current PowerShell window (no terminal restart needed)
$env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")

# Verify installation
python --version
```
> **Windows Store Alias Tip**: If `python --version` opens the Microsoft Store instead of printing `Python 3.12.x`, open **Settings -> Apps -> Advanced app settings -> App execution aliases** and toggle **Off** both `python.exe` and `python3.exe` (or run `py -3` instead of `python`).

### macOS
```bash
brew install python@3.12
python3 --version
```
*(Or download the official installer from [python.org/downloads](https://www.python.org/downloads/).)*

### Linux (Ubuntu / Debian)
```bash
sudo apt update && sudo apt install -y python3
python3 --version
```

---

## 2. Generating Your Onshape API Keys

Onshape's REST API uses HTTP Basic Authentication requiring **both** an **Access Key** and a **Secret Key** ([Onshape API Authentication](https://github.com/onshape-public/go-client/blob/master/onshape/README.md)):

1. Sign in to [cad.onshape.com](https://cad.onshape.com).
2. Click your profile icon (top-right) and select **My account**.
3. In the left sidebar, open **Developer** -> **API keys**.
4. Click **Create new API key**.
5. Give it a descriptive label (for example `YACSim CAD Exporter`) and select **Read** permissions (the exporter only reads assembly data and downloads translations—it never modifies your CAD document).
6. Click **Create**.
7. **Copy both keys immediately**: Onshape displays the **Secret Key** only once. Store it safely in your team password manager or local user environment variables.

> **Security Rule**: **Never** paste your `ONSHAPE_ACCESS_KEY` or `ONSHAPE_SECRET_KEY` into `yacsim-robot.json`, Java files, PowerShell scripts committed to Git, or public chats. Always pass them through local environment variables.

---

## 3. Linking Your Onshape API Keys to Your Machine

### Windows (PowerShell — Current Session Only)
```powershell
$env:ONSHAPE_ACCESS_KEY = Read-Host "Paste Onshape Access Key"
$env:ONSHAPE_SECRET_KEY = Read-Host "Paste Onshape Secret Key"
```

### Windows (PowerShell — Save Permanently for Your User Account)
```powershell
$access = Read-Host "Paste Onshape Access Key"
$secret = Read-Host "Paste Onshape Secret Key"
[Environment]::SetEnvironmentVariable("ONSHAPE_ACCESS_KEY", $access, "User")
[Environment]::SetEnvironmentVariable("ONSHAPE_SECRET_KEY", $secret, "User")
$env:ONSHAPE_ACCESS_KEY = $access
$env:ONSHAPE_SECRET_KEY = $secret
Write-Host "Saved Onshape API keys to your Windows User environment."
```

### macOS / Linux (`~/.zshrc` or `~/.bashrc`)
```bash
export ONSHAPE_ACCESS_KEY="<your-access-key>"
export ONSHAPE_SECRET_KEY="<your-secret-key>"
```

---

## 4. Running the Live Exporter Against an Onshape Assembly

Copy the URL of your top-level robot **Assembly** tab from Onshape (`https://cad.onshape.com/documents/<did>/<w|v|m>/<wvmid>/e/<eid>`) and run:

```powershell
python tools/onshape-exporter/yacsim_onshape_exporter.py `
  --url "https://cad.onshape.com/documents/2da573df9d139d7a1e0f08a8/w/5266e52966df9f56fcb472f0/e/15a6a42133f059b818ad5264" `
  --robot-name Darwin `
  --out examples/commands3/darwin_6328_manifest_2027/src/main/deploy/yacsim/darwin_onshape
```

### Recommended Onshape Mate & Origin Conventions
- **Robot Origin (`YACSIM_ORIGIN`)**: Place a mate connector named `YACSIM_ORIGIN` or `ORIGIN_CUBE` (such as the standard [FRCDesign Origin Cube](https://frcdesign.org/learning-course/stage1/1d/top-down-design/)) in your root assembly with `+X` forward, `+Y` left, and `+Z` up. The exporter automatically uses its inverse 4x4 transform for the entire robot.
- **Arm / Hood / Wrist**: Use a `REVOLUTE` mate with min/max limits set in Onshape.
- **Elevator / Compactor / Climber**: Use a `SLIDER` mate with min/max travel limits.
- **Flywheel / Kicker / Intake Roller**: Use an unbounded `REVOLUTE` or `CYLINDRICAL` mate (or include `roller`, `wheel`, `flywheel`, `kicker`, or `shooter` in the mate name). The exporter automatically classifies it as `continuous`, estimates `wheelRadiusMeters`, and generates a spinning `.glb` mesh.

---

## 5. Offline 3D Spinning Roller Augmentation (`--augment-rollers`)

If you already have AdvantageScope `.glb` files where rollers are baked into the chassis/arm meshes (like 6328's `Robot_Darwin`), you can generate color-striped 3D spinning roller `.glb` meshes for every `continuous` joint in your `yacsim-robot.json` without Onshape API keys:

```powershell
python tools/onshape-exporter/yacsim_onshape_exporter.py `
  --augment-rollers examples/commands3/darwin_6328_manifest_2027/src/main/deploy/yacsim/darwin/yacsim-robot.json
```

### Automatic Parent-Mounted Roller Chaining in Java
When a roller's `parentLink` in `yacsim-robot.json` is a moving mechanism (like `IntakeRoller` mounted on `Slamtake`), `YamsCommands3Bridge.bindFromManifest(robotConfig, "IntakeRoller", slamtake.getRollerMotor())` automatically looks up `Slamtake`'s live `Pose3d` supplier via `CadRobotConfig.getComponentPoseSupplier("Slamtake")` and composes the parent arm's motion with the roller's local spin—zero extra Java code required.

---

## 6. Automatic 4x4 Anchor Point & Floor Grounding Math (`se3_math.py`)

The exporter computes every link's AdvantageScope `zeroedPosition` and `zeroedRotations` automatically from Onshape's `GetAssemblyDefinition` JSON without any manual measurements:

1. **Robot Root Frame**: Applies the inverse of `YACSIM_ORIGIN` / `ORIGIN_CUBE` when present.
2. **Automatic Floor Grounding**: Shifts chassis `zeroedPosition.z` by `-lowZ` from `/boundingboxes` so wheels sit flush at `z = 0.0`.
3. **Exact Per-Link Anchor Solve**: Reads each child subassembly's 4x4 `occurrence.transform` ($T_{\text{robot}\to\text{link}}$) and mate `matedCS` ($T_{\text{robot}\to\text{joint}}$), solves $T_{\text{zero}} = T_{\text{robot}\to\text{joint}}^{-1} \cdot T_{\text{robot}\to\text{link}}$, composes glTF Y-up -> WPILib NWU, and writes exact `zeroedPosition` + ordered `zeroedRotations` alongside each exported subassembly `.glb`.

---

## 7. Running Exporter Unit Tests

```powershell
python -m unittest discover -s tools/onshape-exporter -p "test_*.py" -v
```
