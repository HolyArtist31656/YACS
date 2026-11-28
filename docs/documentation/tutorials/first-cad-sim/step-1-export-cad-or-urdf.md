---
icon: file-import
---

# Step 1: Export CAD or Load URDF

**Goal:** Load your robot's real mass, link geometry, and joint definitions into Java with one line of code.

{% tabs %}
{% tab title="From Onshape Manifest JSON" %}
```java
import java.nio.file.Path;
import org.wpilib.system.Filesystem;
import yacsim.core.manifest.CadManifestLoader;
import yacsim.core.manifest.CadRobotManifest;

Path manifestPath = Filesystem.getDeployDirectory().toPath().resolve("yacsim/robot_manifest.json");
CadRobotManifest manifest = CadManifestLoader.loadJson(manifestPath);

double elevatorMassKg = manifest.getLinkMass("elevator_carriage");
double armMoiKgM2 = manifest.getJointMomentOfInertia("arm_pivot");
```
{% endtab %}

{% tab title="From a URDF / ZIP Export" %}
```java
import java.nio.file.Path;
import yacsim.core.manifest.CadManifestLoader;
import yacsim.core.manifest.CadManifestLoader.UrdfLoadResult;

UrdfLoadResult result = CadManifestLoader.loadUrdf(Path.of("src/main/deploy/cad/robot.urdf"));
CadRobotManifest manifest = result.manifest();
```
{% endtab %}
{% endtabs %}

## Check it

Confirm `manifest.json` exists in `src/main/deploy/yacsim/` and `CadManifestLoader.load(...)` returns your expected links and cameras with zero warnings.

## If it did not work

* **File not found at runtime:** Confirm the file is under `src/main/deploy/`, not `src/main/resources/`.
* **Missing cameras in `manifest.cameras()`:** Name camera parts with a supported prefix (`cam_`, `limelight`, or `photon`) in Onshape or URDF.
