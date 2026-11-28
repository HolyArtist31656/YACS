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

Next: [Step 2: Add Chassis Dynamics & Terrain](step-2-chassis-dynamics-and-terrain.md).
