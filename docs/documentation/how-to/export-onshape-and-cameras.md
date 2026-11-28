---
icon: camera
---

# How do I export Onshape CAD and camera transforms?

**Goal:** Automatically extract `Transform3d` camera mounts from Onshape mate connectors so your PhotonVision / Limelight / YALL camera poses never go out of sync with CAD.

{% stepper %}
{% step %}
### Name your camera mate connectors in Onshape

In your top-level robot assembly, place a Mate Connector at each camera lens (with `+X` pointing out the lens and `+Z` pointing up) and name it with one of these prefixes:

* `cam_*` (e.g., `cam_front_intake`)
* `limelight*` (e.g., `limelight_turret`)
* `photon*` (e.g., `photon_rear_left`)
{% endstep %}

{% step %}
### Run `yacsim_onshape_exporter.py`

Run the exporter script from `tools/onshape-exporter/` against your Onshape document URL. It automatically detects camera mate connectors (or URDF camera links) and populates the `"cameras": []` array inside `robot_manifest.json`.
{% endstep %}

{% step %}
### Read the camera `Transform3d` in Java

```java
CadRobotManifest manifest = CadManifestLoader.loadJson(manifestPath);
manifest.cameras().forEach(cam ->
    System.out.println(cam.name() + " -> " + cam.robotToCamera())
);
```
{% endstep %}
{% endstepper %}
