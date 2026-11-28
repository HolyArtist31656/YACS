---
icon: download
---

# Install YACS

**Goal:** Add YACS `v2027.6.6-alpha1` to your WPILib `2027.0.0-alpha-8` (Java 25) robot project.

{% stepper %}
{% step %}
### Copy `YACSim.json` into `vendordeps/`

Grab `vendordeps/YACSim.json` from [HolyArtist31656/YACS](https://github.com/HolyArtist31656/YACS) and place it in your project's `vendordeps/` directory.
{% endstep %}

{% step %}
### Enable `mavenLocal()` if testing a local build

If you published YACS locally from `YACS/vendordep` with `.\gradlew publishToMavenLocal`, ensure `repositories { mavenLocal() }` is present in `build.gradle`.
{% endstep %}

{% step %}
### Verify your build

```powershell
.\gradlew.bat build test
```
{% endstep %}
{% endstepper %}

{% hint style="info" %}
**YAMS 2027 Compatibility:** YACS `v2027.6.6-alpha1` connects directly to YAMS's public Pose SPI (`getCascadePose3d`, `getContinuousPose3d`, `getUpperPose3d`, `DifferentialMechanism.getPose3d`), no reflection hacks needed.
{% endhint %}
