# FRC Team 2181 (GEARS) — 2027 Commands v3 + YAMS + YACSim Example

Full port of [FRCteam2181/2026-Comp-Code](https://github.com/FRCteam2181/2026-Comp-Code) (`main` branch) to **WPILib 2027 Commands v3**, **REVLib 2027**, **YAMS 2027**, and **YACSim (`v2027.2.0-alpha1`)**, designed to plug straight into Team 2181's Onshape CAD assembly via `yacsim-robot.json`.

## Ported Team 2181 Mechanisms

| Mechanism | YAMS Type | Hardware (from `2026-Comp-Code`) | CAD / AdvantageScope Binding |
| :--- | :--- | :--- | :--- |
| **`IntakeArmMechanism`** | `Arm` | SPARK MAX `14` (leader) + `13` (inverted follower), 2× NEO, `1:1`, `40 A`, `BRAKE` | `model_0.glb` (`IntakeArm`) |
| **`TurretMechanism`** | `Pivot` | SPARK Flex `16`, NEO Vortex, `4:1 × 10:1` (`40:1`), `40 A`, `BRAKE`, MOI `0.2 kg·m²` | `model_1.glb` (`Turret`) |
| **`ClimberMechanism`** | `Elevator` | SPARK MAX `18`, NEO, `40 A`, `BRAKE` | `model_2.glb` (`Climber`) |
| **`ShooterMechanism`** | `FlyWheel` | SPARK MAX `9` (leader) + `10` (inverted follower), 2× NEO, `1:1`, `4 in` dia, `COAST` | `model_3.glb` (auto-mounted on `Turret`!) |
| **`SpindexerMechanism`** | `FlyWheel` | SPARK MAX `11`, NEO, `9:1`, `60 A`, `BRAKE`, `4 in` dia | `model_4.glb` (Z-axis spinner) |
| **`TopIntakeMechanism`** | `FlyWheel` | SPARK Flex `12`, NEO, `5:1`, `60 A`, `COAST`, `4 in` dia | `model_5.glb` (auto-mounted on `IntakeArm`!) |
| **`InputMechanism`** | `FlyWheel` | SPARK Flex `15`, NEO, `1:1`, `40 A`, `BRAKE`, `4 in` dia | `model_6.glb` (feeder roller) |

## Quick Start (Windows PowerShell)

```powershell
cd examples\commands3\gears_2181_2027
.\setup_gears_2181.ps1
.\gradlew run --refresh-dependencies "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## Plugging In Team 2181's Real Onshape CAD

This example ships with lightweight starter placeholder `.glb` meshes so it runs in AdvantageScope immediately. To replace them with Team 2181's actual Onshape assembly with **zero Java changes**:

```powershell
python ..\..\..\tools\onshape-exporter\yacsim_onshape_exporter.py `
  --url "<TEAM_2181_ONSHAPE_ASSEMBLY_URL>" `
  --robot-name GEARS_2181 `
  --out src\main\deploy\yacsim\gears2181
```
