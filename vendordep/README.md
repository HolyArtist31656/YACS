# YACSim Vendordep Project (Commands v3 Only)

Open **this `vendordep/` directory** in **WPILib 2027 VS Code** to build, test, format, and publish YACSim locally.

## Common Commands

```powershell
# Format, build, run all 67 JUnit 5 tests, and generate Javadoc
.\gradlew spotlessApply build test javadoc "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"

# Publish local Maven artifacts (build/maven/releases) and yacsim_commands3.json
.\gradlew publish generateVendordepJson "-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## Public Distribution

- **Online Vendordep JSON**: `https://holyartist31656.github.io/YACS/vendordeps/yacsim_commands3.json`
- **Maven Repository**: `https://holyartist31656.github.io/YACS/maven/releases/`
- **Hosted Javadoc**: `https://holyartist31656.github.io/YACS/javadoc/`

**Current Release:** `v2027.5.0-alpha1` (67 JUnit 5 tests across 9 test classes).
