# YACSim — Yet Another CAD Simulator (WPILib 2027 / Commands v3)

A lightweight, zero-extra-dependency **WPILib 2027 (`2027_alpha7`, Java 25, Commands v3)** FRC vendor library structured after [`Yet-Another-Software-Suite/YAMS`](https://github.com/Yet-Another-Software-Suite/YAMS) (`master` branch).

## Repository Layout

- `yacsim/java/yacsim/{core,commands3}/` — Library source code (`core` + `commands3`).
- `vendordep/` — WPILib GradleRIO `2027.0.0-alpha-7` project with `CommandsV3.json` and `yams_commands3.json` in `vendordep/vendordeps/`, plus JUnit 5 tests in `vendordep/src/test/java/yacsim/`.
- `examples/commands3/arm_and_elevator_2027/` — Complete Commands v3 example robot project.
- `docs/ARCHITECTURE.md` — SystemCore 2 GB hardware budget & drivetrain/mechanism pipeline.

## Building & Testing

Open `vendordep/` in **WPILib 2027 VS Code** (ensuring `gradlew.bat` and `gradle/wrapper/gradle-wrapper.jar` are copied into `vendordep/`), then run:

```cmd
gradlew.bat build test -Dorg.gradle.java.home="C:\Users\Public\wpilib\2027_alpha7\jdk"
```

## License

Licensed under **LGPL-3.0**, matching YASS / YAMS.
