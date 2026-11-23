# Architecture & SystemCore 2 GB Performance Budget

## Hardware Target

YACSim is engineered specifically for the **WPILib 2027 SystemCore** (Raspberry Pi CM5: quad-core Cortex-A76 @ 2.4 GHz, 2 GB LPDDR4x-4267 RAM, Linux 6.12.77, Java 25, AdvantageScope bundled in OS).

## Design Principles

- **Commands v3 Only**: Targets WPILib 2027 `CommandsV3.json` + `yams_commands3.json` exclusively—eliminating Commands v2/v3 vendordep conflicts.
- **Zero Extra Runtime JARs**: Relies strictly on WPILib 2027 and YAMS 2027. Pure-Java `MinimalJson` handles glTF 2.0 JSON chunks and AdvantageScope `config.json` generation with zero Jackson/Gson overhead.
- **Header-Only glTF/GLB Parsing**: `MeshParser` inspects glTF 2.0 `POSITION` accessor `min`/`max` bounds and primitive index counts directly from the JSON chunk without decoding vertex or texture buffers into Java heap memory.
- **Lock-Free Threading (`SimSnapshot`)**: `CadRobotSimulator` stores simulation state in an immutable `record SimSnapshot(Pose2d, Pose3d, Pose3d[], List<CollisionEvent>)` held in a `volatile` field. `simIterate()` (safe to run on a high-rate `Notifier` thread) swaps the reference atomically, while `updateTelemetry()` reads a consistent snapshot on the 50 Hz main thread with zero locking and zero `ArrayIndexOutOfBoundsException` risk.

## Verified SystemCore Stress Benchmark (FRC 6328 "Darwin")

Tested on FRC Team 6328 Mechanical Advantage's 2026 robot (*Darwin*) ported to YAMS 2027 + YACSim (`examples/commands3/darwin_6328_2027`):

| Metric | Observed Result |
| --- | --- |
| Robot JVM Heap Cap | `-Xmx256m` (`-XX:MaxHeapSize=268435456` verified via `-XX:+PrintCommandLineFlags`) |
| CAD Payload | 5 real `.glb` files (`model.glb` = 2,104,546 triangles + 4 mechanism meshes) |
| Out-of-Memory Errors | `0` |
| Steady-State `robotPeriodic()` Time | `0.635 ms` (`0.000635s`) publishing `RobotPose`, `RobotPose3d`, `ComponentPoses`, `CollisionActive`, `CollisionDetails` |
| Steady-State 50 Hz Overruns (2+ min run) | `0` (single cold-start JIT/Phoenix init tick at startup only) |
