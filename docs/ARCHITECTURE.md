# YACSim Architecture & SystemCore 2 GB Performance Budget

- **Hardware Target**: 2 GB SystemCore (Raspberry Pi CM5: quad-core Cortex-A76 @ 2.4 GHz, VideoCore VII GPU, 2 GB LPDDR4x-4267 RAM, Linux 6.12.77, Java 25, WPILib 2027, AdvantageScope bundled in OS).
- **Commands v3 Only**: Targets WPILib 2027 `CommandsV3.json` + `yams_commands3.json` exclusively—eliminating Commands v2/v3 vendordep conflicts.
- **Strict YAMS-Only Resources**: Zero external third-party runtime jars beyond WPILib 2027 and YAMS 2027. Pure-Java `MinimalJson` handles glTF 2.0 parsing and AdvantageScope `config.json` generation.
