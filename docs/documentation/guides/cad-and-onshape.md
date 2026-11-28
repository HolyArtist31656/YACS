# CAD Manifest & Onshape Exporters

`CadManifestLoader` ingests Onshape URDF ZIP exports and JSON manifests (`yacsim-manifest.json`), automatically:
- Grounding the chassis and child joints (`CadRobotSimulator`),
- Extracting CAD-mounted camera transforms (`CadCameraConfig`) for YALL/AdvantageScope,
- Building multi-primitive sub-OBBs for concave assemblies (`ObbCollisionDetector`), and
- Binding YAMS Commands v3 mechanisms via `YamsCommands3Bridge` and `YacSimYamsFactory`.
