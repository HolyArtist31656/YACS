// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.manifest;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import yacsim.commands3.YacSimYamsFactory;
import yacsim.commands3.YamsCommands3Bridge;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.cad.GlbMerger;
import yacsim.core.cad.MeshMetadata;
import yacsim.core.cad.MeshParser;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.model.CadRobotSimulator;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.ObbCollisionDetector;
import yacsim.core.util.MinimalJson;
import yacsim.core.yams.MotionAxis;

class CadManifestLoaderTest {

  private static final String UNIT_CUBE_STL =
      """
      solid cube
        facet normal 0 0 -1
          outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 1 1 0
          endloop
        endfacet
        facet normal 0 0 1
          outer loop
            vertex 0 0 1
            vertex 1 1 1
            vertex 0 1 1
          endloop
        endfacet
      endsolid cube
      """;

  @Test
  void roundTripsManifestJsonAndProvidesYamsUnits(@TempDir Path dir) throws Exception {
    String json =
        """
        {
          "schemaVersion": "1.0",
          "robotName": "DarwinManifestTest",
          "sourceUrl": "https://cad.onshape.com/documents/2da573df9d139d7a1e0f08a8/w/5266e52966df9f56fcb472f0/e/15a6a42133f059b818ad5264",
          "extensionXyMarginMeters": 0.35,
          "extensionMaxHeightMeters": 1.85,
          "ignoreRestingOverlaps": true,
          "autoGroundChassis": true,
          "chassis": {
            "name": "Chassis",
            "meshFile": "chassis.stl",
            "massProperties": {
              "volumeCubicMeters": 0.025,
              "massKilograms": 48.5,
              "centerOfMassMeters": [0.01, -0.02, 0.18],
              "ixxKgM2": 3.2,
              "iyyKgM2": 4.1,
              "izzKgM2": 5.5
            }
          },
          "links": [
            {
              "name": "Hood",
              "meshFile": "hood.stl",
              "massProperties": {
                "massKilograms": 1.36,
                "centerOfMassMeters": [0.0, 0.0, 0.09],
                "ixxKgM2": 0.012,
                "iyyKgM2": 0.018,
                "izzKgM2": 0.014
              }
            }
          ],
          "joints": [
            {
              "name": "Hood",
              "type": "revolute",
              "parentLink": "Chassis",
              "childLink": "Hood",
              "originMeters": [-0.273, 0.0, 0.486],
              "axis": "Y",
              "minLimit": 0.15707963,
              "maxLimit": 0.73303829,
              "softMinLimit": 0.17453293,
              "softMaxLimit": 0.71907565,
              "startingPosition": 0.17453293,
              "lengthMeters": 0.18
            },
            {
              "name": "Flywheel",
              "type": "continuous",
              "parentLink": "Chassis",
              "childLink": "FlywheelVirtual",
              "originMeters": [-0.15, 0.0, 0.48],
              "axis": "Y",
              "wheelRadiusMeters": 0.0508,
              "momentOfInertiaKgM2": 0.02
            }
          ],
          "cameras": [
            {
              "name": "FrontCam",
              "position": [0.28, 0.0, 0.52],
              "rpyRadians": [0.0, -0.2618, 0.0],
              "fovDegrees": 70.0,
              "resolution": [1280, 800]
            }
          ]
        }
        """;

    Path manifestFile = dir.resolve("yacsim-robot.json");
    Files.writeString(manifestFile, json, StandardCharsets.UTF_8);

    CadRobotManifest parsed = CadManifestLoader.loadJson(manifestFile);
    Path roundTripFile = dir.resolve("roundtrip.json");
    CadManifestLoader.writeJson(parsed, roundTripFile);
    CadRobotManifest reparsed = CadManifestLoader.loadJson(roundTripFile);

    assertEquals("DarwinManifestTest", reparsed.robotName());
    assertTrue(reparsed.sourceUrl().orElse("").contains("2da573df9d139d7a1e0f08a8"));
    assertEquals(1.36, reparsed.getLinkMass("Hood").in(Kilograms), 1e-6);
    assertEquals(0.018, reparsed.getJointMomentOfInertia("Hood").in(KilogramSquareMeters), 1e-6);
    assertEquals(0.02, reparsed.getJointMomentOfInertia("Flywheel").in(KilogramSquareMeters), 1e-6);
    assertEquals(9.0, reparsed.getJoint("Hood").minLimitAngle().in(Degrees), 1e-3);
    assertEquals(42.0, reparsed.getJoint("Hood").maxLimitAngle().in(Degrees), 1e-3);
    assertEquals(0.1016, reparsed.getJoint("Flywheel").wheelDiameter().in(Meters), 1e-6);
    assertEquals(1, reparsed.cameras().size());
  }

  @Test
  void importsStandardUrdfWithPackagePrefixAndInertia(@TempDir Path dir) throws Exception {
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="TestBotUrdf">
          <link name="base_link">
            <visual>
              <geometry>
                <mesh filename="package://testbot/meshes/chassis.stl" scale="1 1 1"/>
              </geometry>
            </visual>
            <inertial>
              <origin xyz="0.02 0.0 0.15"/>
              <mass value="42.0"/>
              <inertia ixx="2.1" ixy="0" ixz="0" iyy="3.4" iyz="0" izz="4.5"/>
            </inertial>
          </link>
          <link name="arm_link">
            <visual>
              <geometry>
                <mesh filename="package://testbot/meshes/arm.stl"/>
              </geometry>
            </visual>
            <inertial>
              <origin xyz="0.15 0.0 0.0"/>
              <mass value="3.5"/>
              <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.14" iyz="0" izz="0.12"/>
            </inertial>
          </link>
          <joint name="arm_joint" type="revolute">
            <parent link="base_link"/>
            <child link="arm_link"/>
            <origin xyz="0.25 0.0 0.40" rpy="0 0 0"/>
            <axis xyz="0 -1 0"/>
            <limit lower="0.1" upper="2.2" effort="40" velocity="3.0"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("robot.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotManifest manifest = CadManifestLoader.loadUrdf(urdfPath);
    assertEquals("TestBotUrdf", manifest.robotName());
    assertTrue(manifest.chassis().isPresent());
    assertEquals("base_link", manifest.chassis().get().name());
    assertEquals("meshes/chassis.stl", manifest.chassis().get().meshFile());
    assertEquals(42.0, manifest.getLinkMass("base_link").in(Kilograms), 1e-6);

    assertEquals(1, manifest.links().size());
    CadJointManifest armJoint = manifest.getJoint("arm_joint");
    assertEquals(CadJointType.REVOLUTE, armJoint.type());
    assertEquals(MotionAxis.Y, armJoint.axis());
    assertTrue(armJoint.inverted());
    assertEquals(0.25, armJoint.originMeters().getX(), 1e-6);
    assertEquals(
        0.14, manifest.getJointMomentOfInertia("arm_joint").in(KilogramSquareMeters), 1e-6);
  }

  @Test
  void meshParserAppliesExactCadMassPropertiesFromManifest(@TempDir Path dir) throws Exception {
    MeshParser.clearCache();
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("arm.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);

    String json =
        """
        {
          "schemaVersion": "1.0",
          "robotName": "MassOverrideBot",
          "chassis": {
            "name": "Chassis",
            "meshFile": "chassis.stl",
            "massProperties": {
              "volumeCubicMeters": 0.018,
              "massKilograms": 51.25,
              "centerOfMassMeters": [0.03, -0.01, 0.22],
              "ixxKgM2": 2.5,
              "iyyKgM2": 3.5,
              "izzKgM2": 4.8
            }
          },
          "links": [
            {
              "name": "Arm",
              "meshFile": "arm.stl"
            }
          ]
        }
        """;
    Path manifestFile = dir.resolve("yacsim-robot.json");
    Files.writeString(manifestFile, json, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromManifest(manifestFile);
    MeshMetadata chassisMeta = MeshParser.parse(cfg.getChassisAsset().orElseThrow());
    assertEquals(51.25, chassisMeta.massProperties().massKilograms(), 1e-6);
    assertEquals(3.5, chassisMeta.massProperties().iyyKgM2(), 1e-6);
    assertEquals(0.22, chassisMeta.massProperties().centerOfMassMeters().getZ(), 1e-6);
  }

  @Test
  void bindsPoseSupplierByComponentNameAfterUrdfImport(@TempDir Path dir) throws Exception {
    Files.createDirectories(dir.resolve("meshes"));
    Files.writeString(dir.resolve("meshes/chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("meshes/elevator.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);

    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="UrdfBindBot">
          <link name="Chassis">
            <visual><geometry><mesh filename="meshes/chassis.stl"/></geometry></visual>
          </link>
          <link name="Elevator">
            <visual><geometry><mesh filename="meshes/elevator.stl"/></geometry></visual>
          </link>
          <joint name="ElevatorJoint" type="prismatic">
            <parent link="Chassis"/>
            <child link="Elevator"/>
            <origin xyz="0 0 0.1"/>
            <axis xyz="0 0 1"/>
            <limit lower="0.0" upper="1.2"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("bot.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    Pose3d expected = new Pose3d(new Translation3d(0.0, 0.0, 0.75), new Rotation3d());
    CadRobotConfig cfg =
        CadRobotConfig.fromUrdf(urdfPath).withComponentPoseSupplier("Elevator", () -> expected);

    assertTrue(cfg.getManifest().isPresent());
    assertEquals(1, cfg.getComponents().size());
    assertEquals(expected, cfg.getComponents().get(0).getPoseSupplier().get());
  }

  @Test
  void rejectsInvalidJointTypeAndMissingComponentName(@TempDir Path dir) throws Exception {
    assertThrows(IllegalArgumentException.class, () -> CadJointType.fromString("spherical"));

    String json =
        """
        {
          "schemaVersion": "1.0",
          "robotName": "ValidationBot",
          "links": []
        }
        """;
    Path manifestFile = dir.resolve("yacsim-robot.json");
    Files.writeString(manifestFile, json, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromManifest(manifestFile);
    assertThrows(
        IllegalArgumentException.class,
        () -> cfg.withComponentPoseSupplier("NonExistentLink", Pose3d::new));
  }

  @Test
  void exposesParentPoseSupplierForMountedRollersInManifest(@TempDir Path dir) throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("arm.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("roller.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);

    String json =
        """
        {
          "schemaVersion": "1.0",
          "robotName": "MountedRollerBot",
          "chassis": { "name": "Chassis", "meshFile": "chassis.stl" },
          "links": [
            { "name": "Slamtake", "meshFile": "arm.stl" },
            { "name": "IntakeRollerMesh", "meshFile": "roller.stl", "collisionEnabled": false }
          ],
          "joints": [
            {
              "name": "Slamtake",
              "type": "revolute",
              "parentLink": "Chassis",
              "childLink": "Slamtake",
              "originMeters": [0.3, 0.0, 0.2],
              "axis": "Y"
            },
            {
              "name": "IntakeRoller",
              "type": "continuous",
              "parentLink": "Slamtake",
              "childLink": "IntakeRollerMesh",
              "originMeters": [0.32, 0.0, 0.0],
              "axis": "Y",
              "wheelRadiusMeters": 0.0254
            }
          ]
        }
        """;
    Path manifestFile = dir.resolve("yacsim-robot.json");
    Files.writeString(manifestFile, json, StandardCharsets.UTF_8);

    Pose3d armPose = new Pose3d(new Translation3d(0.3, 0.0, 0.2), new Rotation3d());
    CadRobotConfig cfg =
        CadRobotConfig.fromManifest(manifestFile)
            .withComponentPoseSupplier("Slamtake", () -> armPose);

    assertTrue(cfg.getComponentPoseSupplier("Slamtake").isPresent());
    assertEquals(armPose, cfg.getComponentPoseSupplier("Slamtake").get().get());
    assertTrue(cfg.hasComponent("IntakeRollerMesh"));
    assertFalse(cfg.getComponents().get(1).isCollisionEnabled());
  }

  @Test
  void matchesCadAndCodeNamesCaseAndSeparatorInsensitively(@TempDir Path dir) throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("arm.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);

    String json =
        """
        {
          "schemaVersion": "1.0",
          "robotName": "NamingConventionBot",
          "chassis": { "name": "Chassis", "meshFile": "chassis.stl" },
          "links": [
            { "name": "yacsim_Intake_Arm", "meshFile": "arm.stl" }
          ],
          "joints": [
            {
              "name": "yacsim_Intake_Arm",
              "type": "revolute",
              "parentLink": "Chassis",
              "childLink": "yacsim_Intake_Arm",
              "originMeters": [0.3, 0.0, 0.2],
              "axis": "Y"
            }
          ]
        }
        """;
    Path manifestFile = dir.resolve("yacsim-robot.json");
    Files.writeString(manifestFile, json, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromManifest(manifestFile);
    CadRobotManifest manifest = cfg.getManifest().orElseThrow();
    assertEquals("yacsim_Intake_Arm", manifest.getJoint("IntakeArm").name());
    assertEquals("yacsim_Intake_Arm", manifest.getJoint("intake arm").name());
    assertTrue(cfg.hasComponent("IntakeArm"));
    assertTrue(cfg.hasComponent("intake_arm"));
  }

  @Test
  void importsNativeOnshapeUrdfZipWithNestedMeshesAndInertia(@TempDir Path dir) throws Exception {
    MeshParser.clearCache();
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="OnshapeNativeBot">
          <link name="Chassis">
            <visual>
              <geometry><box size="0.1 0.1 0.1"/></geometry>
            </visual>
            <visual>
              <geometry><mesh filename="./meshes/chassis.stl"/></geometry>
            </visual>
            <inertial>
              <origin xyz="0.01 0.0 0.14"/>
              <mass value="45.0"/>
              <inertia ixx="2.2" ixy="0" ixz="0" iyy="3.1" iyz="0" izz="4.4"/>
            </inertial>
          </link>
          <link name="Intake_Arm">
            <visual>
              <geometry><mesh filename="./meshes/arm.stl"/></geometry>
            </visual>
            <inertial>
              <origin xyz="0.12 0.0 0.0"/>
              <mass value="2.8"/>
              <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.09" iyz="0" izz="0.07"/>
            </inertial>
          </link>
          <joint name="Intake_Arm_Joint" type="revolute">
            <parent link="Chassis"/>
            <child link="Intake_Arm"/>
            <origin xyz="0.28 0.0 0.22" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="0.0" upper="1.5707963"/>
          </joint>
        </robot>
        """;

    Path zipPath = dir.resolve("onshape_robot_urdf.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("onshape_export/robot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();

      zos.putNextEntry(new ZipEntry("onshape_export/meshes/chassis.stl"));
      zos.write(UNIT_CUBE_STL.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();

      zos.putNextEntry(new ZipEntry("onshape_export/meshes/arm.stl"));
      zos.write(UNIT_CUBE_STL.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
    }

    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath);
    CadRobotManifest manifest = cfg.getManifest().orElseThrow();
    assertEquals("OnshapeNativeBot", cfg.getRobotName());
    assertEquals("meshes/chassis.stl", manifest.chassis().orElseThrow().meshFile());
    assertTrue(cfg.hasComponent("IntakeArm"));
    assertEquals(45.0, manifest.getLinkMass("Chassis").in(Kilograms), 1e-6);
    assertEquals(
        0.09, manifest.getJointMomentOfInertia("IntakeArmJoint").in(KilogramSquareMeters), 1e-6);
    assertEquals(90.0, manifest.getJoint("IntakeArmJoint").maxLimitAngle().in(Degrees), 1e-3);

    MeshMetadata chassisMeta = MeshParser.parse(cfg.getChassisAsset().orElseThrow());
    assertEquals(45.0, chassisMeta.massProperties().massKilograms(), 1e-6);
  }

  @Test
  void rejectsZipSlipTraversalInUrdfZip(@TempDir Path dir) throws Exception {
    Path zipPath = dir.resolve("malicious.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("../escaped.txt"));
      zos.write("blocked".getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
    }

    IOException ex =
        assertThrows(
            IOException.class, () -> CadManifestLoader.loadUrdfZip(zipPath, dir.resolve("out")));
    assertTrue(ex.getMessage().contains("Zip-Slip"));
  }

  @Test
  void safeFallbackGettersReturnDefaultsWhenUrdfOmitsLimitsOrMass(@TempDir Path dir)
      throws Exception {
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="MinimalUrdfBot">
          <link name="Chassis"/>
          <link name="Arm"/>
          <joint name="Arm" type="revolute">
            <parent link="Chassis"/>
            <child link="Arm"/>
            <origin xyz="0.2 0.0 0.3"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("minimal.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotManifest manifest = CadManifestLoader.loadUrdf(urdfPath);
    CadJointManifest arm = manifest.getJoint("Arm");

    assertEquals(-5.0, arm.minLimitAngleOr(Degrees.of(-5.0)).in(Degrees), 1e-6);
    assertEquals(110.0, arm.maxLimitAngleOr(Degrees.of(110.0)).in(Degrees), 1e-6);
    assertEquals(0.0, arm.startingAngleOr(Degrees.of(0.0)).in(Degrees), 1e-6);
    assertEquals(0.42, arm.lengthOr(Meters.of(0.42)).in(Meters), 1e-6);
    assertEquals(0.1016, arm.wheelDiameterOr(Meters.of(0.1016)).in(Meters), 1e-6);
    assertEquals(
        0.05, arm.momentOfInertiaOr(KilogramSquareMeters.of(0.05)).in(KilogramSquareMeters), 1e-6);
    assertEquals(3.2, manifest.getLinkMassOr("Arm", Kilograms.of(3.2)).in(Kilograms), 1e-6);
    assertEquals(
        0.08,
        manifest
            .getJointMomentOfInertiaOr("Arm", KilogramSquareMeters.of(0.08))
            .in(KilogramSquareMeters),
        1e-6);
  }

  @SuppressWarnings("unchecked")
  @Test
  void mergesMultiVisualOnshapeUrdfLinkAndResolvesRosPackageRoot(@TempDir Path dir)
      throws Exception {
    MeshParser.clearCache();
    byte[] partAGlb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f);
    byte[] partBGlb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.3f);

    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="main_assembly">
          <link name="root">
            <visual>
              <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
              <geometry><mesh filename="package://main_assembly/meshes/part_a.glb"/></geometry>
            </visual>
            <visual>
              <origin xyz="0.5 0.0 0.1" rpy="0 0 1.5707963"/>
              <geometry><mesh filename="package://main_assembly/meshes/part_a.glb"/></geometry>
            </visual>
            <visual>
              <origin xyz="-0.2 0.3 0.4" rpy="0 0 0"/>
              <geometry><mesh filename="package://main_assembly/meshes/part_b.glb"/></geometry>
            </visual>
            <inertial>
              <origin xyz="0 0 0.15"/>
              <mass value="15.34"/>
              <inertia ixx="1.2" ixy="0" ixz="0" iyy="1.5" iyz="0" izz="2.1"/>
            </inertial>
          </link>
          <link name="left_hood">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://main_assembly/meshes/part_a.glb"/></geometry>
            </visual>
            <visual>
              <origin xyz="0.1 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://main_assembly/meshes/part_b.glb"/></geometry>
            </visual>
          </link>
          <joint name="hood_joint" type="revolute">
            <parent link="root"/>
            <child link="left_hood"/>
            <origin xyz="0.1 0.0 0.4" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="0.0" upper="0.8"/>
          </joint>
        </robot>
        """;

    Path zipPath = dir.resolve("gears_style_urdf.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("main_assembly/urdf/main_assembly.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();

      zos.putNextEntry(new ZipEntry("main_assembly/meshes/part_a.glb"));
      zos.write(partAGlb);
      zos.closeEntry();

      zos.putNextEntry(new ZipEntry("main_assembly/meshes/part_b.glb"));
      zos.write(partBGlb);
      zos.closeEntry();
    }

    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath);
    CadRobotManifest manifest = cfg.getManifest().orElseThrow();
    assertEquals("yacsim_merged/root.glb", manifest.chassis().orElseThrow().meshFile());
    assertTrue(manifest.chassis().orElseThrow().zeroedRotations().isEmpty());
    assertEquals("yacsim_merged/left_hood.glb", manifest.getLink("left_hood").meshFile());
    Pose3d hoodResting = cfg.getComponentPoseSupplier("left_hood").orElseThrow().get();
    assertEquals(0.1, hoodResting.getX(), 1e-6);
    assertEquals(0.4, hoodResting.getZ(), 1e-6);

    Path mergedRoot = cfg.getChassisAsset().orElseThrow().getMeshPath();
    assertTrue(Files.isRegularFile(mergedRoot));
    MeshMetadata meta = MeshParser.parse(cfg.getChassisAsset().orElseThrow());
    assertEquals(15.34, meta.massProperties().massKilograms(), 1e-6);

    // Inspect merged GLB JSON chunk: 2 deduplicated meshes, 3 wrapper nodes in scene 0
    byte[] mergedBytes = Files.readAllBytes(mergedRoot);
    ByteBuffer bb = ByteBuffer.wrap(mergedBytes).order(ByteOrder.LITTLE_ENDIAN);
    assertEquals(0x46546C67, bb.getInt());
    assertEquals(2, bb.getInt());
    bb.getInt();
    int jsonLen = bb.getInt();
    assertEquals(0x4E4F534A, bb.getInt());
    byte[] jsonBytes = new byte[jsonLen];
    bb.get(jsonBytes);
    Map<String, Object> gltf =
        (Map<String, Object>)
            MinimalJson.parse(new String(jsonBytes, StandardCharsets.UTF_8).trim());
    assertEquals(2, ((List<?>) gltf.get("meshes")).size());
    List<Object> scenes = (List<Object>) gltf.get("scenes");
    Map<String, Object> scene0 = (Map<String, Object>) scenes.get(0);
    assertEquals(3, ((List<?>) scene0.get("nodes")).size());
  }

  @Test
  void unifiedUrdfGroundingShiftsChassisAndChildJointsTogether(@TempDir Path dir) throws Exception {
    MeshParser.clearCache();
    byte[] chassisPart =
        buildMinimalTriangleGlb(0.0f, 0.0f, -0.25f, 0.3f, 0.0f, -0.25f, 0.0f, 0.3f, 0.10f);
    byte[] armPart = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.2f, 0.1f);

    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="GroundedUrdfBot">
          <link name="root">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/chassis.glb"/></geometry>
            </visual>
          </link>
          <link name="arm">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/arm.glb"/></geometry>
            </visual>
          </link>
          <joint name="arm_joint" type="revolute">
            <parent link="root"/>
            <child link="arm"/>
            <origin xyz="0.2 0.0 0.15" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;

    Path zipPath = dir.resolve("grounded_urdf.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      zos.putNextEntry(new ZipEntry("bot/meshes/chassis.glb"));
      zos.write(chassisPart);
      zos.closeEntry();
      zos.putNextEntry(new ZipEntry("bot/meshes/arm.glb"));
      zos.write(armPart);
      zos.closeEntry();
    }

    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath);
    // Chassis minZ was -0.25m, so unified grounding shifts both chassis and root-attached arm_joint
    // up by +0.25m (0.15 + 0.25 = 0.40m) with zero separation.
    Pose3d armResting = cfg.getComponentPoseSupplier("arm").orElseThrow().get();
    assertEquals(0.40, armResting.getZ(), 1e-5);
    assertEquals(0.0, cfg.getChassisAsset().orElseThrow().getZeroedPositionMeters().getZ(), 1e-6);
  }

  @SuppressWarnings("unchecked")
  @Test
  void cadUrdfOptionsExcludesFastenersAndCustomPatternsAndAliasesJoints(@TempDir Path dir)
      throws Exception {
    MeshParser.clearCache();
    byte[] glb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.2f, 0.1f);

    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="FastenerFilterBot">
          <link name="root">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/frame.glb"/></geometry>
            </visual>
            <visual>
              <origin xyz="0.1 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/_10_32_Nylock_Jam_Nut.glb"/></geometry>
            </visual>
            <visual>
              <origin xyz="0.3 0.3 0.06" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/Simplified_MAXSwerve_Module.glb"/></geometry>
            </visual>
          </link>
          <link name="bhcs_10_32_screw">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/bhcs_10_32_screw.glb"/></geometry>
            </visual>
          </link>
          <link name="am_5038_turntable_gear">
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry><mesh filename="package://bot/meshes/turntable.glb"/></geometry>
            </visual>
          </link>
          <joint name="revolute_screw" type="continuous">
            <parent link="root"/>
            <child link="bhcs_10_32_screw"/>
            <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="revolute_1" type="continuous">
            <parent link="root"/>
            <child link="am_5038_turntable_gear"/>
            <origin xyz="0.0 0.0 0.35" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>
        """;

    Path zipPath = dir.resolve("fastener_bot.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      for (String mesh :
          List.of(
              "frame.glb",
              "_10_32_Nylock_Jam_Nut.glb",
              "Simplified_MAXSwerve_Module.glb",
              "bhcs_10_32_screw.glb",
              "turntable.glb")) {
        zos.putNextEntry(new ZipEntry("bot/meshes/" + mesh));
        zos.write(glb);
        zos.closeEntry();
      }
    }

    CadUrdfOptions opts =
        CadUrdfOptions.recommended()
            .excludeMeshPatterns("MAXSwerve")
            .withJointAlias("revolute_1", "Turret");
    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath, opts);
    CadRobotManifest manifest = cfg.getManifest().orElseThrow();

    // Only turntable remains as a visual component (screw link filtered out)
    assertEquals(1, cfg.getComponents().size());
    assertTrue(cfg.hasComponent("am_5038_turntable_gear"));
    assertFalse(cfg.hasComponent("bhcs_10_32_screw"));

    // Joint alias works seamlessly
    assertEquals("am_5038_turntable_gear", manifest.getJoint("Turret").childLink());

    // Unbound collision disabled initially, then auto-enabled on bind
    assertFalse(cfg.getComponents().get(0).isCollisionEnabled());
    cfg.withComponentPoseSupplier("am_5038_turntable_gear", Pose3d::new);
    assertTrue(cfg.getComponents().get(0).isCollisionEnabled());
    assertTrue(cfg.isComponentBound("am_5038_turntable_gear"));

    // Chassis root.glb kept only 1 visual node (frame.glb; nut & MAXSwerve stripped)
    byte[] mergedRoot = Files.readAllBytes(cfg.getChassisAsset().orElseThrow().getMeshPath());
    ByteBuffer bb = ByteBuffer.wrap(mergedRoot).order(ByteOrder.LITTLE_ENDIAN);
    bb.position(12);
    int jsonLen = bb.getInt();
    bb.getInt();
    byte[] jsonBytes = new byte[jsonLen];
    bb.get(jsonBytes);
    Map<String, Object> gltf =
        (Map<String, Object>)
            MinimalJson.parse(new String(jsonBytes, StandardCharsets.UTF_8).trim());
    assertEquals(1, ((List<?>) gltf.get("meshes")).size());
  }

  @Test
  void cadUrdfOptionsKeepOnlyJointsRetainsNestedFkRestingPose(@TempDir Path dir) throws Exception {
    byte[] glb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="NestedFilterBot">
          <link name="root">
            <visual><geometry><mesh filename="package://bot/meshes/part.glb"/></geometry></visual>
          </link>
          <link name="turret_base">
            <visual><geometry><mesh filename="package://bot/meshes/part.glb"/></geometry></visual>
          </link>
          <link name="hood_link">
            <visual><geometry><mesh filename="package://bot/meshes/part.glb"/></geometry></visual>
          </link>
          <joint name="revolute_turret" type="continuous">
            <parent link="root"/>
            <child link="turret_base"/>
            <origin xyz="0.1 0.0 0.3" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="revolute_hood" type="revolute">
            <parent link="turret_base"/>
            <child link="hood_link"/>
            <origin xyz="0.05 0.0 0.2" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;

    Path zipPath = dir.resolve("nested_bot.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      zos.putNextEntry(new ZipEntry("bot/meshes/part.glb"));
      zos.write(glb);
      zos.closeEntry();
    }

    CadUrdfOptions opts =
        CadUrdfOptions.recommended()
            .withJointAlias("revolute_hood", "Hood")
            .keepOnlyJoints(List.of("Hood"));
    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath, opts);

    assertEquals(1, cfg.getComponents().size());
    assertTrue(cfg.hasComponent("hood_link"));
    assertFalse(cfg.hasComponent("turret_base"));
    // Cumulative FK resting pose still sums root -> turret_base (0.1, 0, 0.3) + turret_base -> hood
    // (0.05, 0, 0.2)
    Pose3d hoodResting = cfg.getComponentPoseSupplier("hood_link").orElseThrow().get();
    assertEquals(0.15, hoodResting.getX(), 1e-5);
    assertEquals(0.50, hoodResting.getZ(), 1e-5);
  }

  @Test
  void diagnosticReportHighlightsBoundAndUnboundJointsWithAliasHints(@TempDir Path dir)
      throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("turret.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("hood.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);

    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="DiagBot">
          <link name="root">
            <visual><geometry><mesh filename="chassis.stl"/></geometry></visual>
          </link>
          <link name="turret_link">
            <visual><geometry><mesh filename="turret.stl"/></geometry></visual>
          </link>
          <link name="hood_link">
            <visual><geometry><mesh filename="hood.stl"/></geometry></visual>
          </link>
          <joint name="Turret" type="revolute">
            <parent link="root"/>
            <child link="turret_link"/>
            <origin xyz="0 0 0.3"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="revolute_8" type="revolute">
            <parent link="turret_link"/>
            <child link="hood_link"/>
            <origin xyz="0 0 0.2"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("diag.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromUrdf(urdfPath);
    cfg.withComponentPoseSupplier("turret_link", Pose3d::new);

    String report = YacSimDiagnostic.formatReport(cfg);
    assertTrue(report.contains("[BOUND]"));
    assertTrue(report.contains("Turret"));
    assertTrue(report.contains("[UNBOUND]"));
    assertTrue(report.contains("revolute_8"));
    assertTrue(report.contains("withJointAlias(\"revolute_8\""));
  }

  @Test
  void attachSwerveModulesOverlaysSpinningWheelsOntoUrdfChassis(@TempDir Path dir)
      throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("fork.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("wheel.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);

    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="SwerveOverlayBot">
          <link name="root">
            <visual><geometry><mesh filename="chassis.stl"/></geometry></visual>
          </link>
        </robot>
        """;
    Path urdfPath = dir.resolve("swerve.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromUrdf(urdfPath);
    List<Translation3d> corners =
        List.of(
            new Translation3d(0.3, 0.3, 0.05),
            new Translation3d(0.3, -0.3, 0.05),
            new Translation3d(-0.3, 0.3, 0.05),
            new Translation3d(-0.3, -0.3, 0.05));
    SwerveModulePosition[] positions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0.5, Rotation2d.fromDegrees(45.0)),
          new SwerveModulePosition(0.5, Rotation2d.fromDegrees(45.0)),
          new SwerveModulePosition(0.5, Rotation2d.fromDegrees(45.0)),
          new SwerveModulePosition(0.5, Rotation2d.fromDegrees(45.0))
        };

    YamsCommands3Bridge.attachSwerveModules(
        cfg,
        new CadAssetConfig(dir.resolve("fork.stl")),
        new CadAssetConfig(dir.resolve("wheel.stl")),
        0.0508,
        corners,
        Pose2d::new,
        Rotation3d::new,
        () -> positions);

    assertTrue(cfg.getDrivetrainConfig().isPresent());
    // 4 steer forks + 4 spinning wheels = 8 articulated drivetrain components
    assertEquals(8, cfg.getAllArticulatedComponents().size());
  }

  @Test
  void chainFromManifestTracksNestedParentMotionAndIntermediateStaticLinks(@TempDir Path dir)
      throws Exception {
    byte[] glb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="NestedChainBot">
          <link name="root">
            <visual><geometry><mesh filename="package://bot/meshes/part.glb"/></geometry></visual>
          </link>
          <link name="turret_base">
            <visual><geometry><mesh filename="package://bot/meshes/part.glb"/></geometry></visual>
          </link>
          <link name="intermediate_bracket"/>
          <link name="hood_link">
            <visual><geometry><mesh filename="package://bot/meshes/part.glb"/></geometry></visual>
          </link>
          <joint name="Turret" type="continuous">
            <parent link="root"/>
            <child link="turret_base"/>
            <origin xyz="0.0 0.0 0.3" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="bracket_fixed" type="fixed">
            <parent link="turret_base"/>
            <child link="intermediate_bracket"/>
            <origin xyz="0.1 0.0 0.1" rpy="0 0 0"/>
          </joint>
          <joint name="Hood" type="revolute">
            <parent link="intermediate_bracket"/>
            <child link="hood_link"/>
            <origin xyz="0.2 0.0 0.05" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path zipPath = dir.resolve("nested_chain.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      zos.putNextEntry(new ZipEntry("bot/meshes/part.glb"));
      zos.write(glb);
      zos.closeEntry();
    }

    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath);
    // Bind Hood BEFORE Turret to prove dynamic ancestor lookup is order-independent
    java.util.function.Supplier<Pose3d> hoodChained =
        YamsCommands3Bridge.chainFromManifest(
            cfg, "Hood", () -> new Pose3d(0.2, 0.0, 0.05, new Rotation3d()));
    cfg.withComponentPoseSupplier("hood_link", hoodChained);

    // Rotate turret 90 deg around Z at (0, 0, 0.3)
    AtomicReference<Pose3d> turretPose =
        new AtomicReference<>(new Pose3d(0.0, 0.0, 0.3, new Rotation3d(0.0, 0.0, Math.PI / 2.0)));
    cfg.withComponentPoseSupplier("turret_base", turretPose::get);

    // Local X offset = 0.1 (bracket) + 0.2 (hood) = 0.3 along rotated +Y -> (0.0, 0.3, 0.45)
    Pose3d hoodWorld = cfg.getComponentPoseSupplier("hood_link").orElseThrow().get();
    assertEquals(0.0, hoodWorld.getX(), 1e-5);
    assertEquals(0.3, hoodWorld.getY(), 1e-5);
    assertEquals(0.45, hoodWorld.getZ(), 1e-5);
  }

  @Test
  void obbBroadphaseAndZeroAllocSatDetectOverlapAndRejectSeparatedBoxes() {
    BoundingBox3d box = new BoundingBox3d(-0.1, -0.1, -0.1, 0.1, 0.1, 0.1);
    Pose3d poseA = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    Pose3d poseFar = new Pose3d(1.0, 0.0, 0.0, new Rotation3d(0.2, 0.3, 0.4));
    Pose3d poseHit = new Pose3d(0.15, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.PI / 4.0));

    assertFalse(ObbCollisionDetector.spheresOverlap(box, poseA, box, poseFar));
    assertTrue(ObbCollisionDetector.testObbCollision(box, poseA, box, poseFar).isEmpty());
    assertTrue(ObbCollisionDetector.spheresOverlap(box, poseA, box, poseHit));
    OptionalDouble depth = ObbCollisionDetector.testObbCollision(box, poseA, box, poseHit);
    assertTrue(depth.isPresent());
    assertTrue(depth.getAsDouble() > 0.0);

    BoundingBox3d env = new BoundingBox3d(-0.5, -0.5, 0.0, 0.5, 0.5, 1.0);
    assertTrue(
        ObbCollisionDetector.testEnvelopeViolation(
                box, new Pose3d(0.0, 0.0, 0.2, new Rotation3d()), env)
            .isEmpty());
    OptionalDouble viol =
        ObbCollisionDetector.testEnvelopeViolation(
            box, new Pose3d(0.55, 0.0, 0.2, new Rotation3d()), env);
    assertTrue(viol.isPresent());
    assertEquals(0.15, viol.getAsDouble(), 1e-5);
  }

  @Test
  void meshParserAppliesGltfSceneGraphNodeTransformsToBounds(@TempDir Path dir) throws Exception {
    MeshParser.clearCache();
    byte[] partGlb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.2f, 0.1f);
    Path partPath = dir.resolve("part.glb");
    Files.write(partPath, partGlb);

    Path mergedPath = dir.resolve("transformed.glb");
    GlbMerger.merge(
        List.of(
            new GlbMerger.VisualInstance(
                partPath, new Translation3d(1.0, 2.0, 0.5), new Rotation3d(), 1.0, true)),
        mergedPath);

    MeshMetadata meta = MeshParser.parse(new CadAssetConfig(mergedPath));
    assertEquals(1.0, meta.localBounds().minX(), 1e-5);
    assertEquals(1.2, meta.localBounds().maxX(), 1e-5);
    assertEquals(2.0, meta.localBounds().minY(), 1e-5);
    assertEquals(0.5, meta.localBounds().minZ(), 1e-5);
    assertEquals(0.6, meta.localBounds().maxZ(), 1e-5);
  }

  @SuppressWarnings("unchecked")
  @Test
  void cadUrdfOptionsBakeUnboundLinksIntoChassisAndTagNoSimplify(@TempDir Path dir)
      throws Exception {
    MeshParser.clearCache();
    byte[] glb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="BakeBot">
          <link name="root">
            <visual><geometry><mesh filename="package://bot/meshes/frame_rail.glb"/></geometry></visual>
          </link>
          <link name="static_plate">
            <visual><geometry><mesh filename="package://bot/meshes/plate.glb"/></geometry></visual>
          </link>
          <link name="hood_link">
            <visual><geometry><mesh filename="package://bot/meshes/hood.glb"/></geometry></visual>
          </link>
          <joint name="plate_mate" type="revolute">
            <parent link="root"/>
            <child link="static_plate"/>
            <origin xyz="0.4 0.0 0.2" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
          </joint>
          <joint name="Hood" type="revolute">
            <parent link="root"/>
            <child link="hood_link"/>
            <origin xyz="0.0 0.0 0.5" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path zipPath = dir.resolve("bake_bot.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      for (String m : List.of("frame_rail.glb", "plate.glb", "hood.glb")) {
        zos.putNextEntry(new ZipEntry("bot/meshes/" + m));
        zos.write(glb);
        zos.closeEntry();
      }
    }

    CadUrdfOptions opts =
        CadUrdfOptions.recommended()
            .keepOnlyJoints(List.of("Hood"))
            .bakeUnboundLinksIntoChassis(true)
            .noSimplifyMeshPatterns("frame_rail");
    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath, opts);

    // Only Hood remains as an articulated component; static_plate was baked into root.glb
    assertEquals(1, cfg.getComponents().size());
    assertTrue(cfg.hasComponent("hood_link"));
    assertFalse(cfg.hasComponent("static_plate"));

    // Merged root.glb now spans out to X = 0.4 + 0.1 = 0.5 from the baked static_plate
    MeshMetadata rootMeta = MeshParser.parse(cfg.getChassisAsset().orElseThrow());
    assertEquals(0.5, rootMeta.localBounds().maxX(), 1e-5);

    // Verify NOSIMPLIFY node tag in root.glb JSON chunk
    byte[] mergedBytes = Files.readAllBytes(cfg.getChassisAsset().orElseThrow().getMeshPath());
    ByteBuffer bb = ByteBuffer.wrap(mergedBytes).order(ByteOrder.LITTLE_ENDIAN);
    bb.position(12);
    int jsonLen = bb.getInt();
    bb.getInt();
    byte[] jsonBytes = new byte[jsonLen];
    bb.get(jsonBytes);
    String gltfJson = new String(jsonBytes, StandardCharsets.UTF_8);
    assertTrue(gltfJson.contains("NOSIMPLIFY_frame_rail"));
  }

  @Test
  void cadRobotSimulatorInitializesComponentPosesToUrdfRestingPosesBeforeSimIterate(
      @TempDir Path dir) throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("arm.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="InitPoseBot">
          <link name="root">
            <visual><geometry><mesh filename="chassis.stl"/></geometry></visual>
          </link>
          <link name="arm">
            <visual><geometry><mesh filename="arm.stl"/></geometry></visual>
          </link>
          <joint name="Arm" type="revolute">
            <parent link="root"/>
            <child link="arm"/>
            <origin xyz="0.25 0.0 0.45"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("init_pose.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromUrdf(urdfPath);
    try (CadRobotSimulator sim = new CadRobotSimulator(cfg, false)) {
      // Before simIterate() is ever called, initial snapshot already holds true resting pose
      Pose3d[] initial = sim.getLatestComponentPoses();
      assertEquals(1, initial.length);
      assertEquals(0.25, initial[0].getX(), 1e-6);
      assertEquals(0.45, initial[0].getZ(), 1e-6);
    }
  }

  @Test
  void continuousRollerCollisionPreservationAndFuzzyAliasSuggestions(@TempDir Path dir)
      throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("roller.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("turntable.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="RollerAliasBot">
          <link name="root">
            <visual><geometry><mesh filename="chassis.stl"/></geometry></visual>
          </link>
          <link name="intake_roller">
            <visual><geometry><mesh filename="roller.stl"/></geometry></visual>
          </link>
          <link name="am_5038_turntable_gear">
            <visual><geometry><mesh filename="turntable.stl"/></geometry></visual>
          </link>
          <joint name="roller_spin" type="continuous">
            <parent link="root"/>
            <child link="intake_roller"/>
            <origin xyz="0.3 0 0.2"/>
            <axis xyz="0 1 0"/>
          </joint>
          <joint name="revolute_1" type="continuous">
            <parent link="root"/>
            <child link="am_5038_turntable_gear"/>
            <origin xyz="0 0 0.35"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("roller_alias.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromUrdf(urdfPath, CadUrdfOptions.recommended());
    cfg.withComponentPoseSupplier("intake_roller", Pose3d::new, false);
    assertTrue(cfg.isComponentBound("intake_roller"));
    assertFalse(
        cfg.getComponents().stream()
            .filter(c -> c.getName().equals("intake_roller"))
            .findFirst()
            .orElseThrow()
            .isCollisionEnabled());

    CadRobotManifest manifest = cfg.getManifest().orElseThrow();
    assertEquals(
        "revolute_1",
        YacSimDiagnostic.suggestBestJointMatch("Turret", manifest.joints()).orElse(""));
    assertEquals(
        "roller_spin",
        YacSimDiagnostic.suggestBestJointMatch("IntakeRoller", manifest.joints()).orElse(""));
  }

  @Test
  void yacSimYamsFactoryShortCircuitsOnRealRobotAndReturnsMechanismUnchanged(@TempDir Path dir)
      throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("arm.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="RealRobotGuardBot">
          <link name="root"><visual><geometry><mesh filename="chassis.stl"/></geometry></visual></link>
          <link name="arm"><visual><geometry><mesh filename="arm.stl"/></geometry></visual></link>
          <joint name="Arm" type="revolute">
            <parent link="root"/>
            <child link="arm"/>
            <origin xyz="0.2 0.0 0.3"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("real_guard.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromUrdf(urdfPath);
    AtomicInteger simTicks = new AtomicInteger(0);
    // Simulate RobotBase.isReal() by passing isSimulationSupplier = () -> false
    try (YacSimYamsFactory factory = YacSimYamsFactory.of(cfg, () -> false)) {
      assertFalse(factory.isSimulationActive());
      String dummyMech =
          factory.registerComponent(
              "Arm", "RealArmInstance", Pose3d::new, simTicks::incrementAndGet, () -> {});
      assertEquals("RealArmInstance", dummyMech);
      assertFalse(cfg.isComponentBound("arm"));
      assertTrue(factory.startSimulator(false).isEmpty());
      factory.simIterate();
      factory.updateTelemetry();
      assertEquals(0, simTicks.get());
    }
  }

  @Test
  void yacSimYamsFactoryBindsChainsAndStepsMechanismsInSimulation(@TempDir Path dir)
      throws Exception {
    Files.writeString(dir.resolve("chassis.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    Files.writeString(dir.resolve("arm.stl"), UNIT_CUBE_STL, StandardCharsets.UTF_8);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="SimFactoryBot">
          <link name="root"><visual><geometry><mesh filename="chassis.stl"/></geometry></visual></link>
          <link name="arm"><visual><geometry><mesh filename="arm.stl"/></geometry></visual></link>
          <joint name="Arm" type="revolute">
            <parent link="root"/>
            <child link="arm"/>
            <origin xyz="0.2 0.0 0.3"/>
            <axis xyz="0 1 0"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("sim_factory.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotConfig cfg = CadRobotConfig.fromUrdf(urdfPath);
    AtomicInteger simTicks = new AtomicInteger(0);
    AtomicInteger telemetryTicks = new AtomicInteger(0);
    try (YacSimYamsFactory factory = YacSimYamsFactory.of(cfg, () -> true)) {
      assertTrue(factory.isSimulationActive());
      String mech =
          factory.registerComponent(
              "Arm",
              "SimArmInstance",
              () -> new Pose3d(0.25, 0.0, 0.55, new Rotation3d()),
              simTicks::incrementAndGet,
              telemetryTicks::incrementAndGet);
      assertEquals("SimArmInstance", mech);
      assertTrue(cfg.isComponentBound("arm"));
      assertEquals(1, factory.getBoundMechanisms().size());

      CadRobotSimulator sim = factory.startSimulator(false).orElseThrow();
      factory.simIterate();
      factory.updateTelemetry();
      assertEquals(1, simTicks.get());
      assertEquals(1, telemetryTicks.get());
      assertEquals(0.55, sim.getLatestComponentPoses()[0].getZ(), 1e-6);
    }
  }

  @Test
  void bakeUnboundLinksAutomaticallyBakesSubtreeChildrenIntoMovingParentJoint(@TempDir Path dir)
      throws Exception {
    MeshParser.clearCache();
    byte[] glb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="SubtreeBakeBot">
          <link name="root">
            <visual><geometry><mesh filename="package://bot/meshes/chassis.glb"/></geometry></visual>
          </link>
          <link name="turret_gear">
            <visual><geometry><mesh filename="package://bot/meshes/turret.glb"/></geometry></visual>
          </link>
          <link name="turret_bracket">
            <visual><geometry><mesh filename="package://bot/meshes/bracket.glb"/></geometry></visual>
          </link>
          <joint name="revolute_1" type="continuous">
            <parent link="root"/>
            <child link="turret_gear"/>
            <origin xyz="0.0 0.0 0.30" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="cylindrical_sub" type="continuous">
            <parent link="turret_gear"/>
            <child link="turret_bracket"/>
            <origin xyz="0.25 0.0 0.15" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>
        """;
    Path zipPath = dir.resolve("subtree_bake.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      for (String m : List.of("chassis.glb", "turret.glb", "bracket.glb")) {
        zos.putNextEntry(new ZipEntry("bot/meshes/" + m));
        zos.write(glb);
        zos.closeEntry();
      }
    }

    CadUrdfOptions opts =
        CadUrdfOptions.recommended()
            .withJointAlias("revolute_1", "Turret")
            .keepOnlyJoints(List.of("Turret"))
            .bakeUnboundLinksIntoChassis(true);
    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath, opts);

    // Only turret_gear remains as an articulated component; turret_bracket baked into
    // turret_gear.glb!
    assertEquals(1, cfg.getComponents().size());
    assertTrue(cfg.hasComponent("turret_gear"));
    assertFalse(cfg.hasComponent("turret_bracket"));

    // turret_gear.glb now includes turret_bracket at local X = 0.25 (+0.10 = 0.35), Z = 0.15 (+0.10
    // = 0.25)
    MeshMetadata turretMeta = MeshParser.parse(cfg.getComponents().get(0).getAssetConfig());
    assertEquals(0.35, turretMeta.localBounds().maxX(), 1e-5);
    assertEquals(0.25, turretMeta.localBounds().maxZ(), 1e-5);
  }

  @Test
  void attachLinksToJointReparentsMismatedRootPartsIntoMovingMechanismGlb(@TempDir Path dir)
      throws Exception {
    MeshParser.clearCache();
    byte[] glb = buildMinimalTriangleGlb(0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="ReparentBot">
          <link name="root">
            <visual><geometry><mesh filename="package://bot/meshes/chassis.glb"/></geometry></visual>
          </link>
          <link name="turret_gear">
            <visual><geometry><mesh filename="package://bot/meshes/turret.glb"/></geometry></visual>
          </link>
          <link name="launcher_side_plate">
            <visual><geometry><mesh filename="package://bot/meshes/plate.glb"/></geometry></visual>
          </link>
          <joint name="revolute_1" type="continuous">
            <parent link="root"/>
            <child link="turret_gear"/>
            <origin xyz="0.10 0.0 0.30" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="planar_mismated" type="continuous">
            <parent link="root"/>
            <child link="launcher_side_plate"/>
            <origin xyz="0.40 0.0 0.50" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
          </joint>
        </robot>
        """;
    Path zipPath = dir.resolve("reparent.zip");
    try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(zipPath))) {
      zos.putNextEntry(new ZipEntry("bot/urdf/bot.urdf"));
      zos.write(urdf.getBytes(StandardCharsets.UTF_8));
      zos.closeEntry();
      for (String m : List.of("chassis.glb", "turret.glb", "plate.glb")) {
        zos.putNextEntry(new ZipEntry("bot/meshes/" + m));
        zos.write(glb);
        zos.closeEntry();
      }
    }

    CadUrdfOptions opts =
        CadUrdfOptions.recommended()
            .withJointAlias("revolute_1", "Turret")
            .keepOnlyJoints(List.of("Turret"))
            .bakeUnboundLinksIntoChassis(true)
            .attachLinksToJoint("Turret", "launcher_side_plate");
    CadRobotConfig cfg = CadRobotConfig.fromUrdfOrZip(zipPath, opts);

    assertEquals(1, cfg.getComponents().size());
    assertFalse(cfg.hasComponent("launcher_side_plate"));
    // Relative to turret_gear at (0.10, 0, 0.30), launcher_side_plate at (0.40, 0, 0.50) sits at
    // local (0.30, 0, 0.20), so maxX = 0.30 + 0.10 = 0.40 and maxZ = 0.20 + 0.10 = 0.30!
    MeshMetadata turretMeta = MeshParser.parse(cfg.getComponents().get(0).getAssetConfig());
    assertEquals(0.40, turretMeta.localBounds().maxX(), 1e-5);
    assertEquals(0.30, turretMeta.localBounds().maxZ(), 1e-5);
  }

  private static byte[] buildMinimalTriangleGlb(
      float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2) {
    ByteBuffer bin = ByteBuffer.allocate(36).order(ByteOrder.LITTLE_ENDIAN);
    bin.putFloat(x0).putFloat(y0).putFloat(z0);
    bin.putFloat(x1).putFloat(y1).putFloat(z1);
    bin.putFloat(x2).putFloat(y2).putFloat(z2);
    byte[] binBytes = bin.array();

    float minX = Math.min(x0, Math.min(x1, x2));
    float minY = Math.min(y0, Math.min(y1, y2));
    float minZ = Math.min(z0, Math.min(z1, z2));
    float maxX = Math.max(x0, Math.max(x1, x2));
    float maxY = Math.max(y0, Math.max(y1, y2));
    float maxZ = Math.max(z0, Math.max(z1, z2));

    String json =
        String.format(
                java.util.Locale.ROOT,
                """
            {"asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[%f,%f,%f],"max":[%f,%f,%f]}],"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36}],"buffers":[{"byteLength":36}]}
            """,
                minX,
                minY,
                minZ,
                maxX,
                maxY,
                maxZ)
            .trim();
    byte[] rawJson = json.getBytes(StandardCharsets.UTF_8);
    int pad = (4 - (rawJson.length % 4)) % 4;
    int jsonLen = rawJson.length + pad;
    int totalLen = 12 + 8 + jsonLen + 8 + binBytes.length;

    ByteBuffer out = ByteBuffer.allocate(totalLen).order(ByteOrder.LITTLE_ENDIAN);
    out.putInt(0x46546C67);
    out.putInt(2);
    out.putInt(totalLen);
    out.putInt(jsonLen);
    out.putInt(0x4E4F534A);
    out.put(rawJson);
    for (int i = 0; i < pad; i++) {
      out.put((byte) 0x20);
    }
    out.putInt(binBytes.length);
    out.putInt(0x004E4942);
    out.put(binBytes);
    return out.array();
  }
}
