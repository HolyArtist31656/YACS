// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.manifest;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.MeshMetadata;
import yacsim.core.cad.MeshParser;
import yacsim.core.model.CadRobotConfig;
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
}
