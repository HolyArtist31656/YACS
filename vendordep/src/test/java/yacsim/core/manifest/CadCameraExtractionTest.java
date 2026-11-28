// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.manifest;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.advantagescope.CadCameraConfig;

/**
 * Unit and fuzz tests for automated CAD camera coordinate extraction across URDF imports, JSON
 * round-trips, and {@link CadRobotManifest} camera transform accessors.
 */
class CadCameraExtractionTest {

  private static final double LINEAR_TOLERANCE_METERS = 1e-5;
  private static final double ANGULAR_TOLERANCE_RAD = 1e-5;
  private static final long FUZZ_SEED = 2027031656L;
  private static final int FUZZ_TRIALS = 50;

  @Test
  void extractsUrdfCameraLinksAcrossChainedFixedJointsWithModelOptics(@TempDir Path dir)
      throws Exception {
    String urdf =
        """
        <?xml version="1.0"?>
        <robot name="VisionRigBot">
          <link name="base_link"/>
          <link name="mast_link"/>
          <link name="cam_front_left_ov9281"/>
          <link name="Limelight4_Rear"/>
          <link name="questnav_mount"/>
          <joint name="base_to_mast" type="fixed">
            <parent link="base_link"/>
            <child link="mast_link"/>
            <origin xyz="0.10 0.0 0.30" rpy="0 0 0"/>
          </joint>
          <joint name="mast_to_front_cam" type="fixed">
            <parent link="mast_link"/>
            <child link="cam_front_left_ov9281"/>
            <origin xyz="0.18 0.12 0.15" rpy="0 -0.261799 0.523599"/>
          </joint>
          <joint name="base_to_ll4" type="fixed">
            <parent link="base_link"/>
            <child link="Limelight4_Rear"/>
            <origin xyz="-0.32 0.0 0.44" rpy="0 -0.174533 3.141593"/>
          </joint>
          <joint name="base_to_quest" type="fixed">
            <parent link="base_link"/>
            <child link="questnav_mount"/>
            <origin xyz="0.22 -0.15 0.50" rpy="0 0 -1.570796"/>
          </joint>
        </robot>
        """;
    Path urdfPath = dir.resolve("vision_rig.urdf");
    Files.writeString(urdfPath, urdf, StandardCharsets.UTF_8);

    CadRobotManifest manifest = CadManifestLoader.loadUrdf(urdfPath);
    assertEquals(3, manifest.cameras().size());

    CadCameraConfig front = manifest.getCamera("front_left_ov9281");
    assertEquals(70.0, front.fovDegrees(), 1e-6);
    assertEquals(1280, front.resolutionWidthPx());
    assertEquals(800, front.resolutionHeightPx());

    // Chained transform: (0.10, 0, 0.30) + (0.18, 0.12, 0.15) = (0.28, 0.12, 0.45)
    Transform3d frontTf = manifest.getCameraTransform("cam_front_left_ov9281");
    assertEquals(0.28, frontTf.getX(), LINEAR_TOLERANCE_METERS);
    assertEquals(0.12, frontTf.getY(), LINEAR_TOLERANCE_METERS);
    assertEquals(0.45, frontTf.getZ(), LINEAR_TOLERANCE_METERS);
    assertEquals(-0.261799, frontTf.getRotation().getY(), ANGULAR_TOLERANCE_RAD);
    assertEquals(0.523599, frontTf.getRotation().getZ(), ANGULAR_TOLERANCE_RAD);

    // Model presets inferred automatically
    CadCameraConfig ll4 = manifest.getCamera("limelight4-rear");
    assertEquals(82.0, ll4.fovDegrees(), 1e-6);
    assertEquals(1280, ll4.resolutionWidthPx());
    assertEquals(800, ll4.resolutionHeightPx());
    assertEquals(
        -0.32, manifest.getCameraTransform("Limelight4_Rear").getX(), LINEAR_TOLERANCE_METERS);

    CadCameraConfig quest = manifest.getCamera("questnav_mount");
    assertEquals(110.0, quest.fovDegrees(), 1e-6);
    assertEquals(1280, quest.resolutionWidthPx());
    assertEquals(720, quest.resolutionHeightPx());

    Map<String, Transform3d> all = manifest.getCameraTransforms();
    assertEquals(3, all.size());
    assertThrows(UnsupportedOperationException.class, () -> all.put("Extra", new Transform3d()));
  }

  @Test
  void cameraLookupSupportsFallbackAndHelpfulMissingError() {
    CadCameraConfig cam =
        new CadCameraConfig(
            "FrontLeft",
            new Transform3d(new Translation3d(0.25, 0.20, 0.40), new Rotation3d(0.0, -0.2, 0.3)),
            70.0,
            1280,
            800);
    CadRobotManifest manifest =
        new CadRobotManifest(
            CadRobotManifest.CURRENT_SCHEMA_VERSION,
            "LookupBot",
            Optional.empty(),
            OptionalDouble.empty(),
            OptionalDouble.empty(),
            true,
            false,
            false,
            Optional.empty(),
            List.of(),
            List.of(),
            List.of(cam));

    assertTrue(manifest.findCamera("cam_front_left").isPresent());
    assertTrue(manifest.findCamera("yacsim_camera_front-left").isPresent());
    assertEquals(0.25, manifest.getCameraTransform("frontleft").getX(), LINEAR_TOLERANCE_METERS);

    Transform3d fallback = new Transform3d(new Translation3d(1.0, 2.0, 3.0), Rotation3d.ZERO);
    assertEquals(fallback, manifest.getCameraTransformOr("NonExistentCam", fallback));

    IllegalArgumentException ex =
        assertThrows(IllegalArgumentException.class, () -> manifest.getCamera("MissingCam"));
    assertTrue(ex.getMessage().contains("FrontLeft"));
  }

  @Test
  void fuzzCameraTransformsRoundTripThroughJsonDeterministically(@TempDir Path dir)
      throws Exception {
    SplittableRandom rng = new SplittableRandom(FUZZ_SEED);
    for (int i = 0; i < FUZZ_TRIALS; i++) {
      double x = rng.nextDouble(-0.5, 0.5);
      double y = rng.nextDouble(-0.5, 0.5);
      double z = rng.nextDouble(0.05, 1.2);
      double roll = rng.nextDouble(-0.5, 0.5);
      double pitch = rng.nextDouble(-0.7, 0.7);
      double yaw = rng.nextDouble(-Math.PI, Math.PI);
      Transform3d expected =
          new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
      CadCameraConfig cam = new CadCameraConfig("Cam_" + i, expected, 75.0, 1280, 800);
      CadRobotManifest manifest =
          new CadRobotManifest(
              CadRobotManifest.CURRENT_SCHEMA_VERSION,
              "FuzzBot_" + i,
              Optional.empty(),
              OptionalDouble.empty(),
              OptionalDouble.empty(),
              true,
              false,
              false,
              Optional.empty(),
              List.of(),
              List.of(),
              List.of(cam));

      Path out = dir.resolve("fuzz_" + i + ".json");
      CadManifestLoader.writeJson(manifest, out);
      CadRobotManifest reloaded = CadManifestLoader.loadJson(out);
      Transform3d actual = reloaded.getCameraTransform("Cam_" + i);

      assertEquals(expected.getX(), actual.getX(), LINEAR_TOLERANCE_METERS);
      assertEquals(expected.getY(), actual.getY(), LINEAR_TOLERANCE_METERS);
      assertEquals(expected.getZ(), actual.getZ(), LINEAR_TOLERANCE_METERS);
      var qExp = expected.getRotation().getQuaternion();
      var qAct = actual.getRotation().getQuaternion();
      double qDot =
          Math.abs(
              qExp.getW() * qAct.getW()
                  + qExp.getX() * qAct.getX()
                  + qExp.getY() * qAct.getY()
                  + qExp.getZ() * qAct.getZ());
      assertTrue(qDot >= 1.0 - 1e-6, "Quaternion dot mismatch: " + qDot);
    }
  }
}
