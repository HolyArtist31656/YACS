// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

class ObbCollisionDetectorTest {

  @Test
  void detectsSeparatedBoxes() {
    BoundingBox3d box = new BoundingBox3d(-0.1, -0.1, -0.1, 0.1, 0.1, 0.1);
    Pose3d poseA = new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
    Pose3d poseB = new Pose3d(new Translation3d(0.5, 0.0, 0.0), new Rotation3d());
    assertTrue(ObbCollisionDetector.testObbCollision(box, poseA, box, poseB).isEmpty());
  }

  @Test
  void detectsOverlappingRotatedBoxes() {
    BoundingBox3d arm = new BoundingBox3d(0.0, -0.05, -0.05, 0.6, 0.05, 0.05);
    BoundingBox3d bumper = new BoundingBox3d(-0.4, -0.4, 0.0, 0.4, 0.4, 0.15);
    Pose3d armPose =
        new Pose3d(new Translation3d(0.2, 0.0, 0.2), new Rotation3d(0.0, Math.toRadians(35), 0.0));
    OptionalDouble result =
        ObbCollisionDetector.testObbCollision(arm, armPose, bumper, new Pose3d());
    assertTrue(result.isPresent());
    assertTrue(result.getAsDouble() > 0.0);
  }

  @Test
  void detectsExtensionEnvelopeViolation() {
    BoundingBox3d allowed = new BoundingBox3d(-0.5, -0.5, 0.0, 0.5, 0.5, 1.8);
    BoundingBox3d carriage = new BoundingBox3d(-0.1, -0.1, 0.0, 0.1, 0.1, 0.2);
    assertTrue(
        ObbCollisionDetector.testEnvelopeViolation(
                carriage, new Pose3d(new Translation3d(0.0, 0.0, 1.5), new Rotation3d()), allowed)
            .isEmpty());
    assertTrue(
        ObbCollisionDetector.testEnvelopeViolation(
                carriage, new Pose3d(new Translation3d(0.0, 0.0, 1.75), new Rotation3d()), allowed)
            .isPresent());
  }
}
