// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
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

  @Test
  void multiObbEliminatesFalsePositiveInsideLShapedCavity() {
    // L-shaped frame: bottom rail [0..1, 0..0.1, 0..0.1] + vertical post [0..0.1, 0..0.1, 0..1]
    List<BoundingBox3d> lBracket =
        List.of(
            new BoundingBox3d(0.0, 0.0, 0.0, 1.0, 0.1, 0.1),
            new BoundingBox3d(0.0, 0.0, 0.0, 0.1, 0.1, 1.0));
    // Outer enclosing box would cover [0..1, 0..0.1, 0..1] and falsely hit a part sitting in the
    // cavity
    BoundingBox3d partInCavity = new BoundingBox3d(0.4, 0.0, 0.4, 0.6, 0.1, 0.6);

    assertTrue(
        ObbCollisionDetector.testMultiObbCollision(
                lBracket, new Pose3d(), List.of(partInCavity), new Pose3d())
            .isEmpty());

    // Moving the part onto the bottom rail triggers a real hit
    BoundingBox3d partOnRail = new BoundingBox3d(0.4, 0.0, 0.02, 0.6, 0.1, 0.15);
    assertTrue(
        ObbCollisionDetector.testMultiObbCollision(
                lBracket, new Pose3d(), List.of(partOnRail), new Pose3d())
            .isPresent());
  }
}
