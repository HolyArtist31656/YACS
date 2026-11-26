// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.commands3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;
import java.util.SplittableRandom;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.yams.YamsMechanismBridge;
import yams.commands3.mechanisms.DifferentialMechanism;
import yams.commands3.mechanisms.DoubleJointedArm;
import yams.commands3.mechanisms.Elevator;
import yams.commands3.swerve.SwerveDrive;

/**
 * Verifies YACSim's public read-only SPI bridges against YAMS 2027 mechanism and swerve APIs
 * without relying on bytecode-mocking agents on Java 25.
 */
class Yams2027SpiBridgeTest {
  private static final double EPS = 1e-9;
  private static final int FUZZ_TRIALS = 500;
  private static final long FUZZ_SEED = 20270804L;

  @Test
  void yams2027Commands3ClassesExposeExpectedPublicPoseSpiMethods() throws NoSuchMethodException {
    assertEquals(List.class, Elevator.class.getMethod("getCascadePose3d").getReturnType());
    assertEquals(
        List.class, Elevator.class.getMethod("getContinuousPose3d", boolean.class).getReturnType());
    assertEquals(Pose3d.class, DoubleJointedArm.class.getMethod("getPose3d").getReturnType());
    assertEquals(Pose3d.class, DoubleJointedArm.class.getMethod("getUpperPose3d").getReturnType());
    assertEquals(Pose3d.class, DifferentialMechanism.class.getMethod("getPose3d").getReturnType());
    assertEquals(Pose2d.class, SwerveDrive.class.getMethod("getSimPose").getReturnType());
  }

  @Test
  void bridgeOverloadsRejectNullMechanismsCleanly() {
    assertThrows(
        NullPointerException.class, () -> YamsCommands3Bridge.fromYamsCascadeElevator(null, 2));
    assertThrows(
        NullPointerException.class,
        () -> YamsCommands3Bridge.fromYamsContinuousElevator(null, 2, true));
    assertThrows(
        NullPointerException.class,
        () -> YamsCommands3Bridge.fromYamsMechanism((DoubleJointedArm) null));
    assertThrows(
        NullPointerException.class, () -> YamsCommands3Bridge.fromYamsDoubleJointedArmUpper(null));
    assertThrows(
        NullPointerException.class,
        () -> YamsCommands3Bridge.fromYamsMechanism((DifferentialMechanism) null));
  }

  @Test
  void poseListSuppliersExtractMultiStagePosesAndApplyMountOffset() {
    Pose3d stage1 = new Pose3d(0.0, 0.0, 0.45, new Rotation3d());
    Pose3d carriage = new Pose3d(0.0, 0.0, 0.90, new Rotation3d());
    Transform3d mountOffset = new Transform3d(0.05, 0.0, 0.10, new Rotation3d());

    List<Supplier<Pose3d>> stages =
        YamsMechanismBridge.fromPoseListSuppliers(() -> List.of(stage1, carriage), 2, mountOffset);
    assertEquals(2, stages.size());
    assertEquals(0.05, stages.get(0).get().getX(), EPS);
    assertEquals(0.55, stages.get(0).get().getZ(), EPS);
    assertEquals(1.00, stages.get(1).get().getZ(), EPS);

    Supplier<Pose3d> upper =
        YamsMechanismBridge.fromPoseSupplier(
            () -> new Pose3d(0.55, 0.0, 0.95, new Rotation3d(0.0, -0.3, 0.0)), new Transform3d());
    assertNotNull(upper.get());
    assertEquals(0.95, upper.get().getZ(), EPS);
  }

  @Test
  void poseListSupplierHandlesShortListsAndInvalidArgumentsSafely() {
    assertThrows(
        IllegalArgumentException.class,
        () -> YamsMechanismBridge.fromPoseListSupplier(List::of, -1, new Transform3d()));
    assertThrows(
        IllegalArgumentException.class,
        () -> YamsMechanismBridge.fromPoseListSuppliers(List::of, 0));

    Transform3d offset = new Transform3d(new Translation3d(0.1, 0.2, 0.3), new Rotation3d());
    Supplier<Pose3d> outOfRange = YamsMechanismBridge.fromPoseListSupplier(List::of, 2, offset);
    Pose3d fallback = outOfRange.get();
    assertEquals(0.1, fallback.getX(), EPS);
    assertEquals(0.2, fallback.getY(), EPS);
    assertEquals(0.3, fallback.getZ(), EPS);
  }

  @Test
  void seededPropertyFuzzPoseListSuppliersStayBoundedAndFinite() {
    SplittableRandom rng = new SplittableRandom(FUZZ_SEED);
    for (int trial = 0; trial < FUZZ_TRIALS; trial++) {
      int stages = 1 + rng.nextInt(4);
      Pose3d[] generated = new Pose3d[stages];
      for (int i = 0; i < stages; i++) {
        generated[i] =
            new Pose3d(
                rng.nextDouble(-2.0, 2.0),
                rng.nextDouble(-2.0, 2.0),
                rng.nextDouble(0.0, 3.0),
                new Rotation3d(
                    rng.nextDouble(-Math.PI, Math.PI),
                    rng.nextDouble(-Math.PI, Math.PI),
                    rng.nextDouble(-Math.PI, Math.PI)));
      }
      List<Pose3d> poseList = List.of(generated);
      List<Supplier<Pose3d>> suppliers =
          YamsMechanismBridge.fromPoseListSuppliers(() -> poseList, stages);
      for (int i = 0; i < stages; i++) {
        assertEquals(generated[i], suppliers.get(i).get());
      }
    }
  }
}
