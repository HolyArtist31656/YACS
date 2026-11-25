// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.yams;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import yacsim.core.cad.MeshFormat;
import yacsim.core.cad.MeshMetadata;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;

class YamsMechanismBridgeTest {

  @Test
  void fromAngleSupplierComputesPivotPoseAndSupportsInversion() {
    AtomicReference<Angle> angle = new AtomicReference<>(Degrees.of(30.0));
    Translation3d pivot = new Translation3d(0.25, 0.0, 0.45);

    Supplier<Pose3d> normal =
        YamsMechanismBridge.fromAngleSupplier(angle::get, pivot, MotionAxis.Y);
    Supplier<Pose3d> inverted =
        YamsMechanismBridge.fromAngleSupplier(angle::get, pivot, MotionAxis.Y, true);

    Pose3d poseNormal = normal.get();
    Pose3d poseInverted = inverted.get();
    assertEquals(0.25, poseNormal.getX(), 1e-9);
    assertEquals(0.45, poseNormal.getZ(), 1e-9);
    assertEquals(Math.toRadians(30.0), poseNormal.getRotation().getY(), 1e-9);
    assertEquals(Math.toRadians(-30.0), poseInverted.getRotation().getY(), 1e-9);
  }

  @Test
  void fromDistanceSupplierComputesLinearPoseAlongAxisAndZeroOffset() {
    AtomicReference<Distance> height = new AtomicReference<>(Meters.of(0.55));
    Translation3d base = new Translation3d(0.01, 0.0, 0.10);
    Transform3d offset = new Transform3d(0.0, 0.0, 0.05, new Rotation3d());

    Supplier<Pose3d> poseSupplier =
        YamsMechanismBridge.fromDistanceSupplier(height::get, base, MotionAxis.Z, false, offset);

    Pose3d pose = poseSupplier.get();
    assertEquals(0.01, pose.getX(), 1e-9);
    assertEquals(0.0, pose.getY(), 1e-9);
    assertEquals(0.70, pose.getZ(), 1e-9);
  }

  @Test
  void chainParentChildComposesNestedMechanismTransforms() {
    Supplier<Pose3d> elevatorPose =
        () -> new Pose3d(new Translation3d(0.0, 0.0, 1.0), new Rotation3d());
    Supplier<Pose3d> wristLocal =
        () -> new Pose3d(new Translation3d(0.2, 0.0, 0.0), new Rotation3d(0.0, Math.PI / 4.0, 0.0));

    Pose3d worldWrist = YamsMechanismBridge.chainParentChild(elevatorPose, wristLocal).get();
    assertEquals(0.2, worldWrist.getX(), 1e-9);
    assertEquals(1.0, worldWrist.getZ(), 1e-9);
    assertEquals(Math.PI / 4.0, worldWrist.getRotation().getY(), 1e-9);
  }

  @Test
  void estimatesParallelAxisMomentOfInertiaAndMassFromMeshMetadata() {
    // 1.0 m x 0.1 m x 0.1 m arm extending from x=0 to x=1.0, center of mass at x=0.5
    BoundingBox3d armBox = new BoundingBox3d(0.0, -0.05, -0.05, 1.0, 0.05, 0.05);
    CadMassProperties props =
        CadMassProperties.fromBoundingBox(armBox, 2700.0).withTotalMassKg(6.0);
    MeshMetadata meta =
        new MeshMetadata(Path.of("arm.glb"), MeshFormat.GLB, 12, 100L, armBox, props);

    assertEquals(6.0, YamsMechanismBridge.estimateMass(meta).in(Kilograms), 1e-6);
    // I_y about pivot at origin = I_yy_cm + m * (0.5^2)
    double expectedMoi = props.iyyKgM2() + 6.0 * 0.25;
    assertEquals(
        expectedMoi,
        YamsMechanismBridge.estimateArmMoiKgM2(
            meta, new Translation3d(0.0, 0.0, 0.0), MotionAxis.Y),
        1e-6);
    assertEquals(
        expectedMoi,
        YamsMechanismBridge.estimateArmMoi(meta, new Translation3d(0.0, 0.0, 0.0), MotionAxis.Y)
            .in(KilogramSquareMeters),
        1e-6);
  }

  @Test
  void fromCascadingElevatorScalesStageHeightsProportionally() {
    AtomicReference<Distance> totalHeight = new AtomicReference<>(Meters.of(1.20));
    List<Supplier<Pose3d>> stages =
        YamsMechanismBridge.fromCascadingElevator(
            totalHeight::get, 3, new Translation3d(0.0, 0.0, 0.10), MotionAxis.Z);

    assertEquals(3, stages.size());
    assertEquals(0.50, stages.get(0).get().getZ(), 1e-6);
    assertEquals(0.90, stages.get(1).get().getZ(), 1e-6);
    assertEquals(1.30, stages.get(2).get().getZ(), 1e-6);
  }

  @Test
  void fromFourBarKeepsEndEffectorLevelAlongArc() {
    AtomicReference<Angle> angle = new AtomicReference<>(Degrees.of(90.0));
    Supplier<Pose3d> tip =
        YamsMechanismBridge.fromFourBar(
            angle::get, new Translation3d(0.2, 0.0, 0.3), 0.5, MotionAxis.Y, true);

    Pose3d pose = tip.get();
    // Rotating +X=(0.5,0,0) by -90 deg around Y lifts it straight up to (0, 0, +0.5)
    assertEquals(0.2, pose.getX(), 1e-6);
    assertEquals(0.8, pose.getZ(), 1e-6);
    assertEquals(0.0, pose.getRotation().getY(), 1e-6);
  }

  @Test
  void fromSpinningWheelAndMountedRollerComposeParentAndSpinRotation() {
    AtomicReference<Angle> wheelAngle = new AtomicReference<>(Degrees.of(90.0));
    Supplier<Pose3d> parentPose =
        () -> new Pose3d(new Translation3d(0.3, 0.0, 0.2), new Rotation3d());

    Supplier<Pose3d> standalone =
        YamsMechanismBridge.fromSpinningWheel(
            wheelAngle::get, new Translation3d(0.1, 0.0, 0.4), MotionAxis.Y);
    assertEquals(Math.PI / 2.0, standalone.get().getRotation().getY(), 1e-6);

    Supplier<Pose3d> mounted =
        YamsMechanismBridge.fromMountedRoller(
            parentPose, wheelAngle::get, new Translation3d(0.2, 0.0, 0.0), MotionAxis.Y, true);
    Pose3d pose = mounted.get();
    assertEquals(0.5, pose.getX(), 1e-6);
    assertEquals(0.2, pose.getZ(), 1e-6);
    assertEquals(-Math.PI / 2.0, pose.getRotation().getY(), 1e-6);
  }

  @Test
  void fromAngularVelocityIntegratesSpinAngleWithoutNaN() {
    AtomicReference<AngularVelocity> speed = new AtomicReference<>(RadiansPerSecond.of(50.0));
    Supplier<Pose3d> wheel =
        YamsMechanismBridge.fromAngularVelocity(
            speed::get, new Translation3d(0.0, 0.0, 0.5), MotionAxis.Y);

    Pose3d first = wheel.get();
    assertEquals(0.5, first.getZ(), 1e-6);
    assertEquals(1.0, first.getRotation().getY(), 1e-6); // 100 rad/s * 0.02s initial step
  }
}
