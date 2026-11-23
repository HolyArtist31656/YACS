// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.physics;

import java.util.List;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * 15-axis Separating Axis Theorem (SAT) collision detector for oriented bounding boxes (OBBs) and
 * robot extension envelopes.
 */
public final class ObbCollisionDetector {
  private static final double EPSILON = 1e-9;

  private ObbCollisionDetector() {}

  /**
   * Tests two posed local bounding boxes for 3D OBB overlap using the 15-axis Separating Axis
   * Theorem.
   *
   * @param boxA Local bounds of component A.
   * @param poseA World/robot-frame pose of component A.
   * @param boxB Local bounds of component B.
   * @param poseB World/robot-frame pose of component B.
   * @return Minimum penetration depth in meters if overlapping, or empty if separated.
   */
  public static OptionalDouble testObbCollision(
      BoundingBox3d boxA, Pose3d poseA, BoundingBox3d boxB, Pose3d poseB) {
    Translation3d centerA = boxA.center().rotateBy(poseA.getRotation()).plus(poseA.getTranslation());
    Translation3d centerB = boxB.center().rotateBy(poseB.getRotation()).plus(poseB.getTranslation());
    double[][] axesA = basisAxes(poseA.getRotation()), axesB = basisAxes(poseB.getRotation());
    double[] extA = {boxA.halfX(), boxA.halfY(), boxA.halfZ()};
    double[] extB = {boxB.halfX(), boxB.halfY(), boxB.halfZ()};
    double[] tVec = {
      centerB.getX() - centerA.getX(),
      centerB.getY() - centerA.getY(),
      centerB.getZ() - centerA.getZ()
    };

    double minOverlap = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 3; i++) {
      double overlap = axisOverlap(axesA[i], tVec, axesA, extA, axesB, extB);
      if (overlap < 0.0) return OptionalDouble.empty();
      minOverlap = Math.min(minOverlap, overlap);
    }
    for (int j = 0; j < 3; j++) {
      double overlap = axisOverlap(axesB[j], tVec, axesA, extA, axesB, extB);
      if (overlap < 0.0) return OptionalDouble.empty();
      minOverlap = Math.min(minOverlap, overlap);
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double[] c = cross(axesA[i], axesB[j]);
        double lenSq = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
        if (lenSq < EPSILON) continue;
        double invLen = 1.0 / Math.sqrt(lenSq);
        c[0] *= invLen; c[1] *= invLen; c[2] *= invLen;
        double overlap = axisOverlap(c, tVec, axesA, extA, axesB, extB);
        if (overlap < 0.0) return OptionalDouble.empty();
        minOverlap = Math.min(minOverlap, overlap);
      }
    }
    return OptionalDouble.of(minOverlap);
  }

  /**
   * Tests whether any transformed corner of a posed bounding box breaches an allowed axis-aligned
   * extension envelope.
   *
   * @param componentBox Local bounds of the component.
   * @param componentPose Robot-frame pose of the component.
   * @param allowedEnvelope Legal axis-aligned extension envelope in meters.
   * @return Maximum violation distance in meters if breached, or empty if within bounds.
   */
  public static OptionalDouble testEnvelopeViolation(
      BoundingBox3d componentBox, Pose3d componentPose, BoundingBox3d allowedEnvelope) {
    double maxViolation = 0.0;
    for (Translation3d corner : componentBox.transformedCorners(componentPose)) {
      maxViolation = Math.max(maxViolation, allowedEnvelope.minX() - corner.getX());
      maxViolation = Math.max(maxViolation, corner.getX() - allowedEnvelope.maxX());
      maxViolation = Math.max(maxViolation, allowedEnvelope.minY() - corner.getY());
      maxViolation = Math.max(maxViolation, corner.getY() - allowedEnvelope.maxY());
      maxViolation = Math.max(maxViolation, allowedEnvelope.minZ() - corner.getZ());
      maxViolation = Math.max(maxViolation, corner.getZ() - allowedEnvelope.maxZ());
    }
    return maxViolation > 1e-6 ? OptionalDouble.of(maxViolation) : OptionalDouble.empty();
  }


  /**
   * Tests two multi-primitive OBB collections for overlap, returning the maximum penetration depth
   * among any colliding sub-box pair.
   *
   * @param boxesA Local sub-bounding boxes for component A.
   * @param poseA World/robot-frame pose of component A.
   * @param boxesB Local sub-bounding boxes for component B.
   * @param poseB World/robot-frame pose of component B.
   * @return Maximum penetration depth in meters if any sub-box pair overlaps, or empty if all are
   *     separated.
   */
  public static OptionalDouble testMultiObbCollision(
      List<BoundingBox3d> boxesA, Pose3d poseA, List<BoundingBox3d> boxesB, Pose3d poseB) {
    double maxPenetration = -1.0;
    for (BoundingBox3d a : boxesA) {
      for (BoundingBox3d b : boxesB) {
        OptionalDouble hit = testObbCollision(a, poseA, b, poseB);
        if (hit.isPresent()) {
          maxPenetration = Math.max(maxPenetration, hit.getAsDouble());
        }
      }
    }
    return maxPenetration >= 0.0 ? OptionalDouble.of(maxPenetration) : OptionalDouble.empty();
  }

  /**
   * Tests whether any sub-bounding box in {@code boxes} breaches {@code allowedEnvelope}.
   *
   * @param boxes Local sub-bounding boxes for the component.
   * @param componentPose Robot-frame pose of the component.
   * @param allowedEnvelope Legal axis-aligned extension envelope in meters.
   * @return Maximum violation distance in meters if breached, or empty if all sub-boxes are within
   *     bounds.
   */
  public static OptionalDouble testMultiEnvelopeViolation(
      List<BoundingBox3d> boxes, Pose3d componentPose, BoundingBox3d allowedEnvelope) {
    double maxViolation = 0.0;
    for (BoundingBox3d box : boxes) {
      OptionalDouble v = testEnvelopeViolation(box, componentPose, allowedEnvelope);
      if (v.isPresent()) {
        maxViolation = Math.max(maxViolation, v.getAsDouble());
      }
    }
    return maxViolation > 1e-6 ? OptionalDouble.of(maxViolation) : OptionalDouble.empty();
  }

  private static double[][] basisAxes(Rotation3d rot) {
    Translation3d x = new Translation3d(1.0, 0.0, 0.0).rotateBy(rot);
    Translation3d y = new Translation3d(0.0, 1.0, 0.0).rotateBy(rot);
    Translation3d z = new Translation3d(0.0, 0.0, 1.0).rotateBy(rot);
    return new double[][] {{x.getX(), x.getY(), x.getZ()}, {y.getX(), y.getY(), y.getZ()}, {z.getX(), z.getY(), z.getZ()}};
  }

  private static double axisOverlap(
      double[] axis, double[] tVec, double[][] axesA, double[] extA, double[][] axesB, double[] extB) {
    double dist = Math.abs(dot(tVec, axis));
    double projA = extA[0] * Math.abs(dot(axesA[0], axis)) + extA[1] * Math.abs(dot(axesA[1], axis)) + extA[2] * Math.abs(dot(axesA[2], axis));
    double projB = extB[0] * Math.abs(dot(axesB[0], axis)) + extB[1] * Math.abs(dot(axesB[1], axis)) + extB[2] * Math.abs(dot(axesB[2], axis));
    return (projA + projB) - dist;
  }

  private static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  private static double[] cross(double[] a, double[] b) {
    return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  }
}
