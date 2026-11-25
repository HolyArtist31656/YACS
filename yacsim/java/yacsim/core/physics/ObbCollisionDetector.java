// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import java.util.List;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;

/**
 * Allocation-free 15-axis Separating Axis Theorem (SAT) collision detector with bounding-sphere
 * broadphase rejection for oriented bounding boxes (OBBs) and robot extension envelopes.
 */
public final class ObbCollisionDetector {
  private static final double EPSILON = 1e-9;

  private ObbCollisionDetector() {}

  /**
   * Computes the local bounding-sphere radius of {@code box} from its center to any corner.
   *
   * @param box Local bounding box.
   * @return Bounding-sphere radius in meters.
   */
  public static double boundingSphereRadius(BoundingBox3d box) {
    double hx = box.halfX();
    double hy = box.halfY();
    double hz = box.halfZ();
    return Math.sqrt(hx * hx + hy * hy + hz * hz);
  }

  /**
   * Fast zero-allocation bounding-sphere broadphase check between two posed bounding boxes.
   *
   * @param boxA Local bounds of component A.
   * @param poseA World/robot-frame pose of component A.
   * @param boxB Local bounds of component B.
   * @param poseB World/robot-frame pose of component B.
   * @return {@code true} if the bounding spheres overlap (or touch), {@code false} if definitely
   *     separated.
   */
  public static boolean spheresOverlap(
      BoundingBox3d boxA, Pose3d poseA, BoundingBox3d boxB, Pose3d poseB) {
    Quaternion qA = poseA.getRotation().getQuaternion();
    double aw = qA.getW(), ax = qA.getX(), ay = qA.getY(), az = qA.getZ();
    double a00 = 1.0 - 2.0 * (ay * ay + az * az);
    double a10 = 2.0 * (ax * ay + aw * az);
    double a20 = 2.0 * (ax * az - aw * ay);
    double a01 = 2.0 * (ax * ay - aw * az);
    double a11 = 1.0 - 2.0 * (ax * ax + az * az);
    double a21 = 2.0 * (ay * az + aw * ax);
    double a02 = 2.0 * (ax * az + aw * ay);
    double a12 = 2.0 * (ay * az - aw * ax);
    double a22 = 1.0 - 2.0 * (ax * ax + ay * ay);

    Quaternion qB = poseB.getRotation().getQuaternion();
    double bw = qB.getW(), bx = qB.getX(), by = qB.getY(), bz = qB.getZ();
    double b00 = 1.0 - 2.0 * (by * by + bz * bz);
    double b10 = 2.0 * (bx * by + bw * bz);
    double b20 = 2.0 * (bx * bz - bw * by);
    double b01 = 2.0 * (bx * by - bw * bz);
    double b11 = 1.0 - 2.0 * (bx * bx + bz * bz);
    double b21 = 2.0 * (by * bz + bw * bx);
    double b02 = 2.0 * (bx * bz + bw * by);
    double b12 = 2.0 * (by * bz - bw * bx);
    double b22 = 1.0 - 2.0 * (bx * bx + by * by);

    double lcAx = (boxA.minX() + boxA.maxX()) * 0.5;
    double lcAy = (boxA.minY() + boxA.maxY()) * 0.5;
    double lcAz = (boxA.minZ() + boxA.maxZ()) * 0.5;
    double cAx = poseA.getX() + a00 * lcAx + a01 * lcAy + a02 * lcAz;
    double cAy = poseA.getY() + a10 * lcAx + a11 * lcAy + a12 * lcAz;
    double cAz = poseA.getZ() + a20 * lcAx + a21 * lcAy + a22 * lcAz;

    double lcBx = (boxB.minX() + boxB.maxX()) * 0.5;
    double lcBy = (boxB.minY() + boxB.maxY()) * 0.5;
    double lcBz = (boxB.minZ() + boxB.maxZ()) * 0.5;
    double cBx = poseB.getX() + b00 * lcBx + b01 * lcBy + b02 * lcBz;
    double cBy = poseB.getY() + b10 * lcBx + b11 * lcBy + b12 * lcBz;
    double cBz = poseB.getZ() + b20 * lcBx + b21 * lcBy + b22 * lcBz;

    double dx = cBx - cAx;
    double dy = cBy - cAy;
    double dz = cBz - cAz;
    double rSum = boundingSphereRadius(boxA) + boundingSphereRadius(boxB);
    return dx * dx + dy * dy + dz * dz <= rSum * rSum + 1e-12;
  }

  /**
   * Tests two posed local bounding boxes for 3D OBB overlap using a bounding-sphere broadphase
   * followed by an allocation-free 15-axis Separating Axis Theorem test.
   *
   * @param boxA Local bounds of component A.
   * @param poseA World/robot-frame pose of component A.
   * @param boxB Local bounds of component B.
   * @param poseB World/robot-frame pose of component B.
   * @return Minimum penetration depth in meters if overlapping, or empty if separated.
   */
  public static OptionalDouble testObbCollision(
      BoundingBox3d boxA, Pose3d poseA, BoundingBox3d boxB, Pose3d poseB) {
    Quaternion qA = poseA.getRotation().getQuaternion();
    double aw = qA.getW(), ax = qA.getX(), ay = qA.getY(), az = qA.getZ();
    double a00 = 1.0 - 2.0 * (ay * ay + az * az);
    double a10 = 2.0 * (ax * ay + aw * az);
    double a20 = 2.0 * (ax * az - aw * ay);
    double a01 = 2.0 * (ax * ay - aw * az);
    double a11 = 1.0 - 2.0 * (ax * ax + az * az);
    double a21 = 2.0 * (ay * az + aw * ax);
    double a02 = 2.0 * (ax * az + aw * ay);
    double a12 = 2.0 * (ay * az - aw * ax);
    double a22 = 1.0 - 2.0 * (ax * ax + ay * ay);

    Quaternion qB = poseB.getRotation().getQuaternion();
    double bw = qB.getW(), bx = qB.getX(), by = qB.getY(), bz = qB.getZ();
    double b00 = 1.0 - 2.0 * (by * by + bz * bz);
    double b10 = 2.0 * (bx * by + bw * bz);
    double b20 = 2.0 * (bx * bz - bw * by);
    double b01 = 2.0 * (bx * by - bw * bz);
    double b11 = 1.0 - 2.0 * (bx * bx + bz * bz);
    double b21 = 2.0 * (by * bz + bw * bx);
    double b02 = 2.0 * (bx * bz + bw * by);
    double b12 = 2.0 * (by * bz - bw * bx);
    double b22 = 1.0 - 2.0 * (bx * bx + by * by);

    double depth =
        testObbWithMatrices(
            boxA,
            poseA.getX(),
            poseA.getY(),
            poseA.getZ(),
            a00,
            a10,
            a20,
            a01,
            a11,
            a21,
            a02,
            a12,
            a22,
            boxB,
            poseB.getX(),
            poseB.getY(),
            poseB.getZ(),
            b00,
            b10,
            b20,
            b01,
            b11,
            b21,
            b02,
            b12,
            b22);
    return depth >= 0.0 ? OptionalDouble.of(depth) : OptionalDouble.empty();
  }

  /**
   * Tests whether any transformed corner of a posed bounding box breaches an allowed axis-aligned
   * extension envelope without heap allocation.
   *
   * @param componentBox Local bounds of the component.
   * @param componentPose Robot-frame pose of the component.
   * @param allowedEnvelope Legal axis-aligned extension envelope in meters.
   * @return Maximum violation distance in meters if breached, or empty if within bounds.
   */
  public static OptionalDouble testEnvelopeViolation(
      BoundingBox3d componentBox, Pose3d componentPose, BoundingBox3d allowedEnvelope) {
    Quaternion q = componentPose.getRotation().getQuaternion();
    double w = q.getW(), x = q.getX(), y = q.getY(), z = q.getZ();
    double r00 = 1.0 - 2.0 * (y * y + z * z);
    double r10 = 2.0 * (x * y + w * z);
    double r20 = 2.0 * (x * z - w * y);
    double r01 = 2.0 * (x * y - w * z);
    double r11 = 1.0 - 2.0 * (x * x + z * z);
    double r21 = 2.0 * (y * z + w * x);
    double r02 = 2.0 * (x * z + w * y);
    double r12 = 2.0 * (y * z - w * x);
    double r22 = 1.0 - 2.0 * (x * x + y * y);

    double v =
        envelopeViolationWithMatrix(
            componentBox,
            componentPose.getX(),
            componentPose.getY(),
            componentPose.getZ(),
            r00,
            r10,
            r20,
            r01,
            r11,
            r21,
            r02,
            r12,
            r22,
            allowedEnvelope);
    return v > 1e-6 ? OptionalDouble.of(v) : OptionalDouble.empty();
  }

  /**
   * Tests two multi-primitive OBB collections for overlap, extracting pose rotation matrices once
   * per call and using a bounding-sphere broadphase per sub-box pair before running 15-axis SAT.
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
    if (boxesA.isEmpty() || boxesB.isEmpty()) {
      return OptionalDouble.empty();
    }
    Quaternion qA = poseA.getRotation().getQuaternion();
    double aw = qA.getW(), ax = qA.getX(), ay = qA.getY(), az = qA.getZ();
    double a00 = 1.0 - 2.0 * (ay * ay + az * az);
    double a10 = 2.0 * (ax * ay + aw * az);
    double a20 = 2.0 * (ax * az - aw * ay);
    double a01 = 2.0 * (ax * ay - aw * az);
    double a11 = 1.0 - 2.0 * (ax * ax + az * az);
    double a21 = 2.0 * (ay * az + aw * ax);
    double a02 = 2.0 * (ax * az + aw * ay);
    double a12 = 2.0 * (ay * az - aw * ax);
    double a22 = 1.0 - 2.0 * (ax * ax + ay * ay);

    Quaternion qB = poseB.getRotation().getQuaternion();
    double bw = qB.getW(), bx = qB.getX(), by = qB.getY(), bz = qB.getZ();
    double b00 = 1.0 - 2.0 * (by * by + bz * bz);
    double b10 = 2.0 * (bx * by + bw * bz);
    double b20 = 2.0 * (bx * bz - bw * by);
    double b01 = 2.0 * (bx * by - bw * bz);
    double b11 = 1.0 - 2.0 * (bx * bx + bz * bz);
    double b21 = 2.0 * (by * bz + bw * bx);
    double b02 = 2.0 * (bx * bz + bw * by);
    double b12 = 2.0 * (by * bz - bw * bx);
    double b22 = 1.0 - 2.0 * (bx * bx + by * by);

    double pAx = poseA.getX(), pAy = poseA.getY(), pAz = poseA.getZ();
    double pBx = poseB.getX(), pBy = poseB.getY(), pBz = poseB.getZ();

    double maxPenetration = -1.0;
    int sizeA = boxesA.size();
    int sizeB = boxesB.size();
    for (int i = 0; i < sizeA; i++) {
      BoundingBox3d a = boxesA.get(i);
      for (int j = 0; j < sizeB; j++) {
        BoundingBox3d b = boxesB.get(j);
        double depth =
            testObbWithMatrices(
                a,
                pAx,
                pAy,
                pAz,
                a00,
                a10,
                a20,
                a01,
                a11,
                a21,
                a02,
                a12,
                a22,
                b,
                pBx,
                pBy,
                pBz,
                b00,
                b10,
                b20,
                b01,
                b11,
                b21,
                b02,
                b12,
                b22);
        if (depth >= 0.0 && depth > maxPenetration) {
          maxPenetration = depth;
        }
      }
    }
    return maxPenetration >= 0.0 ? OptionalDouble.of(maxPenetration) : OptionalDouble.empty();
  }

  /**
   * Tests whether any sub-bounding box in {@code boxes} breaches {@code allowedEnvelope},
   * extracting the pose rotation matrix once per call.
   *
   * @param boxes Local sub-bounding boxes for the component.
   * @param componentPose Robot-frame pose of the component.
   * @param allowedEnvelope Legal axis-aligned extension envelope in meters.
   * @return Maximum violation distance in meters if breached, or empty if all sub-boxes are within
   *     bounds.
   */
  public static OptionalDouble testMultiEnvelopeViolation(
      List<BoundingBox3d> boxes, Pose3d componentPose, BoundingBox3d allowedEnvelope) {
    if (boxes.isEmpty()) {
      return OptionalDouble.empty();
    }
    Quaternion q = componentPose.getRotation().getQuaternion();
    double w = q.getW(), x = q.getX(), y = q.getY(), z = q.getZ();
    double r00 = 1.0 - 2.0 * (y * y + z * z);
    double r10 = 2.0 * (x * y + w * z);
    double r20 = 2.0 * (x * z - w * y);
    double r01 = 2.0 * (x * y - w * z);
    double r11 = 1.0 - 2.0 * (x * x + z * z);
    double r21 = 2.0 * (y * z + w * x);
    double r02 = 2.0 * (x * z + w * y);
    double r12 = 2.0 * (y * z - w * x);
    double r22 = 1.0 - 2.0 * (x * x + y * y);

    double px = componentPose.getX(), py = componentPose.getY(), pz = componentPose.getZ();
    double maxViolation = 0.0;
    for (int i = 0; i < boxes.size(); i++) {
      double v =
          envelopeViolationWithMatrix(
              boxes.get(i),
              px,
              py,
              pz,
              r00,
              r10,
              r20,
              r01,
              r11,
              r21,
              r02,
              r12,
              r22,
              allowedEnvelope);
      if (v > maxViolation) {
        maxViolation = v;
      }
    }
    return maxViolation > 1e-6 ? OptionalDouble.of(maxViolation) : OptionalDouble.empty();
  }

  private static double testObbWithMatrices(
      BoundingBox3d boxA,
      double pAx,
      double pAy,
      double pAz,
      double a00,
      double a10,
      double a20,
      double a01,
      double a11,
      double a21,
      double a02,
      double a12,
      double a22,
      BoundingBox3d boxB,
      double pBx,
      double pBy,
      double pBz,
      double b00,
      double b10,
      double b20,
      double b01,
      double b11,
      double b21,
      double b02,
      double b12,
      double b22) {
    double eAx = boxA.halfX(), eAy = boxA.halfY(), eAz = boxA.halfZ();
    double eBx = boxB.halfX(), eBy = boxB.halfY(), eBz = boxB.halfZ();

    double lcAx = (boxA.minX() + boxA.maxX()) * 0.5;
    double lcAy = (boxA.minY() + boxA.maxY()) * 0.5;
    double lcAz = (boxA.minZ() + boxA.maxZ()) * 0.5;
    double cAx = pAx + a00 * lcAx + a01 * lcAy + a02 * lcAz;
    double cAy = pAy + a10 * lcAx + a11 * lcAy + a12 * lcAz;
    double cAz = pAz + a20 * lcAx + a21 * lcAy + a22 * lcAz;

    double lcBx = (boxB.minX() + boxB.maxX()) * 0.5;
    double lcBy = (boxB.minY() + boxB.maxY()) * 0.5;
    double lcBz = (boxB.minZ() + boxB.maxZ()) * 0.5;
    double cBx = pBx + b00 * lcBx + b01 * lcBy + b02 * lcBz;
    double cBy = pBy + b10 * lcBx + b11 * lcBy + b12 * lcBz;
    double cBz = pBz + b20 * lcBx + b21 * lcBy + b22 * lcBz;

    double tx = cBx - cAx;
    double ty = cBy - cAy;
    double tz = cBz - cAz;

    // Zero-allocation bounding-sphere broadphase rejection
    double rA = Math.sqrt(eAx * eAx + eAy * eAy + eAz * eAz);
    double rB = Math.sqrt(eBx * eBx + eBy * eBy + eBz * eBz);
    double rSum = rA + rB;
    if (tx * tx + ty * ty + tz * tz > rSum * rSum + 1e-12) {
      return -1.0;
    }

    double minOverlap = Double.POSITIVE_INFINITY;
    double ov;

    // 3 face normals of A
    ov = axisOverlap(a00, a10, a20, tx, ty, tz, a00, a10, a20, a01, a11, a21, a02, a12, a22, eAx, eAy, eAz, b00, b10, b20, b01, b11, b21, b02, b12, b22, eBx, eBy, eBz);
    if (ov < 0.0) return -1.0;
    if (ov < minOverlap) minOverlap = ov;

    ov = axisOverlap(a01, a11, a21, tx, ty, tz, a00, a10, a20, a01, a11, a21, a02, a12, a22, eAx, eAy, eAz, b00, b10, b20, b01, b11, b21, b02, b12, b22, eBx, eBy, eBz);
    if (ov < 0.0) return -1.0;
    if (ov < minOverlap) minOverlap = ov;

    ov = axisOverlap(a02, a12, a22, tx, ty, tz, a00, a10, a20, a01, a11, a21, a02, a12, a22, eAx, eAy, eAz, b00, b10, b20, b01, b11, b21, b02, b12, b22, eBx, eBy, eBz);
    if (ov < 0.0) return -1.0;
    if (ov < minOverlap) minOverlap = ov;

    // 3 face normals of B
    ov = axisOverlap(b00, b10, b20, tx, ty, tz, a00, a10, a20, a01, a11, a21, a02, a12, a22, eAx, eAy, eAz, b00, b10, b20, b01, b11, b21, b02, b12, b22, eBx, eBy, eBz);
    if (ov < 0.0) return -1.0;
    if (ov < minOverlap) minOverlap = ov;

    ov = axisOverlap(b01, b11, b21, tx, ty, tz, a00, a10, a20, a01, a11, a21, a02, a12, a22, eAx, eAy, eAz, b00, b10, b20, b01, b11, b21, b02, b12, b22, eBx, eBy, eBz);
    if (ov < 0.0) return -1.0;
    if (ov < minOverlap) minOverlap = ov;

    ov = axisOverlap(b02, b12, b22, tx, ty, tz, a00, a10, a20, a01, a11, a21, a02, a12, a22, eAx, eAy, eAz, b00, b10, b20, b01, b11, b21, b02, b12, b22, eBx, eBy, eBz);
    if (ov < 0.0) return -1.0;
    if (ov < minOverlap) minOverlap = ov;

    // 9 edge-edge cross-product axes A_i x B_j
    for (int i = 0; i < 3; i++) {
      double aix = (i == 0) ? a00 : (i == 1 ? a01 : a02);
      double aiy = (i == 0) ? a10 : (i == 1 ? a11 : a12);
      double aiz = (i == 0) ? a20 : (i == 1 ? a21 : a22);
      for (int j = 0; j < 3; j++) {
        double bjx = (j == 0) ? b00 : (j == 1 ? b01 : b02);
        double bjy = (j == 0) ? b10 : (j == 1 ? b11 : b12);
        double bjz = (j == 0) ? b20 : (j == 1 ? b21 : b22);
        double cx = aiy * bjz - aiz * bjy;
        double cy = aiz * bjx - aix * bjz;
        double cz = aix * bjy - aiy * bjx;
        double lenSq = cx * cx + cy * cy + cz * cz;
        if (lenSq < EPSILON) {
          continue;
        }
        double invLen = 1.0 / Math.sqrt(lenSq);
        ov =
            axisOverlap(
                cx * invLen,
                cy * invLen,
                cz * invLen,
                tx,
                ty,
                tz,
                a00,
                a10,
                a20,
                a01,
                a11,
                a21,
                a02,
                a12,
                a22,
                eAx,
                eAy,
                eAz,
                b00,
                b10,
                b20,
                b01,
                b11,
                b21,
                b02,
                b12,
                b22,
                eBx,
                eBy,
                eBz);
        if (ov < 0.0) return -1.0;
        if (ov < minOverlap) minOverlap = ov;
      }
    }
    return minOverlap;
  }

  private static double axisOverlap(
      double nx,
      double ny,
      double nz,
      double tx,
      double ty,
      double tz,
      double a00,
      double a10,
      double a20,
      double a01,
      double a11,
      double a21,
      double a02,
      double a12,
      double a22,
      double eAx,
      double eAy,
      double eAz,
      double b00,
      double b10,
      double b20,
      double b01,
      double b11,
      double b21,
      double b02,
      double b12,
      double b22,
      double eBx,
      double eBy,
      double eBz) {
    double dist = Math.abs(tx * nx + ty * ny + tz * nz);
    double projA =
        eAx * Math.abs(a00 * nx + a10 * ny + a20 * nz)
            + eAy * Math.abs(a01 * nx + a11 * ny + a21 * nz)
            + eAz * Math.abs(a02 * nx + a12 * ny + a22 * nz);
    double projB =
        eBx * Math.abs(b00 * nx + b10 * ny + b20 * nz)
            + eBy * Math.abs(b01 * nx + b11 * ny + b21 * nz)
            + eBz * Math.abs(b02 * nx + b12 * ny + b22 * nz);
    return (projA + projB) - dist;
  }

  private static double envelopeViolationWithMatrix(
      BoundingBox3d box,
      double px,
      double py,
      double pz,
      double r00,
      double r10,
      double r20,
      double r01,
      double r11,
      double r21,
      double r02,
      double r12,
      double r22,
      BoundingBox3d env) {
    double minX = box.minX(), maxX = box.maxX();
    double minY = box.minY(), maxY = box.maxY();
    double minZ = box.minZ(), maxZ = box.maxZ();
    double maxViolation = 0.0;
    for (int ix = 0; ix < 2; ix++) {
      double lx = (ix == 0) ? minX : maxX;
      for (int iy = 0; iy < 2; iy++) {
        double ly = (iy == 0) ? minY : maxY;
        for (int iz = 0; iz < 2; iz++) {
          double lz = (iz == 0) ? minZ : maxZ;
          double wx = px + r00 * lx + r01 * ly + r02 * lz;
          double wy = py + r10 * lx + r11 * ly + r12 * lz;
          double wz = pz + r20 * lx + r21 * ly + r22 * lz;
          double v;
          if ((v = env.minX() - wx) > maxViolation) maxViolation = v;
          if ((v = wx - env.maxX()) > maxViolation) maxViolation = v;
          if ((v = env.minY() - wy) > maxViolation) maxViolation = v;
          if ((v = wy - env.maxY()) > maxViolation) maxViolation = v;
          if ((v = env.minZ() - wz) > maxViolation) maxViolation = v;
          if ((v = wz - env.maxZ()) > maxViolation) maxViolation = v;
        }
      }
    }
    return maxViolation;
  }
}
