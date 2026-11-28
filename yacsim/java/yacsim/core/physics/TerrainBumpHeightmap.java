// Copyright (c) 2026 Nathan Moe / Yet Another CAD Simulator (YACSim) Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Analytic 2D field terrain heightmap for FRC bumps, cable protectors, and scoring ramps.
 *
 * <p>Evaluates floor elevation, local pitch/roll tilt, and 6-DoF chassis pose projection from four
 * wheel-contact elevations.
 */
public final class TerrainBumpHeightmap {
  /** Supported terrain feature shapes. */
  public enum FeatureShape {
    /** Smooth raised cosine bump (e.g., field hump or cable protector). */
    COSINE_BUMP,
    /** Linearly sloped ramp rising along +X from minX to maxX. */
    LINEAR_RAMP_X,
    /** Flat elevated plateau. */
    PLATEAU
  }

  /** Immutable field terrain feature definition. */
  public record TerrainFeature(
      String name,
      FeatureShape shape,
      double minX,
      double minY,
      double maxX,
      double maxY,
      double peakHeightMeters) {
    public TerrainFeature {
      Objects.requireNonNull(name, "name must not be null");
      Objects.requireNonNull(shape, "shape must not be null");
      if (maxX <= minX || maxY <= minY) {
        throw new IllegalArgumentException("TerrainFeature bounds must satisfy max > min");
      }
      if (!Double.isFinite(peakHeightMeters) || peakHeightMeters < 0.0) {
        throw new IllegalArgumentException("peakHeightMeters must be finite and >= 0");
      }
    }

    public double heightAt(double x, double y) {
      if (x < minX || x > maxX || y < minY || y > maxY) {
        return 0.0;
      }
      double u = (x - minX) / (maxX - minX);
      return switch (shape) {
        case PLATEAU -> peakHeightMeters;
        case LINEAR_RAMP_X -> peakHeightMeters * u;
        case COSINE_BUMP -> peakHeightMeters * 0.5 * (1.0 - Math.cos(2.0 * Math.PI * u));
      };
    }
  }

  /** Terrain contact sample for a chassis pose. */
  public record ChassisTerrainSample(
      Pose3d pose3d, double pitchRad, double rollRad, double meanHeightMeters) {}

  private final List<TerrainFeature> features = new ArrayList<>();

  public TerrainBumpHeightmap addFeature(TerrainFeature feature) {
    features.add(Objects.requireNonNull(feature, "feature must not be null"));
    return this;
  }

  public TerrainBumpHeightmap addCableProtector(
      String name, double minX, double minY, double maxX, double maxY, double heightMeters) {
    return addFeature(
        new TerrainFeature(
            name, FeatureShape.COSINE_BUMP, minX, minY, maxX, maxY, heightMeters));
  }

  public TerrainBumpHeightmap addRampX(
      String name, double minX, double minY, double maxX, double maxY, double topHeightMeters) {
    return addFeature(
        new TerrainFeature(
            name, FeatureShape.LINEAR_RAMP_X, minX, minY, maxX, maxY, topHeightMeters));
  }

  /** Returns the maximum surface elevation at field coordinate {@code (xMeters, yMeters)}. */
  public double heightAt(double xMeters, double yMeters) {
    if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters)) {
      return 0.0;
    }
    double h = 0.0;
    for (TerrainFeature f : features) {
      h = Math.max(h, f.heightAt(xMeters, yMeters));
    }
    return h;
  }

  /**
   * Projects a 2D robot pose onto the terrain by sampling the four swerve module contact points.
   */
  public ChassisTerrainSample projectPoseToTerrain(
      Pose2d pose2d, double wheelbaseMeters, double trackwidthMeters) {
    Objects.requireNonNull(pose2d, "pose2d must not be null");
    double halfL = 0.5 * wheelbaseMeters;
    double halfW = 0.5 * trackwidthMeters;
    double cos = pose2d.getRotation().getCos();
    double sin = pose2d.getRotation().getSin();
    double x = pose2d.getX();
    double y = pose2d.getY();

    double zFL = heightAt(x + halfL * cos - halfW * sin, y + halfL * sin + halfW * cos);
    double zFR = heightAt(x + halfL * cos + halfW * sin, y + halfL * sin - halfW * cos);
    double zBL = heightAt(x - halfL * cos - halfW * sin, y - halfL * sin + halfW * cos);
    double zBR = heightAt(x - halfL * cos + halfW * sin, y - halfL * sin - halfW * cos);

    double zFront = 0.5 * (zFL + zFR);
    double zBack = 0.5 * (zBL + zBR);
    double zLeft = 0.5 * (zFL + zBL);
    double zRight = 0.5 * (zFR + zBR);
    double meanZ = 0.25 * (zFL + zFR + zBL + zBR);

    // WPILib right-handed: nose up (+Z front > back) is negative pitch around +Y; left up is +roll
    double pitchRad = -Math.atan2(zFront - zBack, wheelbaseMeters);
    double rollRad = Math.atan2(zLeft - zRight, trackwidthMeters);
    Rotation3d rot3d = new Rotation3d(rollRad, pitchRad, pose2d.getRotation().getRadians());
    Pose3d pose3d = new Pose3d(new Translation3d(x, y, meanZ), rot3d);
    return new ChassisTerrainSample(pose3d, pitchRad, rollRad, meanZ);
  }

  public List<TerrainFeature> getFeatures() {
    return List.copyOf(features);
  }
}
