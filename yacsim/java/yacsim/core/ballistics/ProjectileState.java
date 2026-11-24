// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.ballistics;

import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Immutable snapshot of one simulated 3D airborne projectile.
 *
 * @param id Unique monotonically increasing projectile ID.
 * @param spec Physical projectile specification.
 * @param positionMeters Field-frame 3D position in meters.
 * @param velocityMps Field-frame 3D linear velocity in m/s.
 * @param spinRadPerSec Field-frame 3D angular spin vector in rad/s (for Magnus lift).
 * @param ageSeconds Time elapsed since launch in seconds.
 * @param active {@code true} while still flying; {@code false} after hitting a target, obstacle, or
 *     settling on the floor.
 * @param scored {@code true} if the projectile intersected a scoring target volume.
 * @param hitObstacleName Name of the obstacle or scoring target struck, if any.
 */
public record ProjectileState(
    int id,
    ProjectileSpec spec,
    Translation3d positionMeters,
    Translation3d velocityMps,
    Translation3d spinRadPerSec,
    double ageSeconds,
    boolean active,
    boolean scored,
    Optional<String> hitObstacleName) {

  /**
   * Converts the projectile's position and velocity direction into a WPILib {@link Pose3d} for
   * AdvantageScope 3D Field rendering.
   *
   * @return Field-frame {@link Pose3d} aligned with the velocity vector.
   */
  public Pose3d toPose3d() {
    double vx = velocityMps.getX();
    double vy = velocityMps.getY();
    double vz = velocityMps.getZ();
    double horiz = Math.hypot(vx, vy);
    double speed = Math.hypot(horiz, vz);
    if (speed < 1e-6) {
      return new Pose3d(positionMeters, new Rotation3d());
    }
    double yaw = Math.atan2(vy, vx);
    double pitch = -Math.atan2(vz, horiz);
    return new Pose3d(positionMeters, new Rotation3d(0.0, pitch, yaw));
  }
}
