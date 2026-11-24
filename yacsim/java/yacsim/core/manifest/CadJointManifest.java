// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.manifest;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Objects;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;
import yacsim.core.yams.MotionAxis;

/**
 * CAD-extracted kinematic joint definition inside a {@link CadRobotManifest}, bridging CAD mate
 * geometry, hard/soft limits, wheel radii, and axis inertia directly into YAMS 2027 and YACSim.
 *
 * @param name Unique joint name (for example {@code "Hood"}, {@code "Slamtake"}, or {@code
 *     "Flywheel"}).
 * @param type Kinematic joint classification ({@link CadJointType}).
 * @param parentLink Parent link name (typically {@code "Chassis"}).
 * @param childLink Child link name (matches a {@link CadLinkManifest#name()} when a mesh exists).
 * @param originMeters Joint pivot, base, or axle origin in parent-frame meters.
 * @param originRotation Fixed joint frame orientation relative to parent frame.
 * @param zeroOffset Optional fixed transform applied after joint motion.
 * @param axis Motion axis ({@link MotionAxis#X}, {@link MotionAxis#Y}, or {@link MotionAxis#Z});
 *     may be {@code null} for {@link CadJointType#FIXED}.
 * @param inverted {@code true} to negate the joint motion direction.
 * @param minLimit Hard lower limit (radians for {@code REVOLUTE}, meters for {@code PRISMATIC}).
 * @param maxLimit Hard upper limit (radians for {@code REVOLUTE}, meters for {@code PRISMATIC}).
 * @param softMinLimit Optional software lower limit (radians or meters).
 * @param softMaxLimit Optional software upper limit (radians or meters).
 * @param startingPosition Optional simulation starting position (radians or meters).
 * @param lengthMeters Optional mechanism length (for example arm length in meters).
 * @param wheelRadiusMeters Optional wheel, roller, or elevator drum radius in meters.
 * @param momentOfInertiaKgM2 Optional CAD-computed scalar moment of inertia about the joint axis
 *     in kg·m².
 */
public record CadJointManifest(
    String name,
    CadJointType type,
    String parentLink,
    String childLink,
    Translation3d originMeters,
    Rotation3d originRotation,
    Transform3d zeroOffset,
    MotionAxis axis,
    boolean inverted,
    OptionalDouble minLimit,
    OptionalDouble maxLimit,
    OptionalDouble softMinLimit,
    OptionalDouble softMaxLimit,
    OptionalDouble startingPosition,
    OptionalDouble lengthMeters,
    OptionalDouble wheelRadiusMeters,
    OptionalDouble momentOfInertiaKgM2) {

  /** Compact constructor validating joint manifest fields. */
  public CadJointManifest {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(type, "type");
    Objects.requireNonNull(parentLink, "parentLink");
    Objects.requireNonNull(childLink, "childLink");
    if (name.isBlank()) {
      throw new IllegalArgumentException("Joint name cannot be blank.");
    }
    if (type != CadJointType.FIXED && axis == null) {
      throw new IllegalArgumentException("Non-fixed joint '" + name + "' requires a MotionAxis.");
    }
    originMeters = originMeters == null ? new Translation3d() : originMeters;
    originRotation = originRotation == null ? new Rotation3d() : originRotation;
    zeroOffset = zeroOffset == null ? new Transform3d() : zeroOffset;
    minLimit = minLimit == null ? OptionalDouble.empty() : minLimit;
    maxLimit = maxLimit == null ? OptionalDouble.empty() : maxLimit;
    softMinLimit = softMinLimit == null ? OptionalDouble.empty() : softMinLimit;
    softMaxLimit = softMaxLimit == null ? OptionalDouble.empty() : softMaxLimit;
    startingPosition = startingPosition == null ? OptionalDouble.empty() : startingPosition;
    lengthMeters = lengthMeters == null ? OptionalDouble.empty() : lengthMeters;
    wheelRadiusMeters = wheelRadiusMeters == null ? OptionalDouble.empty() : wheelRadiusMeters;
    momentOfInertiaKgM2 =
        momentOfInertiaKgM2 == null ? OptionalDouble.empty() : momentOfInertiaKgM2;
    if (wheelRadiusMeters.isPresent() && wheelRadiusMeters.getAsDouble() <= 0.0) {
      throw new IllegalArgumentException("wheelRadiusMeters must be positive on joint: " + name);
    }
    if (lengthMeters.isPresent() && lengthMeters.getAsDouble() <= 0.0) {
      throw new IllegalArgumentException("lengthMeters must be positive on joint: " + name);
    }
    if (momentOfInertiaKgM2.isPresent() && momentOfInertiaKgM2.getAsDouble() <= 0.0) {
      throw new IllegalArgumentException("momentOfInertiaKgM2 must be positive on joint: " + name);
    }
  }

  /** @return Hard lower angular limit as a WPILib {@link Angle}. */
  public Angle minLimitAngle() {
    return Radians.of(requirePresent(minLimit, "minLimit"));
  }

  /** @return Hard upper angular limit as a WPILib {@link Angle}. */
  public Angle maxLimitAngle() {
    return Radians.of(requirePresent(maxLimit, "maxLimit"));
  }

  /** @return Soft lower angular limit, falling back to {@link #minLimitAngle()} if absent. */
  public Angle softMinLimitAngle() {
    return Radians.of(softMinLimit.orElseGet(() -> requirePresent(minLimit, "minLimit")));
  }

  /** @return Soft upper angular limit, falling back to {@link #maxLimitAngle()} if absent. */
  public Angle softMaxLimitAngle() {
    return Radians.of(softMaxLimit.orElseGet(() -> requirePresent(maxLimit, "maxLimit")));
  }

  /** @return Simulation starting angle, falling back to {@link #minLimitAngle()} if absent. */
  public Angle startingAngle() {
    return Radians.of(startingPosition.orElseGet(() -> requirePresent(minLimit, "minLimit")));
  }

  /** @return Hard lower linear limit as a WPILib {@link Distance}. */
  public Distance minLimitDistance() {
    return Meters.of(requirePresent(minLimit, "minLimit"));
  }

  /** @return Hard upper linear limit as a WPILib {@link Distance}. */
  public Distance maxLimitDistance() {
    return Meters.of(requirePresent(maxLimit, "maxLimit"));
  }

  /** @return Soft lower linear limit, falling back to {@link #minLimitDistance()} if absent. */
  public Distance softMinLimitDistance() {
    return Meters.of(softMinLimit.orElseGet(() -> requirePresent(minLimit, "minLimit")));
  }

  /** @return Soft upper linear limit, falling back to {@link #maxLimitDistance()} if absent. */
  public Distance softMaxLimitDistance() {
    return Meters.of(softMaxLimit.orElseGet(() -> requirePresent(maxLimit, "maxLimit")));
  }

  /** @return Simulation starting distance, falling back to {@link #minLimitDistance()} if absent. */
  public Distance startingDistance() {
    return Meters.of(startingPosition.orElseGet(() -> requirePresent(minLimit, "minLimit")));
  }

  /** @return Mechanism length (such as arm length) as a WPILib {@link Distance}. */
  public Distance length() {
    return Meters.of(requirePresent(lengthMeters, "lengthMeters"));
  }

  /** @return Wheel, roller, or drum radius as a WPILib {@link Distance}. */
  public Distance wheelRadius() {
    return Meters.of(requirePresent(wheelRadiusMeters, "wheelRadiusMeters"));
  }

  /** @return Wheel or roller diameter ({@code 2 * wheelRadius}) as a WPILib {@link Distance}. */
  public Distance wheelDiameter() {
    return Meters.of(2.0 * requirePresent(wheelRadiusMeters, "wheelRadiusMeters"));
  }

  /** @return Moment of inertia about the joint axis as a WPILib {@link MomentOfInertia}. */
  public MomentOfInertia momentOfInertia() {
    return KilogramSquareMeters.of(requirePresent(momentOfInertiaKgM2, "momentOfInertiaKgM2"));
  }

  /**
   * @param fallback Fallback angle used if {@code minLimit} is omitted in CAD/URDF.
   * @return Hard lower angular limit, or {@code fallback} when absent.
   */
  public Angle minLimitAngleOr(Angle fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return minLimit.isPresent() ? Radians.of(minLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback angle used if {@code maxLimit} is omitted in CAD/URDF.
   * @return Hard upper angular limit, or {@code fallback} when absent.
   */
  public Angle maxLimitAngleOr(Angle fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return maxLimit.isPresent() ? Radians.of(maxLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback angle used if both {@code softMinLimit} and {@code minLimit} are
   *     omitted.
   * @return Soft lower angular limit, falling back to hard {@code minLimit} then {@code fallback}.
   */
  public Angle softMinLimitAngleOr(Angle fallback) {
    Objects.requireNonNull(fallback, "fallback");
    if (softMinLimit.isPresent()) {
      return Radians.of(softMinLimit.getAsDouble());
    }
    return minLimit.isPresent() ? Radians.of(minLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback angle used if both {@code softMaxLimit} and {@code maxLimit} are
   *     omitted.
   * @return Soft upper angular limit, falling back to hard {@code maxLimit} then {@code fallback}.
   */
  public Angle softMaxLimitAngleOr(Angle fallback) {
    Objects.requireNonNull(fallback, "fallback");
    if (softMaxLimit.isPresent()) {
      return Radians.of(softMaxLimit.getAsDouble());
    }
    return maxLimit.isPresent() ? Radians.of(maxLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback angle used if both {@code startingPosition} and {@code minLimit} are
   *     omitted.
   * @return Starting angle, falling back to {@code minLimit} then {@code fallback}.
   */
  public Angle startingAngleOr(Angle fallback) {
    Objects.requireNonNull(fallback, "fallback");
    if (startingPosition.isPresent()) {
      return Radians.of(startingPosition.getAsDouble());
    }
    return minLimit.isPresent() ? Radians.of(minLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback distance used if {@code minLimit} is omitted in CAD/URDF.
   * @return Hard lower linear limit, or {@code fallback} when absent.
   */
  public Distance minLimitDistanceOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return minLimit.isPresent() ? Meters.of(minLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback distance used if {@code maxLimit} is omitted in CAD/URDF.
   * @return Hard upper linear limit, or {@code fallback} when absent.
   */
  public Distance maxLimitDistanceOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return maxLimit.isPresent() ? Meters.of(maxLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback distance used if both {@code softMinLimit} and {@code minLimit} are
   *     omitted.
   * @return Soft lower linear limit, falling back to hard {@code minLimit} then {@code fallback}.
   */
  public Distance softMinLimitDistanceOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    if (softMinLimit.isPresent()) {
      return Meters.of(softMinLimit.getAsDouble());
    }
    return minLimit.isPresent() ? Meters.of(minLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback distance used if both {@code softMaxLimit} and {@code maxLimit} are
   *     omitted.
   * @return Soft upper linear limit, falling back to hard {@code maxLimit} then {@code fallback}.
   */
  public Distance softMaxLimitDistanceOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    if (softMaxLimit.isPresent()) {
      return Meters.of(softMaxLimit.getAsDouble());
    }
    return maxLimit.isPresent() ? Meters.of(maxLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback distance used if both {@code startingPosition} and {@code minLimit}
   *     are omitted.
   * @return Starting distance, falling back to {@code minLimit} then {@code fallback}.
   */
  public Distance startingDistanceOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    if (startingPosition.isPresent()) {
      return Meters.of(startingPosition.getAsDouble());
    }
    return minLimit.isPresent() ? Meters.of(minLimit.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback mechanism length used if {@code lengthMeters} is omitted in CAD/URDF.
   * @return Mechanism length, or {@code fallback} when absent.
   */
  public Distance lengthOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return lengthMeters.isPresent() ? Meters.of(lengthMeters.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback wheel/roller radius used if {@code wheelRadiusMeters} is omitted.
   * @return Wheel, roller, or drum radius, or {@code fallback} when absent.
   */
  public Distance wheelRadiusOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return wheelRadiusMeters.isPresent() ? Meters.of(wheelRadiusMeters.getAsDouble()) : fallback;
  }

  /**
   * @param fallback Fallback wheel/roller diameter used if {@code wheelRadiusMeters} is omitted.
   * @return Wheel or roller diameter ({@code 2 * wheelRadius}), or {@code fallback} when absent.
   */
  public Distance wheelDiameterOr(Distance fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return wheelRadiusMeters.isPresent()
        ? Meters.of(2.0 * wheelRadiusMeters.getAsDouble())
        : fallback;
  }

  /**
   * @param fallback Fallback inertia used if {@code momentOfInertiaKgM2} is omitted in CAD/URDF.
   * @return Axis moment of inertia, or {@code fallback} when absent.
   */
  public MomentOfInertia momentOfInertiaOr(MomentOfInertia fallback) {
    Objects.requireNonNull(fallback, "fallback");
    return momentOfInertiaKgM2.isPresent()
        ? KilogramSquareMeters.of(momentOfInertiaKgM2.getAsDouble())
        : fallback;
  }

  private double requirePresent(OptionalDouble opt, String field) {
    if (opt.isEmpty()) {
      throw new IllegalStateException(
          "Joint '" + name + "' in CAD manifest does not specify '" + field + "'.");
    }
    return opt.getAsDouble();
  }
}
