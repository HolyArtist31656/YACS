// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Configuration for publishing live spinning-wheel, roller, or flywheel telemetry ({@code
 * VelocityRPM}, {@code SurfaceSpeedMPS}, and {@code Spinning}) under {@code
 * /<telemetryTable>/Rollers/<name>/}. Useful both alongside separate spinning {@code .glb} meshes
 * and when rollers are visually baked into a chassis or intake CAD mesh.
 *
 * @param name Unique roller or flywheel name (for example {@code "Flywheel"} or {@code
 *     "IntakeRoller"}).
 * @param angularVelocitySupplier Supplier returning the current {@link AngularVelocity}.
 * @param wheelRadiusMeters Positive wheel or roller radius in meters (used to compute surface speed
 *     in m/s).
 */
public record CadRollerTelemetryConfig(
    String name, Supplier<AngularVelocity> angularVelocitySupplier, double wheelRadiusMeters) {

  /** Compact constructor validating roller telemetry inputs. */
  public CadRollerTelemetryConfig {
    Objects.requireNonNull(name, "name");
    Objects.requireNonNull(angularVelocitySupplier, "angularVelocitySupplier");
    if (name.isBlank()) {
      throw new IllegalArgumentException("Roller name cannot be blank.");
    }
    if (wheelRadiusMeters <= 0.0) {
      throw new IllegalArgumentException("wheelRadiusMeters must be positive.");
    }
  }
}
