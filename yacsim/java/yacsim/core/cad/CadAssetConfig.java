// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

public final class CadAssetConfig {
  public static final double DEFAULT_ALUMINUM_DENSITY = 2700.0;
  public static final int DEFAULT_MAX_TRIANGLES = 150_000;

  /** Ordered single-axis rotation matching AdvantageScope config.json semantics. */
  public record AxisRotation(String axis, double degrees) {
    public AxisRotation {
      if (axis == null) throw new IllegalArgumentException("Axis cannot be null.");
      axis = axis.toLowerCase(Locale.ROOT);
      if (!axis.equals("x") && !axis.equals("y") && !axis.equals("z")) {
        throw new IllegalArgumentException("Axis must be 'x', 'y', or 'z': " + axis);
      }
    }
  }

  private final Path meshPath;
  private double unitScaleToMeters = 1.0;
  private double densityKgPerM3 = DEFAULT_ALUMINUM_DENSITY;
  private OptionalDouble massOverrideKg = OptionalDouble.empty();
  private Translation3d zeroedPositionMeters = new Translation3d();
  private Rotation3d zeroedRotation = new Rotation3d();
  private final List<AxisRotation> orderedZeroedRotations = new ArrayList<>();
  private int maxTriangleBudget = DEFAULT_MAX_TRIANGLES;

  public CadAssetConfig(Path meshPath) {
    if (meshPath == null) throw new IllegalArgumentException("meshPath cannot be null.");
    this.meshPath = meshPath;
  }

  public CadAssetConfig withUnitScaleToMeters(double scale) {
    if (scale <= 0.0) throw new IllegalArgumentException("Unit scale must be positive.");
    this.unitScaleToMeters = scale;
    return this;
  }

  public CadAssetConfig withMillimeterUnits() {
    return withUnitScaleToMeters(0.001);
  }

  public CadAssetConfig withInchUnits() {
    return withUnitScaleToMeters(0.0254);
  }

  public CadAssetConfig withMaterialDensity(double densityKgPerM3) {
    if (densityKgPerM3 <= 0.0) throw new IllegalArgumentException("Density must be positive.");
    this.densityKgPerM3 = densityKgPerM3;
    return this;
  }

  public CadAssetConfig withMassOverrideKg(double massKg) {
    if (massKg <= 0.0) throw new IllegalArgumentException("Mass override must be positive.");
    this.massOverrideKg = OptionalDouble.of(massKg);
    return this;
  }

  public CadAssetConfig withZeroedPosition(Translation3d pos) {
    this.zeroedPositionMeters = pos;
    return this;
  }

  public CadAssetConfig withZeroedRotation(Rotation3d rot) {
    this.zeroedRotation = rot;
    return this;
  }

  /**
   * Appends a sequential axis rotation (in degrees) for AdvantageScope config.json generation,
   * preserving exact CAD-to-AdvantageScope rotation order (e.g., x: 90, z: 90, y: 147.5).
   */
  public CadAssetConfig addZeroedRotation(String axis, double degrees) {
    this.orderedZeroedRotations.add(new AxisRotation(axis, degrees));
    return this;
  }

  public CadAssetConfig withMaxTriangleBudget(int maxTriangles) {
    if (maxTriangles <= 0) throw new IllegalArgumentException("maxTriangles must be positive.");
    this.maxTriangleBudget = maxTriangles;
    return this;
  }

  public Path getMeshPath() {
    return meshPath;
  }

  public double getUnitScaleToMeters() {
    return unitScaleToMeters;
  }

  public double getDensityKgPerM3() {
    return densityKgPerM3;
  }

  public OptionalDouble getMassOverrideKg() {
    return massOverrideKg;
  }

  public Translation3d getZeroedPositionMeters() {
    return zeroedPositionMeters;
  }

  public Rotation3d getZeroedRotation() {
    return zeroedRotation;
  }

  public List<AxisRotation> getOrderedZeroedRotations() {
    return Collections.unmodifiableList(orderedZeroedRotations);
  }

  public int getMaxTriangleBudget() {
    return maxTriangleBudget;
  }
}
