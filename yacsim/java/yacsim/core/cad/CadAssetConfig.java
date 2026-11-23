// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.physics.CadMassProperties;

/**
 * Configuration for a single CAD mesh asset ({@code .glb}, {@code .gltf}, {@code .obj}, or {@code
 * .stl}), including unit scaling, material density, AdvantageScope zeroed transforms, and triangle
 * budget limits.
 */
public final class CadAssetConfig {
  /** Default 6061 aluminum density in kg/m³ used when computing bounding-box mass properties. */
  public static final double DEFAULT_ALUMINUM_DENSITY = 2700.0;
  /** Default triangle warning/validation budget per CAD asset. */
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
  private CadMassProperties massPropertiesOverride = null;
  private Translation3d zeroedPositionMeters = new Translation3d();
  private Rotation3d zeroedRotation = new Rotation3d();
  private final List<AxisRotation> orderedZeroedRotations = new ArrayList<>();
  private int maxTriangleBudget = DEFAULT_MAX_TRIANGLES;

  /**
   * Creates a CAD asset configuration pointing to the given mesh file.
   *
   * @param meshPath Path to a {@code .glb}, {@code .gltf}, {@code .obj}, or {@code .stl} file.
   * @throws IllegalArgumentException If {@code meshPath} is {@code null}.
   */
  public CadAssetConfig(Path meshPath) {
    if (meshPath == null) throw new IllegalArgumentException("meshPath cannot be null.");
    this.meshPath = meshPath;
  }

  /**
   * Sets the multiplicative scale factor converting mesh coordinates into meters.
   *
   * @param scale Positive scale factor (for example {@code 0.001} for millimeters).
   * @return This config for chaining.
   */
  public CadAssetConfig withUnitScaleToMeters(double scale) {
    if (scale <= 0.0) throw new IllegalArgumentException("Unit scale must be positive.");
    this.unitScaleToMeters = scale;
    return this;
  }

  /**
   * Configures this asset as millimeter-scaled ({@code 0.001} m/unit).
   *
   * @return This config for chaining.
   */
  public CadAssetConfig withMillimeterUnits() {
    return withUnitScaleToMeters(0.001);
  }

  /**
   * Configures this asset as inch-scaled ({@code 0.0254} m/unit).
   *
   * @return This config for chaining.
   */
  public CadAssetConfig withInchUnits() {
    return withUnitScaleToMeters(0.0254);
  }

  /**
   * Sets the material density used to estimate mass and inertia when no mass override is set.
   *
   * @param densityKgPerM3 Positive density in kg/m³.
   * @return This config for chaining.
   */
  public CadAssetConfig withMaterialDensity(double densityKgPerM3) {
    if (densityKgPerM3 <= 0.0) throw new IllegalArgumentException("Density must be positive.");
    this.densityKgPerM3 = densityKgPerM3;
    return this;
  }

  /**
   * Overrides the computed total mass while scaling principal moments of inertia proportionally.
   *
   * @param massKg Positive mass in kilograms.
   * @return This config for chaining.
   */
  public CadAssetConfig withMassOverrideKg(double massKg) {
    if (massKg <= 0.0) throw new IllegalArgumentException("Mass override must be positive.");
    this.massOverrideKg = OptionalDouble.of(massKg);
    return this;
  }

  /**
   * Overrides bounding-box mass estimation with exact CAD-exported mass, center of mass, and
   * principal inertia tensor ({@code Ixx}, {@code Iyy}, {@code Izz}).
   *
   * @param massProperties Exact {@link CadMassProperties} from CAD or a manifest.
   * @return This config for chaining.
   */
  public CadAssetConfig withMassPropertiesOverride(CadMassProperties massProperties) {
    this.massPropertiesOverride = Objects.requireNonNull(massProperties, "massProperties");
    return this;
  }

  /**
   * Sets the AdvantageScope {@code position} zeroing offset in meters.
   *
   * @param pos Translation offset in meters.
   * @return This config for chaining.
   */
  public CadAssetConfig withZeroedPosition(Translation3d pos) {
    this.zeroedPositionMeters = pos;
    return this;
  }

  /**
   * Sets the fallback Euler zeroed rotation when {@link #addZeroedRotation(String, double)} is
   * not used.
   *
   * @param rot Zeroed {@link Rotation3d}.
   * @return This config for chaining.
   */
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

  /**
   * Sets the maximum triangle budget for this asset. Exceeding the budget throws for {@code
   * .obj}/{@code .stl} meshes and logs a warning for header-only {@code .glb}/{@code .gltf} meshes.
   *
   * @param maxTriangles Positive triangle count limit.
   * @return This config for chaining.
   */

  /**
   * Applies a standard CAD export preset (unit scale plus ordered AdvantageScope zeroed rotations)
   * so the mesh aligns with WPILib's NWU coordinate frame on the first run.
   *
   * @param preset CAD tool coordinate-system preset.
   * @return This config for chaining.
   */
  public CadAssetConfig withCoordinateSystem(CadCoordinateSystem preset) {
    if (preset == null) {
      throw new IllegalArgumentException("preset cannot be null.");
    }
    this.unitScaleToMeters = preset.getUnitScaleToMeters();
    this.orderedZeroedRotations.clear();
    this.orderedZeroedRotations.addAll(preset.getZeroedRotations());
    return this;
  }

  public CadAssetConfig withMaxTriangleBudget(int maxTriangles) {
    if (maxTriangles <= 0) throw new IllegalArgumentException("maxTriangles must be positive.");
    this.maxTriangleBudget = maxTriangles;
    return this;
  }

  /** @return Path to the configured CAD mesh file. */
  public Path getMeshPath() {
    return meshPath;
  }

  /** @return Multiplicative unit scale converting mesh coordinates to meters. */
  public double getUnitScaleToMeters() {
    return unitScaleToMeters;
  }

  /** @return Material density in kg/m³. */
  public double getDensityKgPerM3() {
    return densityKgPerM3;
  }

  /** @return Explicit mass override in kilograms, if configured. */
  public OptionalDouble getMassOverrideKg() {
    return massOverrideKg;
  }

  /** @return Exact CAD mass/inertia override, if configured. */
  public Optional<CadMassProperties> getMassPropertiesOverride() {
    return Optional.ofNullable(massPropertiesOverride);
  }

  /** @return AdvantageScope zeroed position offset in meters. */
  public Translation3d getZeroedPositionMeters() {
    return zeroedPositionMeters;
  }

  /** @return Fallback zeroed {@link Rotation3d}. */
  public Rotation3d getZeroedRotation() {
    return zeroedRotation;
  }

  /** @return Unmodifiable list of ordered AdvantageScope axis rotations. */
  public List<AxisRotation> getOrderedZeroedRotations() {
    return Collections.unmodifiableList(orderedZeroedRotations);
  }

  /** @return Configured triangle budget limit. */
  public int getMaxTriangleBudget() {
    return maxTriangleBudget;
  }
}
