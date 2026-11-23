// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.manifest;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import yacsim.core.advantagescope.CadCameraConfig;
import yacsim.core.physics.CadMassProperties;
import yacsim.core.yams.MotionAxis;

/**
 * Top-level CAD-extracted robot description loaded from {@code yacsim-robot.json} or imported from
 * a standard URDF XML file. Provides a single source of truth for CAD geometry, exact mass/inertia,
 * kinematic joints, hard/soft limits, wheel radii, and cameras across both YAMS 2027 and YACSim.
 *
 * @param schemaVersion Manifest schema version (currently {@code "1.0"}).
 * @param robotName Robot display/export name (for example {@code "Darwin"}).
 * @param sourceUrl Optional URL or identifier of the source CAD assembly (for example an Onshape
 *     document URL).
 * @param extensionXyMarginMeters Optional legal horizontal extension margin in meters.
 * @param extensionMaxHeightMeters Optional legal maximum robot height in meters.
 * @param ignoreRestingOverlaps Whether to auto-ignore resting OBB overlaps at startup.
 * @param autoGroundChassis Whether to auto-shift chassis {@code minZ} to {@code 0.0}.
 * @param disableAdvantageScopeSimplification Whether to disable AdvantageScope mesh simplification.
 * @param chassis Optional chassis link manifest.
 * @param links Articulated mechanism links in AdvantageScope {@code model_N} order.
 * @param joints Kinematic and spinning-roller joints.
 * @param cameras AdvantageScope 3D camera frustum definitions.
 */
public record CadRobotManifest(
    String schemaVersion,
    String robotName,
    Optional<String> sourceUrl,
    OptionalDouble extensionXyMarginMeters,
    OptionalDouble extensionMaxHeightMeters,
    boolean ignoreRestingOverlaps,
    boolean autoGroundChassis,
    boolean disableAdvantageScopeSimplification,
    Optional<CadLinkManifest> chassis,
    List<CadLinkManifest> links,
    List<CadJointManifest> joints,
    List<CadCameraConfig> cameras) {

  /** Current supported manifest schema version. */
  public static final String CURRENT_SCHEMA_VERSION = "1.0";

  /** Compact constructor validating robot manifest fields. */
  public CadRobotManifest {
    schemaVersion =
        (schemaVersion == null || schemaVersion.isBlank())
            ? CURRENT_SCHEMA_VERSION
            : schemaVersion;
    Objects.requireNonNull(robotName, "robotName");
    if (robotName.isBlank()) {
      throw new IllegalArgumentException("robotName cannot be blank.");
    }
    sourceUrl = sourceUrl == null ? Optional.empty() : sourceUrl;
    extensionXyMarginMeters =
        extensionXyMarginMeters == null ? OptionalDouble.empty() : extensionXyMarginMeters;
    extensionMaxHeightMeters =
        extensionMaxHeightMeters == null ? OptionalDouble.empty() : extensionMaxHeightMeters;
    chassis = chassis == null ? Optional.empty() : chassis;
    links = links == null ? List.of() : List.copyOf(links);
    joints = joints == null ? List.of() : List.copyOf(joints);
    cameras = cameras == null ? List.of() : List.copyOf(cameras);
  }

  /**
   * Finds a link by name (checking both {@link #chassis()} and {@link #links()}).
   *
   * @param linkName Link name to look up.
   * @return Matching {@link CadLinkManifest}, if present.
   */
  public Optional<CadLinkManifest> findLink(String linkName) {
    Objects.requireNonNull(linkName, "linkName");
    if (chassis.isPresent() && chassis.get().name().equals(linkName)) {
      return chassis;
    }
    return links.stream().filter(l -> l.name().equals(linkName)).findFirst();
  }

  /**
   * Returns a link by name or throws an {@link IllegalArgumentException} if not found.
   *
   * @param linkName Link name to look up.
   * @return Matching {@link CadLinkManifest}.
   */
  public CadLinkManifest getLink(String linkName) {
    return findLink(linkName)
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "Link '" + linkName + "' not found in CAD manifest for " + robotName + "."));
  }

  /**
   * Finds a joint by name.
   *
   * @param jointName Joint name to look up.
   * @return Matching {@link CadJointManifest}, if present.
   */
  public Optional<CadJointManifest> findJoint(String jointName) {
    Objects.requireNonNull(jointName, "jointName");
    return joints.stream().filter(j -> j.name().equals(jointName)).findFirst();
  }

  /**
   * Returns a joint by name or throws an {@link IllegalArgumentException} if not found.
   *
   * @param jointName Joint name to look up.
   * @return Matching {@link CadJointManifest}.
   */
  public CadJointManifest getJoint(String jointName) {
    return findJoint(jointName)
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "Joint '" + jointName + "' not found in CAD manifest for " + robotName + "."));
  }

  /**
   * Returns the CAD-extracted mass of the named link as a WPILib {@link Mass} for direct use in
   * YAMS (for example {@code ElevatorConfig.withCarriageWeight(...)}).
   *
   * @param linkName Link name whose mass properties should be read.
   * @return Link mass in kilograms.
   * @throws IllegalStateException If the link does not contain CAD mass properties.
   */
  public Mass getLinkMass(String linkName) {
    CadLinkManifest link = getLink(linkName);
    CadMassProperties props =
        link.massProperties()
            .orElseThrow(
                () ->
                    new IllegalStateException(
                        "Link '" + linkName + "' has no CAD massProperties in manifest."));
    return Kilograms.of(props.massKilograms());
  }

  /**
   * Resolves the CAD moment of inertia about the named joint's motion axis as a WPILib {@link
   * MomentOfInertia} for direct use in YAMS {@code SmartMotorControllerConfig.withMomentOfInertia}.
   * Prefers an explicit {@link CadJointManifest#momentOfInertiaKgM2()} on the joint; otherwise
   * extracts {@code Ixx}, {@code Iyy}, or {@code Izz} along the joint's {@link MotionAxis} from its
   * child link's {@link CadMassProperties}.
   *
   * @param jointName Joint name to inspect.
   * @return Axis moment of inertia in kg·m².
   * @throws IllegalStateException If neither the joint nor its child link specifies inertia.
   */
  public MomentOfInertia getJointMomentOfInertia(String jointName) {
    CadJointManifest joint = getJoint(jointName);
    if (joint.momentOfInertiaKgM2().isPresent()) {
      return KilogramSquareMeters.of(joint.momentOfInertiaKgM2().getAsDouble());
    }
    Optional<CadLinkManifest> child = findLink(joint.childLink());
    if (child.isPresent() && child.get().massProperties().isPresent() && joint.axis() != null) {
      CadMassProperties mp = child.get().massProperties().get();
      double inertia =
          switch (joint.axis()) {
            case X -> mp.ixxKgM2();
            case Y -> mp.iyyKgM2();
            case Z -> mp.izzKgM2();
          };
      if (inertia > 0.0) {
        return KilogramSquareMeters.of(inertia);
      }
    }
    throw new IllegalStateException(
        "Joint '"
            + jointName
            + "' has no momentOfInertiaKgM2 and child link '"
            + joint.childLink()
            + "' has no CAD massProperties.");
  }
}
