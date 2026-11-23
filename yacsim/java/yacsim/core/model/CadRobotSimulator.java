// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.model;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import yacsim.core.advantagescope.AdvantageScopeConfigGenerator;
import yacsim.core.advantagescope.AdvantageScopePublisher;
import yacsim.core.cad.MeshMetadata;
import yacsim.core.cad.MeshParser;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.CadMassProperties;
import yacsim.core.physics.CollisionEvent;
import yacsim.core.physics.ObbCollisionDetector;

/**
 * Core thread-safe 3D CAD simulation engine. Uses a lock-free {@code volatile SimSnapshot} record
 * so {@link #simIterate()} can run on a high-rate {@code Notifier} thread while {@link
 * #updateTelemetry()} publishes consistent snapshots on the 50 Hz main robot thread.
 */
public final class CadRobotSimulator implements AutoCloseable {
  private record SimSnapshot(
      Pose2d robotPose2d,
      Pose3d robotPose3d,
      Pose3d[] componentPoses,
      List<CollisionEvent> collisions) {}

  private final CadRobotConfig config;
  private final List<CadComponentConfig> allComponents;
  private final MeshMetadata chassisMetadata;
  private final Map<String, MeshMetadata> componentMetadata = new LinkedHashMap<>();
  private volatile SimSnapshot latestSnapshot;
  private AdvantageScopePublisher publisher;

  /**
   * Parses all configured CAD meshes, optionally exports an AdvantageScope asset bundle, and
   * initializes NetworkTables publishing.
   *
   * @param config Robot CAD configuration.
   * @param enableNetworkTables {@code true} to publish NT4 telemetry, {@code false} for headless unit tests.
   * @throws IOException If parsing a mesh or exporting the AdvantageScope bundle fails.
   */
  public CadRobotSimulator(CadRobotConfig config, boolean enableNetworkTables) throws IOException {
    this.config = config;
    this.allComponents = config.getAllArticulatedComponents();
    this.chassisMetadata =
        config.getChassisAsset().isPresent()
            ? MeshParser.parse(config.getChassisAsset().get())
            : null;
    for (CadComponentConfig comp : allComponents) {
      componentMetadata.put(comp.getName(), MeshParser.parse(comp.getAssetConfig()));
    }
    Pose3d[] initialPoses = new Pose3d[allComponents.size()];
    for (int i = 0; i < initialPoses.length; i++) {
      initialPoses[i] = new Pose3d();
    }
    this.latestSnapshot = new SimSnapshot(new Pose2d(), new Pose3d(), initialPoses, List.of());
    if (config.getAdvantageScopeOutputDir().isPresent()) {
      AdvantageScopeConfigGenerator.exportAssetBundle(
          config, config.getAdvantageScopeOutputDir().get());
    }
    if (enableNetworkTables) {
      this.publisher = new AdvantageScopePublisher(config.getTelemetryTable());
    }
  }

  /**
   * Creates a simulator with NetworkTables telemetry enabled.
   *
   * @param config Robot CAD configuration.
   * @throws IOException If parsing a mesh or exporting the AdvantageScope bundle fails.
   */
  public CadRobotSimulator(CadRobotConfig config) throws IOException {
    this(config, true);
  }

  /**
   * Samples all pose suppliers, runs OBB collision and envelope checks, and atomically publishes a
   * new immutable simulation snapshot.
   */
  public void simIterate() {
    Pose2d robotPose2d = config.getDrivetrainPoseSupplier().get();
    Pose3d robotPose3d =
        config.getDrivetrainConfig().isPresent()
            ? config.getDrivetrainConfig().get().computeRobotPose3d()
            : new Pose3d(robotPose2d);
    Pose3d[] poses = new Pose3d[allComponents.size()];
    for (int i = 0; i < allComponents.size(); i++) {
      poses[i] = allComponents.get(i).getPoseSupplier().get();
    }
    List<CollisionEvent> collisions =
        config.isCollisionCheckingEnabled() ? evaluateCollisions(poses) : List.of();
    this.latestSnapshot =
        new SimSnapshot(robotPose2d, robotPose3d, poses, List.copyOf(collisions));
  }

  /** Publishes the latest immutable simulation snapshot to NetworkTables if enabled. */
  public void updateTelemetry() {
    if (publisher != null) {
      SimSnapshot snap = this.latestSnapshot;
      publisher.publish(
          snap.robotPose2d(), snap.robotPose3d(), snap.componentPoses(), snap.collisions());
    }
  }

  private List<CollisionEvent> evaluateCollisions(Pose3d[] poses) {
    List<CollisionEvent> collisions = new ArrayList<>();
    boolean checkChassis =
        chassisMetadata != null
            && config.getDrivetrainConfig().map(d -> d.isChassisCollisionEnabled()).orElse(true);

    for (int i = 0; i < allComponents.size(); i++) {
      CadComponentConfig compA = allComponents.get(i);
      if (!compA.isCollisionEnabled()) {
        continue;
      }
      BoundingBox3d boxA = componentMetadata.get(compA.getName()).localBounds();

      if (checkChassis
          && !compA.getParentComponentName().map("Chassis"::equals).orElse(false)
          && !config.isCollisionPairIgnored(compA.getName(), "Chassis")) {
        OptionalDouble chassisHit =
            ObbCollisionDetector.testObbCollision(
                boxA, poses[i], chassisMetadata.localBounds(), new Pose3d());
        if (chassisHit.isPresent()) {
          collisions.add(
              new CollisionEvent(compA.getName(), "Chassis", chassisHit.getAsDouble()));
        }
      }

      for (int j = i + 1; j < allComponents.size(); j++) {
        CadComponentConfig compB = allComponents.get(j);
        if (!compB.isCollisionEnabled()
            || areParentChild(compA, compB)
            || config.isCollisionPairIgnored(compA.getName(), compB.getName())) {
          continue;
        }
        BoundingBox3d boxB = componentMetadata.get(compB.getName()).localBounds();
        OptionalDouble depth =
            ObbCollisionDetector.testObbCollision(boxA, poses[i], boxB, poses[j]);
        if (depth.isPresent()) {
          collisions.add(
              new CollisionEvent(compA.getName(), compB.getName(), depth.getAsDouble()));
        }
      }

      if (config.getExtensionEnvelope().isPresent()
          && !config.isCollisionPairIgnored(compA.getName(), "ExtensionEnvelope")) {
        OptionalDouble violation =
            ObbCollisionDetector.testEnvelopeViolation(
                boxA, poses[i], config.getExtensionEnvelope().get());
        if (violation.isPresent()) {
          collisions.add(
              new CollisionEvent(compA.getName(), "ExtensionEnvelope", violation.getAsDouble()));
        }
      }
    }
    return collisions;
  }

  private static boolean areParentChild(CadComponentConfig a, CadComponentConfig b) {
    return a.getParentComponentName().map(p -> p.equals(b.getName())).orElse(false)
        || b.getParentComponentName().map(p -> p.equals(a.getName())).orElse(false);
  }

  /** @return Parsed chassis {@link MeshMetadata}, if a chassis asset is configured. */
  public Optional<MeshMetadata> getChassisMetadata() {
    return Optional.ofNullable(chassisMetadata);
  }

  /**
   * Returns parsed {@link MeshMetadata} for a named articulated component.
   *
   * @param name Component name.
   * @return Metadata if the component exists.
   */
  public Optional<MeshMetadata> getComponentMetadata(String name) {
    return Optional.ofNullable(componentMetadata.get(name));
  }

  /**
   * Returns mass and inertia properties for a named articulated component.
   *
   * @param name Component name.
   * @return Mass properties if the component exists.
   */
  public Optional<CadMassProperties> getComponentMassProperties(String name) {
    return getComponentMetadata(name).map(MeshMetadata::massProperties);
  }

  /** @return Latest sampled 2D field pose. */
  public Pose2d getLatestRobotPose2d() {
    return latestSnapshot.robotPose2d();
  }

  /** @return Latest sampled 3D field pose. */
  public Pose3d getLatestRobotPose3d() {
    return latestSnapshot.robotPose3d();
  }

  /** @return Defensive copy of the latest articulated component poses. */
  public Pose3d[] getLatestComponentPoses() {
    return latestSnapshot.componentPoses().clone();
  }

  /** @return Immutable list of collisions detected on the latest iteration. */
  public List<CollisionEvent> getLatestCollisions() {
    return latestSnapshot.collisions();
  }

  /** @return {@code true} if any collision or envelope violation is active. */
  public boolean hasAnyCollision() {
    return !latestSnapshot.collisions().isEmpty();
  }

  /**
   * Checks whether the two named components collided on the latest iteration (order-independent).
   *
   * @param a First component name.
   * @param b Second component name.
   * @return {@code true} if a collision event involves both names.
   */
  public boolean hasCollisionBetween(String a, String b) {
    return latestSnapshot.collisions().stream()
        .anyMatch(
            ev ->
                (ev.componentA().equals(a) && ev.componentB().equals(b))
                    || (ev.componentA().equals(b) && ev.componentB().equals(a)));
  }

  /** @return {@code true} if any component breached {@code "ExtensionEnvelope"} on the latest iteration. */
  public boolean hasExtensionViolation() {
    return latestSnapshot.collisions().stream().anyMatch(ev -> ev.involves("ExtensionEnvelope"));
  }

  /** @return Sum of triangle counts across the chassis and all articulated components. */
  public int getTotalTriangleCount() {
    int total = chassisMetadata != null ? chassisMetadata.triangleCount() : 0;
    for (MeshMetadata meta : componentMetadata.values()) {
      total += meta.triangleCount();
    }
    return total;
  }

  /** Closes NetworkTables publishers owned by this simulator. */
  @Override
  public void close() {
    if (publisher != null) {
      publisher.close();
      publisher = null;
    }
  }
}
