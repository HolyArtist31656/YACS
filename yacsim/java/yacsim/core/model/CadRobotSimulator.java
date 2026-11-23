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

  public CadRobotSimulator(CadRobotConfig config) throws IOException {
    this(config, true);
  }

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

  public Optional<MeshMetadata> getChassisMetadata() {
    return Optional.ofNullable(chassisMetadata);
  }

  public Optional<MeshMetadata> getComponentMetadata(String name) {
    return Optional.ofNullable(componentMetadata.get(name));
  }

  public Optional<CadMassProperties> getComponentMassProperties(String name) {
    return getComponentMetadata(name).map(MeshMetadata::massProperties);
  }

  public Pose2d getLatestRobotPose2d() {
    return latestSnapshot.robotPose2d();
  }

  public Pose3d getLatestRobotPose3d() {
    return latestSnapshot.robotPose3d();
  }

  public Pose3d[] getLatestComponentPoses() {
    return latestSnapshot.componentPoses().clone();
  }

  public List<CollisionEvent> getLatestCollisions() {
    return latestSnapshot.collisions();
  }

  public boolean hasAnyCollision() {
    return !latestSnapshot.collisions().isEmpty();
  }

  public boolean hasCollisionBetween(String a, String b) {
    return latestSnapshot.collisions().stream()
        .anyMatch(
            ev ->
                (ev.componentA().equals(a) && ev.componentB().equals(b))
                    || (ev.componentA().equals(b) && ev.componentB().equals(a)));
  }

  public boolean hasExtensionViolation() {
    return latestSnapshot.collisions().stream().anyMatch(ev -> ev.involves("ExtensionEnvelope"));
  }

  public int getTotalTriangleCount() {
    int total = chassisMetadata != null ? chassisMetadata.triangleCount() : 0;
    for (MeshMetadata meta : componentMetadata.values()) {
      total += meta.triangleCount();
    }
    return total;
  }

  @Override
  public void close() {
    if (publisher != null) {
      publisher.close();
      publisher = null;
    }
  }
}
