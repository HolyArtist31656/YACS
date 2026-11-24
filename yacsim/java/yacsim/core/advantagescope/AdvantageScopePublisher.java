// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.advantagescope;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.datalog.BooleanLogEntry;
import org.wpilib.datalog.DataLog;
import org.wpilib.datalog.StringArrayLogEntry;
import org.wpilib.datalog.StructArrayLogEntry;
import org.wpilib.datalog.StructLogEntry;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringArrayPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.system.DataLogManager;
import yacsim.core.model.CadRollerTelemetryConfig;
import yacsim.core.physics.CollisionEvent;

/**
 * Publishes YACSim robot poses, optional ground-truth simulation poses ({@code SimTruthPose3d}),
 * articulated component poses, OBB collision telemetry, and spinning roller/flywheel telemetry to
 * NetworkTables 4 and (optionally) directly to WPILib {@link DataLog} for competition-grade
 * AdvantageScope review even when NT4 publishing is disabled.
 */
public final class AdvantageScopePublisher implements AutoCloseable {
  private static final double SPINNING_THRESHOLD_RAD_PER_SEC = 0.5;

  private record RollerPubs(
      DoublePublisher rpmPub,
      DoublePublisher surfaceSpeedPub,
      BooleanPublisher spinningPub,
      double radiusMeters) {}

  private final StructPublisher<Pose2d> robotPosePub;
  private final StructPublisher<Pose3d> robotPose3dPub;
  private final StructPublisher<Pose3d> simTruthPose3dPub;
  private final StructArrayPublisher<Pose3d> componentPosesPub;
  private final BooleanPublisher collisionActivePub;
  private final StringArrayPublisher collisionDetailsPub;
  private final Map<String, RollerPubs> rollerPublishers = new LinkedHashMap<>();

  private final StructLogEntry<Pose2d> robotPoseLog;
  private final StructLogEntry<Pose3d> robotPose3dLog;
  private final StructLogEntry<Pose3d> simTruthPose3dLog;
  private final StructArrayLogEntry<Pose3d> componentPosesLog;
  private final BooleanLogEntry collisionActiveLog;
  private final StringArrayLogEntry collisionDetailsLog;

  /**
   * Opens NetworkTables 4 struct, boolean, and string-array publishers under the given root table.
   *
   * @param rootTableName Root NetworkTables table name (for example {@code "YACSim"}).
   */
  public AdvantageScopePublisher(String rootTableName) {
    this(rootTableName, List.of(), false, false);
  }

  /**
   * Opens NetworkTables 4 publishers under the given root table, plus per-roller telemetry topics
   * under {@code /<rootTableName>/Rollers/<name>/}.
   *
   * @param rootTableName Root NetworkTables table name (for example {@code "YACSim"}).
   * @param rollers Registered spinning roller/flywheel telemetry configurations.
   */
  public AdvantageScopePublisher(String rootTableName, List<CadRollerTelemetryConfig> rollers) {
    this(rootTableName, rollers, false, false);
  }

  /**
   * Opens NetworkTables 4 publishers and optional direct WPILib {@link DataLog} struct entries
   * (plus optional {@code SimTruthPose3d} dual-pose ground-truth publishing).
   *
   * @param rootTableName Root NetworkTables table name (for example {@code "YACSim"}).
   * @param rollers Registered spinning roller/flywheel telemetry configurations.
   * @param enableDataLog {@code true} to also append directly to {@link DataLogManager#getLog()}.
   * @param publishSimTruthPose {@code true} to open the {@code SimTruthPose3d} topic/log entry.
   */
  public AdvantageScopePublisher(
      String rootTableName,
      List<CadRollerTelemetryConfig> rollers,
      boolean enableDataLog,
      boolean publishSimTruthPose) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(rootTableName);
    this.robotPosePub = table.getStructTopic("RobotPose", Pose2d.struct).publish();
    this.robotPose3dPub = table.getStructTopic("RobotPose3d", Pose3d.struct).publish();
    this.simTruthPose3dPub =
        publishSimTruthPose ? table.getStructTopic("SimTruthPose3d", Pose3d.struct).publish() : null;
    this.componentPosesPub = table.getStructArrayTopic("ComponentPoses", Pose3d.struct).publish();
    this.collisionActivePub = table.getBooleanTopic("CollisionActive").publish();
    this.collisionDetailsPub = table.getStringArrayTopic("CollisionDetails").publish();

    if (rollers != null && !rollers.isEmpty()) {
      NetworkTable rollersTable = table.getSubTable("Rollers");
      for (CadRollerTelemetryConfig roller : rollers) {
        NetworkTable sub = rollersTable.getSubTable(roller.name());
        rollerPublishers.put(
            roller.name(),
            new RollerPubs(
                sub.getDoubleTopic("VelocityRPM").publish(),
                sub.getDoubleTopic("SurfaceSpeedMPS").publish(),
                sub.getBooleanTopic("Spinning").publish(),
                roller.wheelRadiusMeters()));
      }
    }

    if (enableDataLog) {
      DataLog log = DataLogManager.getLog();
      String prefix = "/" + rootTableName + "/";
      this.robotPoseLog = StructLogEntry.create(log, prefix + "RobotPose", Pose2d.struct);
      this.robotPose3dLog = StructLogEntry.create(log, prefix + "RobotPose3d", Pose3d.struct);
      this.simTruthPose3dLog =
          publishSimTruthPose
              ? StructLogEntry.create(log, prefix + "SimTruthPose3d", Pose3d.struct)
              : null;
      this.componentPosesLog =
          StructArrayLogEntry.create(log, prefix + "ComponentPoses", Pose3d.struct);
      this.collisionActiveLog = new BooleanLogEntry(log, prefix + "CollisionActive");
      this.collisionDetailsLog = new StringArrayLogEntry(log, prefix + "CollisionDetails");
    } else {
      this.robotPoseLog = null;
      this.robotPose3dLog = null;
      this.simTruthPose3dLog = null;
      this.componentPosesLog = null;
      this.collisionActiveLog = null;
      this.collisionDetailsLog = null;
    }
  }

  /**
   * Publishes a consistent simulation frame to NetworkTables (and DataLog if enabled).
   *
   * @param robotPose 2D robot field pose.
   * @param robotPose3d 3D robot field pose including gyro pitch and roll.
   * @param componentPoses Array of articulated component poses in registration order.
   * @param collisions Active collision events for this frame.
   */
  public void publish(
      Pose2d robotPose,
      Pose3d robotPose3d,
      Pose3d[] componentPoses,
      List<CollisionEvent> collisions) {
    publish(robotPose, robotPose3d, null, componentPoses, collisions, Map.of());
  }

  /**
   * Publishes a consistent simulation frame including spinning roller/flywheel velocities.
   *
   * @param robotPose 2D robot field pose.
   * @param robotPose3d 3D robot field pose including gyro pitch and roll.
   * @param componentPoses Array of articulated component poses in registration order.
   * @param collisions Active collision events for this frame.
   * @param rollerRadPerSec Map from roller name to angular velocity in rad/s.
   */
  public void publish(
      Pose2d robotPose,
      Pose3d robotPose3d,
      Pose3d[] componentPoses,
      List<CollisionEvent> collisions,
      Map<String, Double> rollerRadPerSec) {
    publish(robotPose, robotPose3d, null, componentPoses, collisions, rollerRadPerSec);
  }

  /**
   * Publishes a consistent simulation frame including optional ground-truth {@code simTruthPose3d}
   * and spinning roller/flywheel velocities.
   *
   * @param robotPose 2D robot field pose (fused odometry).
   * @param robotPose3d 3D robot field pose including gyro pitch and roll.
   * @param simTruthPose3d Optional ground-truth 3D simulated pose (may be {@code null}).
   * @param componentPoses Array of articulated component poses in registration order.
   * @param collisions Active collision events for this frame.
   * @param rollerRadPerSec Map from roller name to angular velocity in rad/s.
   */
  public void publish(
      Pose2d robotPose,
      Pose3d robotPose3d,
      Pose3d simTruthPose3d,
      Pose3d[] componentPoses,
      List<CollisionEvent> collisions,
      Map<String, Double> rollerRadPerSec) {
    robotPosePub.set(robotPose);
    robotPose3dPub.set(robotPose3d);
    if (simTruthPose3dPub != null && simTruthPose3d != null) {
      simTruthPose3dPub.set(simTruthPose3d);
    }
    componentPosesPub.set(componentPoses);
    boolean hasCollision = !collisions.isEmpty();
    collisionActivePub.set(hasCollision);
    String[] details = new String[collisions.size()];
    for (int i = 0; i < collisions.size(); i++) {
      CollisionEvent ev = collisions.get(i);
      details[i] =
          String.format(
              "%s <-> %s (%.4f m)", ev.componentA(), ev.componentB(), ev.penetrationDepthMeters());
    }
    collisionDetailsPub.set(details);

    if (robotPoseLog != null) {
      robotPoseLog.append(robotPose);
      robotPose3dLog.append(robotPose3d);
      if (simTruthPose3dLog != null && simTruthPose3d != null) {
        simTruthPose3dLog.append(simTruthPose3d);
      }
      componentPosesLog.append(componentPoses);
      collisionActiveLog.append(hasCollision);
      collisionDetailsLog.append(details);
    }

    for (Map.Entry<String, RollerPubs> entry : rollerPublishers.entrySet()) {
      double radPerSec = rollerRadPerSec.getOrDefault(entry.getKey(), 0.0);
      RollerPubs pubs = entry.getValue();
      pubs.rpmPub().set(radPerSec * 60.0 / (2.0 * Math.PI));
      pubs.surfaceSpeedPub().set(radPerSec * pubs.radiusMeters());
      pubs.spinningPub().set(Math.abs(radPerSec) > SPINNING_THRESHOLD_RAD_PER_SEC);
    }
  }

  /** Closes all NetworkTables publishers and DataLog entries owned by this instance. */
  @Override
  public void close() {
    robotPosePub.close();
    robotPose3dPub.close();
    if (simTruthPose3dPub != null) {
      simTruthPose3dPub.close();
    }
    componentPosesPub.close();
    collisionActivePub.close();
    collisionDetailsPub.close();
    if (robotPoseLog != null) {
      robotPoseLog.finish();
      robotPose3dLog.finish();
      if (simTruthPose3dLog != null) {
        simTruthPose3dLog.finish();
      }
      componentPosesLog.finish();
      collisionActiveLog.finish();
      collisionDetailsLog.finish();
    }
    for (RollerPubs pubs : rollerPublishers.values()) {
      pubs.rpmPub().close();
      pubs.surfaceSpeedPub().close();
      pubs.spinningPub().close();
    }
    rollerPublishers.clear();
  }
}
