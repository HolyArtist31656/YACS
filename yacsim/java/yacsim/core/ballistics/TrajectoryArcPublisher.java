// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.ballistics;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.wpilib.datalog.DataLog;
import org.wpilib.datalog.StructArrayLogEntry;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.system.DataLogManager;

/**
 * Publishes predicted aiming arcs and live airborne projectile poses as {@code Pose3d[]} struct
 * arrays to NetworkTables 4 (and optionally directly to WPILib {@link DataLog}) for AdvantageScope
 * 3D Field visualization.
 *
 * <p>Uses delta/change-detection throttling so unchanged or resting/empty projectile arrays are not
 * redundantly re-streamed every 20 ms (mitigating AdvantageScope 3D FPS degradation during
 * high-count Fuel/ballistics matches).
 */
public final class TrajectoryArcPublisher implements AutoCloseable {
  private final StructArrayPublisher<Pose3d> predictedArcPub;
  private final StructArrayPublisher<Pose3d> activeProjectilesPub;
  private final StructArrayLogEntry<Pose3d> predictedArcLog;
  private final StructArrayLogEntry<Pose3d> activeProjectilesLog;

  private Pose3d[] lastPublishedArc = null;
  private Pose3d[] lastPublishedProjectiles = null;
  private int publishedArcFrames = 0;
  private int publishedProjectileFrames = 0;

  /**
   * Opens NT4 publishers under {@code /<rootTableName>/Ballistics/}.
   *
   * @param rootTableName Root NetworkTables table (for example {@code "YACSim"}).
   */
  public TrajectoryArcPublisher(String rootTableName) {
    this(rootTableName, true, false);
  }

  /**
   * Opens NT4 publishers (and optional direct WPILib {@link DataLog} struct entries) under {@code
   * /<rootTableName>/Ballistics/}.
   *
   * @param rootTableName Root NetworkTables table (for example {@code "YACSim"}).
   * @param enableDataLog {@code true} to also append directly to {@link DataLogManager#getLog()}.
   */
  public TrajectoryArcPublisher(String rootTableName, boolean enableDataLog) {
    this(rootTableName, true, enableDataLog);
  }

  /**
   * Opens optional NT4 publishers and optional direct WPILib {@link DataLog} struct entries under
   * {@code /<rootTableName>/Ballistics/}. Passing {@code enableNetworkTables = false} supports
   * DataLog-only competition logging and headless JUnit 5 tests without native NT4 libraries.
   *
   * @param rootTableName Root NetworkTables table (for example {@code "YACSim"}).
   * @param enableNetworkTables {@code true} to publish to NetworkTables 4.
   * @param enableDataLog {@code true} to append directly to {@link DataLogManager#getLog()}.
   */
  public TrajectoryArcPublisher(
      String rootTableName, boolean enableNetworkTables, boolean enableDataLog) {
    Objects.requireNonNull(rootTableName, "rootTableName");
    if (enableNetworkTables) {
      NetworkTable table =
          NetworkTableInstance.getDefault().getTable(rootTableName).getSubTable("Ballistics");
      this.predictedArcPub = table.getStructArrayTopic("PredictedArc", Pose3d.struct).publish();
      this.activeProjectilesPub =
          table.getStructArrayTopic("ActiveProjectiles", Pose3d.struct).publish();
    } else {
      this.predictedArcPub = null;
      this.activeProjectilesPub = null;
    }
    if (enableDataLog) {
      DataLog log = DataLogManager.getLog();
      String prefix = "/" + rootTableName + "/Ballistics/";
      this.predictedArcLog =
          StructArrayLogEntry.create(log, prefix + "PredictedArc", Pose3d.struct);
      this.activeProjectilesLog =
          StructArrayLogEntry.create(log, prefix + "ActiveProjectiles", Pose3d.struct);
    } else {
      this.predictedArcLog = null;
      this.activeProjectilesLog = null;
    }
  }

  /**
   * Publishes a predicted aim arc to AdvantageScope if it changed since the last published frame.
   *
   * @param arcPoses Predicted {@link Pose3d} samples along the arc.
   */
  public void publishPredictedArc(List<Pose3d> arcPoses) {
    Pose3d[] arr = arcPoses == null ? new Pose3d[0] : arcPoses.toArray(new Pose3d[0]);
    if (lastPublishedArc != null && Arrays.equals(arr, lastPublishedArc)) {
      return;
    }
    lastPublishedArc = arr;
    publishedArcFrames++;
    if (predictedArcPub != null) {
      predictedArcPub.set(arr);
    }
    if (predictedArcLog != null) {
      predictedArcLog.append(arr);
    }
  }

  /**
   * Publishes all currently active airborne projectiles from {@code sim} to AdvantageScope if their
   * poses changed since the last published frame.
   *
   * @param sim Projectile physics simulator.
   */
  public void publishActiveProjectiles(ProjectilePhysicsSim sim) {
    Pose3d[] arr = sim == null ? new Pose3d[0] : sim.getActiveProjectilePoses();
    if (lastPublishedProjectiles != null && Arrays.equals(arr, lastPublishedProjectiles)) {
      return;
    }
    lastPublishedProjectiles = arr;
    publishedProjectileFrames++;
    if (activeProjectilesPub != null) {
      activeProjectilesPub.set(arr);
    }
    if (activeProjectilesLog != null) {
      activeProjectilesLog.append(arr);
    }
  }

  /** @return Number of non-redundant predicted-arc frames published since construction. */
  public int getPublishedArcFrames() {
    return publishedArcFrames;
  }

  /** @return Number of non-redundant active-projectile frames published since construction. */
  public int getPublishedProjectileFrames() {
    return publishedProjectileFrames;
  }

  @Override
  public void close() {
    if (predictedArcPub != null) {
      predictedArcPub.close();
    }
    if (activeProjectilesPub != null) {
      activeProjectilesPub.close();
    }
    if (predictedArcLog != null) {
      predictedArcLog.finish();
    }
    if (activeProjectilesLog != null) {
      activeProjectilesLog.finish();
    }
  }
}