// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.ballistics;

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
 */
public final class TrajectoryArcPublisher implements AutoCloseable {
  private final StructArrayPublisher<Pose3d> predictedArcPub;
  private final StructArrayPublisher<Pose3d> activeProjectilesPub;
  private final StructArrayLogEntry<Pose3d> predictedArcLog;
  private final StructArrayLogEntry<Pose3d> activeProjectilesLog;

  /**
   * Opens NT4 publishers under {@code /<rootTableName>/Ballistics/}.
   *
   * @param rootTableName Root NetworkTables table (for example {@code "YACSim"}).
   */
  public TrajectoryArcPublisher(String rootTableName) {
    this(rootTableName, false);
  }

  /**
   * Opens NT4 publishers (and optional direct WPILib {@link DataLog} struct entries) under {@code
   * /<rootTableName>/Ballistics/}.
   *
   * @param rootTableName Root NetworkTables table (for example {@code "YACSim"}).
   * @param enableDataLog {@code true} to also append directly to {@link DataLogManager#getLog()}.
   */
  public TrajectoryArcPublisher(String rootTableName, boolean enableDataLog) {
    Objects.requireNonNull(rootTableName, "rootTableName");
    NetworkTable table =
        NetworkTableInstance.getDefault().getTable(rootTableName).getSubTable("Ballistics");
    this.predictedArcPub = table.getStructArrayTopic("PredictedArc", Pose3d.struct).publish();
    this.activeProjectilesPub =
        table.getStructArrayTopic("ActiveProjectiles", Pose3d.struct).publish();
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
   * Publishes a predicted aim arc to AdvantageScope.
   *
   * @param arcPoses Predicted {@link Pose3d} samples along the arc.
   */
  public void publishPredictedArc(List<Pose3d> arcPoses) {
    Pose3d[] arr = arcPoses == null ? new Pose3d[0] : arcPoses.toArray(new Pose3d[0]);
    predictedArcPub.set(arr);
    if (predictedArcLog != null) {
      predictedArcLog.append(arr);
    }
  }

  /**
   * Publishes all currently active airborne projectiles from {@code sim} to AdvantageScope.
   *
   * @param sim Projectile physics simulator.
   */
  public void publishActiveProjectiles(ProjectilePhysicsSim sim) {
    Pose3d[] arr = sim == null ? new Pose3d[0] : sim.getActiveProjectilePoses();
    activeProjectilesPub.set(arr);
    if (activeProjectilesLog != null) {
      activeProjectilesLog.append(arr);
    }
  }

  @Override
  public void close() {
    predictedArcPub.close();
    activeProjectilesPub.close();
    if (predictedArcLog != null) {
      predictedArcLog.finish();
    }
    if (activeProjectilesLog != null) {
      activeProjectilesLog.finish();
    }
  }
}
