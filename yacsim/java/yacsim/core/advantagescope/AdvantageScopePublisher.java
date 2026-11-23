// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.advantagescope;

import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringArrayPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import yacsim.core.physics.CollisionEvent;

/**
 * Publishes YACSim robot poses, articulated component poses, and OBB collision telemetry to
 * NetworkTables 4 for live visualization in AdvantageScope.
 */
public final class AdvantageScopePublisher implements AutoCloseable {
  private final StructPublisher<Pose2d> robotPosePub;
  private final StructPublisher<Pose3d> robotPose3dPub;
  private final StructArrayPublisher<Pose3d> componentPosesPub;
  private final BooleanPublisher collisionActivePub;
  private final StringArrayPublisher collisionDetailsPub;

  /**
   * Opens NetworkTables 4 struct, boolean, and string-array publishers under the given root table.
   *
   * @param rootTableName Root NetworkTables table name (for example {@code "YACSim"}).
   */
  public AdvantageScopePublisher(String rootTableName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(rootTableName);
    this.robotPosePub = table.getStructTopic("RobotPose", Pose2d.struct).publish();
    this.robotPose3dPub = table.getStructTopic("RobotPose3d", Pose3d.struct).publish();
    this.componentPosesPub = table.getStructArrayTopic("ComponentPoses", Pose3d.struct).publish();
    this.collisionActivePub = table.getBooleanTopic("CollisionActive").publish();
    this.collisionDetailsPub = table.getStringArrayTopic("CollisionDetails").publish();
  }

  /**
   * Publishes a consistent simulation frame to NetworkTables.
   *
   * @param robotPose 2D robot field pose.
   * @param robotPose3d 3D robot field pose including gyro pitch and roll.
   * @param componentPoses Array of articulated component poses in registration order.
   * @param collisions Active collision events for this frame.
   */
  public void publish(
      Pose2d robotPose, Pose3d robotPose3d, Pose3d[] componentPoses, List<CollisionEvent> collisions) {
    robotPosePub.set(robotPose);
    robotPose3dPub.set(robotPose3d);
    componentPosesPub.set(componentPoses);
    collisionActivePub.set(!collisions.isEmpty());
    String[] details = new String[collisions.size()];
    for (int i = 0; i < collisions.size(); i++) {
      CollisionEvent ev = collisions.get(i);
      details[i] = String.format("%s <-> %s (%.4f m)", ev.componentA(), ev.componentB(), ev.penetrationDepthMeters());
    }
    collisionDetailsPub.set(details);
  }

  /** Closes all NetworkTables publishers owned by this instance. */
  @Override
  public void close() {
    robotPosePub.close();
    robotPose3dPub.close();
    componentPosesPub.close();
    collisionActivePub.close();
    collisionDetailsPub.close();
  }
}
