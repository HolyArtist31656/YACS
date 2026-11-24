// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Unified YACSim v2027.5.0-alpha1 Showcase: Dual-Pose Truth + Direct DataLog + RK4 3D Ballistics + SOTM Aiming

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Filesystem;
import org.wpilib.system.Notifier;
import yacsim.commands3.CadRobotMechanism;
import yacsim.commands3.YamsCommands3Bridge;
import yacsim.core.ballistics.FieldObstacle3d;
import yacsim.core.ballistics.ProjectilePhysicsSim;
import yacsim.core.ballistics.ProjectileSpec;
import yacsim.core.ballistics.TrajectoryArcPublisher;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.yams.YamsMechanismBridge;

/**
 * Self-contained WPILib 2027 Commands v3 showcase for YACSim {@code v2027.5.0-alpha1}:
 *
 * <ul>
 *   <li>Loads 3D field obstacles &amp; Hub scoring target from {@code yacsim-field.json} (generated
 *       by {@code tools/onshape-field-exporter/}).
 *   <li>Drives a 4-corner animated swerve robot with dual-pose publishing (noisy fused {@code
 *       RobotPose3d} alongside exact {@code SimTruthPose3d}).
 *   <li>Computes Shoot-on-the-Move (SOTM) turret yaw &amp; hood pitch aiming into the Blue Hub,
 *       streaming live predicted arcs ({@code /YACSim/Ballistics/PredictedArc}) and airborne RK4
 *       Fuel projectiles ({@code /YACSim/Ballistics/ActiveProjectiles}) to AdvantageScope and
 *       WPILib {@code DataLog}.
 * </ul>
 */
public class Robot extends TimedRobot implements Mechanism {
  private static final double MODULE_HALF_TRACK = 0.30;
  private static final double WHEEL_RADIUS = 0.0508;
  private static final Translation3d BLUE_HUB_TARGET = new Translation3d(4.60, 4.10, 2.20);
  private static final double LAUNCH_SPEED_MPS = 9.5;
  private static final double[][] MODULE_XY = {
    {MODULE_HALF_TRACK, MODULE_HALF_TRACK},
    {MODULE_HALF_TRACK, -MODULE_HALF_TRACK},
    {-MODULE_HALF_TRACK, MODULE_HALF_TRACK},
    {-MODULE_HALF_TRACK, -MODULE_HALF_TRACK}
  };

  private final Scheduler scheduler = Scheduler.getDefault();
  private final CadRobotMechanism cadSim;
  private final ProjectilePhysicsSim ballisticsSim;
  private final TrajectoryArcPublisher arcPublisher;

  private Notifier simNotifier;
  private double simTime = 0.0;
  private double lastShotTime = -1.0;
  private int lastReportedScore = 0;

  private volatile Pose2d truthPose = new Pose2d(2.0, 2.5, new Rotation2d());
  private volatile Pose2d odometryPose = new Pose2d(2.0, 2.5, new Rotation2d());
  private volatile double turretYawRad = 0.0;
  private volatile double hoodPitchRad = Math.toRadians(-45.0);
  private volatile List<Pose3d> latestPredictedArc = List.of();
  private final double[] moduleDistances = new double[4];
  private final Rotation2d[] moduleAngles = {
    new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()
  };
  private volatile SwerveModulePosition[] modulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  public Robot() {
    DataLogManager.start();
    try {
      Path deployDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/rebuilt3d");
      List<FieldObstacle3d> fieldObstacles =
          FieldObstacle3d.loadFieldManifest(deployDir.resolve("yacsim-field.json"));

      this.ballisticsSim = new ProjectilePhysicsSim().addObstacles(fieldObstacles);
      this.arcPublisher = new TrajectoryArcPublisher("YACSim", true);

      java.util.function.Supplier<Pose3d> turretPoseSupplier =
          () -> new Pose3d(0.0, 0.0, 0.24, new Rotation3d(0.0, 0.0, turretYawRad));
      java.util.function.Supplier<Pose3d> hoodPoseSupplier =
          YamsMechanismBridge.chainParentChild(
              turretPoseSupplier,
              () -> new Pose3d(0.08, 0.0, 0.22, new Rotation3d(0.0, hoodPitchRad, 0.0)));

      CadRobotConfig config =
          new CadRobotConfig("Rebuilt3D_Ballistics")
              .withChassisAsset(new CadAssetConfig(deployDir.resolve("chassis.glb")))
              .withDrivetrainPoseSupplier(() -> odometryPose)
              .withSimTruthPose2dSupplier(() -> truthPose)
              .withDataLogEnabled(true)
              .ignoreRestingOverlaps()
              .withAutoInstallToAdvantageScope()
              .addComponent(
                  new CadComponentConfig("Turret", new CadAssetConfig(deployDir.resolve("turret.glb")))
                      .withParentComponent("Chassis")
                      .withPoseSupplier(turretPoseSupplier))
              .addComponent(
                  new CadComponentConfig("Hood", new CadAssetConfig(deployDir.resolve("hood.glb")))
                      .withParentComponent("Turret")
                      .withPoseSupplier(hoodPoseSupplier));

      List<Translation3d> corners =
          List.of(
              new Translation3d(MODULE_HALF_TRACK, MODULE_HALF_TRACK, WHEEL_RADIUS),
              new Translation3d(MODULE_HALF_TRACK, -MODULE_HALF_TRACK, WHEEL_RADIUS),
              new Translation3d(-MODULE_HALF_TRACK, MODULE_HALF_TRACK, WHEEL_RADIUS),
              new Translation3d(-MODULE_HALF_TRACK, -MODULE_HALF_TRACK, WHEEL_RADIUS));
      YamsCommands3Bridge.attachSwerveModules(
          config,
          new CadAssetConfig(deployDir.resolve("swerve_fork.glb")),
          new CadAssetConfig(deployDir.resolve("swerve_wheel.glb")),
          WHEEL_RADIUS,
          corners,
          () -> odometryPose,
          () -> new Rotation3d(0.0, 0.0, odometryPose.getRotation().getRadians()),
          () -> modulePositions);

      this.cadSim = new CadRobotMechanism(this, config);
      System.out.println(
          "[Rebuilt3D] Loaded "
              + fieldObstacles.size()
              + " field OBBs from yacsim-field.json; dual-pose truth + DataLog + RK4 ballistics active.");
    } catch (IOException e) {
      throw new RuntimeException("Failed to initialize Rebuilt3D Ballistics showcase.", e);
    }
  }

  @Override
  public void robotPeriodic() {
    cadSim.periodic();
    arcPublisher.publishPredictedArc(latestPredictedArc);
    arcPublisher.publishActiveProjectiles(ballisticsSim);
    scheduler.run();
  }

  @Override
  public void simulationInit() {
    simNotifier = new Notifier(this::stepSimulation);
    simNotifier.startPeriodic(0.02);
  }

  private void stepSimulation() {
    double dt = 0.02;
    Pose2d prev = truthPose;
    simTime += dt;

    // Orbit around the Blue Hub (4.6, 4.1) at radius ~2.6 m
    double orbitAng = simTime * 0.45;
    double x = 2.2 + 1.2 * Math.cos(orbitAng);
    double y = 4.1 + 1.8 * Math.sin(orbitAng);
    double heading = orbitAng + Math.PI / 2.0;
    Pose2d nextTruth = new Pose2d(x, y, new Rotation2d(heading));

    // Simulate slight odometry drift vs. ground truth
    double driftX = 0.04 * Math.sin(simTime * 0.7);
    double driftY = 0.04 * Math.cos(simTime * 0.5);
    odometryPose = new Pose2d(x + driftX, y + driftY, new Rotation2d(heading + 0.015));
    truthPose = nextTruth;

    // Swerve module steering & wheel spin kinematics
    double vxField = (x - prev.getX()) / dt;
    double vyField = (y - prev.getY()) / dt;
    double dTheta =
        Math.atan2(
            Math.sin(heading - prev.getRotation().getRadians()),
            Math.cos(heading - prev.getRotation().getRadians()));
    double omega = dTheta / dt;
    double cosH = Math.cos(heading), sinH = Math.sin(heading);
    double vxRobot = vxField * cosH + vyField * sinH;
    double vyRobot = -vxField * sinH + vyField * cosH;

    SwerveModulePosition[] nextMods = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      double mvx = vxRobot - omega * MODULE_XY[i][1];
      double mvy = vyRobot + omega * MODULE_XY[i][0];
      double spd = Math.hypot(mvx, mvy);
      moduleDistances[i] += spd * dt;
      if (spd > 1e-4) {
        moduleAngles[i] = new Rotation2d(Math.atan2(mvy, mvx));
      }
      nextMods[i] = new SwerveModulePosition(moduleDistances[i], moduleAngles[i]);
    }
    modulePositions = nextMods;

    // First-order Shoot-on-the-Move (SOTM) virtual target compensation
    Translation3d launchOrigin = new Translation3d(x, y, 0.52);
    double dist = Math.hypot(BLUE_HUB_TARGET.getX() - x, BLUE_HUB_TARGET.getY() - y);
    double estTof = dist / (LAUNCH_SPEED_MPS * 0.65);
    double virtX = BLUE_HUB_TARGET.getX() - vxField * estTof;
    double virtY = BLUE_HUB_TARGET.getY() - vyField * estTof;
    double fieldAimYaw = Math.atan2(virtY - y, virtX - x);
    turretYawRad = fieldAimYaw - heading;
    double launchPitchRad = Math.toRadians(52.0);
    hoodPitchRad = -launchPitchRad;

    double vHoriz = LAUNCH_SPEED_MPS * Math.cos(launchPitchRad);
    double vZ = LAUNCH_SPEED_MPS * Math.sin(launchPitchRad);
    Translation3d worldLaunchVel =
        new Translation3d(
            vxField + vHoriz * Math.cos(fieldAimYaw),
            vyField + vHoriz * Math.sin(fieldAimYaw),
            vZ);
    Translation3d backspin =
        new Translation3d(Math.sin(fieldAimYaw) * 45.0, -Math.cos(fieldAimYaw) * 45.0, 0.0);

    // Update live AdvantageScope aim arc preview
    latestPredictedArc =
        ballisticsSim.predictArc(
            ProjectileSpec.frcFuel2026(), launchOrigin, worldLaunchVel, backspin, 0.04, 45);

    // Fire one Fuel projectile every 1.2 seconds
    if (simTime - lastShotTime >= 1.2) {
      lastShotTime = simTime;
      ballisticsSim.launch(ProjectileSpec.frcFuel2026(), launchOrigin, worldLaunchVel, backspin);
    }
    ballisticsSim.step(dt);

    int scored = ballisticsSim.getScoredCount();
    if (scored > lastReportedScore) {
      lastReportedScore = scored;
      System.out.printf("[Rebuilt3D] SCORED into BlueHub! Total scored: %d%n", scored);
    }

    cadSim.simulationPeriodic();
  }
}
