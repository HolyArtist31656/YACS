// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package frc.robot.mechanisms;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
import yacsim.core.advantagescope.AdvantageScopeConfigGenerator;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.drivetrain.CadDrivetrainConfig;
import yacsim.core.drivetrain.YamsDrivetrainBridge;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.yams.MotionAxis;
import yacsim.core.yams.YamsMechanismBridge;

public class SuperstructureMechanism implements Mechanism {
  private final CadRobotMechanism cadSim;
  private double simTimeSeconds = 0.0;
  private double simulatedElevatorHeightMeters = 0.25;
  private double simulatedArmAngleRadians = Math.toRadians(20.0);
  private Pose2d simulatedChassisPose = new Pose2d(2.0, 2.0, new Rotation2d());
  private Rotation2d simulatedSteerAngle = new Rotation2d();
  private double simulatedWheelDistanceMeters = 0.0;

  public SuperstructureMechanism() throws IOException {
    Path modelsDir = Filesystem.getDeployDirectory().toPath().resolve("yacsim/models");
    Path advantageScopeDir =
        Filesystem.getOperatingDirectory().toPath().resolve("advantagescope-assets");

    CadDrivetrainConfig drivetrainConfig =
        YamsDrivetrainBridge.fromYamsSwerve(
            new CadAssetConfig(modelsDir.resolve("chassis.glb")),
            new CadAssetConfig(modelsDir.resolve("swerve_fork.glb")),
            new CadAssetConfig(modelsDir.resolve("swerve_wheel.glb")),
            0.0508,
            List.of(
                new Translation3d(0.30, 0.30, 0.0508),
                new Translation3d(0.30, -0.30, 0.0508),
                new Translation3d(-0.30, 0.30, 0.0508),
                new Translation3d(-0.30, -0.30, 0.0508)),
            () -> simulatedChassisPose,
            () -> new Rotation3d(0.0, 0.0, simulatedChassisPose.getRotation().getRadians()),
            () ->
                new SwerveModulePosition[] {
                  new SwerveModulePosition(simulatedWheelDistanceMeters, simulatedSteerAngle),
                  new SwerveModulePosition(simulatedWheelDistanceMeters, simulatedSteerAngle),
                  new SwerveModulePosition(simulatedWheelDistanceMeters, simulatedSteerAngle),
                  new SwerveModulePosition(simulatedWheelDistanceMeters, simulatedSteerAngle)
                });

    Supplier<Pose3d> elevatorPose =
        YamsMechanismBridge.fromDistanceSupplier(
            () -> Meters.of(simulatedElevatorHeightMeters), new Translation3d(), MotionAxis.Z);
    Supplier<Pose3d> armLocalPose =
        YamsMechanismBridge.fromAngleSupplier(
            () -> Radians.of(simulatedArmAngleRadians),
            new Translation3d(0.12, 0.0, 0.05),
            MotionAxis.Y,
            true);

    CadRobotConfig robotConfig =
        new CadRobotConfig("ExampleBot_2027")
            .withDrivetrain(drivetrainConfig)
            .withExtensionMargin(0.30, 1.98)
            .ignoreRestingOverlaps()
            .autoGroundChassis()
            .addCamera(
                "FrontCam",
                new Transform3d(
                    new Translation3d(0.25, 0.0, 0.40),
                    new Rotation3d(0.0, Math.toRadians(-15.0), 0.0)),
                70.0,
                1280,
                800)
            .withAdvantageScopeExportDir(advantageScopeDir)
            .withAutoInstallToAdvantageScope()
            .addComponent(
                new CadComponentConfig(
                        "Elevator", new CadAssetConfig(modelsDir.resolve("elevator_carriage.glb")))
                    .withParentComponent("Chassis")
                    .withPoseSupplier(elevatorPose))
            .addComponent(
                new CadComponentConfig("Arm", new CadAssetConfig(modelsDir.resolve("arm.glb")))
                    .withParentComponent("Elevator")
                    .withPoseSupplier(
                        YamsMechanismBridge.chainParentChild(elevatorPose, armLocalPose)));

    AdvantageScopeConfigGenerator.exportStarterLayout(
        robotConfig, advantageScopeDir.resolve("examplebot-layout.json"));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    simTimeSeconds += 0.02;

    // Gentle circular drive path + articulated swerve steering & wheel spin
    double radius = 1.25;
    double omega = 0.45;
    double angle = simTimeSeconds * omega;
    double x = 3.0 + radius * Math.cos(angle);
    double y = 3.0 + radius * Math.sin(angle);
    double heading = angle + Math.PI / 2.0;
    simulatedChassisPose = new Pose2d(x, y, new Rotation2d(heading));
    simulatedSteerAngle = new Rotation2d(0.35 * Math.sin(simTimeSeconds * 0.9));
    simulatedWheelDistanceMeters += 0.02 * (radius * omega);

    // Smooth elevator + arm animation for AdvantageScope 3D verification
    simulatedElevatorHeightMeters = 0.625 + 0.425 * Math.sin(simTimeSeconds * 1.1);
    simulatedArmAngleRadians = Math.toRadians(25.0 + 40.0 * Math.sin(simTimeSeconds * 1.6));

    cadSim.simulationPeriodic();
  }

  public CadRobotMechanism getCadSim() {
    return cadSim;
  }
}
