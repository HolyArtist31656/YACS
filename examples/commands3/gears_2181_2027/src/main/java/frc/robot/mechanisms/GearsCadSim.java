// Copyright (c) FRC Team 2181 GEARS & YACSim Contributors
// Manifest- & URDF-driven YACSim v2027.4.0 3D CAD Simulator wiring for FRC Team 2181 GEARS

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.system.Filesystem;
import yacsim.commands3.CadRobotMechanism;
import yacsim.commands3.YamsCommands3Bridge;
import yacsim.core.advantagescope.AdvantageScopeConfigGenerator;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.manifest.CadRobotManifest;
import yacsim.core.manifest.CadUrdfOptions;
import yacsim.core.manifest.YacSimDiagnostic;
import yacsim.core.model.CadRobotConfig;

/**
 * Wires FRC Team 2181 GEARS to YACSim {@code v2027.4.0-alpha1} demonstrating code-side CAD repair:
 *
 * <ul>
 *   <li>Automatic moving-subtree baking pulls {@code Turret} children ({@code maxtube}, etc.) into
 *       the rotating turret mesh automatically.
 *   <li>{@link CadUrdfOptions#attachLinksToJoint} re-parents mis-mated {@code root}-attached
 *       launcher side plates and kicker back plate straight into the rotating {@code Turret} mesh.
 *   <li>Un-spins {@code tilt_bar} (keeping it baked cleanly with the chassis) and tests {@code
 *       revolute_3} ({@code part_1_15} + its baked subtree {@code part_1_16/17/18}) as the pivoting
 *       {@code IntakeArm}, while {@code left_hood} rides nested on top of the full turret.
 * </ul>
 */
public class GearsCadSim implements Mechanism {
  private static final double WHEEL_RADIUS_METERS = 0.0381; // 3-inch REV MAXSwerve wheel
  private static final double MODULE_XY_METERS = 0.3429; // 13.5-inch corner offset

  private final CadRobotMechanism cadSim;

  public GearsCadSim(
      CadRobotManifest manifest,
      Path manifestDir,
      Supplier<Pose2d> fieldPoseSupplier,
      Supplier<SwerveModulePosition[]> modulePositionsSupplier,
      IntakeArmMechanism intakeArm,
      TurretMechanism turret,
      ClimberMechanism climber,
      ShooterMechanism shooter,
      SpindexerMechanism spindexer,
      TopIntakeMechanism topIntake,
      InputMechanism inputFeeder)
      throws IOException {

    CadRobotConfig robotConfig;
    String urdfEnv = System.getenv("YACSIM_URDF_ZIP");
    if (urdfEnv != null && !urdfEnv.isBlank() && Files.exists(Path.of(urdfEnv))) {
      Path urdfPath = Path.of(urdfEnv);
      System.out.println(
          "[GEARS 2181] Loading native Onshape URDF with automatic subtree baking + attachLinksToJoint: "
              + urdfPath);
      CadUrdfOptions urdfOptions =
          CadUrdfOptions.recommended()
              .excludeMeshPatterns("MAXSwerve")
              .withJointAlias("revolute_1", "Turret")
              .withJointAlias("revolute_3", "IntakeArm") // part_1_15 + subtree part_1_16/17/18!
              .withJointAlias("slider_1_1", "Climber") // vertical prismatic Z slide
              .withJointAlias("revolute_6", "Shooter") // winch/wheel spool
              .withJointAlias("revolute_11", "Input") // hex shaft roller
              .keepOnlyJoints(
                  List.of("Turret", "revolute_8", "IntakeArm", "Climber", "Shooter", "Input"))
              .bakeUnboundLinksIntoChassis(true)
              // Re-parent mis-mated root-attached turret/launcher plates directly into Turret!
              .attachLinksToJoint(
                  "Turret",
                  "launcher_in_a_box",
                  "kicker_back_plate",
                  "2__x_1__maxtube",
                  "part_1_4");
      robotConfig =
          CadRobotConfig.fromUrdfOrZip(urdfPath, urdfOptions)
              .withDrivetrainPoseSupplier(fieldPoseSupplier)
              .withAutoInstallToAdvantageScope()
              .withAdvantageScopeExportDir(
                  Filesystem.getOperatingDirectory().toPath().resolve("advantagescope-assets"));
    } else {
      robotConfig =
          CadRobotConfig.fromManifest(manifest, manifestDir)
              .withDrivetrainPoseSupplier(fieldPoseSupplier)
              .withAutoInstallToAdvantageScope()
              .withAdvantageScopeExportDir(
                  Filesystem.getOperatingDirectory().toPath().resolve("advantagescope-assets"));
    }

    // 1. Auto-bind mechanisms (left_hood rides nested on the full rotating Turret!)
    Map<String, Object> mechanisms = new LinkedHashMap<>();
    mechanisms.put("Turret", turret.getPivot());
    mechanisms.put("IntakeArm", intakeArm.getArm());
    mechanisms.put("Climber", climber.getElevator());
    mechanisms.put("Shooter", shooter.getMotor());
    mechanisms.put("Input", inputFeeder.getMotor());

    YacSimDiagnostic.BindingReport report =
        YamsCommands3Bridge.autoBind(robotConfig, mechanisms);

    // Keep left_hood chained rigidly onto the rotating Turret even without a separate Hood motor
    if (robotConfig.getManifest().isPresent()
        && robotConfig.getManifest().get().findJoint("revolute_8").isPresent()
        && robotConfig.hasComponent("left_hood")) {
      var hoodJoint = robotConfig.getManifest().get().getJoint("revolute_8");
      robotConfig.withComponentPoseSupplier(
          "left_hood",
          YamsCommands3Bridge.chainFromManifest(
              robotConfig,
              "revolute_8",
              () ->
                  new org.wpilib.math.geometry.Pose3d(
                      hoodJoint.originMeters(), hoodJoint.originRotation())),
          false);
    }

    // 2. Overlay 4 animated MAXSwerve steer forks + spoked spinning wheels at the 4 corners
    Path forkMesh = manifestDir.resolve("swerve_fork.glb");
    Path wheelMesh = manifestDir.resolve("swerve_wheel.glb");
    if (Files.isRegularFile(forkMesh) && Files.isRegularFile(wheelMesh)) {
      List<Translation3d> corners =
          List.of(
              new Translation3d(MODULE_XY_METERS, MODULE_XY_METERS, WHEEL_RADIUS_METERS),
              new Translation3d(MODULE_XY_METERS, -MODULE_XY_METERS, WHEEL_RADIUS_METERS),
              new Translation3d(-MODULE_XY_METERS, MODULE_XY_METERS, WHEEL_RADIUS_METERS),
              new Translation3d(-MODULE_XY_METERS, -MODULE_XY_METERS, WHEEL_RADIUS_METERS));
      YamsCommands3Bridge.attachSwerveModules(
          robotConfig,
          new CadAssetConfig(forkMesh),
          new CadAssetConfig(wheelMesh),
          WHEEL_RADIUS_METERS,
          corners,
          fieldPoseSupplier,
          () -> new Rotation3d(0.0, 0.0, fieldPoseSupplier.get().getRotation().getRadians()),
          modulePositionsSupplier);
    }

    YacSimDiagnostic.printReport(robotConfig);
    System.out.println("  Auto-bound mechanisms: " + report.boundNames());

    AdvantageScopeConfigGenerator.exportStarterLayout(
        robotConfig,
        Filesystem.getOperatingDirectory()
            .toPath()
            .resolve("advantagescope-assets/gears-2181-layout.json"));

    this.cadSim = new CadRobotMechanism(this, robotConfig);
  }

  public void periodic() {
    cadSim.periodic();
  }

  public void simulationPeriodic() {
    cadSim.simulationPeriodic();
  }
}
