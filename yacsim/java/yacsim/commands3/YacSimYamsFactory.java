// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.commands3;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.manifest.CadJointManifest;
import yacsim.core.manifest.CadRobotManifest;
import yacsim.core.manifest.CadUrdfOptions;
import yacsim.core.manifest.YacSimDiagnostic;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.model.CadRobotSimulator;
import yams.commands3.mechanisms.Arm;
import yams.commands3.mechanisms.Elevator;
import yams.commands3.mechanisms.FlyWheel;
import yams.commands3.mechanisms.Pivot;
import yams.commands3.swerve.SwerveDrive;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Seamless pass-through factory bridging YAMS 2027 mechanism configs directly to YACSim 3D CAD
 * simulation while remaining zero-overhead and optional on real robots.
 *
 * <p>On a real robot ({@link RobotBase#isReal()}), {@link #fromUrdfOrZip(Path)} and {@link
 * #fromManifest(Path)} skip all CAD disk I/O and GLB parsing, and {@link #createArm}, {@link
 * #createElevator}, {@link #createPivot}, and {@link #createFlyWheel} immediately construct and
 * return the standard YAMS mechanism with zero simulation overhead.
 *
 * <p>In simulation ({@link RobotBase#isSimulation()}), each {@code create*()} call constructs the
 * YAMS mechanism, automatically binds it to its matching CAD joint/link (including nested URDF
 * parent-child kinematic chains), registers its {@code simIterate()} and {@code updateTelemetry()}
 * hooks, and returns the plain YAMS mechanism.
 */
public final class YacSimYamsFactory implements AutoCloseable {
  private final CadRobotConfig cadConfig;
  private final BooleanSupplier isSimulationSupplier;
  private boolean publishOnRealRobot = false;
  private final List<Runnable> simIterateHooks = new ArrayList<>();
  private final List<Runnable> telemetryHooks = new ArrayList<>();
  private final List<String> boundSummary = new ArrayList<>();
  private final List<String> unmatchedSummary = new ArrayList<>();
  private CadRobotSimulator simulator = null;

  private YacSimYamsFactory(CadRobotConfig cadConfig, BooleanSupplier isSimulationSupplier) {
    this.cadConfig = cadConfig;
    this.isSimulationSupplier = Objects.requireNonNull(isSimulationSupplier, "isSimulationSupplier");
  }

  /**
   * Creates a no-op pass-through factory that simply constructs and returns YAMS mechanisms with no
   * CAD loading or simulation binding.
   *
   * @return Pass-through {@link YacSimYamsFactory}.
   */
  public static YacSimYamsFactory passthrough() {
    return new YacSimYamsFactory(null, () -> false);
  }

  /**
   * Creates a factory backed by an already-loaded {@link CadRobotConfig} (using {@link
   * RobotBase#isSimulation()} to gate simulation binding).
   *
   * @param cadConfig Loaded {@link CadRobotConfig}, or {@code null} for pass-through mode.
   * @return Configured {@link YacSimYamsFactory}.
   */
  public static YacSimYamsFactory of(CadRobotConfig cadConfig) {
    return new YacSimYamsFactory(cadConfig, RobotBase::isSimulation);
  }

  /**
   * Creates a factory backed by an already-loaded {@link CadRobotConfig} with an explicit
   * simulation-mode predicate (useful for deterministic unit tests).
   *
   * @param cadConfig Loaded {@link CadRobotConfig}, or {@code null} for pass-through mode.
   * @param isSimulationSupplier Predicate returning {@code true} in simulation mode.
   * @return Configured {@link YacSimYamsFactory}.
   */
  public static YacSimYamsFactory of(
      CadRobotConfig cadConfig, BooleanSupplier isSimulationSupplier) {
    return new YacSimYamsFactory(cadConfig, isSimulationSupplier);
  }

  /**
   * Loads a native Onshape URDF {@code .zip}, {@code .urdf}, or extracted folder using {@link
   * CadUrdfOptions#recommended()} when running in simulation; on a real robot ({@link
   * RobotBase#isReal()}), skips all disk I/O and returns a zero-overhead pass-through factory.
   *
   * @param urdfOrZipPath Path to URDF {@code .zip}, {@code .urdf}, or directory.
   * @return Configured {@link YacSimYamsFactory}.
   * @throws IOException If loading the URDF fails in simulation.
   */
  public static YacSimYamsFactory fromUrdfOrZip(Path urdfOrZipPath) throws IOException {
    return fromUrdfOrZip(urdfOrZipPath, CadUrdfOptions.recommended());
  }

  /**
   * Loads a native Onshape URDF {@code .zip}, {@code .urdf}, or extracted folder with explicit
   * {@link CadUrdfOptions} in simulation; on a real robot, skips all CAD I/O and returns a
   * zero-overhead pass-through factory.
   *
   * @param urdfOrZipPath Path to URDF {@code .zip}, {@code .urdf}, or directory.
   * @param options Filtering, aliasing, and baking options.
   * @return Configured {@link YacSimYamsFactory}.
   * @throws IOException If loading the URDF fails in simulation.
   */
  public static YacSimYamsFactory fromUrdfOrZip(Path urdfOrZipPath, CadUrdfOptions options)
      throws IOException {
    Objects.requireNonNull(urdfOrZipPath, "urdfOrZipPath");
    Objects.requireNonNull(options, "options");
    if (RobotBase.isReal()) {
      return passthrough();
    }
    return new YacSimYamsFactory(
        CadRobotConfig.fromUrdfOrZip(urdfOrZipPath, options), RobotBase::isSimulation);
  }

  /**
   * Loads a {@code yacsim-robot.json} manifest in simulation; on a real robot, skips all CAD I/O
   * and returns a zero-overhead pass-through factory.
   *
   * @param manifestPath Path to {@code yacsim-robot.json}.
   * @return Configured {@link YacSimYamsFactory}.
   * @throws IOException If loading the manifest fails in simulation.
   */
  public static YacSimYamsFactory fromManifest(Path manifestPath) throws IOException {
    Objects.requireNonNull(manifestPath, "manifestPath");
    if (RobotBase.isReal()) {
      return passthrough();
    }
    return new YacSimYamsFactory(
        CadRobotConfig.fromManifest(manifestPath), RobotBase::isSimulation);
  }

  /**
   * Allows lightweight pose binding and telemetry publishing even on a real robot (when constructed
   * via {@link #of(CadRobotConfig)}). Default is {@code false} (pure zero-overhead pass-through on
   * real robots).
   *
   * @param enabled {@code true} to keep binding active on real robots.
   * @return This factory for chaining.
   */
  public YacSimYamsFactory withPublishOnRealRobot(boolean enabled) {
    this.publishOnRealRobot = enabled;
    return this;
  }

  /** @return {@code true} if CAD binding and simulation hooks are active in the current mode. */
  public boolean isSimulationActive() {
    return cadConfig != null && (isSimulationSupplier.getAsBoolean() || publishOnRealRobot);
  }

  /**
   * Constructs a YAMS {@link Arm} from {@code config} and {@code motor}. On a real robot, returns
   * {@code new Arm(config, motor)} immediately; in simulation, also binds it to the matching CAD
   * joint (with automatic nested parent-child chaining) and registers its sim/telemetry hooks.
   *
   * @param jointName CAD joint name (or alias configured in {@link CadUrdfOptions}).
   * @param config YAMS {@link ArmConfig}.
   * @param motor YAMS {@link SmartMotorController}.
   * @return Constructed YAMS {@link Arm}.
   */
  public Arm createArm(String jointName, ArmConfig config, SmartMotorController motor) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(motor, "motor");
    Arm arm = new Arm(config, motor);
    if (!isSimulationActive()) {
      return arm;
    }
    bindMechanismIfPresent(
        jointName,
        joint -> YamsCommands3Bridge.bindFromManifest(cadConfig, joint.name(), arm),
        arm::simIterate,
        arm::updateTelemetry);
    return arm;
  }

  /**
   * Constructs a YAMS {@link Elevator} from {@code config} and {@code motor}. On a real robot,
   * returns {@code new Elevator(config, motor)} immediately; in simulation, also binds it to its
   * matching CAD joint and registers its sim/telemetry hooks.
   *
   * @param jointName CAD joint name (or alias configured in {@link CadUrdfOptions}).
   * @param config YAMS {@link ElevatorConfig}.
   * @param motor YAMS {@link SmartMotorController}.
   * @return Constructed YAMS {@link Elevator}.
   */
  public Elevator createElevator(
      String jointName, ElevatorConfig config, SmartMotorController motor) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(motor, "motor");
    Elevator elevator = new Elevator(config, motor);
    if (!isSimulationActive()) {
      return elevator;
    }
    bindMechanismIfPresent(
        jointName,
        joint -> YamsCommands3Bridge.bindFromManifest(cadConfig, joint.name(), elevator),
        elevator::simIterate,
        elevator::updateTelemetry);
    return elevator;
  }

  /**
   * Constructs a YAMS {@link Pivot} from {@code config} and {@code motor}. On a real robot,
   * returns {@code new Pivot(config, motor)} immediately; in simulation, also binds it to its
   * matching CAD joint and registers its sim/telemetry hooks.
   *
   * @param jointName CAD joint name (or alias configured in {@link CadUrdfOptions}).
   * @param config YAMS {@link PivotConfig}.
   * @param motor YAMS {@link SmartMotorController}.
   * @return Constructed YAMS {@link Pivot}.
   */
  public Pivot createPivot(String jointName, PivotConfig config, SmartMotorController motor) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(motor, "motor");
    Pivot pivot = new Pivot(config, motor);
    if (!isSimulationActive()) {
      return pivot;
    }
    bindMechanismIfPresent(
        jointName,
        joint -> YamsCommands3Bridge.bindFromManifest(cadConfig, joint.name(), pivot),
        pivot::simIterate,
        pivot::updateTelemetry);
    return pivot;
  }

  /**
   * Constructs a YAMS {@link FlyWheel} from {@code config} and {@code motor}. On a real robot,
   * returns {@code new FlyWheel(config, motor)} immediately; in simulation, also binds its roller
   * motor telemetry and spinning CAD link (keeping roller OBB collisions disabled) and registers
   * its sim/telemetry hooks.
   *
   * @param jointName CAD joint name (or alias configured in {@link CadUrdfOptions}).
   * @param config YAMS {@link FlyWheelConfig}.
   * @param motor YAMS {@link SmartMotorController}.
   * @return Constructed YAMS {@link FlyWheel}.
   */
  public FlyWheel createFlyWheel(
      String jointName, FlyWheelConfig config, SmartMotorController motor) {
    Objects.requireNonNull(config, "config");
    Objects.requireNonNull(motor, "motor");
    FlyWheel flywheel = new FlyWheel(config, motor);
    if (!isSimulationActive()) {
      return flywheel;
    }
    bindMechanismIfPresent(
        jointName,
        joint -> YamsCommands3Bridge.bindFromManifest(cadConfig, joint.name(), motor),
        flywheel::simIterate,
        flywheel::updateTelemetry);
    return flywheel;
  }

  /**
   * Attaches animated swerve steer forks and spinning wheels from a YAMS {@link SwerveDrive} in
   * simulation; on a real robot, returns {@code drive} immediately unchanged.
   *
   * @param drive YAMS {@link SwerveDrive}.
   * @param steerForkAsset Optional shared steer-fork CAD asset (may be {@code null}).
   * @param wheelAsset Optional shared wheel CAD asset (may be {@code null}).
   * @param wheelRadiusMeters Positive wheel radius in meters.
   * @param moduleLocations Module mounting locations in chassis frame.
   * @return The same {@code drive} instance for fluent pass-through.
   */
  public SwerveDrive attachSwerve(
      SwerveDrive drive,
      CadAssetConfig steerForkAsset,
      CadAssetConfig wheelAsset,
      double wheelRadiusMeters,
      List<Translation3d> moduleLocations) {
    Objects.requireNonNull(drive, "drive");
    if (isSimulationActive()) {
      YamsCommands3Bridge.attachSwerveModules(
          cadConfig, drive, steerForkAsset, wheelAsset, wheelRadiusMeters, moduleLocations);
    }
    return drive;
  }

  /**
   * Generic pass-through registration for any custom mechanism or supplier-based component. On a
   * real robot, returns {@code mechanism} immediately without touching CAD or hooks; in simulation,
   * chains and binds {@code localPoseSupplier} to the matching CAD joint or component and registers
   * optional sim/telemetry callbacks.
   *
   * @param <T> Mechanism type to pass through.
   * @param jointOrLinkName CAD joint name (preferred) or link name.
   * @param mechanism Mechanism instance to return unchanged.
   * @param localPoseSupplier Local joint/component {@link Pose3d} supplier.
   * @param simIterateHook Optional simulation step callback (may be {@code null}).
   * @param telemetryHook Optional telemetry update callback (may be {@code null}).
   * @return The same {@code mechanism} instance unchanged.
   */
  public <T> T registerComponent(
      String jointOrLinkName,
      T mechanism,
      Supplier<Pose3d> localPoseSupplier,
      Runnable simIterateHook,
      Runnable telemetryHook) {
    Objects.requireNonNull(localPoseSupplier, "localPoseSupplier");
    if (!isSimulationActive()) {
      return mechanism;
    }
    Optional<CadRobotManifest> optManifest = cadConfig.getManifest();
    Optional<CadJointManifest> optJoint =
        optManifest.flatMap(m -> m.findJoint(jointOrLinkName));
    if (optJoint.isPresent() && cadConfig.hasComponent(optJoint.get().childLink())) {
      CadJointManifest joint = optJoint.get();
      Supplier<Pose3d> chained =
          YamsCommands3Bridge.chainFromManifest(cadConfig, joint.name(), localPoseSupplier);
      cadConfig.withComponentPoseSupplier(joint.childLink(), chained);
      boundSummary.add(jointOrLinkName + " -> " + joint.name());
    } else if (cadConfig.hasComponent(jointOrLinkName)) {
      cadConfig.withComponentPoseSupplier(jointOrLinkName, localPoseSupplier);
      boundSummary.add(jointOrLinkName);
    } else {
      unmatchedSummary.add(jointOrLinkName);
    }
    if (simIterateHook != null) {
      simIterateHooks.add(simIterateHook);
    }
    if (telemetryHook != null) {
      telemetryHooks.add(telemetryHook);
    }
    return mechanism;
  }

  /**
   * Builds and starts the underlying {@link CadRobotSimulator} (with NT4 publishing enabled) when
   * simulation is active; returns {@link Optional#empty()} on a real robot.
   *
   * @return Started {@link CadRobotSimulator}, or empty on a real robot.
   * @throws IOException If parsing meshes or exporting AdvantageScope assets fails.
   */
  public Optional<CadRobotSimulator> startSimulator() throws IOException {
    return startSimulator(true);
  }

  /**
   * Builds and starts the underlying {@link CadRobotSimulator} when simulation is active; returns
   * {@link Optional#empty()} on a real robot.
   *
   * @param enableNetworkTables {@code true} to publish NT4 telemetry, {@code false} for headless
   *     tests.
   * @return Started {@link CadRobotSimulator}, or empty on a real robot.
   * @throws IOException If parsing meshes or exporting AdvantageScope assets fails.
   */
  public Optional<CadRobotSimulator> startSimulator(boolean enableNetworkTables)
      throws IOException {
    if (!isSimulationActive()) {
      return Optional.empty();
    }
    if (this.simulator == null) {
      this.simulator = new CadRobotSimulator(cadConfig, enableNetworkTables);
    }
    return Optional.of(this.simulator);
  }

  /**
   * Steps all mechanisms created through this factory and then steps {@link CadRobotSimulator}.
   * No-op on a real robot.
   */
  public void simIterate() {
    if (!isSimulationActive()) {
      return;
    }
    for (Runnable hook : simIterateHooks) {
      hook.run();
    }
    if (simulator != null) {
      simulator.simIterate();
    }
  }

  /**
   * Updates telemetry for all mechanisms created through this factory and publishes the latest
   * {@link CadRobotSimulator} snapshot. No-op on a real robot unless {@link
   * #withPublishOnRealRobot(boolean)} is enabled.
   */
  public void updateTelemetry() {
    if (!isSimulationActive()) {
      return;
    }
    for (Runnable hook : telemetryHooks) {
      hook.run();
    }
    if (simulator != null) {
      simulator.updateTelemetry();
    }
  }

  /** Prints the startup {@link YacSimDiagnostic} report if simulation is active. */
  public void printDiagnosticReport() {
    if (isSimulationActive()) {
      YacSimDiagnostic.printReport(cadConfig);
    }
  }

  /** @return Underlying {@link CadRobotConfig}, if loaded. */
  public Optional<CadRobotConfig> getCadConfig() {
    return Optional.ofNullable(cadConfig);
  }

  /** @return Underlying {@link CadRobotManifest}, if loaded. */
  public Optional<CadRobotManifest> getManifest() {
    return cadConfig != null ? cadConfig.getManifest() : Optional.empty();
  }

  /** @return Started {@link CadRobotSimulator}, if {@link #startSimulator()} was called. */
  public Optional<CadRobotSimulator> getSimulator() {
    return Optional.ofNullable(simulator);
  }

  /** @return Unmodifiable list of mechanisms bound through this factory. */
  public List<String> getBoundMechanisms() {
    return List.copyOf(boundSummary);
  }

  /** @return Unmodifiable list of mechanism names that did not match a CAD joint/link. */
  public List<String> getUnmatchedMechanisms() {
    return List.copyOf(unmatchedSummary);
  }

  @Override
  public void close() {
    if (simulator != null) {
      simulator.close();
      simulator = null;
    }
  }

  private void bindMechanismIfPresent(
      String jointName,
      java.util.function.Consumer<CadJointManifest> binder,
      Runnable simHook,
      Runnable telemetryHook) {
    Optional<CadJointManifest> optJoint =
        cadConfig.getManifest().flatMap(m -> m.findJoint(jointName));
    if (optJoint.isPresent() && cadConfig.hasComponent(optJoint.get().childLink())) {
      CadJointManifest joint = optJoint.get();
      binder.accept(joint);
      boundSummary.add(jointName + " -> " + joint.name());
    } else {
      unmatchedSummary.add(jointName);
    }
    if (simHook != null) {
      simIterateHooks.add(simHook);
    }
    if (telemetryHook != null) {
      telemetryHooks.add(telemetryHook);
    }
  }
}
