// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.ballistics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.physics.BoundingBox3d;
import yacsim.core.physics.ObbCollisionDetector;

/**
 * Zero-dependency pure-Java 4th-order Runge-Kutta (RK4) 3D projectile physics simulator with
 * gravity, quadratic aerodynamic drag, Magnus spin lift, optional buoyancy/added-mass/spin-decay
 * corrections at {@code 0.001 s} sub-steps, floor bounce/stop, and 3D OBB collision against {@link
 * FieldObstacle3d} obstacles and scoring targets.
 *
 * <p>Designed to pair directly with YAECE's Shoot-on-the-Move (SOTM) solver, {@link
 * BallisticCalibrationFitter}, and {@link TrajectoryArcPublisher} for live AdvantageScope 3D Field
 * visualization.
 */
public final class ProjectilePhysicsSim {
  private static final int DEFAULT_MAX_ACTIVE = 32;
  private static final double FLOOR_STOP_VZ_MPS = 0.25;

  /**
   * Captured landing observation when a simulated projectile crosses a target elevation plane while
   * descending.
   *
   * @param landingPositionMeters Field-frame 3D crossing point in meters.
   * @param horizontalRangeMeters Horizontal distance traveled from launch X/Y to landing X/Y in
   *     meters.
   * @param timeOfFlightSeconds Elapsed time from launch to target-plane crossing in seconds.
   * @param entryAngleRad Positive downward entry angle below horizontal in radians ({@code
   *     atan2(-vz, hypot(vx, vy))}).
   * @param impactSpeedMps Total linear speed at crossing in m/s.
   */
  public record LandingObservation(
      Translation3d landingPositionMeters,
      double horizontalRangeMeters,
      double timeOfFlightSeconds,
      double entryAngleRad,
      double impactSpeedMps) {}

  private final List<FieldObstacle3d> obstacles = new ArrayList<>();
  private final List<ProjectileState> projectiles = new ArrayList<>();
  private int nextId = 1;
  private int maxActiveProjectiles = DEFAULT_MAX_ACTIVE;
  private double maxLifetimeSeconds = 6.0;
  private AerodynamicCorrectionConfig aeroConfig = AerodynamicCorrectionConfig.disabled();

  /** Creates an empty projectile physics simulator with baseline RK4 settings. */
  public ProjectilePhysicsSim() {}

  /**
   * Configures optional buoyancy, added-mass, spin-decay, and fine RK4 sub-stepping corrections.
   *
   * @param config Aerodynamic correction settings (non-null).
   * @return This simulator for chaining.
   */
  public ProjectilePhysicsSim withAerodynamicCorrections(AerodynamicCorrectionConfig config) {
    this.aeroConfig = Objects.requireNonNull(config, "config");
    return this;
  }

  /** @return Current {@link AerodynamicCorrectionConfig}. */
  public AerodynamicCorrectionConfig getAerodynamicCorrections() {
    return aeroConfig;
  }

  /**
   * Registers a static 3D field obstacle or scoring target volume.
   *
   * @param obstacle Obstacle or scoring target.
   * @return This simulator for chaining.
   */
  public ProjectilePhysicsSim addObstacle(FieldObstacle3d obstacle) {
    obstacles.add(Objects.requireNonNull(obstacle, "obstacle"));
    return this;
  }

  /**
   * Registers multiple field obstacles or scoring targets (for example loaded from {@link
   * FieldObstacle3d#loadFieldManifest}).
   *
   * @param list Obstacles to add.
   * @return This simulator for chaining.
   */
  public ProjectilePhysicsSim addObstacles(List<FieldObstacle3d> list) {
    Objects.requireNonNull(list, "list");
    for (FieldObstacle3d obs : list) {
      addObstacle(obs);
    }
    return this;
  }

  /**
   * Configures the maximum number of retained projectiles (oldest entries are evicted when exceeded
   * to keep SystemCore memory bounded).
   *
   * @param maxActive Positive cap (default {@code 32}).
   * @return This simulator for chaining.
   */
  public ProjectilePhysicsSim withMaxActiveProjectiles(int maxActive) {
    if (maxActive <= 0) {
      throw new IllegalArgumentException("maxActive must be positive.");
    }
    this.maxActiveProjectiles = maxActive;
    return this;
  }

  /**
   * Configures the maximum flight lifetime before an airborne projectile is retired.
   *
   * @param maxLifetimeSeconds Positive lifetime in seconds (default {@code 6.0}).
   * @return This simulator for chaining.
   */
  public ProjectilePhysicsSim withMaxLifetimeSeconds(double maxLifetimeSeconds) {
    if (maxLifetimeSeconds <= 0.0) {
      throw new IllegalArgumentException("maxLifetimeSeconds must be positive.");
    }
    this.maxLifetimeSeconds = maxLifetimeSeconds;
    return this;
  }

  /**
   * Spawns a new airborne projectile at {@code launchPositionMeters} with initial field-frame
   * velocity and optional spin vector.
   *
   * @param spec Physical projectile specification.
   * @param launchPositionMeters Launch position in field-frame meters.
   * @param initialVelocityMps Initial field-frame velocity in m/s (include robot velocity for
   *     Shoot-on-the-Move!).
   * @param spinRadPerSec Angular spin vector in rad/s (pass {@code new Translation3d()} for zero
   *     spin).
   * @return Spawned {@link ProjectileState}.
   */
  public ProjectileState launch(
      ProjectileSpec spec,
      Translation3d launchPositionMeters,
      Translation3d initialVelocityMps,
      Translation3d spinRadPerSec) {
    Objects.requireNonNull(spec, "spec");
    Objects.requireNonNull(launchPositionMeters, "launchPositionMeters");
    Objects.requireNonNull(initialVelocityMps, "initialVelocityMps");
    Translation3d spin = spinRadPerSec == null ? new Translation3d() : spinRadPerSec;
    ProjectileState state =
        new ProjectileState(
            nextId++,
            spec,
            launchPositionMeters,
            initialVelocityMps,
            spin,
            0.0,
            true,
            false,
            Optional.empty());
    projectiles.add(state);
    while (projectiles.size() > maxActiveProjectiles) {
      projectiles.remove(0);
    }
    return state;
  }

  /**
   * Advances all active projectiles by {@code dtSeconds} using 4th-order Runge-Kutta (RK4)
   * integration and evaluates floor and 3D OBB field obstacle/target collisions.
   *
   * @param dtSeconds Timestep in seconds (for example {@code 0.02}).
   */
  public void step(double dtSeconds) {
    if (dtSeconds <= 0.0 || !Double.isFinite(dtSeconds)) {
      return;
    }
    for (int i = 0; i < projectiles.size(); i++) {
      ProjectileState cur = projectiles.get(i);
      if (!cur.active()) {
        continue;
      }
      projectiles.set(i, integrateWithSubsteps(cur, dtSeconds));
    }
  }

  /**
   * Computes a predicted trajectory arc (as field-frame {@link Pose3d} samples) for live
   * AdvantageScope aim-arc visualization without mutating active projectiles.
   *
   * @param spec Projectile specification.
   * @param launchPositionMeters Launch position in meters.
   * @param initialVelocityMps Initial field-frame velocity in m/s.
   * @param spinRadPerSec Spin vector in rad/s.
   * @param dtSeconds Sample interval in seconds (for example {@code 0.04}).
   * @param maxSteps Maximum number of trajectory points.
   * @return Immutable list of {@link Pose3d} points along the predicted arc.
   */
  public List<Pose3d> predictArc(
      ProjectileSpec spec,
      Translation3d launchPositionMeters,
      Translation3d initialVelocityMps,
      Translation3d spinRadPerSec,
      double dtSeconds,
      int maxSteps) {
    Objects.requireNonNull(spec, "spec");
    Objects.requireNonNull(launchPositionMeters, "launchPositionMeters");
    Objects.requireNonNull(initialVelocityMps, "initialVelocityMps");
    if (dtSeconds <= 0.0 || maxSteps <= 0) {
      return List.of();
    }
    Translation3d spin = spinRadPerSec == null ? new Translation3d() : spinRadPerSec;
    ProjectileState cur =
        new ProjectileState(
            0,
            spec,
            launchPositionMeters,
            initialVelocityMps,
            spin,
            0.0,
            true,
            false,
            Optional.empty());
    List<Pose3d> arc = new ArrayList<>(maxSteps + 1);
    arc.add(cur.toPose3d());
    for (int step = 0; step < maxSteps && cur.active(); step++) {
      cur = integrateWithSubsteps(cur, dtSeconds);
      arc.add(cur.toPose3d());
    }
    return List.copyOf(arc);
  }

  /**
   * Simulates a single projectile without mutating simulator state and captures the exact
   * descending crossing through {@code targetHeightMeters} via linear sub-step interpolation.
   *
   * @param spec Projectile specification.
   * @param launchPositionMeters Launch position in meters.
   * @param initialVelocityMps Initial field-frame velocity in m/s.
   * @param spinRadPerSec Initial angular spin vector in rad/s.
   * @param targetHeightMeters Target plane elevation Z in meters.
   * @param dtSeconds Integration step size in seconds (for example {@code 0.001}).
   * @param maxSeconds Maximum flight duration to simulate in seconds.
   * @return {@link LandingObservation} if the projectile descends through {@code targetHeightMeters},
   *     otherwise empty.
   */
  public Optional<LandingObservation> simulateToLanding(
      ProjectileSpec spec,
      Translation3d launchPositionMeters,
      Translation3d initialVelocityMps,
      Translation3d spinRadPerSec,
      double targetHeightMeters,
      double dtSeconds,
      double maxSeconds) {
    Objects.requireNonNull(spec, "spec");
    Objects.requireNonNull(launchPositionMeters, "launchPositionMeters");
    Objects.requireNonNull(initialVelocityMps, "initialVelocityMps");
    if (dtSeconds <= 0.0 || maxSeconds <= 0.0 || !Double.isFinite(targetHeightMeters)) {
      return Optional.empty();
    }
    Translation3d spin = spinRadPerSec == null ? new Translation3d() : spinRadPerSec;
    double px = launchPositionMeters.getX();
    double py = launchPositionMeters.getY();
    double pz = launchPositionMeters.getZ();
    double vx = initialVelocityMps.getX();
    double vy = initialVelocityMps.getY();
    double vz = initialVelocityMps.getZ();
    double wx = spin.getX();
    double wy = spin.getY();
    double wz = spin.getZ();

    double t = 0.0;
    int maxSteps = (int) Math.ceil(maxSeconds / dtSeconds);
    double floorZ = Math.min(spec.radiusMeters(), targetHeightMeters - 0.05);

    for (int i = 0; i < maxSteps; i++) {
      double prevX = px;
      double prevY = py;
      double prevZ = pz;
      double prevVx = vx;
      double prevVy = vy;
      double prevVz = vz;
      double prevT = t;

      double[] a1 = acceleration(spec, aeroConfig, vx, vy, vz, wx, wy, wz);
      double vx2 = vx + 0.5 * dtSeconds * a1[0];
      double vy2 = vy + 0.5 * dtSeconds * a1[1];
      double vz2 = vz + 0.5 * dtSeconds * a1[2];

      double[] a2 = acceleration(spec, aeroConfig, vx2, vy2, vz2, wx, wy, wz);
      double vx3 = vx + 0.5 * dtSeconds * a2[0];
      double vy3 = vy + 0.5 * dtSeconds * a2[1];
      double vz3 = vz + 0.5 * dtSeconds * a2[2];

      double[] a3 = acceleration(spec, aeroConfig, vx3, vy3, vz3, wx, wy, wz);
      double vx4 = vx + dtSeconds * a3[0];
      double vy4 = vy + dtSeconds * a3[1];
      double vz4 = vz + dtSeconds * a3[2];

      double[] a4 = acceleration(spec, aeroConfig, vx4, vy4, vz4, wx, wy, wz);

      px += (dtSeconds / 6.0) * (vx + 2.0 * vx2 + 2.0 * vx3 + vx4);
      py += (dtSeconds / 6.0) * (vy + 2.0 * vy2 + 2.0 * vy3 + vy4);
      pz += (dtSeconds / 6.0) * (vz + 2.0 * vz2 + 2.0 * vz3 + vz4);

      vx += (dtSeconds / 6.0) * (a1[0] + 2.0 * a2[0] + 2.0 * a3[0] + a4[0]);
      vy += (dtSeconds / 6.0) * (a1[1] + 2.0 * a2[1] + 2.0 * a3[1] + a4[1]);
      vz += (dtSeconds / 6.0) * (a1[2] + 2.0 * a2[2] + 2.0 * a3[2] + a4[2]);

      if (aeroConfig.spinDecayRatePerSec() > 0.0) {
        double decay = Math.exp(-aeroConfig.spinDecayRatePerSec() * dtSeconds);
        wx *= decay;
        wy *= decay;
        wz *= decay;
      }
      t += dtSeconds;

      // Detect descending crossing of targetHeightMeters
      if (prevZ >= targetHeightMeters && pz <= targetHeightMeters && (vz < 0.0 || prevVz < 0.0)) {
        double dz = prevZ - pz;
        double alpha = dz > 1e-9 ? Math.max(0.0, Math.min(1.0, (prevZ - targetHeightMeters) / dz)) : 1.0;
        double cx = prevX + alpha * (px - prevX);
        double cy = prevY + alpha * (py - prevY);
        double cvx = prevVx + alpha * (vx - prevVx);
        double cvy = prevVy + alpha * (vy - prevVy);
        double cvz = prevVz + alpha * (vz - prevVz);
        double ct = prevT + alpha * dtSeconds;
        double range = Math.hypot(cx - launchPositionMeters.getX(), cy - launchPositionMeters.getY());
        double horizSpeed = Math.hypot(cvx, cvy);
        double entryAngle = Math.atan2(-cvz, Math.max(1e-9, horizSpeed));
        double impactSpeed = Math.hypot(horizSpeed, cvz);
        return Optional.of(
            new LandingObservation(
                new Translation3d(cx, cy, targetHeightMeters),
                range,
                ct,
                entryAngle,
                impactSpeed));
      }

      if (pz < floorZ && vz <= 0.0) {
        return Optional.empty();
      }
    }
    return Optional.empty();
  }

  /** @return Immutable snapshot of all tracked projectiles (active and recently finished). */
  public List<ProjectileState> getProjectiles() {
    return List.copyOf(projectiles);
  }

  /** @return Field-frame {@link Pose3d} array of all currently active projectiles for NT4/DataLog. */
  public Pose3d[] getActiveProjectilePoses() {
    List<Pose3d> active = new ArrayList<>();
    for (ProjectileState p : projectiles) {
      if (p.active()) {
        active.add(p.toPose3d());
      }
    }
    return active.toArray(new Pose3d[0]);
  }

  /** @return Count of projectiles that have intersected a scoring target. */
  public int getScoredCount() {
    int count = 0;
    for (ProjectileState p : projectiles) {
      if (p.scored()) {
        count++;
      }
    }
    return count;
  }

  /** Clears all tracked projectiles. */
  public void clearProjectiles() {
    projectiles.clear();
  }

  private ProjectileState integrateWithSubsteps(ProjectileState s, double dt) {
    double maxSub = aeroConfig.maxInternalSubstepSeconds();
    if (maxSub <= 0.0 || dt <= maxSub) {
      return integrateOne(s, dt);
    }
    int steps = Math.max(1, (int) Math.ceil(dt / maxSub));
    double h = dt / steps;
    ProjectileState cur = s;
    for (int i = 0; i < steps && cur.active(); i++) {
      cur = integrateOne(cur, h);
    }
    return cur;
  }

  private ProjectileState integrateOne(ProjectileState s, double dt) {
    ProjectileSpec spec = s.spec();
    double px = s.positionMeters().getX();
    double py = s.positionMeters().getY();
    double pz = s.positionMeters().getZ();
    double vx = s.velocityMps().getX();
    double vy = s.velocityMps().getY();
    double vz = s.velocityMps().getZ();
    double wx = s.spinRadPerSec().getX();
    double wy = s.spinRadPerSec().getY();
    double wz = s.spinRadPerSec().getZ();

    double[] a1 = acceleration(spec, aeroConfig, vx, vy, vz, wx, wy, wz);
    double vx2 = vx + 0.5 * dt * a1[0];
    double vy2 = vy + 0.5 * dt * a1[1];
    double vz2 = vz + 0.5 * dt * a1[2];

    double[] a2 = acceleration(spec, aeroConfig, vx2, vy2, vz2, wx, wy, wz);
    double vx3 = vx + 0.5 * dt * a2[0];
    double vy3 = vy + 0.5 * dt * a2[1];
    double vz3 = vz + 0.5 * dt * a2[2];

    double[] a3 = acceleration(spec, aeroConfig, vx3, vy3, vz3, wx, wy, wz);
    double vx4 = vx + dt * a3[0];
    double vy4 = vy + dt * a3[1];
    double vz4 = vz + dt * a3[2];

    double[] a4 = acceleration(spec, aeroConfig, vx4, vy4, vz4, wx, wy, wz);

    double nx = px + (dt / 6.0) * (vx + 2.0 * vx2 + 2.0 * vx3 + vx4);
    double ny = py + (dt / 6.0) * (vy + 2.0 * vy2 + 2.0 * vy3 + vy4);
    double nz = pz + (dt / 6.0) * (vz + 2.0 * vz2 + 2.0 * vz3 + vz4);

    double nvx = vx + (dt / 6.0) * (a1[0] + 2.0 * a2[0] + 2.0 * a3[0] + a4[0]);
    double nvy = vy + (dt / 6.0) * (a1[1] + 2.0 * a2[1] + 2.0 * a3[1] + a4[1]);
    double nvz = vz + (dt / 6.0) * (a1[2] + 2.0 * a2[2] + 2.0 * a3[2] + a4[2]);

    Translation3d nextSpin = s.spinRadPerSec();
    if (aeroConfig.spinDecayRatePerSec() > 0.0) {
      double decay = Math.exp(-aeroConfig.spinDecayRatePerSec() * dt);
      nextSpin = new Translation3d(wx * decay, wy * decay, wz * decay);
    }

    double nextAge = s.ageSeconds() + dt;
    boolean active = nextAge < maxLifetimeSeconds;
    boolean scored = false;
    Optional<String> hitName = Optional.empty();

    // Check 3D OBB obstacles & scoring targets
    double r = spec.radiusMeters();
    if (!obstacles.isEmpty()) {
      BoundingBox3d projBox = new BoundingBox3d(-r, -r, -r, r, r, r);
      Pose3d projPose = new Pose3d(new Translation3d(nx, ny, nz), new Rotation3d());
      for (FieldObstacle3d obs : obstacles) {
        if (ObbCollisionDetector.testObbCollision(projBox, projPose, obs.localBounds(), obs.pose())
            .isPresent()) {
          active = false;
          scored = obs.scoringTarget();
          hitName = Optional.of(obs.name());
          break;
        }
      }
    }

    // Floor contact at z = radiusMeters
    if (active && nz <= r) {
      nz = r;
      if (spec.restitution() > 0.0 && Math.abs(nvz) > FLOOR_STOP_VZ_MPS) {
        nvz = Math.abs(nvz) * spec.restitution();
        nvx *= 0.85;
        nvy *= 0.85;
      } else {
        nvz = 0.0;
        active = false;
        hitName = Optional.of("Floor");
      }
    }

    return new ProjectileState(
        s.id(),
        spec,
        new Translation3d(nx, ny, nz),
        new Translation3d(nvx, nvy, nvz),
        nextSpin,
        nextAge,
        active,
        scored,
        hitName);
  }

  private static double[] acceleration(
      ProjectileSpec spec,
      AerodynamicCorrectionConfig aero,
      double vx,
      double vy,
      double vz,
      double wx,
      double wy,
      double wz) {
    double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    double invEffMass = 1.0 / aero.effectiveMassKg(spec);
    double ax = 0.0;
    double ay = 0.0;
    // Net vertical force: downward weight (-m*g) + optional upward air buoyancy (+rho*V*g)
    double az = (-spec.massKg() * spec.gravityMps2() + aero.buoyancyForceNewtons(spec)) * invEffMass;

    if (speed > 1e-6 && spec.airDensityKgPerM3() > 0.0) {
      double area = spec.crossSectionAreaM2();
      // Quadratic aerodynamic drag: F_d = -0.5 * rho * Cd * A * |v| * v
      if (spec.dragCoefficient() > 0.0) {
        double dragFactor =
            -0.5 * spec.airDensityKgPerM3() * spec.dragCoefficient() * area * speed * invEffMass;
        ax += dragFactor * vx;
        ay += dragFactor * vy;
        az += dragFactor * vz;
      }
      // Magnus spin lift: F_m = 0.5 * rho * Cm * A * r * (omega x v)
      double spinMag = Math.sqrt(wx * wx + wy * wy + wz * wz);
      if (spec.magnusLiftCoefficient() > 0.0 && spinMag > 1e-6) {
        double cx = wy * vz - wz * vy;
        double cy = wz * vx - wx * vz;
        double cz = wx * vy - wy * vx;
        double magnusFactor =
            0.5
                * spec.airDensityKgPerM3()
                * spec.magnusLiftCoefficient()
                * area
                * spec.radiusMeters()
                * invEffMass;
        ax += magnusFactor * cx;
        ay += magnusFactor * cy;
        az += magnusFactor * cz;
      }
    }
    return new double[] {ax, ay, az};
  }
}
