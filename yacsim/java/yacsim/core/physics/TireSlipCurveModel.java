// Copyright (c) 2026 Nathan Moe / Yet Another CAD Simulator (YACSim) Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.physics;

import java.util.Objects;

/**
 * Simplified Pacejka Magic-Formula tire traction model for FRC swerve treads on competition
 * carpet, capturing peak static grip and post-peak kinetic slip drop-off.
 */
public final class TireSlipCurveModel {
  /** Common FRC tread presets calibrated for dry competition carpet. */
  public enum TreadPreset {
    /** Billet / nitrile high-grip swerve tread (e.g., SDS / WCP black nitrile). */
    NITRILE_CARPET(1.25, 0.92, 0.12, 10.0, 1.45),
    /** Molded neoprene tread. */
    NEOPRENE_CARPET(1.15, 0.85, 0.14, 9.0, 1.40),
    /** Blue/gray Colson Performa wheel on carpet. */
    COLSON_CARPET(1.00, 0.76, 0.15, 8.5, 1.35),
    /** Worn tread / dusty carpet conservative floor. */
    WORN_TREAD_CARPET(0.82, 0.62, 0.16, 7.5, 1.30);

    private final TireConfig config;

    TreadPreset(
        double peakMu,
        double kineticMu,
        double peakSlipRatio,
        double stiffnessB,
        double shapeC) {
      this.config = new TireConfig(peakMu, kineticMu, peakSlipRatio, stiffnessB, shapeC);
    }

    public TireConfig config() {
      return config;
    }
  }

  /**
   * Tire curve parameters.
   *
   * @param peakMu Peak static traction coefficient (&gt; 0).
   * @param kineticMu Asymptotic sliding friction coefficient in {@code (0, peakMu]}.
   * @param peakSlipRatio Longitudinal/combined slip ratio where peak traction occurs (&gt; 0).
   * @param stiffnessB Pacejka stiffness factor (&gt; 0).
   * @param shapeC Pacejka shape factor (&gt; 1).
   */
  public record TireConfig(
      double peakMu,
      double kineticMu,
      double peakSlipRatio,
      double stiffnessB,
      double shapeC) {
    public TireConfig {
      if (!Double.isFinite(peakMu) || peakMu <= 0.0) {
        throw new IllegalArgumentException("peakMu must be positive");
      }
      if (!Double.isFinite(kineticMu) || kineticMu <= 0.0 || kineticMu > peakMu) {
        throw new IllegalArgumentException("kineticMu must be in (0, peakMu]");
      }
      if (!Double.isFinite(peakSlipRatio) || peakSlipRatio <= 0.0) {
        throw new IllegalArgumentException("peakSlipRatio must be positive");
      }
      if (!Double.isFinite(stiffnessB) || stiffnessB <= 0.0) {
        throw new IllegalArgumentException("stiffnessB must be positive");
      }
      if (!Double.isFinite(shapeC) || shapeC <= 1.0) {
        throw new IllegalArgumentException("shapeC must be > 1.0");
      }
    }
  }

  private final TireConfig config;

  public TireSlipCurveModel(TreadPreset preset) {
    this(Objects.requireNonNull(preset, "preset must not be null").config());
  }

  public TireSlipCurveModel(TireConfig config) {
    this.config = Objects.requireNonNull(config, "config must not be null");
  }

  /**
   * Evaluates effective traction coefficient {@code mu} for a non-negative slip ratio {@code s}.
   * Rises smoothly to {@code peakMu} near {@code peakSlipRatio}, then decays toward {@code
   * kineticMu} under excessive wheelspin.
   */
  public double frictionCoefficient(double slipRatio) {
    if (!Double.isFinite(slipRatio)) {
      return 0.0;
    }
    double s = Math.abs(slipRatio);
    if (s <= 1e-9) {
      return 0.0;
    }
    if (s <= config.peakSlipRatio()) {
      double norm = s / config.peakSlipRatio();
      double magic =
          Math.sin(config.shapeC() * Math.atan(config.stiffnessB() * norm))
              / Math.sin(config.shapeC() * Math.atan(config.stiffnessB()));
      return Math.clamp(config.peakMu() * magic, 0.0, config.peakMu());
    }
    double excess = (s - config.peakSlipRatio()) / Math.max(0.05, 1.0 - config.peakSlipRatio());
    double decay = Math.exp(-3.0 * excess);
    return config.kineticMu() + (config.peakMu() - config.kineticMu()) * decay;
  }

  /** Maximum horizontal tire force available at the given normal load and slip ratio. */
  public double maxTractionForceNewtons(double normalLoadNewtons, double slipRatio) {
    if (!Double.isFinite(normalLoadNewtons) || normalLoadNewtons <= 0.0) {
      return 0.0;
    }
    return normalLoadNewtons * frictionCoefficient(slipRatio);
  }

  /** Maximum static peak force available at optimal slip. */
  public double peakTractionForceNewtons(double normalLoadNewtons) {
    if (!Double.isFinite(normalLoadNewtons) || normalLoadNewtons <= 0.0) {
      return 0.0;
    }
    return normalLoadNewtons * config.peakMu();
  }

  /** Clamps a requested wheel force magnitude to the available traction envelope. */
  public double clampWheelForceNewtons(
      double normalLoadNewtons, double requestedForceNewtons, double slipRatio) {
    if (!Double.isFinite(requestedForceNewtons)) {
      return 0.0;
    }
    double maxF = maxTractionForceNewtons(normalLoadNewtons, slipRatio);
    return Math.clamp(requestedForceNewtons, -maxF, maxF);
  }

  public TireConfig getConfig() {
    return config;
  }
}
