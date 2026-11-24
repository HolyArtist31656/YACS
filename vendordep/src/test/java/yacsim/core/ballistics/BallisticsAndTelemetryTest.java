// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import yacsim.core.cad.CadAssetConfig;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.model.CadRobotSimulator;
import yacsim.core.physics.BoundingBox3d;

class BallisticsAndTelemetryTest {
  private static final String UNIT_CUBE_STL =
      """
      solid cube
        facet normal 0 0 1
          outer loop
            vertex 0 0 0
            vertex 0.1 0 0
            vertex 0 0.1 0.1
          endloop
        endfacet
      endsolid cube
      """;

  @Test
  void rk4VacuumTrajectoryMatchesAnalyticalKinematics() {
    ProjectilePhysicsSim sim = new ProjectilePhysicsSim();
    ProjectileSpec spec = ProjectileSpec.vacuum("TestBall", 0.25, 0.05);
    Translation3d p0 = new Translation3d(0.0, 0.0, 1.0);
    Translation3d v0 = new Translation3d(8.0, 2.0, 6.0);
    sim.launch(spec, p0, v0, new Translation3d());

    double t = 0.5;
    for (int i = 0; i < 25; i++) {
      sim.step(0.02);
    }
    ProjectileState state = sim.getProjectiles().get(0);
    assertTrue(state.active());
    assertEquals(8.0 * t, state.positionMeters().getX(), 1e-4);
    assertEquals(2.0 * t, state.positionMeters().getY(), 1e-4);
    double expectedZ = 1.0 + 6.0 * t - 0.5 * ProjectileSpec.STANDARD_GRAVITY_MPS2 * t * t;
    assertEquals(expectedZ, state.positionMeters().getZ(), 1e-4);
  }

  @Test
  void aerodynamicDragReducesRangeAndMagnusBackspinIncreasesApex() {
    Translation3d p0 = new Translation3d(0.0, 0.0, 1.0);
    Translation3d v0 = new Translation3d(12.0, 0.0, 8.0);

    ProjectilePhysicsSim vacSim = new ProjectilePhysicsSim();
    List<Pose3d> vacArc =
        vacSim.predictArc(
            ProjectileSpec.vacuum("Vac", 0.22, 0.075), p0, v0, new Translation3d(), 0.02, 120);

    ProjectilePhysicsSim dragSim = new ProjectilePhysicsSim();
    List<Pose3d> dragArc =
        dragSim.predictArc(ProjectileSpec.frcFuel2026(), p0, v0, new Translation3d(), 0.02, 120);

    // Backspin about -Y lifts a projectile moving in +X: (-Y) x (+X) = +Z
    ProjectilePhysicsSim spinSim = new ProjectilePhysicsSim();
    List<Pose3d> spinArc =
        spinSim.predictArc(
            ProjectileSpec.frcFuel2026(), p0, v0, new Translation3d(0.0, -80.0, 0.0), 0.02, 120);

    double vacEndX = vacArc.get(vacArc.size() - 1).getX();
    double dragEndX = dragArc.get(dragArc.size() - 1).getX();
    assertTrue(dragEndX < vacEndX, "Aerodynamic drag should shorten horizontal range");

    double dragApexZ = dragArc.stream().mapToDouble(Pose3d::getZ).max().orElse(0.0);
    double spinApexZ = spinArc.stream().mapToDouble(Pose3d::getZ).max().orElse(0.0);
    assertTrue(spinApexZ > dragApexZ, "Backspin Magnus lift should increase trajectory apex");
  }

  @Test
  void projectileDetectsScoringTargetAndBlockingObstacleObb() {
    ProjectilePhysicsSim sim =
        new ProjectilePhysicsSim()
            .addObstacle(
                FieldObstacle3d.axisAlignedTarget(
                    "BlueHubTarget", new BoundingBox3d(3.8, -0.5, 1.6, 4.4, 0.5, 2.4)))
            .addObstacle(
                FieldObstacle3d.axisAlignedObstacle(
                    "MidWall", new BoundingBox3d(1.8, 2.0, 0.0, 2.2, 3.0, 2.0)));

    // Shot 1: aimed cleanly into BlueHubTarget
    sim.launch(
        ProjectileSpec.vacuum("Fuel", 0.22, 0.075),
        new Translation3d(0.0, 0.0, 1.0),
        new Translation3d(8.0, 0.0, 4.5),
        new Translation3d());
    // Shot 2: aimed straight into MidWall
    sim.launch(
        ProjectileSpec.vacuum("Fuel", 0.22, 0.075),
        new Translation3d(0.0, 2.5, 1.0),
        new Translation3d(10.0, 0.0, 0.5),
        new Translation3d());

    for (int i = 0; i < 60; i++) {
      sim.step(0.02);
    }

    assertEquals(1, sim.getScoredCount());
    ProjectileState s1 = sim.getProjectiles().get(0);
    ProjectileState s2 = sim.getProjectiles().get(1);
    assertTrue(s1.scored());
    assertEquals("BlueHubTarget", s1.hitObstacleName().orElse(""));
    assertFalse(s2.scored());
    assertEquals("MidWall", s2.hitObstacleName().orElse(""));
  }

  @Test
  void loadsFieldObstaclesFromYacsimFieldJsonManifest(@TempDir Path dir) throws Exception {
    String fieldJson =
        """
        {
          "schemaVersion": "1.0",
          "seasonName": "REBUILT_2026",
          "obstacles": [
            {
              "name": "OBSTACLE_blue_hub_base",
              "scoringTarget": false,
              "minMeters": [4.0, 3.5, 0.0],
              "maxMeters": [5.2, 4.7, 1.1]
            },
            {
              "name": "SCORING_SITE_blue_hub_opening",
              "scoringTarget": true,
              "minMeters": [4.2, 3.7, 2.0],
              "maxMeters": [5.0, 4.5, 2.4]
            }
          ]
        }
        """;
    Path p = dir.resolve("yacsim-field.json");
    Files.writeString(p, fieldJson, StandardCharsets.UTF_8);

    List<FieldObstacle3d> loaded = FieldObstacle3d.loadFieldManifest(p);
    assertEquals(2, loaded.size());
    assertFalse(loaded.get(0).scoringTarget());
    assertTrue(loaded.get(1).scoringTarget());
    assertEquals(4.2, loaded.get(1).localBounds().minX(), 1e-6);
  }

  @Test
  void predictArcProducesOrientedPosesWithoutMutatingActiveProjectiles() {
    ProjectilePhysicsSim sim = new ProjectilePhysicsSim();
    List<Pose3d> arc =
        sim.predictArc(
            ProjectileSpec.frcFuel2026(),
            new Translation3d(0.0, 0.0, 0.8),
            new Translation3d(7.0, 0.0, 5.0),
            new Translation3d(),
            0.05,
            40);
    assertTrue(arc.size() > 5);
    assertTrue(sim.getProjectiles().isEmpty());
    assertEquals(0.8, arc.get(0).getZ(), 1e-6);
  }

  @Test
  void cadRobotSimulatorSamplesDualPoseTruthAndDataLogConfigFlag(@TempDir Path dir)
      throws Exception {
    Path chassis = dir.resolve("chassis.stl");
    Files.writeString(chassis, UNIT_CUBE_STL, StandardCharsets.UTF_8);

    CadRobotConfig cfg =
        new CadRobotConfig("DualPoseBot")
            .withChassisAsset(new CadAssetConfig(chassis))
            .withDrivetrainPoseSupplier(() -> new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(10.0)))
            .withSimTruthPose2dSupplier(() -> new Pose2d(2.05, 3.02, Rotation2d.fromDegrees(11.0)))
            .withDataLogEnabled(true);

    assertTrue(cfg.isDataLogEnabled());
    try (CadRobotSimulator sim = new CadRobotSimulator(cfg, false)) {
      sim.simIterate();
      Pose3d truth = sim.getLatestSimTruthPose3d().orElseThrow();
      assertEquals(2.05, truth.getX(), 1e-6);
      assertEquals(3.02, truth.getY(), 1e-6);
      assertEquals(2.0, sim.getLatestRobotPose2d().getX(), 1e-6);
    }
  }
}
