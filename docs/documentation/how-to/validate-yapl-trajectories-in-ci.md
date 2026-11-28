---
icon: list-check
---

# How do I validate YAPL trajectories before competition?

**Goal:** Simulate a full **YAPL** autonomous drive loop in JUnit 5 and hand the realized `TrajectoryWaypoint` list straight to `PathTrajectoryValidator`.

```java
List<TrajectoryWaypoint> realized = new ArrayList<>();
Pose2d pose = startPose;
double simTime = 0.0;
double dt = 0.02;

for (int step = 0; step < 500 && !pathController.isFinished(); step++) {
    ChassisVelocities v = pathController.update(pose, dt);
    realized.add(new TrajectoryWaypoint(simTime, pose, v));
    pose = new Pose2d(
        pose.getX() + v.vx * dt,
        pose.getY() + v.vy * dt,
        pose.getRotation().plus(Rotation2d.fromRadians(v.omega * dt))
    );
    simTime += dt;
}

ValidationReport report = validator.validate(
    realized, 56.0, new Translation3d(0.0, 0.0, 0.40),
    new ValidationLimits(0.85, 1.05, 12.0)
);
assertTrue(report.feasible(), report.summary());
```

See the complete working implementation in [YAECE-RebuiltSim-Example (`FullStack2027Coordinator.java`)](https://github.com/HolyArtist31656/YAECE-RebuiltSim-Example).
