# 3D Ballistics & AdvantageScope Export

- **`ProjectilePhysicsSim`**: Deterministic RK4 3D trajectory integration with quadratic drag, Magnus lift, spin decay, buoyancy, added mass, and 3D field obstacle OBB collisions.
- **`AdvantageScopeConfigGenerator` & `AdvantageScopePublisher`**: Generates AdvantageScope custom-robot `config.json` and publishes 3D robot/mechanism/projectile poses over NT4 + WPILib `DataLog`.
