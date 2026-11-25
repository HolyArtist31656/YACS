// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Zero-dependency deterministic 3D environment and chassis physics sensor models for WPILib 2027 /
 * SystemCore: 120 Hz wired QuestNav VIO drift ({@link yacsim.core.sensors.QuestNavSim}), 360-degree
 * 3D LiDAR clustering with static-OBB occlusion ({@link yacsim.core.sensors.Lidar3dSim}), and
 * friction-circle wheel slip ({@link yacsim.core.sensors.FrictionCircleSlipModel}).
 *
 * <p>Camera and AprilTag simulation is intentionally handled by YALL ({@code yall.sim.VisionSim})
 * rather than YACSim.
 */
package yacsim.core.sensors;
