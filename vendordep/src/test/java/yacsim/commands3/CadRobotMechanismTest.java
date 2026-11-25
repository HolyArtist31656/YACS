// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.commands3;

import static org.junit.jupiter.api.Assertions.assertFalse;

import java.io.IOException;
import org.junit.jupiter.api.Test;
import yacsim.core.model.CadRobotConfig;
import yacsim.core.model.CadRobotSimulator;

class CadRobotMechanismTest {

  @Test
  void mechanismStepsSimulatorSafely() throws IOException {
    CadRobotConfig config = new CadRobotConfig("Commands3Bot");
    try (CadRobotMechanism mech =
        new CadRobotMechanism(null, new CadRobotSimulator(config, false))) {
      mech.simulationPeriodic();
      mech.periodic();
      assertFalse(mech.getSimulator().hasAnyCollision());
    }
  }
}
