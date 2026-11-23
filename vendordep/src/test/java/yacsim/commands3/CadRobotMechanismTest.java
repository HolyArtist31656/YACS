// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

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
