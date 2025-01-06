// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double PERIODIC_LOOP_SEC = 0.02;

  public static RobotType ROBOT_TYPE = RobotType.COMP; // FIXME

  /* running mode of robot */
  public static Mode getRobotMode() {
    return switch (ROBOT_TYPE) {
      case COMP, DEV -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM -> Mode.SIM;
    };
  }

  /* iteration of robot */
  public static RobotType getRobotType() {
    return ROBOT_TYPE;
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /* */
  public enum RobotType {
    COMP,
    DEV,
    SIM;
  }
}
