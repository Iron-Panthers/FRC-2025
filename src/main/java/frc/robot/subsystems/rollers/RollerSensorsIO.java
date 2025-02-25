package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
  @AutoLog
  class RollerSensorsIOInputs {
    public boolean intakeDetected = false;
  }

  default void updateInputs(RollerSensorsIOInputs inputs) {}
}
