package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollersIO {
  @AutoLog
  class GenericRollersIOInputs {
    public boolean connected = true;
    public double positionRads = 0;
    public double velocityRadsPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
  }

  default void updateInputs(GenericRollersIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void stop() {}
}
