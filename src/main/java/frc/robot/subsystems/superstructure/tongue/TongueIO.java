package frc.robot.subsystems.superstructure.tongue;

import org.littletonrobotics.junction.AutoLog;

public interface TongueIO {
  @AutoLog
  class TongueIOInputs {
    public double angle = 0;
    public boolean pole1Detected = false;
    public boolean pole2Detected = false;
  }

  default void updateInputs(TongueIOInputsAutoLogged inputs) {}

  default void runPosition(double position) {}

  default void stop() {}
}
