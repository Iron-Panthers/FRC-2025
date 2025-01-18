package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.GravityTypeValue;
import org.littletonrobotics.junction.AutoLog;

public interface GenericSuperstructureIO {
  @AutoLog
  class GenericSuperstructureIOInputs {
    public boolean connected = true;
    public double positionRotations = 0;
    public double velocityRotPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
    public double tempCelsius = 0;
    public boolean zeroing = false;
  }

  default void updateInputs(GenericSuperstructureIOInputs inputs) {}

  default void runPosition(double position) {}

  default void runCharacterization() {}

  default void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      GravityTypeValue gravityTypeValue) {}

  default void stop() {}

  default void setOffset() {}

  default double getPositionTargetEpsilon() {
    return 0;
  }

  default double getZeroingVoltageThreshold() {
    return Double.POSITIVE_INFINITY;
  }
}
