package frc.robot.subsystems.superstructure.pivot;

import static frc.robot.subsystems.superstructure.pivot.PivotConstants.*;
import static frc.robot.subsystems.superstructure.pivot.PivotConstants.POSITION_TARGET_EPSILON;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class PivotIOTalonFX extends GenericSuperstructureIOTalonFX implements PivotIO {

  public PivotIOTalonFX() {
    super(
        PIVOT_CONFIG.motorID(),
        Optional.empty(),
        INVERT_MOTOR,
        Optional.empty(),
        SUPPLY_CURRENT_LIMIT,
        PIVOT_CONFIG.canCoderID(),
        PIVOT_CONFIG.canCoderOffset(),
        PIVOT_CONFIG.reduction(),
        UPPER_EXTENSION_LIMIT,
        LOWER_EXTENSION_LIMIT,
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT,
        ZEROING_VOLTS,
        ZEROING_OFFSET,
        ZEROING_VOLTAGE_THRESHOLD,
        POSITION_TARGET_EPSILON);
    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }

  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
