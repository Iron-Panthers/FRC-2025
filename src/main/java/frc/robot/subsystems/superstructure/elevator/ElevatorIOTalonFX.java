package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.superstructure.pivot.PivotConstants.ZEROING_OFFSET;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class ElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {
    super(
        ELEVATOR_CONFIG.motorID(),
        INVERT_MOTOR,
        SUPPLY_CURRENT_LIMIT,
        Optional.empty(),
        ELEVATOR_CONFIG.reduction(),
        UPPER_EXTENSION_LIMIT,
        UPPER_VOLT_LIMIT,
        LOWER_VOLT_LIMIT,
        ZEROING_VOLTS,
        ZEROING_OFFSET);
    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        GRAVITY_TYPE);
  }
}
