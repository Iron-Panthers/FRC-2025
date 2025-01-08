package frc.robot.subsystems.superstructure.elevator;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class ElevatorIOTalonFX extends GenericSuperstructureIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {
    super(
        ElevatorConstants.ELEVATOR_CONFIG.motorID(),
        ElevatorConstants.INVERT_MOTOR,
        ElevatorConstants.SUPPLY_CURRENT_LIMIT,
        Optional.empty(),
        ElevatorConstants.ELEVATOR_CONFIG.reduction(),
        ElevatorConstants.UPPER_EXTENSION_LIMIT,
        ElevatorConstants.UPPER_VOLT_LIMIT,
        ElevatorConstants.LOWER_VOLT_LIMIT);
    setSlot0(
        ElevatorConstants.GAINS.kP(),
        ElevatorConstants.GAINS.kI(),
        ElevatorConstants.GAINS.kD(),
        ElevatorConstants.GAINS.kS(),
        ElevatorConstants.GAINS.kV(),
        ElevatorConstants.GAINS.kA(),
        ElevatorConstants.GRAVITY_TYPE);
  }
}
