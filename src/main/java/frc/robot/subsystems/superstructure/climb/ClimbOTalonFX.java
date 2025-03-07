package frc.robot.subsystems.superstructure.climb;

import static frc.robot.subsystems.superstructure.climb.ClimbConstants.*;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

public class ClimbOTalonFX extends GenericSuperstructureIOTalonFX implements ClimbIO {

  public ClimbOTalonFX() {
    super(
        CLIMB_CONFIG.motorID(),
        Optional.empty(),
        INVERT_MOTOR,
        Optional.empty(),
        SUPPLY_CURRENT_LIMIT,
        CLIMB_CONFIG.canCoderID(),
        CLIMB_CONFIG.canCoderOffset(),
        CANCODER_DIRECTION,
        SENSOR_DISCONTINUITY_POINT,
        CLIMB_CONFIG.reduction(),
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
}
