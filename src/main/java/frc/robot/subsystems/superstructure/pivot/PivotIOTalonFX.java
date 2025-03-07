package frc.robot.subsystems.superstructure.pivot;

import static frc.robot.subsystems.superstructure.pivot.PivotConstants.*;

import frc.robot.subsystems.superstructure.GenericSuperstructureIOTalonFX;
import java.util.Optional;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;

import edu.wpi.first.wpilibj.RobotState;

public class PivotIOTalonFX extends GenericSuperstructureIOTalonFX implements PivotIO {

  // Dynamic motion magic control mode, only to be used in auto - Jacob didn't want it doing anything during teleop
  private final DynamicMotionMagicVoltage dynamicPositionControl = new DynamicMotionMagicVoltage(0, MOTION_MAGIC_CONFIG.cruiseVelocity(), MOTION_MAGIC_CONFIG.autonomousForwardAcceleration(), 0);

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
    double positionInRotations = position / 360d;
    if(RobotState.isAutonomous()){ // if were in the autonomous state we can do this weird dynamic acceleration thing
      dynamicPositionControl.Acceleration = talon.getPosition().getValueAsDouble() - positionInRotations < 0 ? MOTION_MAGIC_CONFIG.autonomousForwardAcceleration() : MOTION_MAGIC_CONFIG.autonomousBackwardAcceleration();
      talon.setControl(dynamicPositionControl.withPosition(position / 360d));
    }else{
      super.runPosition(positionInRotations); // convert degrees to rotations
    }
  }
}
