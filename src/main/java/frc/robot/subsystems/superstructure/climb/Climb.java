package frc.robot.subsystems.superstructure.climb;

import static frc.robot.subsystems.superstructure.climb.ClimbConstants.INDUCTION_PORT_NUMBER;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.rgb.RGB.RGBMessages;
import frc.robot.subsystems.superstructure.GenericSuperstructure;
import org.littletonrobotics.junction.Logger;

public class Climb extends GenericSuperstructure<Climb.ClimbTarget> {
  public enum ClimbTarget implements GenericSuperstructure.PositionTarget {

    // "Bottom" is ready to ram into cage
    BOTTOM(0.07),

    // "Top" is apex of climb
    TOP(0.307),

    // To get coral out
    CLEAR(-0.15),

    // When not climbing
    STOW(0.25);
    private double position = 0;

    private ClimbTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
  // induction sensor
  private DigitalInput inductionSensor;

  // run tino the cage - sensor triggers - flash leds to tell driver - button presses : reels it in
  // or out
  // CAN'T BACKOUT
  // set position for intake in a cage, a button to climb up or down
  public Climb(ClimbIO io) {
    super("Climb", io);
    inductionSensor = new DigitalInput(INDUCTION_PORT_NUMBER);
    setPositionTarget(ClimbTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  // checks if the sensor has hit the cage
  public boolean hitCage() {
    return !inductionSensor.get();
  }

  @Override
  public void periodic() {

    super.periodic();
    Logger.recordOutput("Superstructure/Climb/Hit Cage?", hitCage());
    Logger.recordOutput("Superstructure/Climb/Climb State", getPositionTarget());
    SmartDashboard.putBoolean("Has Cage?", hitCage());
    RGBMessages.CLIMB.setIsExpired(!hitCage());
  }
}
