package frc.robot.subsystems.superstructure.tongue;

import static frc.robot.subsystems.superstructure.tongue.TongueConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class TongueIOServo implements TongueIO {

  private final Servo tongueIOServo;
  private final DigitalInput topSensor1;
  private final DigitalInput topSensor2;

  public TongueIOServo() {
    tongueIOServo = new Servo(TongueConstants.TONGUE_CONFIG.servoID());
    topSensor1 = new DigitalInput(8);
    topSensor2 = new DigitalInput(9);
    // tongueIOServo.setZeroLatch();
    // tongueIOServo.

    // set servo settings
  }

  @Override
  public void runPosition(double position) {
    tongueIOServo.setAngle(position);
  }

  @Override
  public void stop() {
    tongueIOServo.setDisabled();
  }

  @Override
  public void updateInputs(TongueIOInputsAutoLogged inputs) {
    inputs.angle = tongueIOServo.get();
    inputs.pole1Detected = !topSensor1.get();
    inputs.pole2Detected = !topSensor2.get();
  }
}
