package frc.robot.subsystems.superstructure.tongue;

import org.littletonrobotics.junction.Logger;

public class Tongue {
  public enum TongueTarget {
    TOP(90),
    INTAKE(115),
    STOW(115), // FIXME
    L1(100),
    L2(100),
    L3(60),
    L4(0);

    private double position;

    private TongueTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public enum ControlMode {
    POSITION,
    STOP,
  }

  private ControlMode controlMode = ControlMode.STOP;

  private final TongueIO io;

  private TongueIOInputsAutoLogged inputs = new TongueIOInputsAutoLogged(); // FIXME
  private TongueTarget positionTarget;

  public Tongue(TongueIO io) {
    this.io = io;

    setPositionTarget(TongueTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Tongue", inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        io.runPosition(positionTarget.getPosition());
      }
      case STOP -> {
        io.stop();
      }
    }
    Logger.recordOutput("Tongue/Target", positionTarget.toString());
    Logger.recordOutput("Tongue/Control Mode", controlMode.toString());
    Logger.recordOutput("Tongue/Reached target", reachedTarget());

    Logger.recordOutput("Superstructure/" + "Tongue" + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + "Tongue" + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + "Tongue" + "/Reached target", reachedTarget());
  }

  public TongueTarget getPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(TongueTarget positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  public double position() {
    return inputs.angle;
  }

  public boolean reachedTarget() {
    return true;
  }

  public boolean poleDetected() {
    return inputs.pole1Detected && inputs.pole2Detected;
  }
}
