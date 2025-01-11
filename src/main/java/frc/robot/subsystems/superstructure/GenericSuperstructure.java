package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

public class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> {
  public interface PositionTarget {
    double getPosition();
  }

  public enum ControlMode {
    POSITION,
    ZERO,
    STOP,
  }

  private ControlMode controlMode = ControlMode.STOP;

  private final String name;
  private final GenericSuperstructureIO superstructureIO;

  private GenericSuperstructureIOInputsAutoLogged inputs =
      new GenericSuperstructureIOInputsAutoLogged();
  private G positionTarget;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;
  }

  public void periodic() {
    // Process inputs
    superstructureIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        superstructureIO.runPosition(positionTarget.getPosition());
      }
      case ZERO -> {
        superstructureIO.runCharacterization();
      }
      case STOP -> {
        superstructureIO.stop();
      }
    }

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", controlMode.toString());
  }

  public G getGetPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(G positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  public void setOffset() {
    superstructureIO.setOffset();
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double position() {
    return inputs.positionRotations;
  }
}
