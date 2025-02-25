package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

public class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> {
  public interface PositionTarget {
    double getPosition();
  }

  public enum ControlMode {
    POSITION,
    STOP,
  }

  private ControlMode controlMode = ControlMode.STOP;

  protected final String name;
  protected final GenericSuperstructureIO superstructureIO;

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
      case STOP -> {
        superstructureIO.stop();
      }
    }

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + name + "/Reached target", reachedTarget());
  }

  public G getPositionTarget() {
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

  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(inputs.positionRotations - (positionTarget.getPosition()))
        <= superstructureIO.getPositionTargetEpsilon();
  }

  // public Command zeroingCommand() {
  //   return new FunctionalCommand(
  //       () -> {},
  //       () -> { // execute
  //         // nothing needs to happen here
  //         setControlMode(ControlMode.ZERO);
  //       },
  //       (e) -> { // on end
  //         setOffset();
  //         setControlMode(ControlMode.POSITION);
  //       },
  //       () ->
  //           (getFilteredSupplyCurrentAmps()
  //               > superstructureIO.getZeroingVoltageThreshold()) // TODO: Make this work for both
  //       ,
  //       this);
  // }

  // public Command goToPositionCommand(G position) {
  //   return new FunctionalCommand(
  //       () -> {
  //         setPositionTarget(position);
  //       },
  //       () -> { // execute
  //       },
  //       (e) -> { // on end
  //       },
  //       () -> reachedTarget(),
  //       this);
  // }
}
