package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;

import org.littletonrobotics.junction.Logger;

public class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> extends SubsystemBase{
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

  // linear filter for superstrucure
  private final LinearFilter supplyCurrentFilter;
  private double filteredSupplyCurrentAmps = 0;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);
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

    // calculate our new filtered supply current for the elevator
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());

    Logger.recordOutput("Superstructure/" + name + "/Target", getPositionTarget().toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", getControlMode().toString());
    Logger.recordOutput(
        "Superstructure/" + name + "/Filtered supply current amps", getFilteredSupplyCurrentAmps());
    Logger.recordOutput(
        "Superstructure/" + name + "/Reached target", reachedTarget());
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

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

  public double position() {
    return inputs.positionRotations;
  }

  /**
   * This function returns weather or not the subsystem has reached its position target
   *
   * @return weather the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(inputs.positionRotations - positionTarget.getPosition())
        <= superstructureIO.getPositionTargetEpsilon();
  }

  public Command zeroingCommand(){
    return new FunctionalCommand(() -> {
      setControlMode(ControlMode.ZERO);
    }, () -> { // execute
      // nothing needs to happen here
    }, (e) -> { // on end
      setOffset();
      setControlMode(ControlMode.POSITION);
    }, () -> (getFilteredSupplyCurrentAmps() > ElevatorConstants.ZEROING_VOLTAGE_THRESHOLD)// TODO: Make this work for both
    , this);
  }

  public Command goToPositionCommand(G position){
    return new FunctionalCommand(() -> {
      setPositionTarget(position);
    }, () -> { // execute
    }, (e) -> { // on end
    }, () -> reachedTarget(), this);
  }
}
