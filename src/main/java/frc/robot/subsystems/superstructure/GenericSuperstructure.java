package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
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

  // linear filter for superstrucure
  private final LinearFilter supplyCurrentFilter;
  private double filteredSupplyCurrentAmps = 0;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(10);
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
        if (filteredSupplyCurrentAmps > 4) {
          setOffset();
          setControlMode(ControlMode.POSITION);
        }
      }
      case STOP -> {
        superstructureIO.stop();
      }
    }

    // calculate our new filtered supply current for the elevator
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", controlMode.toString());
    Logger.recordOutput(
        "Superstructure/" + name + "/Filtered supply current amps", controlMode.toString());
  }

  public G getGetPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(G positionTarget) {
    if (getControlMode() != ControlMode.ZERO) setControlMode(ControlMode.POSITION);
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
