package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VoltageTarget> {
  public interface VoltageTarget {
    double getVolts();
  }

  private LinearFilter filter;
  private double filteredCurrent;

  private final String name;
  private final GenericRollersIO rollerIO;
  private GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

  private G voltageTarget;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
    this.filter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    rollerIO.runVolts(voltageTarget.getVolts());
    Logger.recordOutput("Rollers/" + name + "/Target", voltageTarget.toString());

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput("Rollers/" + name + "/FilteredCurrent", filteredCurrent);
  }

  public G getVoltageTarget() {
    return voltageTarget;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public void setVoltageTarget(G voltageTarget) {
    this.voltageTarget = voltageTarget;
  }
}
