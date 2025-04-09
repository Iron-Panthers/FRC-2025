package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;

import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.RollerTarget> {
  public interface RollerTarget {
    boolean isVoltage = true;
    double getValue();
  }
  public interface VoltageTarget extends RollerTarget {
    boolean isVoltage = true;
  }
  public interface VelocityTarget extends RollerTarget {
    boolean isVoltage = false;
  }

  private LinearFilter filter;
  private double filteredCurrent;

  protected final String name;
  protected final GenericRollersIO rollerIO;
  private GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

  protected G rollerTarget;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
    this.filter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (G.isVoltage){
      rollerIO.runVolts(rollerTarget.getValue());
    }else{
      rollerIO.runVelocity(rollerTarget.getValue());
    } 
    Logger.recordOutput("Rollers/" + name + "/Target", rollerTarget.toString());

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput("Rollers/" + name + "/FilteredCurrent", filteredCurrent);
  }

  public G getRollerTarget() {
    return rollerTarget;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public void setRollerTarget(G rollerTarget) {
    this.rollerTarget = rollerTarget;
  }
}
