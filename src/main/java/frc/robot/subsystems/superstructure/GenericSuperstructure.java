package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

public class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> {
  public interface PositionTarget {
    double getPosition();
  }

  private final String name;
  private boolean stop;
  private final GenericSuperstructureIO superstructureIO;

  private GenericSuperstructureIOInputsAutoLogged inputs =
      new GenericSuperstructureIOInputsAutoLogged();
  private G positionTarget;
  private boolean zeroing;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;
  }

  public void periodic() {
    superstructureIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    if (stop) {
      superstructureIO.stop();
    } else if (zeroing) {
      superstructureIO.runCharacterization();
      if (inputs.supplyCurrentAmps > 4) {
        zeroing = false;
        superstructureIO.setOffset();
      }
    } else {
      superstructureIO.runPosition(positionTarget.getPosition());
    }

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
  }

  public G getGetPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(G positionTarget) {
    stop = false;
    this.positionTarget = positionTarget;
  }

  public void runCharacterization() {
    stop = false;
    zeroing = true;
  }

  public double position() {
    return inputs.positionRotations;
  }

  public double supplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public void stop() {
    stop = true;
  }
}
