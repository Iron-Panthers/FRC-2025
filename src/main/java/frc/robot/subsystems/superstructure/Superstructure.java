package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    STOW,
    INTAKE,
    SUBWOOF_SHOT,
    SHUTTLE,
    AMP,
    ZERO,
    STOP,
    PRESCORE,
    SCORE;
  }

  private SuperstructureState targetState = SuperstructureState.STOW;

  private final Elevator elevator;
  private final Pivot pivot;

  public Superstructure(Elevator elevator, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void periodic() {
    switch (targetState) {
      case STOW -> { // basically just the default value
        pivot.setPositionTarget(PivotTarget.STOW);
        // elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case SCORE -> {
        pivot.setPositionTarget(PivotTarget.SCORE);
        // elevator.setPositionTarget(ElevatorTarget.STOW); //FIXME: FIX ALL OF THESE MODES SO THEY
        // CORRESPOND WITH REEFSCAPES
      }
      case INTAKE -> {
        pivot.setPositionTarget(PivotTarget.STOW);
        // elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case SUBWOOF_SHOT -> {
        // pivot.setPositionTarget(PivotTarget.SUBWOOF_SHOT);
        // elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case SHUTTLE -> {
        // pivot.setPositionTarget(PivotTarget.SHUTTLE);
        // elevator.setPositionTarget(ElevatorTarget.STOW);
      }
      case AMP -> {
        pivot.setPositionTarget(PivotTarget.STOW);
        // elevator.setPositionTarget(ElevatorTarget.AMP);
      }
      case ZERO -> {
        pivot.runCharacterization();
        elevator.runCharacterization();
      }
      case STOP -> {
        pivot.stop();
        elevator.stop();
        setTargetState(SuperstructureState.STOW);
      }
    }
    elevator.periodic();
    pivot.periodic();
    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public void setTargetState(SuperstructureState superstructureState) {
    targetState = superstructureState;
  }

  public double elevatorPosition() {
    return elevator.position();
  }

  public double getElevatorSupplyCurrentAmps() {
    return elevator.supplyCurrentAmps();
  }

  public double getPivotSupplyCurrentAmps() {
    return pivot.supplyCurrentAmps();
  }

  public SuperstructureState getTargetState() {
    return targetState;
  }
}
