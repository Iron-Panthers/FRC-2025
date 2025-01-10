package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  private LinearFilter filter;
  private double filteredCurrent;

  public enum RollerState {
    IDLE,
    INTAKE,
    EJECT,
    HOLD
  }

  private final Intake intake;

  private RollerState targetState = RollerState.IDLE;

  public Rollers(Intake intake) {
    this.intake = intake;
    this.filter = LinearFilter.movingAverage(35);
  }

  @Override
  public void periodic() {
    intake.setVoltageTarget(Intake.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        intake.setVoltageTarget(Intake.Target.IDLE);
      }
      case INTAKE -> {
        intake.setVoltageTarget(Intake.Target.INTAKE);
        if (filteredCurrent > 35) {
          this.targetState = RollerState.HOLD;
        }
      }
      case HOLD -> {
        intake.setVoltageTarget(Intake.Target.HOLD);
      }
      case EJECT -> {
        intake.setVoltageTarget(Intake.Target.EJECT);
      }
    }

    intake.periodic();
    this.filteredCurrent = this.filter.calculate(intake.getSupplyCurrentAmps());
    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public RollerState getTargetState() {
    return targetState;
  }

  public void setTargetState(RollerState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(RollerState target) {
    return startEnd(
        () -> {
          this.targetState = target;
        },
        () -> {
          this.targetState = RollerState.IDLE;
        });
  }
}
