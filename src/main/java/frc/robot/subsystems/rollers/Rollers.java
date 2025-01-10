package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

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
        if (intake.getFilteredCurrent() < -0.5) {
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

    Logger.recordOutput("Rollers/TargetState", targetState);
  }

  public RollerState getTargetState() {
    return targetState;
  }

  public void setTargetState(RollerState targetState) {
    this.targetState = targetState;
  }

  public Command setTargetCommand(RollerState target) {
    return new InstantCommand(
        () -> {
          this.targetState = target;
        });
  }
}
