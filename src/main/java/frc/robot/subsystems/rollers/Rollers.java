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
    FORCE_INTAKE,
    EJECT,
    HOLD
  }

  private final Intake intake;
  private final RollerSensorsIO sensorsIO;
  private double ejectTime = 0;
  private double intakeTime = 0;

  private RollerState targetState = RollerState.IDLE;
  private RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

  public Rollers(Intake intake, RollerSensorsIO sensorsIO) {
    this.intake = intake;
    this.sensorsIO = sensorsIO;
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("RollerSensors", sensorsInputs);
    intake.setVoltageTarget(Intake.Target.IDLE);

    switch (targetState) {
      case IDLE -> {
        intake.setVoltageTarget(Intake.Target.IDLE);
      }
      case INTAKE -> {
        intake.setVoltageTarget(Intake.Target.INTAKE);
        if (intakeDetected()) {
          this.targetState = RollerState.HOLD;
        }
      }
      case FORCE_INTAKE -> {
        intakeTime += 0.02;
        intake.setVoltageTarget(Intake.Target.INTAKE);
        if (intakeTime > 0.5) {
          this.targetState = RollerState.INTAKE;
          intakeTime = 0;
        }
      }
      case HOLD -> {
        intake.setVoltageTarget(Intake.Target.HOLD);
      }
      case EJECT -> {
        ejectTime += 0.02;
        intake.setVoltageTarget(Intake.Target.EJECT);
        if (ejectTime > 0.5) {
          this.targetState = RollerState.IDLE;
          ejectTime = 0;
        }
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

  public boolean intakeDetected() {
    return sensorsInputs.intakeDetected;
  }
}
