package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO moduleIO;
  private final int index;

  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO moduleIO, int index) {
    this.moduleIO = moduleIO;
    this.index = index;
  }

  public void updateInputs() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
  }

  public void runToSetpoint(SwerveModuleState targetState) {
    moduleIO.runSteerPositionSetpoint(targetState.angle.getRadians());

    // reduce "skew" when changing directions ; from Phoenix6
    double steerError = targetState.angle.getRadians() - getSteerHeading().getRadians();
    double cosineScalar = Math.cos(steerError);
    if (cosineScalar < 0) {
      cosineScalar = 0;
    }

    /* Back out the expected shimmy the drive motor will see */
    /* Find the angular rate to determine what to back out */
    double azimuthTurnRps = inputs.steerVelocityRadsPerSec;
    /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
    double driveRateBackOut =
        azimuthTurnRps * DriveConstants.MODULE_CONSTANTS.couplingGearReduction();

    double driveVelocityRads =
        ((targetState.speedMetersPerSecond * cosineScalar)
                / DriveConstants.DRIVE_CONFIG.wheelRadius())
            + driveRateBackOut;

    moduleIO.runDriveVelocitySetpoint(driveVelocityRads);

    Logger.recordOutput("Swerve/Module" + index + "/SteerSetpoint", targetState.angle.getRadians());
    Logger.recordOutput(
        "Swerve/Module" + index + "/SteerError",
        targetState.angle.getRadians() - inputs.steerAbsolutePostion.getRadians());
    Logger.recordOutput("Swerve/Module" + index + "/DriveVelRadsScalar", driveVelocityRads);
  }

  public Rotation2d getSteerHeading() {
    return inputs.steerAbsolutePostion;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.steerAbsolutePostion);
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.steerAbsolutePostion);
  }
}
