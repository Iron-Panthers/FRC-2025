// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.autonomous.PathCommand;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve; // FIXME make final, implement other robot types
  private Intake intake;
  private Rollers rollers;

  private SendableChooser<Command> autoChooser;

  // superstructure
  private Elevator elevator;
  private Superstructure superstructure;

  public RobotContainer() {
    intake = null;
    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
        }
        case DEV -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
        }
        case SIM -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
        }
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    rollers = new Rollers(intake);

    // superstructure
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    // TODO: add pivot
    superstructure = new Superstructure(elevator);

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis());
                })
            .withName("Drive Teleop"));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    // -----Intake Controls-----

    // -----Flywheel Controls-----

    // -----Superstructure Controls-----
    driverA // GO TO BOTTOM
        .b()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setTargetState(Superstructure.SuperstructureState.STOW),
                superstructure));
    driverA // GO TO L3
        .a()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setTargetState(Superstructure.SuperstructureState.SCORE_L3),
                superstructure));

    driverA // GO TO L4
        .y()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setTargetState(Superstructure.SuperstructureState.SCORE_L4),
                superstructure));

    driverA // ZERO
        .x()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setTargetState(Superstructure.SuperstructureState.ZERO),
                superstructure));

    // -----Intake Controls-----

    driverB.b().onTrue(rollers.setTargetCommand(Rollers.RollerState.INTAKE));
    driverB.a().onTrue(rollers.setTargetCommand(Rollers.RollerState.IDLE));
    driverB.x().onTrue(rollers.setTargetCommand(Rollers.RollerState.HOLD));
    driverB.y().onTrue(rollers.setTargetCommand(Rollers.RollerState.EJECT));

    // -----Flywheel Controls-----

  }

  private void configureAutos() {
    NamedCommands.registerCommand(
        "TestPrintCommand",
        new InstantCommand(
            () ->
                System.out.println(
                    "\nWe'd do something if we had the subsystems to do it :( \n"))); // FIXME Only
    // for testing
    // event
    // markers
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround

    BooleanSupplier flipAlliance =
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configureCustom(
        (path) -> new PathCommand(path, flipAlliance, swerve, passRobotConfig),
        () -> RobotState.getInstance().getOdometryPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        flipAlliance,
        true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }
}
