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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.PathCommand;
import frc.robot.commands.ScoringSequenceCommand;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotIO;
import frc.robot.subsystems.superstructure.pivot.PivotIOTalonFX;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;

import java.lang.annotation.ElementType;
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
  private Pivot pivot;
  // private Superstructure superstructure;

  public RobotContainer() {
    intake = null;
    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case PROG -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
          pivot = new Pivot(new PivotIOTalonFX());
        }
        case ALPHA -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          pivot = new Pivot(new PivotIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
        }
        case SIM -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          pivot = new Pivot(new PivotIOTalonFX());
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
    if (pivot == null) {
      pivot = new Pivot(new PivotIO() {});
    }
    // superstructure = new Superstructure(elevator, pivot);
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

    // -----Flywheel Controls-----

    // -----Superstructure Controls-----
    driverA // GO TO BOTTOM
        .b()
        .onTrue(new ParallelCommandGroup(elevator.goToPositionCommand(ElevatorTarget.BOTTOM), pivot.goToPositionCommand(PivotTarget.TOP)));
        
    driverA // GO TO L2
        .a()
        .onTrue(new ScoringSequenceCommand(elevator, pivot, rollers, ElevatorTarget.L2, PivotTarget.SETUP_L2, PivotTarget.SCORE_L2));

    driverA // GO TO L4
        .y()
        .onTrue(new ScoringSequenceCommand(elevator, pivot, rollers, ElevatorTarget.L4, PivotTarget.SETUP_L4, PivotTarget.SCORE_L4));

    driverA // ZERO our mechanism
        .x()
        .onTrue(new ParallelCommandGroup(
          elevator.zeroingCommand().andThen(elevator.goToPositionCommand(ElevatorTarget.BOTTOM)),
          pivot.zeroingCommand().andThen(pivot.goToPositionCommand(PivotTarget.TOP))
        ));
    
    driverA.rightTrigger().onTrue(
      new ParallelCommandGroup(
        elevator.goToPositionCommand(ElevatorTarget.SETUP_INTAKE),
        pivot.goToPositionCommand(PivotTarget.INTAKE)
      ).andThen(
        rollers.setTargetCommand(RollerState.INTAKE)
      ).andThen(
        elevator.goToPositionCommand(ElevatorTarget.INTAKE)
      ).andThen(elevator.goToPositionCommand(ElevatorTarget.L4))
    );

    // -----Intake Controls-----
    driverB.b().onTrue(rollers.setTargetCommand(Rollers.RollerState.INTAKE));
    driverB.a().onTrue(rollers.setTargetCommand(Rollers.RollerState.IDLE));
    driverB.x().onTrue(rollers.setTargetCommand(Rollers.RollerState.HOLD));
    driverB.y().onTrue(rollers.setTargetCommand(Rollers.RollerState.EJECT));
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
