// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.ApproachReef.LevelOffsets;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.subsystems.canWatchdog.CANWatchdog;
import frc.robot.subsystems.canWatchdog.CANWatchdogIO;
import frc.robot.subsystems.canWatchdog.CANWatchdogIOComp;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGB.RGBMessages;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.rgb.RGBIOCANdle;
import frc.robot.subsystems.rollers.RollerSensorsIOComp;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.superstructure.ClimbController;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.climb.Climb.ClimbTarget;
import frc.robot.subsystems.superstructure.climb.ClimbOTalonFX;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
import frc.robot.subsystems.superstructure.pivot.PivotIO;
import frc.robot.subsystems.superstructure.pivot.PivotIOTalonFX;
import frc.robot.subsystems.superstructure.tongue.Tongue;
import frc.robot.subsystems.superstructure.tongue.TongueIO;
import frc.robot.subsystems.superstructure.tongue.TongueIOServo;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();

  private SendableChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  @AutoLogOutput(key = "CommandedOffset")
  private LevelOffsets levelOffsets = LevelOffsets.PREP_L4_OFFSET;

  private boolean eject = false;

  private boolean autoAngle = true;

  private Drive swerve;
  private Vision vision;
  private Intake intake;
  private Rollers rollers;
  private Elevator elevator;
  private Pivot pivot;
  private Tongue tongue;
  private Climb climb;
  private Superstructure superstructure;
  private RGB rgb;
  private CANWatchdog canWatchdog;
  private ApproachReef approachReef;
  private ClimbController climbController;

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
          vision = new Vision(new VisionIOPhotonvision(4), new VisionIOPhotonvision(5));
          intake = new Intake(new IntakeIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
          pivot = new Pivot(new PivotIOTalonFX());
          tongue = new Tongue(new TongueIOServo());
          rgb = new RGB(new RGBIOCANdle());
          canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          climb = new Climb(new ClimbOTalonFX());
        }
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
    if (vision == null) {
      vision = new Vision();
    }

    rollers = new Rollers(intake, new RollerSensorsIOComp());

    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (pivot == null) {
      pivot = new Pivot(new PivotIO() {});
    }

    if (canWatchdog == null) {
      canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);
    }

    if (rgb == null) {
      rgb = new RGB(new RGBIO() {});
    }

    if (tongue == null) {
      tongue = new Tongue(new TongueIO() {});
    }
    superstructure = new Superstructure(elevator, pivot, tongue);

    nameCommands();
    if (climb == null) {
      climb = new Climb(new ClimbOTalonFX());
    }
    climbController = new ClimbController(climb);

    configureAutos();
    configureBindings();
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  private void nameCommands() {
    // Register Command Names
    NamedCommands.registerCommand(
        "Intake",
        new SequentialCommandGroup(
            superstructure.goToStateCommand(SuperstructureState.INTAKE),
            rollers.setTargetCommand(RollerState.INTAKE)));
    NamedCommands.registerCommand(
        "Score_L4",
        new SequentialCommandGroup(
                new WaitUntilCommand(() -> rollers.intakeDetected()),
                new FunctionalCommand(
                    () -> superstructure.setTargetState(SuperstructureState.SETUP_L4),
                    () -> {},
                    (e) -> {},
                    () ->
                        superstructure.getCurrentState() == SuperstructureState.SETUP_L4
                            && superstructure.superstructureReachedTarget(),
                    superstructure))
            .withTimeout(2.6));

    NamedCommands.registerCommand("Eject", rollers.setTargetCommand(RollerState.EJECT_TOP));

    NamedCommands.registerCommand(
        "Eject_L4",
        new SequentialCommandGroup(
            rollers.setTargetCommand(RollerState.EJECT_TOP),
            new WaitCommand(0.2),
            superstructure
                .goToStateCommand(SuperstructureState.INTAKE)
                .alongWith(
                    new WaitCommand(0.4).andThen(rollers.setTargetCommand(RollerState.INTAKE)))));

    new EventTrigger("Score_L4")
        .onTrue(superstructure.goToStateCommand(SuperstructureState.SCORE_L4));

    new EventTrigger("Intake")
        .onTrue(
            new SequentialCommandGroup(
                superstructure.goToStateCommand(SuperstructureState.INTAKE),
                rollers.setTargetCommand(RollerState.INTAKE)));
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
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      superstructure.getElevatorPosition() > 3
                          ? 3
                          : DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());
                  if (Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                      || Math.abs(driverA.getRightTriggerAxis()) > 0.1) {
                    swerve.clearHeadingControl();
                  } else if (autoAngle) {
                    // Station snaps
                    if (RobotState.getInstance()
                            .getEstimatedPose()
                            .getTranslation()
                            .getDistance(DriveConstants.RIGHT_CORNER)
                        < 3) {
                      swerve.setTargetHeading(new Rotation2d(Math.toRadians(232)));
                    } else if (RobotState.getInstance()
                            .getEstimatedPose()
                            .getTranslation()
                            .getDistance(DriveConstants.LEFT_CORNER)
                        < 3) {
                      swerve.setTargetHeading(new Rotation2d(Math.toRadians(128)));
                      // close up reef snaps
                    } else if (RobotState.getInstance()
                            .getEstimatedPose()
                            .getTranslation()
                            .getDistance(DriveConstants.REEF_TRANSLATION2D)
                        < 2) {
                      swerve.setTargetHeading(
                          DriverStation.getAlliance().isPresent()
                                  && DriverStation.getAlliance().get() == Alliance.Red
                              ? calculateSnapTargetHeading(
                                  RobotState.getInstance()
                                      .getEstimatedPose()
                                      .getTranslation()
                                      .minus(DriveConstants.REEF_TRANSLATION2D)
                                      .getAngle())
                              : FlippingUtil.flipFieldRotation(
                                  calculateSnapTargetHeading(
                                      RobotState.getInstance()
                                          .getEstimatedPose()
                                          .getTranslation()
                                          .minus(DriveConstants.REEF_TRANSLATION2D)
                                          .getAngle())));
                      // climb snaps
                    } else if (MathUtil.isNear(
                            DriveConstants.CLIMB_ZONE_CENTER.getX(),
                            RobotState.getInstance().getEstimatedPose().getTranslation().getX(),
                            2)
                        && MathUtil.isNear(
                            DriveConstants.CLIMB_ZONE_CENTER.getY(),
                            RobotState.getInstance().getEstimatedPose().getTranslation().getY(),
                            2)
                        && superstructure.getTargetState() == SuperstructureState.CLIMB) {
                      swerve.setTargetHeading(new Rotation2d(Math.PI / 2));
                      // default gradual far from reef snaps
                    } else {
                      swerve.setTargetHeading(
                          RobotState.getInstance()
                              .getEstimatedPose()
                              .getTranslation()
                              .minus(DriveConstants.REEF_TRANSLATION2D)
                              .getAngle()
                              .minus(
                                  DriverStation.getAlliance().isPresent()
                                          && DriverStation.getAlliance().get() == Alliance.Red
                                      ? Rotation2d.kPi
                                      : Rotation2d.kZero));
                    }
                  }
                })
            .withName("Drive Teleop"));

    new Trigger(
            () -> (Math.abs(driverA.getRightY()) > 0.2) || (Math.abs(driverA.getRightX()) > 0.2))
        .whileTrue(
            new FunctionalCommand(
                () -> {},
                () ->
                    swerve.setTargetHeading(
                        calculateSnapTargetHeading(
                            new Rotation2d(
                                Math.atan2(
                                    MathUtil.applyDeadband(-driverA.getRightX(), 0.1),
                                    MathUtil.applyDeadband(-driverA.getRightY(), 0.1))))),
                interrupted -> {},
                () -> false));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));

    driverA.y().onTrue(new InstantCommand(() -> autoAngle = !autoAngle));

    // driverA.povUp().onTrue(new InstantCommand(() -> levelOffsets = LevelOffsets.L4_OFFSET));
    // driverA.povRight().onTrue(new InstantCommand(() -> levelOffsets = LevelOffsets.L3_OFFSET));
    // driverA.povDown().onTrue(new InstantCommand(() -> levelOffsets = LevelOffsets.L2_OFFSET));
    // driverA.povLeft().onTrue(new InstantCommand(() -> levelOffsets = LevelOffsets.L1_OFFSET));

    // auto align
    driverA
        .leftBumper()
        .whileTrue(
            (new ApproachReef(() -> levelOffsets, true, swerve)
                    .alongWith(new InstantCommand(() -> swerve.clearHeadingControl()))
                    .andThen(
                        new InstantCommand(
                            () -> eject = levelOffsets != LevelOffsets.PREP_L4_OFFSET))
                    .andThen(
                        (new WaitUntilCommand(() -> RobotState.getInstance().alignError() > 0.5)
                                .andThen(new ApproachReef(() -> levelOffsets, true, swerve)))
                            .repeatedly()
                            .until(() -> levelOffsets == LevelOffsets.L4_OFFSET)))
                .repeatedly()); // so if it aligns to L4 prep, it will then try to align to L4
    // auto align
    driverA
        .rightBumper()
        .whileTrue(
            (new ApproachReef(() -> levelOffsets, false, swerve)
                    .alongWith(new InstantCommand(() -> swerve.clearHeadingControl()))
                    .andThen(
                        new InstantCommand(
                            () -> eject = levelOffsets != LevelOffsets.PREP_L4_OFFSET))
                    .andThen(
                        (new WaitUntilCommand(
                                    () ->
                                        RobotState.getInstance().alignError() > 0.5
                                            || (RobotState.getInstance().alignError() < 2
                                                && levelOffsets == LevelOffsets.PREP_L4_OFFSET))
                                .andThen(new ApproachReef(() -> levelOffsets, false, swerve)))
                            .repeatedly()
                            .until(() -> levelOffsets == LevelOffsets.L4_OFFSET)))
                .repeatedly()); // so if it aligns to L4 prep, it will then try to align to L4

    // if superstructure at L4 pos. move to score
    new Trigger(
            () ->
                levelOffsets == LevelOffsets.PREP_L4_OFFSET
                    && !swerve.isTeleop()
                    && superstructure.superstructureReachedTarget()
                    && superstructure.getCurrentState() == SuperstructureState.SETUP_L4
                    && RobotState.getInstance().alignError() < 2)
        .onTrue(new InstantCommand(() -> levelOffsets = LevelOffsets.L4_OFFSET));
    // after ejecting or ending auto align early, when you move away make L4 auto align be prep
    new Trigger(() -> eject)
        .onTrue(
            new WaitUntilCommand(() -> RobotState.getInstance().alignError() > 3)
                .andThen(new InstantCommand(() -> eject = false))
                .alongWith(
                    new InstantCommand(
                        () ->
                            levelOffsets =
                                levelOffsets == LevelOffsets.L4_OFFSET
                                    ? LevelOffsets.PREP_L4_OFFSET
                                    : levelOffsets)));
    driverA
        .leftBumper()
        .onFalse(
            new InstantCommand(
                () ->
                    levelOffsets =
                        levelOffsets == LevelOffsets.L4_OFFSET
                            ? LevelOffsets.PREP_L4_OFFSET
                            : levelOffsets));
    driverA
        .rightBumper()
        .onFalse(
            new InstantCommand(
                () ->
                    levelOffsets =
                        levelOffsets == LevelOffsets.L4_OFFSET
                            ? LevelOffsets.PREP_L4_OFFSET
                            : levelOffsets));
    // station angle snap (no longer all that important)
    driverA
        .x()
        .onTrue(
            new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(Math.toRadians(128)))));

    driverA
        .b()
        .onTrue(
            new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(Math.toRadians(232)))));

    // -----Superstructure Controls-----
    // auto go to L1
    new Trigger(
            () ->
                2.5
                        > RobotState.getInstance()
                            .getEstimatedPose()
                            .getTranslation()
                            .minus(DriveConstants.REEF_TRANSLATION2D)
                            .getNorm()
                    && !swerve.isTeleop()
                    && DriverStation.isTeleop()
                    && levelOffsets == LevelOffsets.L1_OFFSET
                    && (rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
        .onTrue(superstructure.goToStateCommand(SuperstructureState.L1));

    // auto go to L2
    new Trigger(
            () ->
                3
                        > RobotState.getInstance()
                            .getEstimatedPose()
                            .getTranslation()
                            .minus(DriveConstants.REEF_TRANSLATION2D)
                            .getNorm()
                    && !swerve.isTeleop()
                    && DriverStation.isTeleop()
                    && levelOffsets == LevelOffsets.L2_OFFSET
                    && (rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
        .onTrue(superstructure.goToStateCommand(SuperstructureState.L2));

    // auto go to L3
    new Trigger(
            () ->
                !swerve.isTeleop()
                    && DriverStation.isTeleop()
                    && levelOffsets == LevelOffsets.L3_OFFSET
                    && (rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
        .onTrue(superstructure.goToStateCommand(SuperstructureState.SCORE_L3));

    // auto go to L4
    new Trigger(
            () ->
                !swerve.isTeleop()
                    && DriverStation.isTeleop()
                    && levelOffsets == LevelOffsets.PREP_L4_OFFSET
                    && (superstructure.getTargetState() == SuperstructureState.PREVENT_TIPPING
                        || superstructure.getTargetState() == SuperstructureState.SETUP_L3))
        .onTrue(superstructure.goToStateCommand(SuperstructureState.SCORE_L4));
    // auto go half to L4 after intaking
    new Trigger(
            () ->
                levelOffsets == LevelOffsets.PREP_L4_OFFSET
                    && rollers.readyToRaise()
                    && DriverStation.isTeleop())
        .whileTrue(superstructure.goToStateCommand(SuperstructureState.PREVENT_TIPPING));
    // L1
    driverB
        .povDown()
        .onTrue(
            // superstructure
            //     .goToStateCommand(SuperstructureState.L1)
            new InstantCommand(() -> levelOffsets = LevelOffsets.L1_OFFSET)
                .alongWith(rgb.clearLevelCommands())
                .andThen(rgb.startMessageCommand(RGBMessages.L1)));
    // L2

    driverB
        .povRight()
        .onTrue(
            // superstructure
            // .goToStateCommand(SuperstructureState.L2)
            new InstantCommand(() -> levelOffsets = LevelOffsets.L2_OFFSET)
                .alongWith(rgb.clearLevelCommands())
                .andThen(rgb.startMessageCommand(RGBMessages.L2)));
    // L3
    driverB
        .povLeft()
        .onTrue(
            // superstructure
            //     .goToStateCommand(SuperstructureState.SCORE_L3)
            new InstantCommand(() -> levelOffsets = LevelOffsets.L3_OFFSET)
                .alongWith(rgb.clearLevelCommands())
                .andThen(rgb.startMessageCommand(RGBMessages.L3)));

    // L4
    driverB
        .povUp()
        .onTrue(
            // superstructure
            // .goToStateCommand(SuperstructureState.SCORE_L4)
            new InstantCommand(() -> levelOffsets = LevelOffsets.PREP_L4_OFFSET)
                .alongWith(rgb.clearLevelCommands())
                .andThen(rgb.startMessageCommand(RGBMessages.L4)));

    new Trigger(() -> driverB.a().getAsBoolean() && driverB.start().getAsBoolean())
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setCurrentState(SuperstructureState.ZERO);
                },
                superstructure));

    // Stop everything
    driverB
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setStopped(true);
                  climbController.setStopped(true);
                  rollers.setTargetState(RollerState.IDLE);
                }));

    // kinda manual commands
    driverB.leftBumper().onTrue(climbController.setPositionTargetCommand(ClimbTarget.STOW));
    driverB
        .rightBumper()
        .onTrue(
            superstructure
                .goToStateCommand(SuperstructureState.SETUP_L3)
                .alongWith(superstructure.oneTimeOverrideCommand()));
    // climb
    driverB
        .y()
        .onTrue(
            climbController.setPositionTargetCommand(
                ClimbTarget.TOP) // FIXME: We need to add elevator position up
            );

    new Trigger(() -> driverB.b().getAsBoolean() && driverB.start().getAsBoolean())
        .onTrue(
            climbController
                .setPositionTargetCommand(ClimbTarget.BOTTOM)
                .alongWith(superstructure.goToStateCommand(SuperstructureState.CLIMB)));
    // Descore
    driverB
        .rightStick()
        .onTrue(
            superstructure
                .goToStateCommand(SuperstructureState.DESCORE_LOW)
                .andThen(rollers.setTargetCommand(RollerState.INTAKE)));
    driverB
        .leftStick()
        .onTrue(
            superstructure
                .goToStateCommand(SuperstructureState.DESCORE_HIGH)
                .andThen(rollers.setTargetCommand(RollerState.INTAKE)));

    driverB // intake
        .leftTrigger()
        .onTrue(
            new SequentialCommandGroup(
                superstructure.goToStateCommand(SuperstructureState.INTAKE),
                rollers.setTargetCommand(RollerState.INTAKE)));

    // RGB for intaking
    new Trigger(() -> rollers.intakeDetected())
        .onTrue(rgb.startMessageCommand(RGBMessages.CORAL_DETECTED));
    new Trigger(() -> rollers.getTargetState().equals(RollerState.INTAKE))
        .onTrue(rgb.endMessageCommand(RGBMessages.CORAL_DETECTED));

    // Eject on L1
    new Trigger(
            () ->
                (superstructure.getTargetState().equals(SuperstructureState.L1)
                        && superstructure.getCurrentState().equals(SuperstructureState.L1)
                        && superstructure.superstructureReachedTarget())
                    && (driverB.rightTrigger().getAsBoolean() || eject))
        .onTrue(
            new InstantCommand(() -> eject = false)
                .andThen(rollers.setTargetCommand(RollerState.EJECT_L1))
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.INTAKE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE))));
    // Eject L2
    new Trigger(
            () ->
                (superstructure.getTargetState().equals(SuperstructureState.L2)
                        && superstructure.getCurrentState().equals(SuperstructureState.L2)
                        && superstructure.superstructureReachedTarget())
                    && (driverB.rightTrigger().getAsBoolean() || eject))
        .onTrue(
            new InstantCommand(() -> eject = false)
                .andThen(rollers.setTargetCommand(RollerState.EJECT_L2))
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.EJECT_TOP))
                        .andThen(new WaitCommand(0.03))
                        .andThen(rollers.setTargetCommand(RollerState.IDLE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE)))
                .andThen(new WaitCommand(0.5))
                .andThen(rollers.setTargetCommand(RollerState.INTAKE)));

    // Eject L3
    new Trigger(
            () ->
                (superstructure.getTargetState().equals(SuperstructureState.SCORE_L3)
                        && superstructure.getCurrentState().equals(SuperstructureState.SCORE_L3)
                        && superstructure.superstructureReachedTarget())
                    && (driverB.rightTrigger().getAsBoolean()
                        || (eject && superstructure.superstructureReachedTarget())))
        .onTrue(
            new InstantCommand(() -> eject = false)
                .andThen(rollers.setTargetCommand(RollerState.EJECT_L3))
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.INTAKE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE))));

    // Eject Intake - ONLY IF ITS EXACTLY AT INTAKE
    new Trigger(
            () ->
                (superstructure.getTargetState().equals(SuperstructureState.INTAKE)
                        && superstructure.getCurrentState().equals(SuperstructureState.INTAKE)
                        && superstructure.superstructureReachedTarget())
                    && driverB.rightTrigger().getAsBoolean())
        .onTrue(
            rollers
                .setTargetCommand(RollerState.EJECT_TOP)
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.INTAKE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE))));
    // Eject if not at L1 or L2 or L3 or Intake
    new Trigger(
            () ->
                !(superstructure.getTargetState().equals(SuperstructureState.L1)
                        || superstructure.getTargetState().equals(SuperstructureState.L2)
                        || superstructure.getTargetState().equals(SuperstructureState.SCORE_L3)
                        || superstructure.getTargetState().equals(SuperstructureState.INTAKE))
                    && (driverB.rightTrigger().getAsBoolean()
                        || (eject
                            && superstructure.getTargetState().equals(SuperstructureState.SETUP_L4)
                            && superstructure.superstructureReachedTarget())))
        .onTrue(
            new InstantCommand(() -> eject = false)
                .andThen(rollers.setTargetCommand(RollerState.EJECT_TOP))
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.INTAKE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE))));
    // Eject on L4 with sensors
    new Trigger(() -> (superstructure.getCurrentState() == SuperstructureState.SCORE_L4))
        .onTrue(
            new SequentialCommandGroup(
                    // new WaitCommand(0.1),
                    rollers.setTargetCommand(RollerState.EJECT_TOP),
                    new WaitCommand(0.2),
                    superstructure.goToStateCommand(SuperstructureState.INTAKE),
                    new WaitCommand(0.9),
                    rollers.setTargetCommand(RollerState.FORCE_INTAKE))
                .andThen(new InstantCommand(() -> eject = false))
                .alongWith(
                    new InstantCommand(
                        () ->
                            levelOffsets =
                                levelOffsets == LevelOffsets.L4_OFFSET
                                    ? LevelOffsets.PREP_L4_OFFSET
                                    : levelOffsets)));
  }

  private void configureAutos() {
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

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        flipAlliance,
        swerve);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }

  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance()
        .schedule(new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 5, .5)));

    // vibrate controller at 30 seconds left
    CommandScheduler.getInstance()
        .schedule(
            new WaitCommand(105)
                .andThen(
                    new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 3, 0.4))));
  }

  public void updateDashboardStatus() {
    SmartDashboard.putBoolean(
        "Elevator Initial",
        elevator.getPositionTarget() == ElevatorTarget.BOTTOM && elevator.reachedTarget());
    SmartDashboard.putBoolean(
        "Arm Initial", (pivot.getPositionTarget() == PivotTarget.STOW) && pivot.reachedTarget());
    SmartDashboard.putBoolean(
        "Climb Initial", climb.getPositionTarget() == ClimbTarget.STOW && climb.reachedTarget());

    SmartDashboard.putBoolean("Tongue 1", tongue.pole1Detected());
    SmartDashboard.putBoolean("Tongue 2", tongue.pole2Detected());

    SmartDashboard.putBoolean("Coral Intaked", rollers.intakeDetected());
    SmartDashboard.putBoolean("Climb Cage", !climb.hitCage());

    SmartDashboard.putString("Current Auto", autoChooser.getSelected().getName());
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    double a = ((currentAngle - newAngle) % 360 + 360) % 360;
    double b = ((currentAngle - newAngle) % 360 + 360) % 360;
    return a < b ? a : -b;
  }

  public static Rotation2d calculateSnapTargetHeading(Rotation2d targetHeading) {
    targetHeading =
        targetHeading.rotateBy(
            new Rotation2d(Math.PI + Math.toRadians(30))); // because back of robot
    double closest = DriveConstants.REEF_SNAP_ANGLES[0];
    for (double snap : DriveConstants.REEF_SNAP_ANGLES) {
      if (Math.abs(relativeAngularDifference(targetHeading.getDegrees(), snap))
          < Math.abs(relativeAngularDifference(targetHeading.getDegrees(), closest))) {
        closest = snap;
      }
    }
    return new Rotation2d(Math.toRadians(closest));
  }
}
