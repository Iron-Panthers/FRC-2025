package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.DoubleSupplier;

public class ApproachReef extends Command {
  private final boolean bSide;
  private final Drive drive;
  private Command approachReef;
  private DoubleSupplier levelOffsetSupplier;

  public enum LevelOffsets {
    // metres
    L4_OFFSET(0.14),
    L3_OFFSET(0.05),
    L2_OFFSET(0),
    L1_OFFSET(0);
    public double levelOffset;

    private LevelOffsets(double levelOffset) {
      this.levelOffset = levelOffset;
    }

    public double getLevelOffset() {
      return levelOffset;
    }
  }

  public ApproachReef(DoubleSupplier levelOffsetSupplier, boolean bSide, Drive drive) {
    this.levelOffsetSupplier = levelOffsetSupplier;
    this.bSide = bSide;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    approachReef =
        RobotState.getInstance()
            .approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide, drive.getRobotSpeeds());
    approachReef.initialize();

    System.out.println(
        "Initializing ApproachReefCommand, offset of "
            + levelOffsetSupplier.getAsDouble()
            + ", bSide is "
            + bSide);
  }

  @Override
  public void execute() {
    approachReef.execute();
  }

  @Override
  public boolean isFinished() {
    return approachReef.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    approachReef.end(interrupted);
    approachReef = null;
  }
}
