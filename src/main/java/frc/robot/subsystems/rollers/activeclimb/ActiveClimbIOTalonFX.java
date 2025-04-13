package frc.robot.subsystems.rollers.activeclimb;

import frc.robot.Constants;
import frc.robot.subsystems.rollers.GenericRollersIOTalonFX;

public class ActiveClimbIOTalonFX extends GenericRollersIOTalonFX implements ActiveClimbIO {
  private static final int id =
      switch (Constants.getRobotType()) {
        case COMP -> 30;
        case ALPHA -> 62;
        default -> 0;
      };
  private static final int currentLimitAmps =
      switch (Constants.getRobotType()) {
        case COMP -> 40;
        case ALPHA -> 40;
        default -> 40;
      };
  private static final boolean inverted =
      switch (Constants.getRobotType()) {
        case COMP -> false;
        case ALPHA -> false;
        default -> false;
      };
  private static final boolean brake =
      switch (Constants.getRobotType()) {
        default -> false;
      };
  private static final double reduction =
      switch (Constants.getRobotType()) {
        case COMP -> 1;
        case ALPHA -> 1;
        default -> 1;
      };

  public ActiveClimbIOTalonFX() {
    super(id, currentLimitAmps, inverted, brake, reduction);
  }
}
