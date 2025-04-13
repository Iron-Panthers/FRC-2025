package frc.robot.subsystems.rollers.activeclimb;

import frc.robot.subsystems.rollers.GenericRollers;
import org.littletonrobotics.junction.Logger;

public class ActiveClimb extends GenericRollers<ActiveClimb.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    CLIMB(6);

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public ActiveClimb(ActiveClimbIO activeClimbIO) {
    super("ActiveClimb", activeClimbIO);

    Logger.recordOutput("activeclimb", "eggnog");
  }
}
