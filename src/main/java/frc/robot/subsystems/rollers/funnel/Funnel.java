package frc.robot.subsystems.rollers.funnel;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.rollers.GenericRollers;

public class Funnel extends GenericRollers<Funnel.Target> {
  public enum Target implements GenericRollers.VelocityTarget {
    IDLE(0),
    INTAKE(3),
    HOLD(0),
    EJECT(-4);

    private double velocity;

    private Target(double velocity) {
      this.velocity = velocity;
    }

    public double getValue() {
      return velocity;
    }
  }

  public Funnel(FunnelIO funnelIO) {
    super("Funnel", funnelIO);
  }
}
