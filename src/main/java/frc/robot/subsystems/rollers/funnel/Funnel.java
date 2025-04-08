package frc.robot.subsystems.rollers.funnel;

import frc.robot.subsystems.rollers.GenericRollers;

public class Funnel extends GenericRollers<Funnel.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(3),
    HOLD(0),
    EJECT(-4);

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  public Funnel(FunnelIO funnelIO) {
    super("Funnel", funnelIO);
  }
}
