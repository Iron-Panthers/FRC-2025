package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollers;

public class Intake extends GenericRollers<Intake.Target> {
  public enum Target implements GenericRollers.VoltageTarget {
    IDLE(0),
    INTAKE(4),
    HOLD(0),
    EJECT_TOP(-8),
    EJECT_L3(3.9),
    EJECT_L1(2.7),
    EJECT_L2(3.9);

    private double volts;

    private Target(double volts) {
      this.volts = volts;
    }

    public double getValue() {
      return volts;
    }
  }

  public Intake(IntakeIO intakeIO) {
    super("Intake", intakeIO);
  }
}
