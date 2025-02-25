package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.subsystems.superstructure.GenericSuperstructure;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GenericSuperstructure<Elevator.ElevatorTarget> {
  public enum ElevatorTarget implements GenericSuperstructure.PositionTarget {
    BOTTOM(0), // 25 and 7.25, made it a bit bigger
    L1(11.8), // FIXME: 26 and 21.5
    L2(20.88), // 24 and 53.75
    L3(0), // 0 and 53.75
    SETUP_L4(31.6),
    SCORE_L4(28),
    TOP(31),
    INTAKE(0);
    // TEST_BOTTOM(2),
    // TEST_TOP(31),
    // TEST_MIDDLE(15); // FIXME
    private double position = 0;

    private ElevatorTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  // linear filter for superstrucure
  private final LinearFilter supplyCurrentFilter;
  private double filteredSupplyCurrentAmps = 0;

  private boolean zeroing = false;

  public Elevator(ElevatorIO io) {
    super("Elevator", io);
    setPositionTarget(ElevatorTarget.L3);
    setControlMode(ControlMode.STOP);

    // setup the linear filter
    supplyCurrentFilter = LinearFilter.movingAverage(30);
  }

  @Override
  public void periodic() {

    super.periodic();

    // for zeroing
    // calculate our new filtered supply current for the elevator
    filteredSupplyCurrentAmps = supplyCurrentFilter.calculate(getSupplyCurrentAmps());
    if (zeroing) {
      superstructureIO.runCharacterization();
    }
    Logger.recordOutput(
        "Superstructure/" + name + "/Filtered supply current amps", getFilteredSupplyCurrentAmps());
  }

  public double getFilteredSupplyCurrentAmps() {
    return filteredSupplyCurrentAmps;
  }

  public boolean aboveSafeHeightForPivot() {
    return this.getPosition() > ElevatorConstants.MIN_SAFE_HEIGHT_FOR_PIVOT;
  }

  public void setZeroing(boolean zeroing) {
    this.zeroing = zeroing;
  }

  public boolean isZeroing() {
    return zeroing;
  }
}
