package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    STOP, // Stop the superstructure
    // add more states here
  }

  private SuperstructureState targetState = SuperstructureState.STOP; // current target state

  private final Elevator elevator;

  public Superstructure(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    switch (targetState) { // switch on the target state
      // TODO add more states here
      case STOP -> {
        // TODO run logic here
      }
    }
    elevator.periodic();


    Logger.recordOutput("Superstructure/TargetState", targetState);
    // add more logging outputs here
  }

  // add other classes and a target state getter and setter
}
