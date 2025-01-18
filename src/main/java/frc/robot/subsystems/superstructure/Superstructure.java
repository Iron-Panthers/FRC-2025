// EXPERIMENTAL CHANGE
// WE *hopefully* no longer need this

// package frc.robot.subsystems.superstructure;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.superstructure.GenericSuperstructure.ControlMode;
// import frc.robot.subsystems.superstructure.elevator.Elevator;
// import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
// import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
// import frc.robot.subsystems.superstructure.pivot.Pivot;
// import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;
// import frc.robot.subsystems.superstructure.pivot.PivotConstants;
// import java.util.Optional;
// import org.littletonrobotics.junction.Logger;

// public class Superstructure extends SubsystemBase {
//   public enum SuperstructureState {
//     STOP, // Stop the superstructure
//     SCORE_L4, // Scoring in L4
//     SETUP_L4(SuperstructureState.SCORE_L4), // Setting up for scoring in L4
//     SCORE_L3, // Scoring in L3
//     SETUP_L3(SuperstructureState.SCORE_L3), // Setting up for scoring in L3
//     SCORE_L2, // Scoring in L2
//     SETUP_L2(SuperstructureState.SCORE_L2), // Setting up for scoring in L2
//     SCORE_L1, // Scoring in the trough
//     SETUP_L1(SuperstructureState.SCORE_L1), // Setting up the position to score in the trough
//     INTAKE,
//     SETUP_INTAKE(SuperstructureState.INTAKE),
//     STOW, // Going to the lowest position
//     ZERO; // Zero the motor

//     private Optional<SuperstructureState> nextState;

//     public SuperstructureState getNextState() {
//       // If it exists, just return the next state
//       if (nextState.isPresent()) return nextState.get();

//       // Default is to just return to stow
//       return STOW;
//     }

//     private SuperstructureState() {
//       this.nextState = Optional.empty();
//     }

//     private SuperstructureState(SuperstructureState nextState) {
//       this.nextState = Optional.of(nextState);
//     }
//   }

//   private SuperstructureState targetState = SuperstructureState.ZERO; // current target state

//   private final Elevator elevator;
//   private final Pivot pivot;

//   public Superstructure(Elevator elevator, Pivot pivot) {
//     this.elevator = elevator;
//     this.pivot = pivot;
//     pivot.setPositionTarget(PivotTarget.TOP);
//     elevator.setPositionTarget(ElevatorTarget.BOTTOM);
//   }

//   @Override
//   public void periodic() {
//     switch (targetState) { // switch on the target state
//       case SETUP_L1 -> { // go to the position before scoring
//         elevator.setPositionTarget(ElevatorTarget.L1);
//         pivot.setPositionTarget(PivotTarget.SETUP_L1);
//         if (superstructureReachedTarget()) {
//           setTargetState(SuperstructureState.SETUP_L1.getNextState());
//         }
//       }
//       case SCORE_L1 -> {
//         elevator.setPositionTarget(ElevatorTarget.L1);
//         pivot.setPositionTarget(PivotTarget.SCORE_L1);
//       }
//       case SETUP_L2 -> { // go to the position before scoring
//         elevator.setPositionTarget(ElevatorTarget.L2);
//         pivot.setPositionTarget(PivotTarget.SETUP_L2);
//         if (superstructureReachedTarget()) {
//           setTargetState(SuperstructureState.SETUP_L2.getNextState());
//         }
//       }
//       case SCORE_L2 -> {
//         elevator.setPositionTarget(ElevatorTarget.L2);
//         pivot.setPositionTarget(PivotTarget.SCORE_L2);
//       }
//       case SETUP_L3 -> { // go to the position before scoring
//         elevator.setPositionTarget(ElevatorTarget.L3);
//         pivot.setPositionTarget(PivotTarget.SETUP_L3);
//         if (superstructureReachedTarget()) {
//           setTargetState(SuperstructureState.SETUP_L3.getNextState());
//         }
//       }
//       case SCORE_L3 -> {
//         elevator.setPositionTarget(ElevatorTarget.L3);
//         pivot.setPositionTarget(PivotTarget.SCORE_L3);
//       }
//       case SETUP_L4 -> { // go to the position before scoring
//         elevator.setPositionTarget(ElevatorTarget.L4);
//         pivot.setPositionTarget(PivotTarget.SETUP_L4);
//         if (superstructureReachedTarget()) {
//           setTargetState(SuperstructureState.SETUP_L4.getNextState());
//         }
//       }
//       case SCORE_L4 -> {
//         elevator.setPositionTarget(ElevatorTarget.L4);
//         pivot.setPositionTarget(PivotTarget.SCORE_L4);
//       }
//       case STOW -> {
//         elevator.setPositionTarget(ElevatorTarget.BOTTOM);
//         pivot.setPositionTarget(PivotTarget.TOP);
//       }
//       case SETUP_INTAKE -> {
//         elevator.setPositionTarget(ElevatorTarget.SETUP_INTAKE);
//         pivot.setPositionTarget(PivotTarget.INTAKE);
//       }
//       case INTAKE -> {
//         elevator.setPositionTarget(ElevatorTarget.INTAKE);
//         pivot.setPositionTarget(PivotTarget.INTAKE);
//       }
//       case ZERO -> {
//         if (notZeroing()) { // set our mechanisms to zero if they aren't already
//           elevator.setControlMode(ControlMode.ZERO);
//           pivot.setControlMode(ControlMode.ZERO);
//         } else { // if our mechanisms are currently zeroing run this logic
//           if (true) { // check if the elevator is done zeroing and set
//             // offsets accordingly
//             elevator.setOffset();
//             elevator.setControlMode(ControlMode.POSITION);
//           }
//           if (pivot.getFilteredSupplyCurrentAmps()
//               > PivotConstants
//                   .ZEROING_VOLTAGE_THRESHOLD) { // check if pivot is done zeroing and set offsets
//             // accordingly
//             pivot.setOffset();
//             pivot.setControlMode(ControlMode.POSITION);
//           }
//           if (notZeroing()) { // if both of our mechanisms aren't zeroing anymore, exit out of
// this
//             // madness
//             setTargetState(SuperstructureState.STOW);
//           }
//         }
//       }
//       case STOP -> {
//         elevator.setControlMode(ControlMode.STOP);
//         pivot.setControlMode(ControlMode.STOP);
//       }
//     }
//     elevator.periodic();
//     pivot.periodic();

//     Logger.recordOutput("Superstructure/TargetState", targetState);
//     Logger.recordOutput("Superstructure/Elevator reached target", elevator.reachedTarget());
//     Logger.recordOutput("Superstructure/Pivot reached target", pivot.reachedTarget());
//     Logger.recordOutput("Superstructure/Reached Target", superstructureReachedTarget());
//   }

//   // Target state getter and setter
//   public void setTargetState(SuperstructureState superstructureState) {
//     targetState = superstructureState;
//   }

//   public SuperstructureState getTargetState() {
//     return targetState;
//   }

//   /**
//    * Get the position of the elevator
//    *
//    * @return the position of the elevator
//    */
//   public double getElevatorPosition() {
//     return elevator.position();
//   }

//   /**
//    * Get the position of the pivot
//    *
//    * @return the position of the pivot
//    */
//   public double getPivotPosition() {
//     return pivot.position();
//   }

//   /**
//    * Get the supply current of the elevator
//    *
//    * @return the supply current of the elevator
//    */
//   public double getElevatorSupplyCurrentAmps() {
//     return elevator.getSupplyCurrentAmps();
//   }
//   /**
//    * Get the supply current of the pivot
//    *
//    * @return the supply current of the pivot
//    */
//   public double getPivotSupplyCurrentAmps() {
//     return pivot.getSupplyCurrentAmps();
//   }

//   /**
//    * @return a boolean that says weather or not both of our mechanisms have finished zeroing
//    */
//   public boolean notZeroing() {
//     return elevator.getControlMode() != ControlMode.ZERO
//         && pivot.getControlMode() != ControlMode.ZERO;
//   }

//   /**
//    * @return if both subsystems in the superstructure have reached their target
//    */
//   public boolean superstructureReachedTarget() {
//     return elevator.reachedTarget() && pivot.reachedTarget();
//   }

//   public Command initiateScoringSequence(SuperstructureState superstructureState) {
//     return new FunctionalCommand(
//         () -> this.setTargetState(superstructureState),
//         () -> {
//           // Execute logic here (if any)
//         },
//         interrupted -> {
//           // End logic here (if any)
//         },
//         () -> {
//           // Ends when we've transitioned to the next state (the score state)
//           // AND we've reached the target (we're ready to place)
//           return this.targetState == superstructureState.getNextState()
//               && this.superstructureReachedTarget();
//         },
//         this);
//   }
// }
