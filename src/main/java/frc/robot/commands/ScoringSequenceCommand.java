// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.Pivot.PivotTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringSequenceCommand extends SequentialCommandGroup {
  /** Creates a new ScoringSequenceCommand. */
  public ScoringSequenceCommand(
      Elevator elevator,
      Pivot pivot,
      Rollers rollers,
      ElevatorTarget elevatorTarget,
      PivotTarget pivotSetupTarget,
      PivotTarget pivotScoreTarget) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            elevator.goToPositionCommand(elevatorTarget),
            pivot.goToPositionCommand(pivotSetupTarget)),
        pivot.goToPositionCommand(pivotScoreTarget),
        new WaitCommand(1),
        rollers.setTargetCommand(RollerState.EJECT));
  }
}
