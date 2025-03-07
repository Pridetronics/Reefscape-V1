// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnStowManipulator extends SequentialCommandGroup {
  /** Creates a new UnStowManipulator. */
  public UnStowManipulator(ManipulatorSubsystem manipulatorSubsystem, IntakeSubsystem intakeSubsystem, ClawHeightLevel targetState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addRequirements(manipulatorSubsystem, intakeSubsystem);

    //Move Intake (Only if stowed)
    //Move elevator to safe height
    //Parallel:
      //Move shoulder
      //Move Intake back
    //Adjust elevator  
    addCommands(
      new SequentialCommandGroup(

        new UnstowIntake(intakeSubsystem).onlyIf(manipulatorSubsystem::getStowedState),

        new ConditionalCommand(
  
          new ParallelCommandGroup(
            new MoveElevatorToTargetPosition(manipulatorSubsystem, targetState),
            new SequentialCommandGroup(
              new WaitUntilCommand(manipulatorSubsystem::isClawOutOfWay),
              new ParallelCommandGroup(
                new MoveShoulderToTargetPosition(manipulatorSubsystem, targetState),
                new StowIntake(intakeSubsystem)
              )
            )
          ),
          new SequentialCommandGroup(
            new MoveElevatorToTargetPosition(manipulatorSubsystem, ClawHeightLevel.ElevatorSafeHeight),
            new ParallelCommandGroup(
              new MoveShoulderToTargetPosition(manipulatorSubsystem, targetState),
              new StowIntake(intakeSubsystem)
            ),
            new MoveElevatorToTargetPosition(manipulatorSubsystem, targetState)
          ), 
          () -> manipulatorSubsystem.getElevatorHeightFromEnum(targetState) > manipulatorSubsystem.getElevatorHeightFromEnum(ClawHeightLevel.ElevatorSafeHeight)
  
        )

      )
    );
  }
}
