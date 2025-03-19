// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopCollectCoral extends SequentialCommandGroup {
  /** Creates a new StopCollectCoral. */
  public StopCollectCoral(ManipulatorSubsystem manipulatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addRequirements(manipulatorSubsystem, intakeSubsystem);

    addCommands(
      new InstantCommand(manipulatorSubsystem::clawGrab),
      new InstantCommand(intakeSubsystem::startIntake),
      new InstantCommand(),
      new ParallelCommandGroup(
        new MoveElevatorToTargetPosition(manipulatorSubsystem, ClawHeightLevel.CoralExtract),
        new MoveShoulderToTargetPosition(manipulatorSubsystem, ClawHeightLevel.CoralExtract)
      ),
      new ParallelCommandGroup(
        new MoveElevatorToTargetPosition(manipulatorSubsystem, ClawHeightLevel.Stow),
        new MoveShoulderToTargetPosition(manipulatorSubsystem, ClawHeightLevel.Stow)
      ),
      new InstantCommand(manipulatorSubsystem::stopClaw),
      new InstantCommand(intakeSubsystem::stopIntake),
      new StowIntake(intakeSubsystem)
    );
  }
  
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
