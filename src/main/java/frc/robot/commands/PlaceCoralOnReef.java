// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralOnReef extends SequentialCommandGroup {
  /** Creates a new PlaceCoralOnReef. */
  public PlaceCoralOnReef(ManipulatorSubsystem manipulatorSubsystem, IntakeSubsystem intakeSubsystem, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(manipulatorSubsystem, intakeSubsystem);

    BackUpCommand backUpCommand = new BackUpCommand(swerveSubsystem);
    addCommands(
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(0.5),
          new ReleaseCoral(manipulatorSubsystem)
        ),
        new InstantCommand(() -> {
          System.out.println("STARTNG BACK UP");
          backUpCommand.schedule();
        }),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new WaitUntilCommand(() -> {
            System.out.println(!backUpCommand.isScheduled());
            return !backUpCommand.isScheduled();
          })
        ),
        new StowManipulator(manipulatorSubsystem, intakeSubsystem)
      ).onlyIf(() -> !manipulatorSubsystem.getStowedState())
    );
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
