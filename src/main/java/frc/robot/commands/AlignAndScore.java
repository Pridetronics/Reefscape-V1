// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;
import frc.robot.subsystems.VisionSubsystem.ReefSide;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndScore extends SequentialCommandGroup {
  /** Creates a new AlignAndScore. */
  public AlignAndScore(
    SwerveSubsystem swerveSubsystem, 
    VisionSubsystem visionSubsystem,
    ManipulatorSubsystem manipulatorSubsystem, 
    IntakeSubsystem intakeSubsystem, 
    ReefSide side, 
    ClawHeightLevel heightLevel) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(manipulatorSubsystem, intakeSubsystem);

    AlignWithReef alignmentCommand = new AlignWithReef(swerveSubsystem, visionSubsystem, side);

    addCommands(
      new ParallelCommandGroup(
        new UnStowManipulator(manipulatorSubsystem, intakeSubsystem, heightLevel),
        new InstantCommand(() -> {
          alignmentCommand.schedule();
        }),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new WaitUntilCommand(() -> !alignmentCommand.isScheduled())
        )
      ),
      new PlaceCoralOnReef(manipulatorSubsystem, intakeSubsystem)
    );
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
