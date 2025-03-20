// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectAlgaeHigh extends SequentialCommandGroup {
  /** Creates a new CollectAlgae. */
  public CollectAlgaeHigh(ManipulatorSubsystem manipulatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(manipulatorSubsystem, intakeSubsystem);
    addCommands(
      new InstantCommand(manipulatorSubsystem::clawGrab),
      new UnStowManipulator(manipulatorSubsystem, intakeSubsystem, ClawHeightLevel.Level3)
    );
  }
}
