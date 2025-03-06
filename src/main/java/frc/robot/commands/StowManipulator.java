// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowManipulator extends SequentialCommandGroup {
  /** Creates a new StowManipulator. */
  public StowManipulator(ManipulatorSubsystem manipulatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addRequirements(manipulatorSubsystem, intakeSubsystem);

    //Parallel:
      //Move Intake
      //Sequence:
        //Ensure elevator is above safe zone
        //Move Shoulder down
    //Move elevator down
    //Move intake back

    addCommands();
  }
}
