// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorTargetHeight extends InstantCommand {
  private ClawHeightLevel targetHeight;
  private ManipulatorSubsystem m_ManipulatorSubsystem;

  public SetElevatorTargetHeight(ClawHeightLevel heightLevel, ManipulatorSubsystem manipulatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetHeight = heightLevel;
    m_ManipulatorSubsystem = manipulatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
