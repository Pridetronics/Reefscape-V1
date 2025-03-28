// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectCoralUntilStopped extends Command {
  ManipulatorSubsystem m_ManipulatorSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  /** Creates a new CollectCoralUntilStopped. */
  public CollectCoralUntilStopped(ManipulatorSubsystem manipulatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_IntakeSubsystem = intakeSubsystem;
    m_ManipulatorSubsystem = manipulatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ManipulatorSubsystem.clawGrab();
    m_IntakeSubsystem.startIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ManipulatorSubsystem.stopClaw();
    m_IntakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
