// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveShoulderToTargetPosition extends Command {
  
  private final ManipulatorSubsystem m_ManipulatorSubsystem;
  private final ClawHeightLevel targetHeightLevel;

  /** Creates a new MoveShoulderToTargetPosition. */
  public MoveShoulderToTargetPosition(ManipulatorSubsystem manipulatorSubsystem, ClawHeightLevel targetHeightLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ManipulatorSubsystem = manipulatorSubsystem;
    this.targetHeightLevel = targetHeightLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SET SHOULDER TO "+m_ManipulatorSubsystem.getShoulderAngleFromEnum(targetHeightLevel));
    m_ManipulatorSubsystem.setShoulderPositionalState(targetHeightLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SHOULDER MOVED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ManipulatorSubsystem.isShoulderAtAngle(targetHeightLevel);
  }
}
