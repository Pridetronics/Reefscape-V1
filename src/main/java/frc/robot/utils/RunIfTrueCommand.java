// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIfTrueCommand extends Command {
  private Boolean condition;
  private Command m_command;

  /** Creates a new RunIfTrueCommand. */
  public RunIfTrueCommand(Command commandToRun, BooleanSupplier supplier) {
    condition = supplier.getAsBoolean();
    m_command = commandToRun;

    if (!condition) return;

    CommandScheduler.getInstance().registerComposedCommands(commandToRun);
    // copy the wrapped command's name
    setName(commandToRun.getName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (condition) {
      m_command.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (condition) {
      m_command.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (condition) {
      m_command.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !condition || m_command.isFinished();
  }

    /**
   * Specifies the set of subsystems used by this command. Two commands cannot use the same
   * subsystem at the same time. If the command is scheduled as interruptible and another command is
   * scheduled that shares a requirement, the command will be interrupted. Else, the command will
   * not be scheduled. If no subsystems are required, return an empty set.
   *
   * <p>Note: it is recommended that user implementations contain the requirements as a field, and
   * return that field here, rather than allocating a new set every time this is called.
   *
   * @return the set of subsystems that are required
   */
  @Override
  public Set<Subsystem> getRequirements() {
    return m_command.getRequirements();
  }

    /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   *
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return m_command.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_command.getInterruptionBehavior();
  }
}
