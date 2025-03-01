// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;
import frc.robot.utils.RunIfTrueCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionClawAtHeight extends SequentialCommandGroup {
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
  /** Creates a new PositionClawAtHeight. */
  public PositionClawAtHeight(ClawHeightLevel targetHeight, IntakeSubsystem m_IntakeSubsystem, ManipulatorSubsystem m_ManipulatorSubsystem) {

    
    

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
        }),
  
        new RunIfTrueCommand(new UnstowIntake(), () -> m_IntakeSubsystem.getStowedState() && m_ManipulatorSubsystem.getStowedState()),
  
        new ParallelCommandGroup(
          new InstantCommand(() -> {
            m_interruptBehavior = InterruptionBehavior.kCancelSelf;
          }),
          new SetClawTargetHeight(targetHeight, m_ManipulatorSubsystem),
          new WaitUntilCommand(() -> m_ManipulatorSubsystem.isClawAtPositionalHeight(targetHeight)),
          new SequentialCommandGroup(
            new WaitUntilCommand(() -> m_ManipulatorSubsystem.isClawOutOfWay()),
            new StowIntake(m_IntakeSubsystem)
          )
        )
      ).finallyDo((boolean interrupted) -> {
        if (interrupted) {

        }
      })

    );
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }
}
