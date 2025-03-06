// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeJoystickControl extends Command {
  IntakeSubsystem intakeSubsystem;

  Supplier<Double> intakeAngleJoystick;

  GenericEntry intakeRelativeEntry = Shuffleboard.getTab("Test Data").add("Intake Relative Angle", 0).getEntry();
  GenericEntry intakeAbsoluteEntry = Shuffleboard.getTab("Test Data").add("Intake Absolute Angle", 0).getEntry();

  /** Creates a new IntakeJoystickControl. */
  public IntakeJoystickControl(IntakeSubsystem intakeSubsystem, Supplier<Double> intakeAngleJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.intakeAngleJoystick = intakeAngleJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakeJoystickValue = intakeAngleJoystick.get();
    if (Math.abs(intakeJoystickValue) <= 0.01) intakeJoystickValue = 0.000;
    double intakeTargetIncrement = intakeJoystickValue * 30;
    intakeSubsystem.setIntakeAngle(intakeSubsystem.getIntakeAngle() + intakeTargetIncrement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
