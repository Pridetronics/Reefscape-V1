// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.utils.ShuffleboardRateLimiter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorJoystickControl extends Command {
 
  private final ManipulatorSubsystem manipulatorSubsystem;
  Supplier<Double> elevatorJoystickSupplier;
  Supplier<Double> shoulderJoystickSupplier;

  boolean wasElevatorJoystickDeadBandActive = true;
  boolean wasShoulderJoystickDeadBandActive = true;

  GenericEntry elevatorHeightEntry = Shuffleboard.getTab("Test Data").add("Elevator Height", 0).getEntry();
  GenericEntry shoulderAngleEntry = Shuffleboard.getTab("Test Data").add("Shoulder Angle", 0).getEntry();
  GenericEntry shoulderAbsoluteAngleEntry = Shuffleboard.getTab("Test Data").add("Shoulder Absolute Angle", 0).getEntry();

  /** Creates a new ManipulatorJoystickControl. */
  public ManipulatorJoystickControl(ManipulatorSubsystem manipulatorSubsystem, Supplier<Double> elevatorJoystickSupplier, Supplier<Double> shoulderJoystickSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulatorSubsystem);
    this.manipulatorSubsystem = manipulatorSubsystem;
    this.elevatorJoystickSupplier = elevatorJoystickSupplier;
    this.shoulderJoystickSupplier = shoulderJoystickSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorJoystickValue = elevatorJoystickSupplier.get();
    if (Math.abs(elevatorJoystickValue) <= 0.05) {
      if (!wasElevatorJoystickDeadBandActive) {
        wasElevatorJoystickDeadBandActive = true;
        manipulatorSubsystem.elevatorHelper.setPosition(
          manipulatorSubsystem.elevatorHelper.getPosition() + manipulatorSubsystem.elevatorHelper.getVelocity()*0.25
        );
      }
    } else {
      wasElevatorJoystickDeadBandActive = false;
      double elevatorTargetIncrement = elevatorJoystickValue * 12;
      manipulatorSubsystem.elevatorHelper.setPosition(manipulatorSubsystem.elevatorHelper.getPosition() + elevatorTargetIncrement);  
    }

    double shoulderJoystickValue = shoulderJoystickSupplier.get();
    if (Math.abs(shoulderJoystickValue) < 0.05) {
      if (!wasShoulderJoystickDeadBandActive) {
        wasShoulderJoystickDeadBandActive = true;
        manipulatorSubsystem.shoulderHelper.setPosition(
          manipulatorSubsystem.shoulderHelper.getPosition() + manipulatorSubsystem.shoulderHelper.getVelocity() * 0.25
        );

      }
    } else {
      wasShoulderJoystickDeadBandActive = false;
      double shoulderTargetIncrement = shoulderJoystickValue * 300;
      manipulatorSubsystem.shoulderHelper.setPosition(manipulatorSubsystem.shoulderHelper.getPosition() + shoulderTargetIncrement);
      System.out.println(manipulatorSubsystem.shoulderHelper.getPosition());
      System.out.println(manipulatorSubsystem.shoulderHelper.getVelocity());
    };

    ShuffleboardRateLimiter.queueDataForShuffleboard(elevatorHeightEntry, manipulatorSubsystem.elevatorHelper.getPosition());
    ShuffleboardRateLimiter.queueDataForShuffleboard(shoulderAngleEntry, manipulatorSubsystem.shoulderHelper.getPosition());
    ShuffleboardRateLimiter.queueDataForShuffleboard(shoulderAbsoluteAngleEntry, manipulatorSubsystem.shoulderHelper.getAbsolutePosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
