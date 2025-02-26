// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public enum CommandSelector {
    centerL1Auto,
    centerL4Auto,
  }

  public static HashMap<CommandSelector, Supplier<Command>> commandHashMap = new HashMap<>(
    Map.of(
      CommandSelector.centerL1Auto, () -> Coral1Auto(),
      CommandSelector.centerL4Auto, () -> Coral4Auto()
    )
  );

  public static SwerveSubsystem robotSwerveSubsystem;
  public static void setSwerveSubsystem(SwerveSubsystem subsystem) {
    robotSwerveSubsystem = subsystem;
  }

  public static Command Coral1Auto() {
    return new SequentialCommandGroup(
      
    );
  }

  public static Command Coral4Auto() {
    return new SequentialCommandGroup(
      
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
