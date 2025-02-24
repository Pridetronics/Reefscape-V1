// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WheelConstants;

public final class SwerveAutoPaths {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public static Trajectory TestPath = TrajectoryGenerator.generateTrajectory(
    List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 1, Rotation2d.fromDegrees(45)),
      new Pose2d(1, 0, Rotation2d.fromDegrees(90))
    ),
    new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecond
    ).setKinematics(WheelConstants.kDriveKinematics)
  );


  private SwerveAutoPaths() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
