// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.commands.TrackCoralWithCamera;
import frc.robot.commands.UnstowIntake;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.PositionClawAtHeight;
import frc.robot.commands.RetrieveCoral;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public static enum CommandSelector {
    centerL1Auto,
    centerL4Auto,
  }

  public static final AutoPosePosition coralStationPosition = new AutoPosePosition(new Pose2d(
    2.71,
    6.59,
    Rotation2d.fromDegrees(156)
  ));

  public static final AutoPosePosition aroundReefManuverPosition = new AutoPosePosition(new Pose2d(
    6.11,
    5.32,
    Rotation2d.fromDegrees(180)
  ));

  public static class coralPositions {
    public static final AutoPosePosition bargeSide1 = new AutoPosePosition(new Pose2d(
      5.89,
      4.17,
      Rotation2d.fromDegrees(180)
    ));
    public static final AutoPosePosition bargeSide2 = new AutoPosePosition(new Pose2d(
      5.36,
      5.19,
      Rotation2d.fromDegrees(-120)
    ));
    public static final AutoPosePosition bargeSide3 = new AutoPosePosition(new Pose2d(
      5.05,
      5.32,
      Rotation2d.fromDegrees(-120)
    ));
    public static final AutoPosePosition stationSide3 = new AutoPosePosition(new Pose2d(
      3.94,
      5.34,
      Rotation2d.fromDegrees(-60)
    ));
    public static final AutoPosePosition stationSide2 = new AutoPosePosition(new Pose2d(
      3.65,
      5.14,
      Rotation2d.fromDegrees(-60)
    ));
    public static final AutoPosePosition stationSide1 = new AutoPosePosition(new Pose2d(
      3.10,
      4.19,
      Rotation2d.fromDegrees(0)
    ));
  }

  public static final HashMap<String, AutoPosePosition> startingPositions = new HashMap<>(
    Map.of(
      "Scoring side", new AutoPosePosition(
        new Pose2d(
          0, 
          0, 
          Rotation2d.fromDegrees(180)
        ),
        true
      ),
      "Center side", new AutoPosePosition(
        new Pose2d(
          0, 
          0, 
          Rotation2d.fromDegrees(180)
        ),
        true
      ),
      "Audience side", new AutoPosePosition(
        new Pose2d(
          0, 
          0, 
          Rotation2d.fromDegrees(180)
        ),
        true
      )
    ) 
  );

  public static final HashMap<CommandSelector, BiFunction<Pose2d, RobotContainer, Command>> commandHashMap = new HashMap<>(
    Map.of(
      CommandSelector.centerL1Auto, Autos::Coral1Auto,
      CommandSelector.centerL4Auto, Autos::Coral4Auto
    )
  );



  private static Command Coral1Auto(Pose2d initialPose, RobotContainer robotContainer) {
    // //Controllers to keep the robot on the main path since it will not follow it too well without it
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    Trajectory startToReefTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        initialPose,
        coralPositions.bargeSide1.getPose()
      ),
      Constants.kTrajectoryConfig
    );

    SwerveControllerCommand startToReef = new SwerveControllerCommand(
      startToReefTrajectory, 
      robotContainer.swerveSubsystem::getPose, 
      WheelConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      robotContainer.swerveSubsystem::setModuleStates,
      robotContainer.swerveSubsystem
    );

    Trajectory firstReefToStationTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        coralPositions.bargeSide1.getPose(),
        aroundReefManuverPosition.getPose(),
        coralStationPosition.getPose()
      ), 
      Constants.kTrajectoryConfig.setEndVelocity(DriveConstants.kCameraCoralTrackingSpeedMetersPerSecond)
    );

    SwerveControllerCommand firstReefToStation = new SwerveControllerCommand(
      firstReefToStationTrajectory, 
      robotContainer.swerveSubsystem::getPose, 
      WheelConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      robotContainer.swerveSubsystem::setModuleStates,
      robotContainer.swerveSubsystem
    );

    Trajectory firstStationToReefTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        coralStationPosition.getPose(),
        coralPositions.stationSide2.getPose()
      ), 
      Constants.kTrajectoryConfig
    );

    SwerveControllerCommand firstStationToReef = new SwerveControllerCommand(
      firstStationToReefTrajectory, 
      robotContainer.swerveSubsystem::getPose, 
      WheelConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      robotContainer.swerveSubsystem::setModuleStates,
      robotContainer.swerveSubsystem
    );

    return new SequentialCommandGroup(
      //Drive to reef,
      new ParallelCommandGroup(
        startToReef,
        new PositionClawAtHeight(
          ManipulatorSubsystem.ClawHeightLevel.Level3, 
          robotContainer.intakeSubsystem, 
          robotContainer.manipulatorSubsystem
        )
      ),
      //place coral
      new PlaceCoral(robotContainer.manipulatorSubsystem),
      //drive to coral station
      new ParallelCommandGroup(
        firstReefToStation,
        new SequentialCommandGroup(
          new WaitCommand(1),
          new UnstowIntake()
        )
      ),
      //pick up with AI
      new ParallelDeadlineGroup(
        new RetrieveCoral(),
        new UnstowIntake(),
        new TrackCoralWithCamera(robotContainer.swerveSubsystem, robotContainer.visionSubsystem)
      ),
      //drive to reef
      new ParallelCommandGroup(
        firstStationToReef,
        new PositionClawAtHeight(
          ManipulatorSubsystem.ClawHeightLevel.Level3, 
          robotContainer.intakeSubsystem, 
          robotContainer.manipulatorSubsystem
        )
      ),
      //place coral
      new PlaceCoral(robotContainer.manipulatorSubsystem),
    );
  }

  private static Command Coral4Auto(Pose2d initialPose, RobotContainer robotContainer) {
    return new SequentialCommandGroup(
      
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
