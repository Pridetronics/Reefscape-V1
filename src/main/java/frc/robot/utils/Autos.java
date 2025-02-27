// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public static enum CommandSelector {
    centerL1Auto,
    centerL4Auto,
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

  public static final HashMap<CommandSelector, Function<Pose2d, Command>> commandHashMap = new HashMap<>(
    Map.of(
      CommandSelector.centerL1Auto, Autos::Coral1Auto,
      CommandSelector.centerL4Auto, Autos::Coral4Auto
    )
  );



  private static Command Coral1Auto(Pose2d initialPose) {
    return new SequentialCommandGroup(
      
    );
  }

  private static Command Coral4Auto(Pose2d initialPose) {
    return new SequentialCommandGroup(
      
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
