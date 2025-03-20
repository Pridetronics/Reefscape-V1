// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.VisionSubsystem.ReefSide;

/** Add your docs here. */
public class AutoPosePosition {

    private static Boolean swapFieldSides = false;
    public static void setFieldSideSwap(Boolean value) {
      swapFieldSides = value;
    }
    public static ReefSide setReefSideInverted(ReefSide side) {
        ReefSide firstInvert = side;
        if (swapFieldSides) {
            if (side == ReefSide.Left) {
                firstInvert = ReefSide.Right;
            } else if (side == ReefSide.Right) {
                firstInvert = ReefSide.Left;
            }
        }
        ReefSide secondInvert = firstInvert;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (firstInvert == ReefSide.Left) {
                secondInvert = ReefSide.Right;
            } else if (firstInvert == ReefSide.Right) {
                secondInvert = ReefSide.Left;
            }
        }
        return secondInvert;
    } 

    private final Pose2d position;
    private final Boolean ignoreFieldSideSwapValue;
    
    public AutoPosePosition(Pose2d pose) {
        position = pose;
        ignoreFieldSideSwapValue = false;
    }

    public AutoPosePosition(Pose2d pose, Boolean ignoreFieldSideSwap) {
        position = pose;
        ignoreFieldSideSwapValue = ignoreFieldSideSwap;
    }

    public Pose2d getPose() {
        Pose2d teamOrientedPose = TrajectoryHelper.toAllianceRelativePosition(position);
        if (ignoreFieldSideSwapValue) {
            System.out.println("POSITION INIT "+teamOrientedPose.toString());
            return teamOrientedPose;
        }
        Pose2d fieldSideOrientedPose = TrajectoryHelper.setFieldSidePosition(teamOrientedPose, swapFieldSides);
        return fieldSideOrientedPose;
    }

    public Translation2d getPosition() {
        Translation2d teamOrientedPosition = TrajectoryHelper.toAllianceRelativePosition(position.getTranslation());
        if (ignoreFieldSideSwapValue) {
            return teamOrientedPosition;
        }
        Translation2d fieldSideOrientedPosition = TrajectoryHelper.setFieldSidePosition(teamOrientedPosition, swapFieldSides);
        return fieldSideOrientedPosition;
    }

    public Rotation2d getRotation() {
        Pose2d teamOrientedPose = TrajectoryHelper.toAllianceRelativePosition(position);
        if (ignoreFieldSideSwapValue) {
            return teamOrientedPose.getRotation();
        }
        Pose2d fieldSideOrientedPose = TrajectoryHelper.setFieldSidePosition(teamOrientedPose, swapFieldSides);
        return fieldSideOrientedPose.getRotation();
    }
}
