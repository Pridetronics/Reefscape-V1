// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AutoPosePosition {
    private final Pose2d kPosition;
    
    public AutoPosePosition(Pose2d pose) {
        kPosition = pose;
    }

    public Pose2d getPose() {
        SendableChooser<Boolean> chooser = SmartDashboard.getData("Side of field scoring");
        //Pose2d teamOrientedPose = TrajectoryHelper.
        return kPosition;
    }

    public Translation2d getPosition() {
        return kPosition.getTranslation();
    }

    public Rotation2d getRotation() {
        return kPosition.getRotation();
    }
}
