// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.Timer;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ShuffleboardRateLimiter;

public class FieldPositionUpdate extends Command {
  /** Creates a new FieldPositionUpdate. */
  VisionSubsystem m_VisionSubsystem;
  SwerveSubsystem m_SwerveSubsystem;
  private Field2d m_fieldTele = new Field2d();

  private Field2d m_fieldAuto = new Field2d();

  StructPublisher<Pose3d> camera3DPublisher = NetworkTableInstance.getDefault().getStructTopic("CameraPose", Pose3d.struct).publish();
  StructPublisher<Pose3d> fieldPositionPublisher = NetworkTableInstance.getDefault().getStructTopic("FieldPose", Pose3d.struct).publish();

  private final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  private final GenericEntry lookingAtAprilTag = teleOpTab.add("Looking at april Tag", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    private final GenericEntry useFieldUpdating  = teleOpTab.add("Disable Limelight", false)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .getEntry();

  public Timer timer = new Timer();
  public FieldPositionUpdate(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    m_VisionSubsystem = visionSubsystem;
    m_SwerveSubsystem = swerveSubsystem;


    teleOpTab.add(m_fieldTele);
    autoTab.add(m_fieldAuto);

    addRequirements(m_VisionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useFieldUpdating.getBoolean(false)) return;
    //Get the caculated robot position on the field from the last camera rendering cycle
    Optional<EstimatedRobotPose> robotPose = m_VisionSubsystem.getEstimatedPose();
    //Check if a field position was caculated last cycle
    if (robotPose.isPresent()) {
      //Update the swerve drive odometry to work with this position
      m_SwerveSubsystem.addVisionMeasurement(robotPose.get().estimatedPose.toPose2d(), robotPose.get().timestampSeconds);

      camera3DPublisher.set(robotPose.get().estimatedPose);
    }

    fieldPositionPublisher.set(new Pose3d(m_SwerveSubsystem.getPose()));

    ShuffleboardRateLimiter.queueDataForShuffleboard(lookingAtAprilTag, robotPose.isPresent());
    
    m_fieldTele.setRobotPose(m_SwerveSubsystem.getPose());
    m_fieldAuto.setRobotPose(m_SwerveSubsystem.getPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //Allows the command to run when disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // //Makes it so other commands do not cancel this command
  // @Override
  // public InterruptionBehavior getInterruptionBehavior() {
  //   return InterruptionBehavior.kCancelIncoming;
  // }
}