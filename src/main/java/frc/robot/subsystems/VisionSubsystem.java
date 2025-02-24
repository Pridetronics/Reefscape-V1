// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimeLightHelpers.LimelightResults;
import frc.robot.LimeLightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera(CameraConstants.kCameraName);
  private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    fieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    CameraConstants.kRobotToCamera
  );
  // private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
  //   fieldLayout, 
  //   PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
  //   camera, 
  //   CameraConstants.kRobotToCamera
  // );
  private Optional<EstimatedRobotPose> currentRobotPose = Optional.empty();
  private boolean currentlyLookingAtAprilTag = false;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    //Done so the camera and camera settings can be viewed at "<HOSTNAME>.local:5800" on google when tethered
    //            ^^^replace <HOSTNAME> with the constants variable for the hostname
    PortForwarder.add(5800, CameraConstants.kHostName + ".local", 5800);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult targetData = camera.getLatestResult();
    Optional<EstimatedRobotPose> robotPose = poseEstimator.update(targetData);
    if (robotPose.isPresent()) currentRobotPose = robotPose;
    currentlyLookingAtAprilTag = robotPose.isPresent();

    
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return currentRobotPose;
  }
  
  public boolean lookingAtAprilTag() {
    return currentlyLookingAtAprilTag;
  }

  public Optional<LimelightTarget_Detector[]> getDetectorCameraOffsets() {
    LimelightResults results = LimeLightHelpers.getLatestResults(CameraConstants.kDetectorCameraName);
    LimelightTarget_Detector[] detectedResults = results.targets_Detector;

    if (detectedResults.length > 0) 
      return Optional.of(detectedResults); 
    else 
      return Optional.empty();
  }
}