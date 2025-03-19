// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
// import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimeLightHelpers.LimelightResults;
import frc.robot.LimeLightHelpers.LimelightTarget_Detector;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera(CameraConstants.kCameraName);
  private AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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

  private ArrayList<Pose2d> reefTargetingPositions = new ArrayList<>();

  public enum ReefSide {
    Left,
    Right
  }

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    //Done so the camera and camera settings can be viewed at "<HOSTNAME>.local:5800" on google when tethered
    //            ^^^replace <HOSTNAME> with the constants variable for the hostname
    PortForwarder.add(5800, CameraConstants.kHostName + ".local", 5800);


    int[] reefAprilTagIds = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    for (int id : reefAprilTagIds) {
      Optional<Pose3d> aprilTagPose = fieldLayout.getTagPose(id);
      if (aprilTagPose.isPresent()) {
        Pose2d pose = aprilTagPose.get().toPose2d().transformBy(
          new Transform2d(
            new Translation2d(Units.inchesToMeters(20), 0), 
            Rotation2d.k180deg
          )
        );
        reefTargetingPositions.add(pose);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult targetData = camera.getLatestResult();
    Optional<EstimatedRobotPose> robotPose = poseEstimator.update(targetData);
    currentRobotPose = robotPose;
    currentlyLookingAtAprilTag = robotPose.isPresent();
    
  }

  public Pose2d getClosestReefPose(Pose2d robotPosition, ReefSide side) {
    Pose2d closestPose = null;
    double closestPoseDistance = 0;
    for (Pose2d pose : reefTargetingPositions) {
      double distance = robotPosition.getTranslation().getDistance(pose.getTranslation());
      if (closestPose == null || distance < closestPoseDistance) {
        closestPose = pose;
        closestPoseDistance = distance;
      }
    }
    return closestPose.transformBy(
      new Transform2d(
        new Translation2d(
          0,
          Units.inchesToMeters(side == ReefSide.Left ? AutoConstants.kReefPipeWidth/2 : -AutoConstants.kReefPipeWidth/2)
        ),
        Rotation2d.kZero
      )
    );
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return currentRobotPose;
  }
  
  public boolean lookingAtAprilTag() {
    return currentlyLookingAtAprilTag;
  }

  private Optional<LimelightTarget_Detector[]> getDetectedCoralTargets() {
    LimelightResults results = LimeLightHelpers.getLatestResults(CameraConstants.kDetectorCameraName);
    LimelightTarget_Detector[] detectedResults = results.targets_Detector;

    if (detectedResults.length > 0) 
      return Optional.of(detectedResults); 
    else 
      return Optional.empty();
  }

  public Optional<Translation3d[]> getAllDetectedCoralOffsets() {
    Optional<LimelightTarget_Detector[]> coralsDetected = getDetectedCoralTargets();
    ArrayList<Translation3d> coralOffsets = new ArrayList<>();

    if (coralsDetected.isEmpty()) {
      return Optional.empty();
    }

    //Converts detected corals from angle offsets of the camera to 3d positions of the robot
    for (LimelightTarget_Detector detectedResult : coralsDetected.get()) {
      if (detectedResult.confidence < CameraConstants.kConfidenceThreshold) continue;

      double angleXDegrees = detectedResult.tx;
      double angleYDegrees = -detectedResult.ty-CameraConstants.kDetectorYawDegrees;

      //Parametric equation resulting in the final position
      Translation3d cameraToCoral = new Translation3d(
        1,
        -Math.tan(angleXDegrees),
        Math.tan(angleYDegrees)
      ).times( -CameraConstants.kDetectorRobotToCamera.getZ() / Math.tan(angleYDegrees) );

      //Add offsets for global offset
      Translation3d coralPosition = CameraConstants.kDetectorRobotToCamera.plus(cameraToCoral);

      //Append final position to array
      coralOffsets.add(coralPosition);
    }

    if (coralOffsets.size() == 0) return Optional.empty();

    return Optional.of(coralOffsets.toArray( new Translation3d[coralOffsets.size()] ));
  }

  public Optional<Translation3d> getClosestCoralOffsetByDistance() {
    Optional<Translation3d[]> detectedCoralOffsets = getAllDetectedCoralOffsets();

    if (detectedCoralOffsets.isEmpty()) return Optional.empty();

    Translation3d closestOffset = null;
    for (Translation3d offset : detectedCoralOffsets.get()) {
      if (closestOffset == null || offset.getDistance(Translation3d.kZero) < closestOffset.getDistance(Translation3d.kZero)) {
        closestOffset = offset;
      }
    }
    return Optional.of(closestOffset);
  }

  public Optional<Translation3d> getClosestCoralOffsetByXAxisValue() {
    Optional<Translation3d[]> detectedCoralOffsets = getAllDetectedCoralOffsets();

    if (detectedCoralOffsets.isEmpty()) return Optional.empty();

    Translation3d closestOffset = null;
    for (Translation3d offset : detectedCoralOffsets.get()) {
      if (closestOffset == null || offset.getX() < closestOffset.getX()) {
        closestOffset = offset;
      }
    }
    return Optional.of(closestOffset);
  }
}