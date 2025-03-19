// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTrackCoralWithCamera extends Command {
  SwerveSubsystem m_SwerveSubsystem;
  VisionSubsystem m_VisionSubsystem;

  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecond);
  SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecond);
  SlewRateLimiter turningSpeedLimiter = new SlewRateLimiter(AutoConstants.kMaxTurningAccelerationRadiansPerSecond);

  /** Creates a new RetrieveCoralWithDetectorCamera. */
  public AutoTrackCoralWithCamera(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);

    m_SwerveSubsystem = swerveSubsystem;
    m_VisionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xSpeedLimiter.reset(DriveConstants.kCameraCoralTrackingDriveSpeedMetersPerMeter*1);
    ySpeedLimiter.reset(0);
    turningSpeedLimiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Translation3d> closestCoral = m_VisionSubsystem.getClosestCoralOffsetByDistance();
    if (closestCoral.isEmpty()) {
      xSpeedLimiter.calculate(0);
      ySpeedLimiter.calculate(0);
      turningSpeedLimiter.calculate(0);
    } else {
      Translation3d coralOffset = closestCoral.get();

      double angleRadians = Math.atan2(coralOffset.getY(), coralOffset.getX());
      double distance = coralOffset.getDistance(Translation3d.kZero);

      xSpeedLimiter.calculate(
        Math.min(
          Math.sin(angleRadians) * distance * DriveConstants.kCameraCoralTrackingDriveSpeedMetersPerMeter,
          AutoConstants.kMaxSpeedMetersPerSecond
        )
      );
      ySpeedLimiter.calculate(
        Math.min(
          Math.cos(angleRadians) * distance * DriveConstants.kCameraCoralTrackingDriveSpeedMetersPerMeter,
          AutoConstants.kMaxSpeedMetersPerSecond
        )
      );
      turningSpeedLimiter.calculate(angleRadians*DriveConstants.kCameraCoralTrackingTurnignSpeedRadiansPerRadian);
    }
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      xSpeedLimiter.lastValue(), 
      ySpeedLimiter.lastValue(), 
      turningSpeedLimiter.lastValue()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
