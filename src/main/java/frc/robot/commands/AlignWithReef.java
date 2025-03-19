// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefSide;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithReef extends Command {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final VisionSubsystem m_VisionSubsystem;
  private final ReefSide kSide;

  SwerveControllerCommand controllerCommand;

  /** Creates a new AlignWithReef. */
  public AlignWithReef(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, ReefSide side) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);


    m_SwerveSubsystem = swerveSubsystem;
    m_VisionSubsystem = visionSubsystem;
    kSide = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        m_SwerveSubsystem.getPose(),
        m_VisionSubsystem.getClosestReefPose(m_SwerveSubsystem.getPose(), kSide)
      ), 
      Constants.kReefAlignmentTrajectoryConfig
    );

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    controllerCommand = new SwerveControllerCommand(
      trajectory, 
      m_SwerveSubsystem::getPose, 
      WheelConstants.kDriveKinematics, 
      xController,
      yController,
      thetaController,
      m_SwerveSubsystem::setModuleStates,
      m_SwerveSubsystem
    );
    controllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
    controllerCommand = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}
