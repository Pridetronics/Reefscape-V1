// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically imCANID this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class shuffleboardConstants {
    public static final double kRateLimitTime = 1;
  }

  //Constants for features related to user controller input
  public static class IOConstants {
    //Deadband for the joysticks (for driving inputs so the robot doesnt move when not touching controller)
    public static final double kDeadband = 0.1;
    //Identifier for the driver's controller
    public static final int kDriveJoystickID = 0;
    //Identifier for the manipulator's controller
    public static final int kManipulatorJoystickID = 1;
    
    //Button ID for reseting the orientation of the robot to the forward direction of the robot
    public static final int kZeroHeadingBtnID = 2;

    //Axis for right/left movement
    public static final int kDriveJoystickXAxis = 1;
    //Axis for forward/backward movement
    public static final int kDriveJoystickYAxis = 0;
    //Axis for turning
    public static final int kDriveJoystickTurningAxis = 4;
    //Button ID for robot oriented drive (when holding)
    public static final int kDriveFieldOrientedDriveBtnID = 16;
    //Time it takes before you can press the zero heading button again (seconds)
    public static final double kZeroHeadingDebounceTime = 2;
  }

  //Constants for data related to the wheels of each module (not the module itself)
  public static class WheelConstants {

    //Diameter of the wheel
    public static final double kSwerveWheelDiameterMeters = Units.inchesToMeters(4);

    //Gear ratio of the propultion motor (number of wheel rotations per motor rotation)
    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    //Gear ratio of the turning motor (number of wheel rotations per motor rotation)
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    //The distance traveled, in meters, per rotation of the wheel
    public static final double kDistancePerWheelRotation = kSwerveWheelDiameterMeters*Math.PI;
    //Just because im lazy
    public static final double k360DegreesToRadians = 2*Math.PI;

    //Power value in the PIDController that controls the wheel direction
    public static final double kPTurning = 0.8;

    
    //Distance between the right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(23);
    //DIstance between the front and back wheels
    public static final double kWheelBaseLength = Units.inchesToMeters(23);

    //Kinematics system that solves for each wheel's direction based on the given target direction ahd turn velocity
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBaseLength/2, kTrackWidth/2), //Front Left
      new Translation2d(kWheelBaseLength/2, -kTrackWidth/2), //Front Right
      new Translation2d(-kWheelBaseLength/2, kTrackWidth/2), //Back Left
      new Translation2d(-kWheelBaseLength/2, -kTrackWidth/2) //Beck Right
    );

  }

  //Constants for the movement of the robot
  public static class DriveConstants {

    public static final double kFieldWidthMeters = Units.inchesToMeters(653.2);


    //The literal max speed each wheel is allowed to go
    public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(15.1);

    /*
    When talking about these acceleration values, 
    these values influence the rate of change in a number in such a way that the number being influenced 
    is multiplied by the max speed of the robot so that the influeced number being 0 means no speed, 1 means 
    100% speed, and -1 mean -100% speed
      -A value of 1 means the 0 to max time is 1 second
      -A value of 3 means the 0 to max time is 0.333333 seconds
      -A value of 0.5 means the 0 to max time is 2 seconds
      -A value 0f 0.2 means the 0 to max time is 5 seconds
    You get the idea, the number is the max change in velocity, as a percent of the robot's full speed
    That means that the number is inversley related t0 the 0 to max time
   */
    public static final double kTeleMaxDriveAccelerationUnitsPerSecond = 5;
    public static final double kTeleMaxTurningAccelerationUnitsPerSecond = 5;

    //Max speed of the robot itself
    public static final double kTeleMaxDriveSpeedMetersPerSecond = 3;
    //Max turning speed of the robot specified in degrees but converted to radians (with the "(Math.PI/180)")
    public static final double kTeleMaxTurningSpeedRadiansPerSecond = 360 * (Math.PI/180);

    //CAN ID for the gyro needed for swerve drive
    public static final int gyroCANID = 13;

    //ID of the Can Spark Max the propels the swerve module wheel
    public static final int kFrontLeftDriveMotorCANID = 4;
    //ID of the Can Spark Max that turns the swerve module wheel
    public static final int kFrontLeftTurningMotorCANID = 5;
    //Whether the turning direction of the wheel is reversed
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    //Whether the propultion direction of the wheel is reversed
    public static final boolean kFrontLeftDriveEncoderReversed = true;
    //ID of the CTRE CANCoder for getting the absolute position of the wheel 
      //(the encoder is the silly little device on top of the module that is wedged bwtween the two motors)
    public static final int kFrontLeftDriveAbsoluteEncoderCANID = 6;
    //Whether the CANCoder direction is reversed
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    /* Offset of the absolute encoder relative to the forward direction of the wheel
          My advice for setting this value to prevent a headache experience:
            1. Face all wheels in the forward direction of the robot, being as close to perfect as you can
            2. Open the Phoenix Tuner X, connect laptop to RoboRio via USB cable (Not by ethenet cable)
            3. Go through each encoder on the device list, press refresh to get the data for it, and get the current rotation of the encoder in degrees
              - The data you see when pressing refresh will include the absolute position data (NOT THE ONE CALLED "POSITION"; ITS CALLED "ABSOLUTE POSITION"), you will use the Absolute position (without sensor/magnet offset)
              - The number will be a number between 0 and 1 (NOT A NEGATIVE NUMBER)
              - Multiply that number by 360 to convert it to degrees
            4. Take the number and simply just place it below! it will subtract that number from the encoder to make the wheel's forward direction always face forward on the robot

      (make sure the value's units are in degrees, not rotations or radians )
       Your welcome! -Guy who made this program   
    */
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 36.91;

    //Everything above applies for the below modules
    
    public static final int kFrontRightDriveMotorCANID = 1;
    public static final int kFrontRightTurningMotorCANID = 2;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderCANID = 3;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = -10.01;


    public static final int kBackRightDriveMotorCANID = 10;
    public static final int kBackRightTurningMotorCANID = 11;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderCANID = 12;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
    public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = -35.50;

    public static final int kBackLeftDriveMotorCANID = 7;
    public static final int kBackLeftTurningMotorCANID = 8;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final int kBackLeftDriveAbsoluteEncoderCANID = 9;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 163.65; 
  }

  // Constants for our intake
  public static class IntakeConstants {
    // Set the motor ID for the motor that spins the Intake itself
    public static final int kIntakeMotorCanID = 14; 
    // Set the motor ID for the motor that controls the angle of the Intake
    public static final int kIntakeAngleMotorCanID = 15; 

    public static final int kIntakeRPM = 50;
    public static final double kIntakeGearRatio = 1.0/9;
    public static final boolean kIntakeReversed = false;


    public static final double kIntakeAngleGearRatio = 1.0/36;
    public static final boolean kIntakeAngleMotorReversed = false;
    public static final double kIntakeAbsoluteEncoderOffsetDegrees = 3;

    // Proportional, Integral, and Derivitive values for the angle PID controller
    public static final double kIntakeAnglePValue = 0.03;
    public static final double kIntakeAngleIValue = 0;
    public static final double kIntakeAngleDValue = 0;

    // Proportional, Integral, and Derivitive values for the velocity PID controller
    public static final double kIntakePValue = 0.037;
    public static final double kIntakeIValue = 0;
    public static final double kIntakeDValue = 0;

    public static final double kIntakeAngleMaxVelocityDegreesPerSecond = 15;
    public static final double kIntakeAngleMaxAccelerationDegreesPerSecondSquared = 30;

    public static final double kIntakeUnStowAngle = -30;
    public static final double kIntakeUsageAngle = -60;
    public static final double kIntakeStowAngle = -15;

    
    public static final double kIntakeAngleFuzzyEqDegrees = 5;

  }

  // Constants for manipulators (not including intake)
  public static class ManipulatorConstants {
    // Set the motor ID for the motor that shoots in the claw
    public static final int kClawMotorID = 16; 
    // Set the motor ID for the motor that controls the elevator
    public static final int kElevatorMotorID = 17; 
    // Set the motor ID for our shoulder motor
    public static final int kShoulderMotorID = 18;
    // Set the shoulder CAN encoder ID
    public static final int kShoulderEncoderID = 19;

    // Inverted elevator
    public static final boolean kShoulderEncoderReversed = true;
    // Value for position and velocity conversion for shoulder
    public static final double kShoulderGearRatio = 1.0/25 * 15.00/54;
    public static final double kShoulderEncoderOffsetDegrees = -0.260742*360 - 54.462;


    // Value for position and velocity conversion for elevator
    public static final double kElevatorGearRatio = 1.0/25.0 * 0.375*13.0 * 2.0;

    public static final boolean kElevatorMotorReversed = false;

    public static final double kClawGearRatio = 1.0/16;

    public static final double kClawSpeedRPM = 5;

    public static final boolean kClawMotorReversed = false;

    // Proportional, Integral, and Derivitive values for the velocity PID controller
    public static final double kClawPValue = 0.038;
    public static final double kClawIValue = 0;
    public static final double kClawDValue = 0;
    // Proportional, Integral, and Derivitive values for the velocity PID controller
    public static final double kElevatorPValue = 0.04;
    public static final double kElevatorIValue = 0;
    public static final double kElevatorDValue = 0;
    public static final double kElevatorGValue = 0.027;
    public static final double kElevatorSValue = 0.05;
    // Proportional, Integral, and Derivitive values for the velocity PID controller
    public static final double kShoulderPValue = 0.04;
    public static final double kShoulderIValue = 0.00;
    public static final double kShoulderDValue = 0.00;
    public static final double kShoulderGValue = 0.03;
    public static final double kShoulderSValue = 0.00;
    public static final double kShoulderMaxVelocityDegreesPerSecond = 180;
    public static final double kShoulderMaxAccelerationDegreesPerSecondSquared = 360;

    //The threshold/region that the elevator will be considered at its target height
    public static final double kElevatorFuzzyEqInches = 2;
    public static final double kShoulderFuzzyEqDegrees = 5;

    public static final double kElevatorSpeedInchesPerSecond = 42;
    public static final double kElevatorAccelerationInchesPerSecondSquared = 66;

    // Elevator Heights
    // Trough (L1) Height
    public static final int kElevatorHeightL1Inches = 12;
    public static final int kShoulderAngleL1Degrees = -45;
    // L2 Height
    public static final int kElevatorHeightL2Inches = 12;
    public static final int kShoulderAngleL2Degrees = -10;
    // L3 Height
    public static final int kElevatorHeightL3Inches = 36;
    public static final int kShoulderAngleL3Degrees = 0;
    // L4 Height
    public static final int kElevatorHeightL4Inches = 60;
    public static final int kShoulderAngleL4Degrees = 0;
    // Max Height
    public static final int kElevatorMaxHeightInches = 60;
    public static final int kShoulderHigherLimitDegrees = 35;
    // Homing Height
    public static final double kElevatorHomingHeightInches = 6.5;
    
    public static final double kElevatorMinimumHeightInches = 7.5;
    public static final int kShoulderLowerLimitDegrees = -90;

    public static final int kElevatorStowHeightInches = 0;
    public static final int kElevatorSafeFromIntakeHeightInches = 12;
    public static final int kShoulderStowAngleDegrees = -100;
  }

  //Constants related to the autonomous period
  public static class AutoConstants {


    //Max speed during autonomous
    public static final double kMaxSpeedMetersPerSecond = 4;
    //Acceleration during autonomous (note its in meters, not units)
    public static final double kMaxAccelerationMetersPerSecond = 4;

    //Max turning speed during autonomous
    public static final double kMaxTurningSpeedRadiansPerSecond = 270 * (Math.PI / 180);
    //Acceleration during autonomous (note its in radians, not units)
    public static final double kMaxTurningAccelerationRadiansPerSecond = 360 * (Math.PI / 180);

    //Power Controllers for the robot to keep it on course
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    //This is for the Profiled PID Controller that controls the robot direction
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
      kMaxTurningSpeedRadiansPerSecond,
      kMaxTurningAccelerationRadiansPerSecond
    );
  }
  //Data for autonomous trajectories
  public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecond
  ).setKinematics(WheelConstants.kDriveKinematics);

  //Class that can store swerve module data for the swerve module class
  public static class SwerveModuleConstants {
      public final int kDriveMotorCANID;
      public final int kTurningMotorCANID;
      public final int kTurningEncoderID;
      public final double kAbsoluteEncoderOffsetDegrees;
      public final boolean kAbsoluteEncoderReversed;
      public final boolean kDriveEncoderReversed;
      public final boolean kTurningEncoderReversed;

      SwerveModuleConstants(
              int driveMotorCANID, 
              int turningMotorCANID, 
              int CTRETurningEncoderID,
              double absoluteEncoderOffsetDegrees,
              boolean absoluteEncoderReversed,
              boolean driveEncoderReversed,
              boolean turningEncoderReversed
          ) {

          kDriveMotorCANID = driveMotorCANID;
          kTurningMotorCANID = turningMotorCANID;
          kTurningEncoderID = CTRETurningEncoderID;
          kAbsoluteEncoderOffsetDegrees = absoluteEncoderOffsetDegrees;// * (Math.PI/180);
          kAbsoluteEncoderReversed = absoluteEncoderReversed;
          kDriveEncoderReversed = driveEncoderReversed;
          kTurningEncoderReversed = turningEncoderReversed;
          
      }
  }

  //Class that stores swerve module constants fot the swerve module class
  public static class SwerveModuleClasses {

    public SwerveModuleConstants backLeft = new SwerveModuleConstants(
      DriveConstants.kBackLeftDriveMotorCANID, 
      DriveConstants.kBackLeftTurningMotorCANID, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderCANID, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed, 
      DriveConstants.kBackLeftDriveEncoderReversed, 
      DriveConstants.kBackLeftTurningEncoderReversed
    );

    public SwerveModuleConstants backRight = new SwerveModuleConstants(
      DriveConstants.kBackRightDriveMotorCANID, 
      DriveConstants.kBackRightTurningMotorCANID, 
      DriveConstants.kBackRightDriveAbsoluteEncoderCANID, 
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed, 
      DriveConstants.kBackRightDriveEncoderReversed, 
      DriveConstants.kBackRightTurningEncoderReversed
    );
    public SwerveModuleConstants frontLeft = new SwerveModuleConstants(
      DriveConstants.kFrontLeftDriveMotorCANID, 
      DriveConstants.kFrontLeftTurningMotorCANID, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderCANID, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, 
      DriveConstants.kFrontLeftDriveEncoderReversed, 
      DriveConstants.kFrontLeftTurningEncoderReversed
    );
    public SwerveModuleConstants frontRight = new SwerveModuleConstants(
      DriveConstants.kFrontRightDriveMotorCANID, 
      DriveConstants.kFrontRightTurningMotorCANID, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderCANID, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg, 
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed, 
      DriveConstants.kFrontRightDriveEncoderReversed, 
      DriveConstants.kFrontRightTurningEncoderReversed
    );
  }
}
