// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ShoulderHelper {

// Set new SparkMax motor for the elevator
  public final SparkMax shoulderMotor = new SparkMax(ManipulatorConstants.kShoulderMotorID, MotorType.kBrushless);

  // CAN Encoder for shoulder motor
  public final CANcoder absoluteEncoder = new CANcoder(ManipulatorConstants.kShoulderEncoderID);

// Relative encoder for the elevator motor
  public final RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();

// Set PID controller
  public final SparkClosedLoopController shoulderPIDController = shoulderMotor.getClosedLoopController();

  private double targetAngle;
  private boolean targetAngleSet = false;

  public ShoulderHelper() {

    // Configurating elevator
    SparkMaxConfig shoulderConfig = new SparkMaxConfig();
    
    shoulderConfig
    // Set what we are configurating
    .inverted(ManipulatorConstants.kShoulderEncoderReversed)
    .idleMode(IdleMode.kBrake)
    // Set current limit
    .smartCurrentLimit(80, 80);
    // Conversion factors to convert from rotations to inches
    shoulderConfig.encoder
    // Conversion for elevator
    .positionConversionFactor(ManipulatorConstants.kShoulderGearRatio * 360)
    // Converts to degrees per second
    .velocityConversionFactor(ManipulatorConstants.kShoulderGearRatio * 360 / 60 );
    
    shoulderConfig.closedLoop
    .pid(ManipulatorConstants.kShoulderPValue, ManipulatorConstants.kShoulderIValue, ManipulatorConstants.kShoulderDValue);
    shoulderConfig.closedLoop.maxMotion
    .maxVelocity(ManipulatorConstants.kShoulderMaxVelocityDegreesPerSecond / ManipulatorConstants.kShoulderGearRatio / 360 * 60)
    .maxAcceleration(ManipulatorConstants.kShoulderMaxAccelerationDegreesPerSecondSquared / ManipulatorConstants.kShoulderGearRatio / 360 * 60);
    

    shoulderConfig.softLimit
    // The limit itself
    .forwardSoftLimit(ManipulatorConstants.kShoulderHigherLimitDegrees)
    // Is the limit enabled
    .forwardSoftLimitEnabled(true)
    // The limit itself
    .reverseSoftLimit(ManipulatorConstants.kShoulderLowerLimitDegrees)
    // Is the limit enabled
    .reverseSoftLimitEnabled(true);
    
    // Apply changes
    shoulderMotor.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Finish configurating elevator

    CANcoderConfiguration absoluteEncoderConfigs = new CANcoderConfiguration();
    absoluteEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    absoluteEncoderConfigs.MagnetSensor.MagnetOffset = ManipulatorConstants.kShoulderEncoderOffsetDegrees/360;
    absoluteEncoder.getConfigurator().apply(absoluteEncoderConfigs);
    

    //Zeros the shoulder encoder
    shoulderEncoder.setPosition(getAbsolutePosition());
  }

  public void periodic() {
    if (targetAngleSet) {
      System.out.println(getAbsolutePosition());
      shoulderPIDController.setReference(
        targetAngle, 
        ControlType.kMAXMotionPositionControl, 
        ClosedLoopSlot.kSlot0, 
        ManipulatorConstants.kShoulderFFValue*Math.cos(Math.toRadians(getAbsolutePosition())), 
        ArbFFUnits.kPercentOut
      );
    }
  }

    // Gets current position
  public double getPosition() {
    return shoulderEncoder.getPosition();
  }

    // Sets target position
  public void setPosition(double position) {
    targetAngleSet = true;
    // Set our goal for PID
    targetAngle = position;
    //shoulderPIDController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  public double getAbsolutePosition() {
    StatusSignal<Angle> absolutePos = absoluteEncoder.getAbsolutePosition();
    double absolutePosDegrees = absolutePos.getValueAsDouble()*360;
    return absolutePosDegrees;
  }
}
