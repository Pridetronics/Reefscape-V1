// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
  public final TalonFX shoulderMotor = new TalonFX(ManipulatorConstants.kShoulderMotorID);

  // CAN Encoder for shoulder motor
  public final CANcoder absoluteEncoder = new CANcoder(ManipulatorConstants.kShoulderEncoderID);

  private double targetAngle;
  private boolean targetAngleSet = false;

  public ShoulderHelper() {

    // Configurating Elevator
    TalonFXConfiguration talonFXElevatorConfig = new TalonFXConfiguration();
    // enable stator current limit
    talonFXElevatorConfig.CurrentLimits.StatorCurrentLimit = 120;
    talonFXElevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID
    talonFXElevatorConfig.Slot0.kP = ManipulatorConstants.kShoulderPValue;
    talonFXElevatorConfig.Slot0.kI = ManipulatorConstants.kShoulderIValue;
    talonFXElevatorConfig.Slot0.kD = ManipulatorConstants.kShoulderDValue;
    talonFXElevatorConfig.Slot0.kG = ManipulatorConstants.kShoulderGValue;
    talonFXElevatorConfig.Slot0.kS = ManipulatorConstants.kShoulderSValue;
    talonFXElevatorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    //More PID Configs for max velocity & Acceleration
    talonFXElevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ManipulatorConstants.kShoulderMaxVelocityDegreesPerSecond/ManipulatorConstants.kShoulderGearRatio/360;
    talonFXElevatorConfig.MotionMagic.MotionMagicAcceleration = ManipulatorConstants.kShoulderMaxAccelerationDegreesPerSecondSquared/ManipulatorConstants.kShoulderGearRatio/360;

    talonFXElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ManipulatorConstants.kShoulderHigherLimitDegrees/ManipulatorConstants.kShoulderGearRatio;
    talonFXElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ManipulatorConstants.kShoulderLowerLimitDegrees/ManipulatorConstants.kShoulderGearRatio;

    talonFXElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXElevatorConfig.MotorOutput.Inverted = ManipulatorConstants.kShoulderEncoderReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Finish Configurating Elevator
    shoulderMotor.getConfigurator().apply(talonFXElevatorConfig);

    // Finish configurating elevator

    CANcoderConfiguration absoluteEncoderConfigs = new CANcoderConfiguration();
    absoluteEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    absoluteEncoderConfigs.MagnetSensor.MagnetOffset = ManipulatorConstants.kShoulderEncoderOffsetDegrees/360;
    absoluteEncoder.getConfigurator().apply(absoluteEncoderConfigs);
    


    //Zeros the shoulder encoder
    shoulderMotor.setPosition(getAbsolutePosition()/360/ManipulatorConstants.kShoulderGearRatio);
  }

    // Gets current position
  public double getPosition() {
    StatusSignal<Angle> angleSignal = shoulderMotor.getPosition();
    return angleSignal.getValueAsDouble() * ManipulatorConstants.kShoulderGearRatio * 360;
  }

    // Sets target position
  public void setPosition(double position) {
    MotionMagicDutyCycle positionTargetRequest = new MotionMagicDutyCycle(position/360/ManipulatorConstants.kShoulderGearRatio);
    
    shoulderMotor.setControl(positionTargetRequest);
  }

  public double getAbsolutePosition() {
    StatusSignal<Angle> absolutePos = absoluteEncoder.getAbsolutePosition();
    double absolutePosDegrees = absolutePos.getValueAsDouble()*360;
    return absolutePosDegrees;
  }
}
