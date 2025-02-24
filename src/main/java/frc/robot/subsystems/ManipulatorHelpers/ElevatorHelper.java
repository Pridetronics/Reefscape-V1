// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ElevatorHelper {

//Set new SparkMax motor for the elevator
  public final SparkMax elevatorMotor = new SparkMax(ManipulatorConstants.kElevatorMotorID, MotorType.kBrushless);

//Relative encoder for the elevator motor
  public final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  public ElevatorHelper() {

    //Configurating elevator
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    elevatorConfig
    //Set what we are configurating
    .inverted(ManipulatorConstants.kElevatorEncoderReversed)
    .idleMode(IdleMode.kBrake)
    //Set current limit
    .smartCurrentLimit(80, 80);
    //Conversion factors to convert from rotations to inches
    elevatorConfig.encoder
    //Conversion for elevator
    .positionConversionFactor(ManipulatorConstants.kDistancePerElevatorGearRotation*2)
    //converts to inches per second
    .velocityConversionFactor((ManipulatorConstants.kDistancePerElevatorGearRotation*2) /60 );
    
    elevatorConfig.closedLoop
    .pid(ManipulatorConstants.kElevatorPValue, ManipulatorConstants.kElevatorIValue, ManipulatorConstants.kElevatorDValue);
    
    //Elevator limit config
    SoftLimitConfig softConfig = new SoftLimitConfig();
    
    //Maximum limit
    softConfig
    //The limit itself
    .forwardSoftLimit(ManipulatorConstants.kElevatorMaxHeightInches)
    //Is the limit enabled
    .forwardSoftLimitEnabled(true);
    //Minimum limit
    softConfig
    //The limit itself
    .reverseSoftLimit(ManipulatorConstants.kElevatorHomingHeightInches)
    //Is the limit enabled
    .reverseSoftLimitEnabled(true);
    
    //Apply changes
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Finish configurating elevator
    }

    public double getPosition() {
        //Gets current position
        
        get
    }

    public void setPosition() {
        //Sets target position
    }
}