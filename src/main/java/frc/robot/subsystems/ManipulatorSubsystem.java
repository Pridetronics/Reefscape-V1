// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Method for start shooter, stop shooter, set shooter height (shooter includes elevator)

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.WheelConstants;

public class ManipulatorSubsystem extends SubsystemBase {
  /** Creates a new ManipulatorSubsystem. */

  //Setting new TalonFX motor for the claw
  public final TalonFX clawMotor = new TalonFX(ManipulatorConstants.kClawMotorID);

  //Set new SparkMax motor for the elevator
  public final SparkMax elevatorMotor = new SparkMax(ManipulatorConstants.kElevatorMotorID, MotorType.kBrushless);

  //Set new SparkMax motor for the shoulder
  public final TalonFX shoulderMotor = new TalonFX(ManipulatorConstants.kShoulderMotorID);

  //CAN Encoder for shoulder motor
  private final CANcoder absoluteEncoder = new CANcoder(ManipulatorConstants.kShoulderEncoderID);
  
  //Relative encoder for the elevator motor
  public final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


  public ManipulatorSubsystem() {

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


    //Configurating Claw
    TalonFXConfigurator talonFXConfigurator = clawMotor.getConfigurator();
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 120;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  public void setElevatorHeight() {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 2. Which direction and/or what type of motor is spinning?
     * 3. How tall?
     * 4. How fast?
     */

     //Conversion factor Circumfirance of gear x2


  }

  public void startClaw() {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 3. Which direction is the motor going to spin?
     * 4. How long?
     * 5. How fast?
     */
  }

  public void stopClaw() {
    /* Steps
     * 1. How does it stop?
     * 2. Why does it stop?
     * 3. When does it stop?
     * 4. Does it trigger anything after it stopped?
     */
  }

  //Note: When do the motors stop?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
