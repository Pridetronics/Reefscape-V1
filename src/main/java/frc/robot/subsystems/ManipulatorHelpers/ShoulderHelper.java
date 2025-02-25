// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ShoulderHelper {

    // Set new SparkMax motor for the shoulder
  public final TalonFX shoulderMotor = new TalonFX(ManipulatorConstants.kShoulderMotorID);

  // CAN Encoder for shoulder motor
  public final CANcoder absoluteEncoder = new CANcoder(ManipulatorConstants.kShoulderEncoderID);

  public ShoulderHelper() {

    // Configurating shoulder motor
    TalonFXConfigurator talonFXShoulderConfigurator = shoulderMotor.getConfigurator();
    CurrentLimitsConfigs shoulderLimitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    shoulderLimitConfigs.StatorCurrentLimit = 120;
    shoulderLimitConfigs.StatorCurrentLimitEnable = true;

    talonFXShoulderConfigurator.apply(shoulderLimitConfigs);

    // PID
    Slot0Configs PIDShoulderConfigs = new Slot0Configs();

    PIDShoulderConfigs.kP = ManipulatorConstants.kShoulderPValue;
    PIDShoulderConfigs.kI = ManipulatorConstants.kShoulderIValue;
    PIDShoulderConfigs.kD = ManipulatorConstants.kShoulderDValue;

    talonFXShoulderConfigurator.apply(PIDShoulderConfigs);
    // Finish configurating shoulder
  }

  // Gets current position
  public double getPosition() {}

  // Sets current position
  public void setPosition() {}
}
