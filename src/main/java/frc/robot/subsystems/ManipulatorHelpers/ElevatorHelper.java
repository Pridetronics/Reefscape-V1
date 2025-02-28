// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ElevatorHelper {

// Set new TalonFX motor for the elevator
  public final TalonFX elevatorMotor = new TalonFX(ManipulatorConstants.kElevatorMotorID);

  public ElevatorHelper() {


    // Configurating Elevator
    TalonFXConfigurator talonFXElevatorConfigurator = elevatorMotor.getConfigurator();
    CurrentLimitsConfigs elevatorLimitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    elevatorLimitConfigs.StatorCurrentLimit = 120;
    elevatorLimitConfigs.StatorCurrentLimitEnable = true;

    talonFXElevatorConfigurator.apply(elevatorLimitConfigs);

    // PID
    Slot0Configs PIDElevatorConfigs = new Slot0Configs();

    PIDElevatorConfigs.kP = ManipulatorConstants.kElevatorPValue;
    PIDElevatorConfigs.kI = ManipulatorConstants.kElevatorIValue;
    PIDElevatorConfigs.kD = ManipulatorConstants.kElevatorDValue;

    talonFXElevatorConfigurator.apply(PIDElevatorConfigs);
    // Finish Configurating Elevator
  }

    // Gets current position
   public double getPosition() {}

    // Sets current position
    public void setPosition() {}

    // Brings elevator back to start
    public void beginHoming() {}

    // Is the homing finished?
    public boolean isFinishedHoming() {}
}