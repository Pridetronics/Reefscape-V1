// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ElevatorHelper {

  // Set new class
  private final boolean hasHomed = false;

  // Set new TalonFX motor for the elevator
  private final TalonFX elevatorMotor = new TalonFX(ManipulatorConstants.kElevatorMotorID);

  public ElevatorHelper() {


    // Configurating Elevator
    TalonFXConfigurator talonFXElevatorConfigurator = elevatorMotor.getConfigurator();
    CurrentLimitsConfigs elevatorCurrentLimitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    elevatorCurrentLimitConfigs.StatorCurrentLimit = 120;
    elevatorCurrentLimitConfigs.StatorCurrentLimitEnable = true;

    talonFXElevatorConfigurator.apply(elevatorCurrentLimitConfigs);

    // PID
    Slot0Configs PIDElevatorConfigs = new Slot0Configs();

    PIDElevatorConfigs.kP = ManipulatorConstants.kElevatorPValue;
    PIDElevatorConfigs.kI = ManipulatorConstants.kElevatorIValue;
    PIDElevatorConfigs.kD = ManipulatorConstants.kElevatorDValue;

    talonFXElevatorConfigurator.apply(PIDElevatorConfigs);

    SoftwareLimitSwitchConfigs elevatorLimitConfigs = new SoftwareLimitSwitchConfigs();
    elevatorLimitConfigs.ForwardSoftLimitEnable = true;
    elevatorLimitConfigs.ForwardSoftLimitThreshold = ManipulatorConstants.kElevatorMaxHeightInches/ManipulatorConstants.kElevatorGearRatio;
    elevatorLimitConfigs.ReverseSoftLimitEnable = true;
    elevatorLimitConfigs.ReverseSoftLimitThreshold = ManipulatorConstants.kElevatorHomingHeightInches/ManipulatorConstants.kElevatorGearRatio;
    talonFXElevatorConfigurator.apply(elevatorLimitConfigs);

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    if (ManipulatorConstants.kElevatorMotorReversed) {
      motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    }
    talonFXElevatorConfigurator.apply(motorConfigs);

    // Finish Configurating Elevator
  }

    // Gets current position
   public double getPosition() {
    StatusSignal<Angle> angleSignal = elevatorMotor.getPosition();
    double angleRotations = angleSignal.getValueAsDouble();
    return angleRotations * ManipulatorConstants.kElevatorGearRatio;
   }

    // Sets current position
    public void setPosition(double height) {
      elevatorMotor.setPosition(height / ManipulatorConstants.kElevatorGearRatio);
    }

    // Brings elevator back to start
    public void beginHoming() {
      elevatorMotor.set(-0.05);
    }

    // Is the homing finished?
    public boolean isFinishedHoming() {
      return hasHomed;
    }
}