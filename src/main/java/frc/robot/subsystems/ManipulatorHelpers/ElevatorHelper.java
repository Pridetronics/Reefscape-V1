// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    TalonFXConfiguration talonFXElevatorConfig = new TalonFXConfiguration();
    // enable stator current limit
    talonFXElevatorConfig.CurrentLimits.StatorCurrentLimit = 120;
    talonFXElevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID
    talonFXElevatorConfig.Slot0.kP = ManipulatorConstants.kElevatorPValue;
    talonFXElevatorConfig.Slot0.kI = ManipulatorConstants.kElevatorIValue;
    talonFXElevatorConfig.Slot0.kD = ManipulatorConstants.kElevatorDValue;
    talonFXElevatorConfig.Slot0.kG = ManipulatorConstants.kElevatorGValue;
    talonFXElevatorConfig.Slot0.kS = ManipulatorConstants.kElevatorSValue;

    //More PID Configs for max velocity & Acceleration
    talonFXElevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ManipulatorConstants.kElevatorSpeedInchesPerSecond/ManipulatorConstants.kElevatorGearRatio;
    talonFXElevatorConfig.MotionMagic.MotionMagicAcceleration = ManipulatorConstants.kElevatorAccelerationInchesPerSecondSquared/ManipulatorConstants.kElevatorGearRatio;

    talonFXElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ManipulatorConstants.kElevatorMaxHeightInches/ManipulatorConstants.kElevatorGearRatio;
    talonFXElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ManipulatorConstants.kElevatorHomingHeightInches/ManipulatorConstants.kElevatorGearRatio;

    talonFXElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXElevatorConfig.MotorOutput.Inverted = ManipulatorConstants.kElevatorMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Finish Configurating Elevator
    elevatorMotor.getConfigurator().apply(talonFXElevatorConfig);

    //Set the robot starting height
    elevatorMotor.setPosition(ManipulatorConstants.kElevatorHomingHeightInches / ManipulatorConstants.kElevatorGearRatio);
  }

    // Gets current position
  public double getPosition() {
    StatusSignal<Angle> angleSignal = elevatorMotor.getPosition();
    double angleRotations = angleSignal.getValueAsDouble();
    return angleRotations * ManipulatorConstants.kElevatorGearRatio;
  }

    // Sets current position
    public void setPosition(double height) {

      MotionMagicDutyCycle positionTargetRequest = new MotionMagicDutyCycle(height/ManipulatorConstants.kElevatorGearRatio);
      
      elevatorMotor.setControl(positionTargetRequest);

    }
}