// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ClawHelper {

    // Setting new TalonFX motor for the claw
  public final TalonFX clawMotor = new TalonFX(ManipulatorConstants.kClawMotorID);

  public ClawHelper() {

// Configurating Claw
    TalonFXConfiguration talonFXClawConfiguration = new TalonFXConfiguration();

    talonFXClawConfiguration.MotorOutput.Inverted = ManipulatorConstants.kClawMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonFXClawConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable stator current limit
    talonFXClawConfiguration.CurrentLimits.StatorCurrentLimit = 120;
    talonFXClawConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID
    talonFXClawConfiguration.Slot0.kP = ManipulatorConstants.kClawPValue;
    talonFXClawConfiguration.Slot0.kI = ManipulatorConstants.kClawIValue;
    talonFXClawConfiguration.Slot0.kD = ManipulatorConstants.kClawDValue;

    // Finish configuring claw
    clawMotor.getConfigurator().apply(talonFXClawConfiguration);

  }

  // Gets current velocity
  public double getVelocity() {
    StatusSignal<AngularVelocity> velocitySignal = clawMotor.getVelocity();
    double velocity = velocitySignal.getValueAsDouble();
    return velocity * ManipulatorConstants.kClawGearRatio;
  }

  // Sets target velocity
  public void setVelocity(double velocityRPM) {
    VelocityDutyCycle velocityTargetRequest = new VelocityDutyCycle(velocityRPM/ManipulatorConstants.kClawGearRatio);
    
    clawMotor.setControl(velocityTargetRequest);
  }

  public void stop() {
    VelocityDutyCycle velocityTargetRequest = new VelocityDutyCycle(0);
    
    clawMotor.setControl(velocityTargetRequest);
  }
}
