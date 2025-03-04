// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ClawHelper {

    // Setting new TalonFX motor for the claw
  public final TalonFX clawMotor = new TalonFX(ManipulatorConstants.kClawMotorID);

  public ClawHelper() {

// Configurating Claw
    TalonFXConfigurator talonFXClawConfigurator = clawMotor.getConfigurator();
    CurrentLimitsConfigs clawLimitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    clawLimitConfigs.StatorCurrentLimit = 120;
    clawLimitConfigs.StatorCurrentLimitEnable = true;

    talonFXClawConfigurator.apply(clawLimitConfigs);

    // PID
    Slot0Configs PIDClawConfigs = new Slot0Configs();

    PIDClawConfigs.kP = ManipulatorConstants.kClawPValue;
    PIDClawConfigs.kI = ManipulatorConstants.kClawIValue;
    PIDClawConfigs.kD = ManipulatorConstants.kClawDValue;

    talonFXClawConfigurator.apply(PIDClawConfigs);
    // Finish configuring claw
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
}
