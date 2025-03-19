// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ManipulatorHelpers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ManipulatorConstants;

/** Add your docs here. */
public class ClawHelper {

    // Setting new TalonFX motor for the claw
  public final SparkMax clawMotor = new SparkMax(ManipulatorConstants.kClawMotorID, MotorType.kBrushless);
  
  public final RelativeEncoder clawEncoder = clawMotor.getEncoder();
  
  public final SparkClosedLoopController clawPIDController = clawMotor.getClosedLoopController();

  public ClawHelper() {

    // Configurating Claw
    SparkMaxConfig clawConfig = new SparkMaxConfig();

    clawConfig.encoder
    .positionConversionFactor(ManipulatorConstants.kClawGearRatio)
    .velocityConversionFactor(ManipulatorConstants.kClawGearRatio / 60);
    clawConfig.closedLoop
    .pid(ManipulatorConstants.kClawPValue, ManipulatorConstants.kClawIValue, ManipulatorConstants.kClawDValue);
    clawConfig
    .inverted(ManipulatorConstants.kClawMotorReversed)
    .smartCurrentLimit(50, 50)
    .idleMode(IdleMode.kBrake);

    clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Gets current velocity
  public double getVelocity() {
    return clawEncoder.getVelocity();
  }

  // Sets target velocity
  public void setVelocity(double velocityRPM) {
    clawPIDController.setReference(velocityRPM, ControlType.kVelocity);
  }

  // Sets target velocity
  public void setDutyCycle(double percentSpeed) {
    clawPIDController.setReference(percentSpeed, ControlType.kDutyCycle);
  }

  public void stop() {
    clawPIDController.setReference(0, ControlType.kVelocity);
  }
}
