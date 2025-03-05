// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Set smart current limit and PID controlling for the motors

package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // Setting new TalonFX motor
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorCanID);

  // Set new SparkMax motor
  private final SparkMax intakeAngleMotor = new SparkMax(IntakeConstants.kIntakeAngleMotorCanID, MotorType.kBrushless);

  // Encoder for our new motor
  private final SparkAbsoluteEncoder intakeAbsoluteEncoder = intakeAngleMotor.getAbsoluteEncoder();
  
  // Relative encoder for our motor
  private final RelativeEncoder intakeEncoder = intakeAngleMotor.getEncoder();

  private final SparkClosedLoopController intakeAnglePIDController = intakeAngleMotor.getClosedLoopController();

  public IntakeSubsystem() {

    // Configurating elevator
    SparkMaxConfig intakeAngleConfig = new SparkMaxConfig();
    
    intakeAngleConfig
    // Set what we are configurating
    .inverted(IntakeConstants.kIntakeAngleMotorReversed)
    .idleMode(IdleMode.kBrake)
    // Set current limit
    .smartCurrentLimit(80, 80);
    // Conversion factors to convert from rotations to inches
    intakeAngleConfig.encoder
    // Conversion for elevator
    .positionConversionFactor(IntakeConstants.kIntakeAngleGearRatio * 360)
    // Converts to degrees per second
    .velocityConversionFactor(IntakeConstants.kIntakeAngleGearRatio * 360 / 60 );
    
    intakeAngleConfig.closedLoop
    .pid(IntakeConstants.kIntakeAnglePValue, IntakeConstants.kIntakeAngleIValue, IntakeConstants.kIntakeAngleDValue);

    intakeAngleConfig.apply(intakeAngleConfig);


    // Configurating Claw
    TalonFXConfigurator talonFXIntakeConfigurator = intakeMotor.getConfigurator();

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    if (IntakeConstants.kIntakeReversed) {
      outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    }

    CurrentLimitsConfigs intakeMotorLimitConfigs = new CurrentLimitsConfigs();

    // enable stator current limit
    intakeMotorLimitConfigs.StatorCurrentLimit = 120;
    intakeMotorLimitConfigs.StatorCurrentLimitEnable = true;

    talonFXIntakeConfigurator.apply(intakeMotorLimitConfigs);

    // PID
    Slot0Configs PIDClawConfigs = new Slot0Configs();

    PIDClawConfigs.kP = IntakeConstants.kIntakePValue;
    PIDClawConfigs.kI = IntakeConstants.kIntakeIValue;
    PIDClawConfigs.kD = IntakeConstants.kIntakeDValue;

    talonFXIntakeConfigurator.apply(PIDClawConfigs);
  }

  public void setIntakeAngle(double angleDegrees) {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 2. Which direction is the motor going to spin?
     * 3. How far?
     * 4. How fast?
     */
    intakeAnglePIDController.setReference(angleDegrees, ControlType.kPosition);
  }

  public void startIntake() {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 2. Which direction is the motor going to spin?
     * 3. How far?
     * 4. How fast?
     * 5. What happens after the coral has been intaked?
     * 6. Source or Floor?
     */

    VelocityDutyCycle velocityRequest = new VelocityDutyCycle(IntakeConstants.kIntakeRPM / IntakeConstants.kIntakeGearRatio);

     intakeMotor.setControl(velocityRequest);
  }

  public void stopIntake() {
    /* Steps
     * 1. How does it stop?
     * 2. Why does it stop?
     * 3. When does it stop?
     */

     VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

     intakeMotor.setControl(velocityRequest);
  }

  // Note: When do the motors stop?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
