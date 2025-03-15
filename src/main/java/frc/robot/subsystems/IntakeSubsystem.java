// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Set smart current limit and PID controlling for the motors

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // Setting new TalonFX motor
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorCanID);

  // Set new SparkMax motor
  private final SparkMax intakeAngleMotor = new SparkMax(IntakeConstants.kIntakeAngleMotorCanID, MotorType.kBrushless);

  // Encoder for our new motor
  private final AnalogInput intakeAbsoluteEncoder = new AnalogInput(1);
  private double accumAbsValue = 0;
  private int accumAbsKeys = 0;
  double currentAverageIntakeAngleAbsoluteValue = 0;
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
    //PID config
    intakeAngleConfig.closedLoop
    .pid(IntakeConstants.kIntakeAnglePValue, IntakeConstants.kIntakeAngleIValue, IntakeConstants.kIntakeAngleDValue);
    
    intakeAngleConfig.closedLoop.maxMotion
    .maxVelocity(IntakeConstants.kIntakeAngleMaxVelocityDegreesPerSecond / IntakeConstants.kIntakeAngleGearRatio / 360 * 60)
    .maxAcceleration(IntakeConstants.kIntakeAngleMaxAccelerationDegreesPerSecondSquared / IntakeConstants.kIntakeAngleGearRatio / 360 * 60);

    intakeAngleMotor.configure(intakeAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Configurating intake motor
    TalonFXConfiguration talonFXIntakeConfiguration = new TalonFXConfiguration();

    talonFXIntakeConfiguration.MotorOutput.Inverted = IntakeConstants.kIntakeReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    talonFXIntakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enable stator current limit
    talonFXIntakeConfiguration.CurrentLimits.StatorCurrentLimit = 120;
    talonFXIntakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID
    talonFXIntakeConfiguration.Slot0.kP = IntakeConstants.kIntakePValue;
    talonFXIntakeConfiguration.Slot0.kI = IntakeConstants.kIntakeIValue;
    talonFXIntakeConfiguration.Slot0.kD = IntakeConstants.kIntakeDValue;

    intakeMotor.getConfigurator().apply(talonFXIntakeConfiguration);

    intakeEncoder.setPosition(getIntakeAbsoluteAngle());
  }

  public void setIntakeAngle(double angleDegrees) {
    //intakeAnglePIDController.setReference(angleDegrees, ControlType.kMAXMotionPositionControl);
  }

  public double getIntakeAngle() {
    return intakeEncoder.getPosition();
  }

  public boolean isIntakeAtAngle(double targetAngle) {
    return Math.abs(getIntakeAngle() - targetAngle) <= IntakeConstants.kIntakeAngleFuzzyEqDegrees;
  }

  public double getIntakeAbsoluteAngle() {
    double encoderAngle = accumAbsValue/accumAbsKeys;
    return encoderAngle+IntakeConstants.kIntakeAbsoluteEncoderOffsetDegrees;
  }

  public void startIntake() {

    VelocityDutyCycle velocityRequest = new VelocityDutyCycle(IntakeConstants.kIntakeRPM / IntakeConstants.kIntakeGearRatio);

    //intakeMotor.setControl(velocityRequest);
  }

  public void startOuttake() {

    VelocityDutyCycle velocityRequest = new VelocityDutyCycle(-IntakeConstants.kIntakeRPM / IntakeConstants.kIntakeGearRatio);

    //intakeMotor.setControl(velocityRequest);
  }

  public void stopIntake() {
    /* Steps
     * 1. How does it stop?
     * 2. Why does it stop?
     * 3. When does it stop?
     */

     VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

     //intakeMotor.setControl(velocityRequest);
  }

  // Note: When do the motors stop?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    accumAbsValue += intakeAbsoluteEncoder.getAverageVoltage()/RobotController.getVoltage3V3() * 360;
    accumAbsKeys += 1;
  }
}
