// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Set smart current limit and PID controlling for the motors

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardRateLimiter;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  public Boolean intakeStowed = true;

  /** Creates a new IntakeSubsystem. */


  GenericEntry intakeRelativeEntry = Shuffleboard.getTab("Test Data").add("Intake Relative Angle", 0).getEntry();
  GenericEntry intakeAbsoluteEntry = Shuffleboard.getTab("Test Data").add("Intake Absolute Angle", 0).getEntry();


  // Setting new TalonFX motor
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorCanID);

  // Set new SparkMax motor
  private final SparkMax intakeAngleMotor = new SparkMax(IntakeConstants.kIntakeAngleMotorCanID, MotorType.kBrushless);
  private double targetAngle = 0;

  // Encoder for our new motor
  private final SparkAbsoluteEncoder intakeAbsoluteEncoder = intakeAngleMotor.getAbsoluteEncoder();

  // Relative encoder for our motor
  private final RelativeEncoder intakeAngleEncoder = intakeAngleMotor.getEncoder();

  private final ProfiledPIDController intakeAnglePIDController = new ProfiledPIDController(
    IntakeConstants.kIntakeAnglePValue, 
    IntakeConstants.kIntakeAngleIValue, 
    IntakeConstants.kIntakeAngleDValue, 
    new TrapezoidProfile.Constraints(
      IntakeConstants.kIntakeAngleMaxVelocityDegreesPerSecond, 
      IntakeConstants.kIntakeAngleMaxAccelerationDegreesPerSecondSquared
    )
  );

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

    intakeAnglePIDController.setTolerance(1);

    resetEncoder();
    setIntakeAngle(IntakeConstants.kIntakeStowAngle);
  }

  public void resetEncoder() {
    intakeAngleEncoder.setPosition(getAbsoluteAngle());
    intakeAnglePIDController.reset(getAbsoluteAngle());
    targetAngle = getAbsoluteAngle();
  }

  public double boundAngle(double num) {
    return ((num + 180) % 360 + 360) % 360 - 180;
  }

  public double getAbsoluteAngle() {
    return boundAngle(-intakeAbsoluteEncoder.getPosition()*360);
  }

  public double getAbsoluteAngleVelocity() {
    return -intakeAbsoluteEncoder.getVelocity()*360;
  }

  public void setIntakeAngle(double angleDegrees) {
    resetEncoder();
    targetAngle = boundAngle(angleDegrees);
  }

  public double getIntakeAngle() {
    return intakeAngleEncoder.getPosition();
  }

  public boolean isIntakeAtAngle(double targetAngle) {
    return Math.abs(getIntakeAngle() - targetAngle) <= IntakeConstants.kIntakeAngleFuzzyEqDegrees;
  }

  public void startIntake() {

    // VelocityDutyCycle velocityRequest = new VelocityDutyCycle(IntakeConstants.kIntakeRPM / IntakeConstants.kIntakeGearRatio);

    // intakeMotor.setControl(velocityRequest);

    intakeMotor.set(0.6);
  }

  public void startOuttake() {

    // VelocityDutyCycle velocityRequest = new VelocityDutyCycle(-IntakeConstants.kIntakeRPM / IntakeConstants.kIntakeGearRatio);

    // intakeMotor.setControl(velocityRequest);

    intakeMotor.set(-0.6);
  }

  public void stopIntake() {

    //  VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    //  intakeMotor.setControl(velocityRequest);
    intakeMotor.set(0);
  }

  // Note: When do the motors stop?

  @Override
  public void periodic() {
    ShuffleboardRateLimiter.queueDataForShuffleboard(intakeRelativeEntry, getIntakeAngle());
    ShuffleboardRateLimiter.queueDataForShuffleboard(intakeAbsoluteEntry, getAbsoluteAngle());

    // if (getAbsoluteAngle() > 20) {
    //   intakeAnglePIDController.setP(IntakeConstants.kIntakeHighAnglePValue);
    // } else {
    //   intakeAnglePIDController.setP(IntakeConstants.kIntakeAnglePValue);
    // }

    // This method will be called once per scheduler run
    double newAngleSetPoint = intakeAnglePIDController.calculate(getIntakeAngle(), targetAngle);

    double feedForward = IntakeConstants.kIntakeAngleFFValue*Math.sin(-Units.degreesToRadians(getAbsoluteAngle()-20));

    // double maxSpeed = 1;
    // if (Math.signum(getAbsoluteAngleVelocity()) == -1) {
    //   maxSpeed = 0.05;
    // }
    //intakeAngleMotor.set();
    // double maxOutput = Math.max(Math.min(1-((getAbsoluteAngle()+10)/360*6), 1), 0.1);
    // double speed = newAngleSetPoint + feedForward;
    // double finalSpeed = Math.max(-maxOutput, Math.min(maxOutput, speed));
    // intakeAngleMotor.set(finalSpeed);
    // System.out.println(finalSpeed);
    intakeAngleMotor.set(newAngleSetPoint + feedForward);
  }

  public Boolean getStowedState() {
    return intakeStowed;
  }

  public Boolean isIntakeOutOfWay() {
    return null;
  }
}
