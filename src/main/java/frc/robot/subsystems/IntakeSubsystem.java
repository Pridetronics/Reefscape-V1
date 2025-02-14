// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Set smart current limit and PID controlling for the motors

package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  //Setting new TalonFX motor
  public final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorCanID);

  //Set new SparkMax motor
  public final SparkMax intakeAngleMotor = new SparkMax(IntakeConstants.kIntakeAngleMotorCanID, MotorType.kBrushless);

  //Encoder for our new motor
  public final SparkAbsoluteEncoder intakeAbsoluteEncoder = intakeAngleMotor.getAbsoluteEncoder();
  
  //Relative encoder for our motor
  public final RelativeEncoder intakeEncoder = intakeAngleMotor.getEncoder();

  public IntakeSubsystem() {
  }

  public void setIntakeAngle() {
    //Method guts go here
  }

  public boolean startIntake() {
    //Method guts go here
  }

  public boolean stopIntake() {
    //Method guts go here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
