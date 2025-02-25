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

  // Setting new TalonFX motor
  public final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorCanID);

  // Set new SparkMax motor
  public final SparkMax intakeAngleMotor = new SparkMax(IntakeConstants.kIntakeAngleMotorCanID, MotorType.kBrushless);

  // Encoder for our new motor
  public final SparkAbsoluteEncoder intakeAbsoluteEncoder = intakeAngleMotor.getAbsoluteEncoder();
  
  // Relative encoder for our motor
  public final RelativeEncoder intakeEncoder = intakeAngleMotor.getEncoder();

  public IntakeSubsystem() {
  }

  public void setIntakeAngle() {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 2. Which direction is the motor going to spin?
     * 3. How far?
     * 4. How fast?
     */
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
  }

  public void stopIntake() {
    /* Steps
     * 1. How does it stop?
     * 2. Why does it stop?
     * 3. When does it stop?
     */
  }

  // Note: When do the motors stop?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
