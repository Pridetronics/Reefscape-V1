// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Method for start shooter, stop shooter, set shooter height (shooter includes elevator)

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  //Setting new TalonFX motor
  public final TalonFX shooterMotor = new TalonFX(ShooterConstants.kShooterMotorID);

  //Set new SparkMax motor
  public final SparkMax elevatorMotor = new SparkMax(ShooterConstants.kElevatorMotorID, MotorType.kBrushless);

  //Encoder for our new motor
  public final SparkAbsoluteEncoder elevatorAbsoluteEncoder = elevatorMotor.getAbsoluteEncoder();
  
  //Relative encoder for our motor
  public final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
