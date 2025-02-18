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

  //Set new SparkMax motor
  public final TalonFX shoulderMotor = new TalonFX(ShooterConstants.kShoulderMotorID);

  //Encoder for our shoulder motor (temporarily commented off)
  //public final SparkAbsoluteEncoder shoulderAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder();
  
  //Relative encoder for our elevator motor
  public final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


  public ShooterSubsystem() {}

  public void setElevatorHeight() {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 2. Which direction and/or what type of motor is spinning?
     * 3. How tall?
     * 4. How fast?
     */
  }

  public void startShooter() {
    /* Steps
     * 1. Driver input (button or joystick?)
     * 3. Which direction is the motor going to spin?
     * 4. How long?
     * 5. How fast?
     */
  }

  public void stopShooter() {
    /* Steps
     * 1. How does it stop?
     * 2. Why does it stop?
     * 3. When does it stop?
     * 4. Does it trigger anything after it stopped?
     */
  }

  //Note: When do the motors stop?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
