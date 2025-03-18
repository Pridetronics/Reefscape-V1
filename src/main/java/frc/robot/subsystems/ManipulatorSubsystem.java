// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Method for start shooter, stop shooter, set shooter height (shooter includes elevator)

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorHelpers.ClawHelper;
import frc.robot.subsystems.ManipulatorHelpers.ElevatorHelper;
import frc.robot.subsystems.ManipulatorHelpers.ShoulderHelper;
import frc.robot.utils.ShuffleboardRateLimiter;

public class ManipulatorSubsystem extends SubsystemBase {
  /** Creates a new ManipulatorSubsystem. */

  GenericEntry elevatorHeightEntry = Shuffleboard.getTab("Test Data").add("Elevator Height", 0).getEntry();
  GenericEntry shoulderAngleEntry = Shuffleboard.getTab("Test Data").add("Shoulder Angle", 0).getEntry();
  GenericEntry shoulderAbsoluteAngleEntry = Shuffleboard.getTab("Test Data").add("Shoulder Absolute Angle", 0).getEntry();


  public static enum ClawHeightLevel {
    Stow,
    Level1,
    Level2,
    Level3,
    Level4,
    Barge,
    ElevatorSafeHeight,
    CoralExtract
  }

  //TODO make helpers private later

  // So we can call upon subsystem ClawHelper
  public final ClawHelper clawHelper = new ClawHelper();

  // So we can call upon subsystem ElevatorHelper
  public final ElevatorHelper elevatorHelper = new ElevatorHelper();

  // So we can call upon subsystem ShoulderHelper
  public final ShoulderHelper shoulderHelper = new ShoulderHelper();

  public ManipulatorSubsystem() {}

  public double getElevatorHeightFromEnum(ClawHeightLevel heightLevel) {
    switch (heightLevel) {
      case Level1:
        return ManipulatorConstants.kElevatorHeightL1Inches;
      case Level2:
        return ManipulatorConstants.kElevatorHeightL2Inches;
      case Level3:
        return ManipulatorConstants.kElevatorHeightL3Inches;
      case Level4:
        return ManipulatorConstants.kElevatorHeightL4Inches;
      case Barge:
        return ManipulatorConstants.kElevatorMaxHeightInches;
      case Stow:
        return ManipulatorConstants.kElevatorStowHeightInches;
      case ElevatorSafeHeight:
        return ManipulatorConstants.kElevatorSafeFromIntakeHeightInches;
      case CoralExtract:
        throw new RuntimeException("CoralExtract is not availiable for the elevator");
      default:
        return ManipulatorConstants.kElevatorHomingHeightInches;
    }
  }

  public double getShoulderAngleFromEnum(ClawHeightLevel heightLevel) {
    switch (heightLevel) {
      case Level1:
        return ManipulatorConstants.kShoulderAngleL1Degrees;
      case Level2:
        return ManipulatorConstants.kShoulderAngleL2Degrees;
      case Level3:
        return ManipulatorConstants.kShoulderAngleL3Degrees;
      case Level4:
        return ManipulatorConstants.kShoulderAngleL4Degrees;
      case Barge:
        return ManipulatorConstants.kShoulderAngleL4Degrees;
      case Stow:
        return ManipulatorConstants.kShoulderStowAngleDegrees;
      case ElevatorSafeHeight:
        throw new RuntimeException("ElevatorSafeHeight is not availiable for the shoulder");
      case CoralExtract:
        return ManipulatorConstants.kShoulderIntakeAngleDegrees;
      default:
        return ManipulatorConstants.kShoulderLowerLimitDegrees;
    }
  }

  public Boolean getStowedState() {
    return ( shoulderHelper.getPosition() < ManipulatorConstants.kShoulderStowAngleDegrees + 20 
    && elevatorHelper.getPosition() < ManipulatorConstants.kElevatorStowHeightInches + ManipulatorConstants.kElevatorFuzzyEqInches );
  }

  public double getElevatorHeight() {
    return elevatorHelper.getPosition();
  }

  public Boolean isClawOutOfWay() {
    return elevatorHelper.getPosition() >= ManipulatorConstants.kElevatorSafeFromIntakeHeightInches - ManipulatorConstants.kElevatorFuzzyEqInches;
  }

  public Boolean isElevatorAtHeight(ClawHeightLevel heightLevel) {
    return Math.abs(elevatorHelper.getPosition() - getElevatorHeightFromEnum(heightLevel)) < ManipulatorConstants.kElevatorFuzzyEqInches;
  }

  public Boolean isShoulderAtAngle(ClawHeightLevel heightLevel) {
    return Math.abs(shoulderHelper.getPosition() - getShoulderAngleFromEnum(heightLevel)) < ManipulatorConstants.kShoulderFuzzyEqDegrees;
  }

  public void setElevatorPositionalState(ClawHeightLevel heightLevel) {
    elevatorHelper.setPosition(getElevatorHeightFromEnum(heightLevel));
  }

  public void setShoulderPositionalState(ClawHeightLevel heightLevel) {
    shoulderHelper.setPosition(getShoulderAngleFromEnum(heightLevel));
  }

  public void clawGrab() {
    clawHelper.setVelocity(ManipulatorConstants.kClawSpeedRPM);
  }

  public void clawRemove() {
    clawHelper.setVelocity(-ManipulatorConstants.kClawSpeedRPM);
  }

  public void stopClaw() {
    clawHelper.stop();
  }

  // Note: When do the motors stop?
  @Override
  public void periodic() {
    ShuffleboardRateLimiter.queueDataForShuffleboard(elevatorHeightEntry, elevatorHelper.getPosition());
    ShuffleboardRateLimiter.queueDataForShuffleboard(shoulderAngleEntry, shoulderHelper.getPosition());
    ShuffleboardRateLimiter.queueDataForShuffleboard(shoulderAbsoluteAngleEntry, shoulderHelper.getAbsolutePosition());


    // This method will be called once per scheduler run
    shoulderHelper.periodic();
  }
}
