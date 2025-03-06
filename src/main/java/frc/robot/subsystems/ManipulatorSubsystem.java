// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Method for start shooter, stop shooter, set shooter height (shooter includes elevator)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorHelpers.ClawHelper;
import frc.robot.subsystems.ManipulatorHelpers.ElevatorHelper;
import frc.robot.subsystems.ManipulatorHelpers.ShoulderHelper;

public class ManipulatorSubsystem extends SubsystemBase {
  /** Creates a new ManipulatorSubsystem. */

  public static enum ClawHeightLevel {
    Stow,
    Level1,
    Level2,
    Level3,
    Level4,
    Barge
  }

  //TODO make helpers private later

  // So we can call upon subsystem ClawHelper
  public final ClawHelper clawHelper = new ClawHelper();

  // So we can call upon subsystem ElevatorHelper
  public final ElevatorHelper elevatorHelper = new ElevatorHelper();

  // So we can call upon subsystem ShoulderHelper
  public final ShoulderHelper shoulderHelper = new ShoulderHelper();

  public ManipulatorSubsystem() {}

  private double getElevatorHeightFromEnum(ClawHeightLevel heightLevel) {
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
      default:
        return ManipulatorConstants.kElevatorHomingHeightInches;
    }
  }

  private double getShoulderAngleFromEnum(ClawHeightLevel heightLevel) {
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
      default:
        return ManipulatorConstants.kShoulderLowerLimitDegrees;
    }
  }

  public Boolean getStowedState() {
    return shoulderHelper.shoulderStowed;
  }

  public Boolean isClawOutOfWay() {
    return null;
  }

  public Boolean isClawAtPositionalHeight(ClawHeightLevel heightLevel) {
    return Math.abs(elevatorHelper.getPosition() - getElevatorHeightFromEnum(heightLevel)) < ManipulatorConstants.kElevatorFuzzyEqInches;
  }

  public void setManipulatorPositionalState(ClawHeightLevel heightLevel) {
    elevatorHelper.setPosition(getElevatorHeightFromEnum(heightLevel));
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
    // This method will be called once per scheduler run
  }
}
