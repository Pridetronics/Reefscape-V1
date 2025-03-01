// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Method for start shooter, stop shooter, set shooter height (shooter includes elevator)

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

  public static enum ClawHeightLevel {
    Level1,
    Level2,
    Level3,
    Level4,
    Barge
  }

  public Boolean shoulderStowed = true;
  /** Creates a new ShooterSubsystem. */
  public ManipulatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Boolean getStowedState() {
    return shoulderStowed;
  }

  public Boolean isClawOutOfWay() {
    return null;
  }

  public Boolean isClawAtPositionalHeight(ClawHeightLevel heightLevel) {
    return null;
  }
}
