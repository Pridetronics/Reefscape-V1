// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.commands.FieldPositionUpdate;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroRobotHeading;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.Autos;
import frc.robot.utils.Autos.CommandSelector;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final SendableChooser<Autos.CommandSelector> autoCommandChooser = new SendableChooser<>();
  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);

  //Create a shuffleboard tab for the drivers to see all teleop info
  private static final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");

  private final GenericEntry forwardDirectionEntry = teleOpTab.add("Reverse Field Forward", false)
  .withWidget(BuiltInWidgets.kToggleSwitch)
  .getEntry();

  private final FieldPositionUpdate fieldUpdateCommand = new FieldPositionUpdate(
    visionSubsystem, 
    swerveSubsystem
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Create your auto paths here (By which the trajectories are made in SwerveAutoPaths)
    autoCommandChooser.setDefaultOption("Do Nothing", null);
    // autoCommandChooser.addOption("test auto", SwerveAutoPaths.TestAutoPath());
    // autoCommandChooser.addOption("weird path", SwerveAutoPaths.WeirdPath());
    autoCommandChooser.addOption("Forward Right", null);

    SmartDashboard.putData("Autonomous Mode", autoCommandChooser);

    //Command set to run periodicly to register joystick inputs
    //It uses suppliers/mini methods to give up to date info easily
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickXAxis) * (forwardDirectionEntry.getBoolean(false) ? -1 : 1), 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickYAxis) * (forwardDirectionEntry.getBoolean(false) ? -1 : 1), 
        () -> -driverJoystick.getRawAxis(IOConstants.kDriveJoystickTurningAxis),
        () -> true
      )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  public void disableCameraUpdating() {
    visionSubsystem.removeDefaultCommand();
  }

  public void enableCameraUpdating() {
    visionSubsystem.setDefaultCommand(fieldUpdateCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Activates an Instant Command to reset field direction when button is pressed down
    new JoystickButton(driverJoystick, IOConstants.kZeroHeadingBtnID)
    .onTrue(new ZeroRobotHeading(swerveSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Get the trajectory selected in the drop down on the shuffleboard

    // if (chosenTrajectory == null) return null;
    // //Controllers to keep the robot on the main path since it will not follow it too well without it
    // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // //Creates a command that will run the robot along the path
    // SwerveControllerCommand swerveAutoCommand = new SwerveControllerCommand(
    // chosenTrajectory, 
    // swerveSubsystem::getPose, 
    // WheelConstants.kDriveKinematics,
    // xController,
    // yController,
    // thetaController,
    // swerveSubsystem::setModuleStates,
    // swerveSubsystem);

    Autos.setSwerveSubsystem(swerveSubsystem);
    CommandSelector commandSeleted = autoCommandChooser.getSelected();

    Supplier<Command> autoCommandSupplier = Autos.commandHashMap.get(commandSeleted);
    return autoCommandSupplier.get();
  }
}
