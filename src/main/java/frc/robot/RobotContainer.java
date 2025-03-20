// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.BiFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.AlignAndScore;
import frc.robot.commands.AlignWithReef;
import frc.robot.commands.CollectCoralSequence;
import frc.robot.commands.FieldPositionUpdate;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.ReverseIntakeUntilStopped;
import frc.robot.commands.StopCollectCoral;
import frc.robot.commands.StowManipulator;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.UnStowManipulator;
import frc.robot.commands.ZeroRobotHeading;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.ReefSide;
import frc.robot.utils.AutoPosePosition;
import frc.robot.utils.Autos;
import frc.robot.utils.Autos.CommandSelector;
import frc.robot.subsystems.ManipulatorSubsystem.ClawHeightLevel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  private final Joystick driverJoystick = new Joystick(IOConstants.kDriveJoystickID);
  private final Joystick manipulatorJoystick = new Joystick(IOConstants.kManipulatorJoystickID);

  //Create a shuffleboard tab for the drivers to see all teleop info
  private static final ShuffleboardTab teleOpTab = Shuffleboard.getTab("Teleoperation");
  private static final ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

  private static final SendableChooser<CommandSelector> autoCommandChooser = new SendableChooser<>();
  public static final SendableChooser<AutoPosePosition> emergencyStartingPoseChooser = new SendableChooser<>();
  public static final SendableChooser<Boolean> sideOfFieldChooser = new SendableChooser<>();

  private final GenericEntry forwardDirectionEntry = teleOpTab.add("Reverse Field Forward", false)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .getEntry();

  private final FieldPositionUpdate fieldUpdateCommand = new FieldPositionUpdate(
    visionSubsystem, 
    swerveSubsystem
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Adds a dropdown list of auto commands
    autoCommandChooser.setDefaultOption("Do Nothing", null);
    for (CommandSelector autoCommand : Autos.commandHashMap.keySet()) {
      autoCommandChooser.addOption(autoCommand.toString(), autoCommand);
    }
    autoTab.add("Auto mode", autoCommandChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

    //Sets emergency starting pose dropdown
    Boolean firstStartingPositionAdded = false;
    for (String optionName : Autos.startingPositions.keySet()) {
      AutoPosePosition position = Autos.startingPositions.get(optionName);
      if (firstStartingPositionAdded) {
        emergencyStartingPoseChooser.addOption(optionName, position);
      } else {
        emergencyStartingPoseChooser.setDefaultOption(optionName, position);
        firstStartingPositionAdded = true;
      }
    }
    autoTab.add("Emergency Starting Pose", emergencyStartingPoseChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

    sideOfFieldChooser.setDefaultOption("Scoring table side", true);
    sideOfFieldChooser.addOption("Audience side", false);
    autoTab.add("Side of field scoring", sideOfFieldChooser)
    .withWidget(BuiltInWidgets.kComboBoxChooser);
    
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

    enableCameraUpdating();
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

    // new JoystickButton(manipulatorJoystick, 8).onTrue(
    //   new StowManipulator(manipulatorSubsystem, intakeSubsystem)
    // );

    // new JoystickButton(manipulatorJoystick, 3).onTrue(
    //   new UnStowManipulator(manipulatorSubsystem, intakeSubsystem, ClawHeightLevel.Level1)
    // );

    // new JoystickButton(manipulatorJoystick, 4).onTrue(
    //   new UnStowManipulator(manipulatorSubsystem, intakeSubsystem, ClawHeightLevel.Level2)
    // );

    // new JoystickButton(manipulatorJoystick, 2).onTrue(
    //   new UnStowManipulator(manipulatorSubsystem, intakeSubsystem, ClawHeightLevel.Level3)
    // );

    // new JoystickButton(manipulatorJoystick, 1).onTrue(
    //   new UnStowManipulator(manipulatorSubsystem, intakeSubsystem, ClawHeightLevel.Level4)
    // );

    // CollectCoralSequence collectSequence = new CollectCoralSequence(manipulatorSubsystem, intakeSubsystem);
    // StopCollectCoral stopCollectSequence = new StopCollectCoral(manipulatorSubsystem, intakeSubsystem);
    // SequentialCommandGroup stopCollectFullSequence = new SequentialCommandGroup(
    //   new WaitUntilCommand(() -> collectSequence.intakeCleared == true),
    //   new InstantCommand(() -> collectSequence.intakeCleared = false),
    //   new InstantCommand(() -> collectSequence.cancel()),
    //   new InstantCommand(() -> {
    //     stopCollectSequence.schedule();
    //   })
    // );

    // new JoystickButton(manipulatorJoystick, 7).onTrue(
    //   collectSequence
    // ).onFalse(
    //   stopCollectFullSequence.onlyIf(
    //     () -> collectSequence.isScheduled() && !stopCollectFullSequence.isScheduled()
    //   )
    // );

    // new JoystickButton(manipulatorJoystick, 5).whileTrue(
    //   new ReverseIntakeUntilStopped(intakeSubsystem)
    // );

    // new JoystickButton(driverJoystick, 6)
    // .whileTrue(new ReleaseCoral(manipulatorSubsystem));

    new JoystickButton(manipulatorJoystick, 1).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Left, ClawHeightLevel.Level4)
    );
    new JoystickButton(manipulatorJoystick, 2).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Right, ClawHeightLevel.Level4)
    );
    new JoystickButton(manipulatorJoystick, 3).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Left, ClawHeightLevel.Level3)
    );
    new JoystickButton(manipulatorJoystick, 4).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Right, ClawHeightLevel.Level3)
    );
    new JoystickButton(manipulatorJoystick, 5).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Left, ClawHeightLevel.Level2)
    );
    new JoystickButton(manipulatorJoystick, 6).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Right, ClawHeightLevel.Level2)
    );
    new JoystickButton(manipulatorJoystick, 7).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Left, ClawHeightLevel.Level1)
    );
    new JoystickButton(manipulatorJoystick, 8).onTrue(
      new AlignAndScore(swerveSubsystem, visionSubsystem, manipulatorSubsystem, intakeSubsystem, ReefSide.Right, ClawHeightLevel.Level1)
    );

    CollectCoralSequence collectSequence = new CollectCoralSequence(manipulatorSubsystem, intakeSubsystem);
    StopCollectCoral stopCollectSequence = new StopCollectCoral(manipulatorSubsystem, intakeSubsystem);
    SequentialCommandGroup stopCollectFullSequence = new SequentialCommandGroup(
      new WaitUntilCommand(() -> collectSequence.intakeCleared == true),
      new InstantCommand(() -> collectSequence.intakeCleared = false),
      new InstantCommand(() -> collectSequence.cancel()),
      new InstantCommand(() -> {
        stopCollectSequence.schedule();
      })
    );

    new JoystickButton(driverJoystick, 6).onTrue(
      collectSequence
    ).onFalse(
      stopCollectFullSequence.onlyIf(
        () -> collectSequence.isScheduled() && !stopCollectFullSequence.isScheduled()
      )
    );
    
    new JoystickButton(manipulatorJoystick, 5).whileTrue(
      new ReverseIntakeUntilStopped(intakeSubsystem)
    );

    new JoystickButton(driverJoystick, 7)
    .whileTrue(new ReleaseCoral(manipulatorSubsystem));

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

    if (!visionSubsystem.lookingAtAprilTag()) {
      System.out.println("NO CAMERA THINGY YAAAASSSSSSSSS");
      //If camera is not used
      swerveSubsystem.resetOdometry(emergencyStartingPoseChooser.getSelected().getPose());
      //Set robot position to emergency starting pose
      if ( emergencyStartingPoseChooser.getSelected().getPose().getTranslation().getDistance(Autos.startingPositions.get("Scoring side").getPose().getTranslation()) < 0.25 ) {
        //Swap field side if on scoring side
        AutoPosePosition.setFieldSideSwap(true);
      } else if ( emergencyStartingPoseChooser.getSelected().getPose().getTranslation().getDistance(Autos.startingPositions.get("Center side").getPose().getTranslation()) < 0.25 ) {
        //Use field side option if in center
        AutoPosePosition.setFieldSideSwap(sideOfFieldChooser.getSelected());
      }
    } else {
      //If camera is used

      //Get starting poses as Pose2d objects
      ArrayList<Pose2d> startingPoses = new ArrayList<>();
      for (AutoPosePosition posePosition : Autos.startingPositions.values()) {
        startingPoses.add(posePosition.getPose());
      }
      //Get closest pose to robot
      Pose2d nearestStartingPose = swerveSubsystem.getPose().nearest(startingPoses);

      if ( nearestStartingPose.equals(Autos.startingPositions.get("Scoring side").getPose()) ) {
        //Swap field side if on scoring side
        AutoPosePosition.setFieldSideSwap(true);
      } else if ( nearestStartingPose.equals(Autos.startingPositions.get("Center side").getPose()) ) {
        //Use field side option if in center
        AutoPosePosition.setFieldSideSwap(sideOfFieldChooser.getSelected());
      }
    }

    //Get selected command enum
    CommandSelector commandSeleted = autoCommandChooser.getSelected();

    if (commandSeleted == null) {
      return null;
    }

    //Get command supplier
    BiFunction<Pose2d, RobotContainer, Command> autoCommandSupplier = Autos.commandHashMap.get(commandSeleted);
    //return the command supplied
    return autoCommandSupplier.apply(swerveSubsystem.getPose(), this);
  }
}
