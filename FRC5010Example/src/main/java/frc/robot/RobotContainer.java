// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.GenericMechanism;
import frc.robot.FRC5010.VisionSystem;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.PersistedEnums;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.mechanisms.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends GenericMechanism {
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> command = new SendableChooser<>();
  private Controller driver;
  private Controller operator;
  private Drive drive;
  private VisionSystem vision;
  private static Alliance alliance;
  private Mechanism2d robotVisual;
  public static Constants constants;

  // Examples of how to use a persisted constants
  // These can live in specific constants files, however
  private static Persisted<Integer> driveVisualH;
  private static Persisted<Integer> driveVisualV;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Create a Mechanism2d display for simulating robot functions
    super(new Mechanism2d(10,10));
    constants = new Constants();

    driveVisualH = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_H, 60);
    driveVisualV = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_V, 60);
    drivetrainVisual = new Mechanism2d(driveVisualH.getInteger(), driveVisualV.getInteger());

    // Setup controllers
    driver = new Controller(Controller.JoystickPorts.ZERO.ordinal());
    operator = new Controller(Controller.JoystickPorts.ONE.ordinal());
    if (!operator.isPluggedIn()) {
      operator = driver;
      driver.setSingleControllerMode(true);
    }

    alliance = determineAllianceColor();
    initRealOrSim();

    // Put Mechanism 2d to SmartDashboard
    robotVisual = new Mechanism2d(PersistedEnums.ROBOT_VISUAL_H.getInteger(), RobotConstantsDef.robotVisualV.getInteger());
    SmartDashboard.putData("Drivetrain Visual", drivetrainVisual);
    SmartDashboard.putData("Robot Visual", robotVisual);
    drive = new Drive(driver, vision, drivetrainVisual);

    /** 
     * TODO: Add other mechanisms 
     * Pass robotVisual to the mechanisms in order to visualize the robot
     * */

    initAutoCommands();
    // Configure the button bindings
    configureButtonBindings(driver, operator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings(Controller driver, Controller operator) {
    drive.configureButtonBindings(driver, operator);
    if (driver.isSingleControllerMode()) {
      // TODO: Add code to handle single driver mode
    } else {
      if (RobotBase.isReal()) {
      }
    }
  }

  /**
   * For things being initialized in RobotContainer, provide a simulation version
   */
  protected void initRealOrSim() {
    if (RobotBase.isReal()) {
      /**
       * TODO: Initialize expected vision subsystem
       */
      VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1);
      multiVision.addPhotonCamera("photonvision", 
        new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
          new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)), 
          new Rotation3d(0, Units.degreesToRadians(-20), 0)
        )
      );
    } else {
      vision = new VisionLimeLightSim("limelight-sim", 1);
    }
  }
  
  public Alliance determineAllianceColor() {
    Alliance color = DriverStation.getAlliance();
    if (Alliance.Red.equals(color)) {
      /**
       * TODO: What to setup if alliance is Red
       */
    } else if (Alliance.Blue.equals(color)) {
      /**
       * TODO: What to setup if alliance is Blue
       */
    } else {
      /**
       * TODO: What to setup if alliance is not set?
       */
    }
    // Return alliance color so that setup functions can also store/pass
    return color;
  }
  public static Alliance getAlliance() { return alliance; }
  
  // Just sets up defalt commands (setUpDeftCom)
  public void setupDefaultCommands() {
    if (!DriverStation.isTest()) {
      drive.setupDefaultCommands();
      /**
       * TODO: Call setupDefaultCommands for other mechanisms
       */
    } else {
      /**
       * TODO: Test mode default commands
       */
    }

  }
  private void initAutoCommands() {
    /**
     * TODO: Add actual auto commands
     */
    command.addOption("Do nothing", new InstantCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return command.getSelected();
  }
}
