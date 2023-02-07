// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.PersistedEnums;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.robots.RobotFactory;
import frc.robot.FRC5010.robots.RobotFactory.Parts;
import frc.robot.FRC5010.sensors.Controller;

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
  private GenericMechanism drive;
  private VisionSystem vision;
  private static Alliance alliance;
  public static Constants constants;
  private RobotFactory robotFactory;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Create a Mechanism2d display for simulating robot functions
    constants = new Constants();

    // Setup controllers
    driver = new Controller(Controller.JoystickPorts.ZERO.ordinal());
    operator = new Controller(Controller.JoystickPorts.ONE.ordinal());
    if (!operator.isPluggedIn()) {
      operator = driver;
      driver.setSingleControllerMode(true);
    }

    // Put Mechanism 2d to SmartDashboard
    mechVisual = new Mechanism2d(PersistedEnums.ROBOT_VISUAL_H.getInteger(), RobotConstantsDef.robotVisualV.getInteger());
    SmartDashboard.putData("Robot Visual", mechVisual);

    alliance = determineAllianceColor();
    initRealOrSim();

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
    } else {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();
      instance.stopServer();
      // set the NT server if simulating this code.
      // "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]" for coprocessor
      instance.setServer("localhost");
      instance.startClient4("myRobot");
    }

    robotFactory = new RobotFactory();
    vision = (VisionSystem)robotFactory.getParts().get(RobotFactory.Parts.VISION);
    drive = (GenericMechanism)robotFactory.getParts().get(RobotFactory.Parts.DRIVE);
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
    Map<String,Command> parts = (Map<String,Command>) robotFactory.getParts().get(Parts.AUTO);
    
    if (null != parts){
      for (String name : parts.keySet()){
          command.addOption(name, parts.get(name));
      }
    }
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

  @Override
  public Map<String,Command> setAutoCommands(Map<String,List<PathPlannerTrajectory>> paths, HashMap<String, Command> eventMap) {
    // TODO Auto-generated method stub
    return null;
  }
}
