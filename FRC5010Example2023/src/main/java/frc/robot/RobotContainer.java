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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.PersistedEnums;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.robots.BabySwerve;
import frc.robot.FRC5010.robots.CurtsLaptopSimulator;
import frc.robot.FRC5010.robots.PracticeBot;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.chargedup.CompBot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends GenericMechanism {
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> command = new SendableChooser<>();
  private Controller driver;
  private Controller operator;
  private static Alliance alliance;
  public static Constants constants;
  private GenericMechanism robot;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super("Robot");
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
    mechVisual = new Mechanism2d(PersistedEnums.ROBOT_VISUAL_H.getInteger(),
        RobotConstantsDef.robotVisualV.getInteger());
    SmartDashboard.putData("Robot Visual", mechVisual);

    alliance = determineAllianceColor();
    initRealOrSim();

    initAutoCommands();
    // Configure the button bindings
    configureButtonBindings(driver, operator);
  }

  public static String WHO_AM_I = "WhoAmI";
  private Persisted<String> whoAmI;

  // Robot types
  public static class Robots {
    public static final String COMP_BOT_2023 = "2023CompBot";
    public static final String BABY_SWERVE = "BabySwerve";
    public static final String PRACTICE_BOT = "PracticeBot";
    public static final String CURTS_LAPTOP_SIM = "CurtsLaptop";
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
      // "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]"
      // for coprocessor
      instance.setServer("localhost");
      instance.startClient4("myRobot");
    }

    robotFactory();
  }

  private void robotFactory() {
    whoAmI = new Persisted<>(WHO_AM_I, String.class);
    String whichRobot = whoAmI.get();

    switch (whichRobot) {
      case Robots.COMP_BOT_2023: {
        robot = new CompBot(mechVisual, shuffleTab);
        break;
      }
      case Robots.BABY_SWERVE: {
        robot = new BabySwerve(mechVisual, shuffleTab);
        break;
      }
      case Robots.PRACTICE_BOT: {
        robot = new PracticeBot(mechVisual, shuffleTab);
        break;
      }
      case Robots.CURTS_LAPTOP_SIM: {
        robot = new CurtsLaptopSimulator(mechVisual, shuffleTab);
        break;
      }
      default: {
        robot = new CompBot(mechVisual, shuffleTab);
        break;
      }
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings(Controller driver, Controller operator) {
    robot.configureButtonBindings(driver, operator);
    if (driver.isSingleControllerMode()) {
      // TODO: Add code to handle single driver mode
    } else {
      if (RobotBase.isReal()) {
      }
    }
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    if (!DriverStation.isTest()) {
      robot.setupDefaultCommands(driver, operator);
    } else {
      /**
       * TODO: Test mode default commands
       */
    }
  }

  // Just sets up defalt commands (setUpDeftCom)
  public void setupDefaultCommands() {
    setupDefaultCommands(driver, operator);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return command.getSelected();
  }

  @Override
  public Map<String, Command> initAutoCommands() {
    Map<String, Command> autoCommands = robot.initAutoCommands();
    command.setDefaultOption("Do nothing auto", new InstantCommand(() -> System.out.println("Auto Ran")));
    if (null != autoCommands) {
      for (String name : autoCommands.keySet()) {
        command.addOption(name, autoCommands.get(name));
      }
      shuffleTab.add("Auto Modes", command).withSize(2, 1);
    }
    return autoCommands;
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

  public static Alliance getAlliance() {
    return alliance;
  }
}
