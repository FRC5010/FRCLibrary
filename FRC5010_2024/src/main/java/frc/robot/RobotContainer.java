// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.FRC5010.arch.Persisted;
import frc.robot.FRC5010.arch.PersistedEnums;
import frc.robot.FRC5010.arch.WpiNetworkTableValuesHelper;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.FRC5010.robots.BabySwerve;
import frc.robot.FRC5010.robots.CurtsLaptopSimulator;
import frc.robot.FRC5010.robots.DefaultRobot;
import frc.robot.FRC5010.robots.PracticeBot;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.subsystems.Color;
import frc.robot.FRC5010.telemetery.WpiDataLogging;
import frc.robot.chargedup.CompBot_2023_T1G3R;
import frc.robot.chargedup.KitBot2024;
import frc.robot.crescendo.CompBot_2024;

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
	private SendableChooser<Command> command;
	private Controller driver;
	private Controller operator;
	private static Alliance alliance;
	public static Constants constants;
	private GenericMechanism robot;
	private static String MAC_Address = "MAC ADDRESS";
	private DigitalInput startupBypass;

	public enum LogLevel {
		DEBUG,
		PRODUCTION
	}

	public static LogLevel logLevel = LogLevel.DEBUG;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		super("Robot");
		// Create a Mechanism2d display for simulating robot functions
		constants = new Constants();
		values.declare(MAC_Address, "");

		startupBypass = new DigitalInput(9); // TODO: Add correct port

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

		DriverStation.silenceJoystickConnectionWarning(true);
		alliance = determineAllianceColor();
		SmartDashboard.putString("Alliance", alliance.toString());

		initRealOrSim();

		// Configure the button bindings
		configureButtonBindings(driver, operator);
		initAutoCommands();
		WpiNetworkTableValuesHelper.loadRegisteredToNetworkTables();
	}

	public static LogLevel getLoggingLevel() {
		return logLevel;
	}

	public static void setLoggingLevel(LogLevel level) {
		logLevel = level;
	}

	public static String WHO_AM_I = "WhoAmI";
	private static Persisted<String> whoAmI = new Persisted<>(WHO_AM_I, "Simulator");

	// Robot types
	public static class Robots {
		public static final String KIT_BOT_2024 = "00:80:2F:34:B2:C5";
		public static final String COMP_BOT_2023 = "00:80:2F:33:04:33";
		public static final String BABY_SWERVE = "BabySwerve";
		public static final String PRACTICE_BOT = "PracticeBot";
		public static final String CURTS_LAPTOP_SIM = "4E:82:A9:77:48:61";// "D2:57:7B:3E:C0:47";
		public static final String MAIN_5010_LAPTOP = "04:EC:D8:22:DA:59";
	}

	/**
	 * For things being initialized in RobotContainer, provide a simulation version
	 */
	protected void initRealOrSim() {
		if (RobotBase.isReal()) {
			WpiDataLogging.start(false);
		} else {
			WpiDataLogging.start(false);
			// NetworkTableInstance instance = NetworkTableInstance.getDefault();
			// instance.stopServer();
			// set the NT server if simulating this code.
			// "localhost" for photon on desktop, or "photonvision.local" / "[ip-address]"
			// for coprocessor
			// instance.setServer("localhost");
			// instance.startClient4("myRobot");
		}

		robotFactory();
	}

	private String whereAmI() {
		try {
			NetworkInterface myNI = NetworkInterface.networkInterfaces().filter(it -> {
				try {
					byte[] MA = it.getHardwareAddress();
					return null != MA;
				} catch (Exception e) {
					e.printStackTrace();
				}
				return false;
			}).findFirst().orElse(NetworkInterface.networkInterfaces().findFirst().get());
			byte[] MAC_ADDRESS = myNI.getHardwareAddress();
			final List<Byte> macList = new ArrayList<>();
			if (null != MAC_ADDRESS) {
				for (byte b : MAC_ADDRESS) {
					macList.add(b);
				}
			}
			String whichRobot = macList.stream().map(it -> String.format("%02X", it))
					.collect(Collectors.joining(":"));
			values.set(MAC_Address, whichRobot.toString());
			return whichRobot.toString();
		} catch (SocketException e) {
			e.printStackTrace();
		}
		return "unknown";
	}

	private void robotFactory() {
		String whichRobot = whereAmI();
		if (!startupBypass.get()) {
			robot = new CompBot_2024(mechVisual, shuffleTab);
			log("Bypassed MAC Address Switch");
		} else {
			switch (whichRobot) {
				case Robots.KIT_BOT_2024: {
					robot = new KitBot2024(mechVisual, shuffleTab);
					// robot = new CompBot_2024(mechVisual, shuffleTab);
					break;
				}
				case Robots.COMP_BOT_2023: {
					robot = new CompBot_2023_T1G3R(mechVisual, shuffleTab);
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
				case Robots.MAIN_5010_LAPTOP: {
					robot = new CompBot_2024(mechVisual, shuffleTab);
					// robot = new CompBot_2023_T1G3R(mechVisual, shuffleTab);
					break;
				}
				case Robots.CURTS_LAPTOP_SIM: {
					switch (whoAmI.get()) {
						case "BabySwerve": {
							robot = new BabySwerve(mechVisual, shuffleTab);
							break;
						}
						case "T1G3R": {
							robot = new CompBot_2023_T1G3R(mechVisual, shuffleTab);
							break;
						}
						case "Simulator": {
							robot = new CurtsLaptopSimulator(mechVisual, shuffleTab);
							break;
						}
						default: {
							robot = new CompBot_2024(mechVisual, shuffleTab);
						}
					}
					break;
				}
				default: {
					robot = new DefaultRobot(mechVisual, shuffleTab);
					break;
				}

			}
		}
		log(">>>>> MAC ADDRESS: " + whichRobot + "<<<<");
		log(">>>>>>>>>> Running " + robot.getClass().getSimpleName() + " <<<<<<<<<<");

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
		if (DriverStation.isTeleop()) {
			robot.setupDefaultCommands(driver, operator);
		} else if (DriverStation.isTest()) {
			robot.setupTestDefaultCommmands(driver, operator);
		}
	}

	// Just sets up defalt commands (setUpDeftCom)
	public void setupDefaults() {
		alliance = determineAllianceColor();
		SmartDashboard.putString("Alliance", alliance.toString());

		AprilTags.aprilTagFieldLayout
				.setOrigin(RobotContainer.getAlliance() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
						: OriginPosition.kRedAllianceWallRightSide);

		setupDefaultCommands(driver, operator);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		return generateAutoCommand(command.getSelected());
	}

	@Override
	public void initAutoCommands() {
		robot.initAutoCommands();
		// This crashes 2024 right now
		command = AutoBuilder.buildAutoChooser();
		if (null != command) {
			shuffleTab.add("Auto Modes", command).withSize(2, 1);
		}

	}

	public Alliance determineAllianceColor() {
		Optional<Alliance> color = DriverStation.getAlliance();
		return color.orElse(Alliance.Blue);
	}

	public static Color chooseAllianceColor() {
		Optional<Alliance> alllianceColor = DriverStation.getAlliance();
		if (alllianceColor.isPresent()) {
			return alllianceColor.get() == Alliance.Red ? Color.RED : Color.BLUE;
		}
		return Color.ORANGE;
	}

	public static Alliance getAlliance() {
		return alliance;
	}

	public void disabledBehavior() {
		robot.disabledBehavior();
	}

	@Override
	public Command generateAutoCommand(Command autoCommand) {
		return robot.generateAutoCommand(autoCommand);
	}
}
