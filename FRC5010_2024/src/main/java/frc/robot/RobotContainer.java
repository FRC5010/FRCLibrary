// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.Persisted;
import org.frc5010.common.arch.WpiHelperInterface;
import org.frc5010.common.arch.WpiNetworkTableValuesHelper;
import org.frc5010.common.config.RobotParser;
import org.frc5010.common.telemetery.WpiDataLogging;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.robots.BabySwerve;
import frc.robot.FRC5010.robots.CurtsLaptopSimulator;
import frc.robot.FRC5010.robots.DefaultRobot;
import frc.robot.FRC5010.robots.PracticeBot;
import frc.robot.chargedup.CompBot_2023_T1G3R;
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
public class RobotContainer implements WpiHelperInterface {
	// The robot's subsystems and commands are defined here...
	private SendableChooser<Command> command;
	public static Constants constants;
	private GenericRobot robot;
	private static String MAC_Address = "MAC ADDRESS";
	private DigitalInput startupBypass;

	public enum LogLevel {
		DEBUG,
		COMPETITION
	}

	public static LogLevel logLevel = LogLevel.DEBUG;

	// Different power modes for demo and competition
    public enum PowerMode {
        DEMO,
        COMPETITION
    }

    public static PowerMode powerMode = PowerMode.COMPETITION;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Create a Mechanism2d display for simulating robot functions
		constants = new Constants();

		startupBypass = new DigitalInput(9); // TODO: Add correct port

		initRealOrSim();

		// Configure the button bindings
		configureButtonBindings();
		initAutoCommands();
		WpiNetworkTableValuesHelper.loadRegisteredToNetworkTables();
	}

	public static LogLevel getLoggingLevel() {
		return logLevel;
	}

	public static void setLoggingLevel(LogLevel level) {
		logLevel = level;
	}

	public static PowerMode getPowerMode() {
        return powerMode;
    }

    public static void setPowerMode(PowerMode mode) {
        powerMode = mode;
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
			WpiDataLogging.start(true);
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
			SmartDashboard.putString(MAC_Address, whichRobot.toString());
			return whichRobot.toString();
		} catch (SocketException e) {
			e.printStackTrace();
		}
		return "unknown";
	}

	private void robotFactory() {
		String whichRobot = whereAmI();
		try {
			if (!startupBypass.get()) {
				robot = new CompBot_2024();
				log("Bypassed MAC Address Switch");
			} else {
				switch (whichRobot) {
					case Robots.KIT_BOT_2024: {
						// robot = new KitBot2024(mechVisual, shuffleTab);
						robot = new CompBot_2024();
						break;
					}
					case Robots.COMP_BOT_2023: {
						robot = new CompBot_2023_T1G3R();
						break;
					}
					case Robots.BABY_SWERVE: {
						robot = new BabySwerve();
						break;
					}
					case Robots.PRACTICE_BOT: {
						robot = new PracticeBot();
						break;
					}
					case Robots.MAIN_5010_LAPTOP: {
						robot = new CompBot_2024();
						// robot = new CompBot_2023_T1G3R(mechVisual, shuffleTab);
						break;
					}
					case Robots.CURTS_LAPTOP_SIM: {
						switch (whoAmI.get()) {
							case "BabySwerve": {
								robot = new BabySwerve();
								break;
							}
							case "T1G3R": {
								robot = new CompBot_2023_T1G3R();
								break;
							}
							default: {
								robot = new CurtsLaptopSimulator("basic_robot");
								break;
							}
						}
						break;
					}
					default: {
						robot = new DefaultRobot();
						break;
					}

				}
			}
		} catch (Exception e) {
			e.printStackTrace();
			return;
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
	public void configureButtonBindings() {
		robot.configureButtonBindings();
	}

	// Just sets up defalt commands (setUpDeftCom)
	public void setupDefaults() {
		robot.setupDefaultCommands();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return robot.getAutonomousCommand();
	}

	public void initAutoCommands() {
		robot.buildAutoCommands();
	}

	public void disabledBehavior() {
		robot.disabledBehavior();
	}
}
