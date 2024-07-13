package org.frc5010.common.arch;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.frc5010.common.config.RobotParser;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.subsystems.Color;
import org.frc5010.common.telemetery.WpiDataLogging;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class GenericRobot extends GenericMechanism {
	private SendableChooser<Command> command;
	private Controller driver;
	private Controller operator;
	private static Alliance alliance;
	private Map<String, GenericSubsystem> subsystems = new HashMap<>();
	private Map<String, Controller> controllers = new HashMap<>();
	private RobotParser parser;

	public enum LogLevel {
		DEBUG,
		COMPETITION
	}

	public static LogLevel logLevel = LogLevel.DEBUG;

	public GenericRobot(String directory) {
		super(Class.class.getName());
		try {
			parser = new RobotParser(directory, this);
			parser.createRobot(this);

			driver = controllers.get("driver");
			operator = controllers.get("operator");
			if (!operator.isPluggedIn()) {
				operator = driver;
				driver.setSingleControllerMode(true);
			}
			DriverStation.silenceJoystickConnectionWarning(true);
			alliance = determineAllianceColor();
			values.declare("Alliance", alliance.toString());
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
	}

	public GenericRobot() {
		super(Class.class.getName());

		// Setup controllers
		driver = new Controller(Controller.JoystickPorts.ZERO.ordinal());
		operator = new Controller(Controller.JoystickPorts.ONE.ordinal());
		if (!operator.isPluggedIn()) {
			operator = driver;
			driver.setSingleControllerMode(true);
		}
		// Put Mechanism 2d to SmartDashboard
		mechVisual = new Mechanism2d(
				PersistedEnums.ROBOT_VISUAL_H.getInteger(),
				RobotConstantsDef.robotVisualV.getInteger());
		SmartDashboard.putData("Robot Visual", mechVisual);

		DriverStation.silenceJoystickConnectionWarning(true);
		alliance = determineAllianceColor();
		values.declare("Alliance", alliance.toString());
	}

	public GenericSubsystem getSubsystem(String name) {
		return subsystems.get(name);
	}

	public static LogLevel getLoggingLevel() {
		return logLevel;
	}

	public static void setLoggingLevel(LogLevel level) {
		logLevel = level;
	}

	@Override
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
		configureButtonBindings(driver, operator);
	}

	public void setupDefaultCommands() {
		if (DriverStation.isTeleop() || DriverStation.isAutonomous()) {
			setupDefaultCommands(driver, operator);
		} else if (DriverStation.isTest()) {
			setupTestDefaultCommmands(driver, operator);
		}
	}

	public void buildAutoCommands() {
		initAutoCommands();

		// TODO: Figure out Pathplanner Warmup Command

		command = AutoBuilder.buildAutoChooser();
		if (null != command) {
			shuffleTab.add("Auto Modes", command).withSize(2, 1);
		}
	}

	public Command getAutonomousCommand() {
		return generateAutoCommand(command.getSelected());
	}

	public Alliance determineAllianceColor() {
		Optional<Alliance> color = DriverStation.getAlliance();
		return color.orElse(Alliance.Blue);
	}

	public static Color chooseAllianceColor() {
		Optional<Alliance> allianceColor = DriverStation.getAlliance();
		if (allianceColor.isPresent()) {
			return allianceColor.get() == Alliance.Red ? Color.RED : Color.BLUE;
		}
		return Color.ORANGE;
	}

	public static Alliance getAlliance() {
		return alliance;
	}

	public void addController(String name, Controller controller) {
		controllers.put(name, controller);
	}

	public void addSubsystem(String name, GenericSubsystem subsystem) {
		subsystems.put(name, subsystem);
	}
}
