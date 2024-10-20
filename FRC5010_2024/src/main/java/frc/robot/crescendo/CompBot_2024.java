// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.function.Supplier;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.commands.JoystickToSwerve;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.swerve.MK4iSwerveModule;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.mechanisms.Drive;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.hardware.KrakenX60;
import org.frc5010.common.motors.hardware.NEO;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.LimeLightCamera;
import org.frc5010.common.sensors.camera.PhotonVisionCamera;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.sensors.gyro.PigeonGyro;
import org.frc5010.common.subsystems.AprilTagPoseSystem;
import org.frc5010.common.subsystems.Color;
import org.frc5010.common.subsystems.LEDStripSegment;
import org.frc5010.common.subsystems.PowerDistribution5010;
import org.frc5010.common.subsystems.SegmentedLedSystem;
import org.frc5010.common.vision.AprilTags;
import org.frc5010.common.vision.VisionLimeLight;
import org.frc5010.common.vision.VisionPhotonAprilTagTarget;
import org.frc5010.common.vision.VisionSystem.Rotation;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.crescendo.FeederSubsystem.NoteState;
import frc.robot.crescendo.commands.AutoAim;
import frc.robot.crescendo.commands.AutoIntake;
import frc.robot.crescendo.commands.PredefinedAutoShot;
import frc.robot.crescendo.commands.RunClimb;
import frc.robot.crescendo.commands.RunIntake;
import frc.robot.crescendo.commands.RunPivot;
import frc.robot.crescendo.commands.RunShooter;

/** Add your docs here. */
public class CompBot_2024 extends GenericRobot {
	private SwerveConstants swerveConstants;
	private MotorController5010 pivotMotor;
	private PivotSubsystem pivotSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private FeederSubsystem feederSubsystem;
	private KrakenX60 topShooterMotor;
	private KrakenX60 bottomShooterMotor;
	private MotorController5010 feederMotor;
	private IntakeSubsystem intakeSubsystem;
	private MotorController5010 innerIntakeMotor;
	private MotorController5010 outerIntakeMotor;

	private AprilTagPoseSystem visionSystem;
	private VisionPhotonAprilTagTarget shooterCamera;
	private VisionLimeLight noteCamera;
	private GenericGyro gyro;
	private ClimbSubsystem climbSubsystem;
	private MotorController5010 leftClimbMotor;
	private MotorController5010 rightClimbMotor;
	private Drive drive;
	private SegmentedLedSystem ledSubsystem;
	private LEDStripSegment allLEDs;
	private TargetingSystem targetingSystem;
	private PowerDistribution5010 powerDistribution5010;
	private boolean neverEnabled = true;

	Supplier<Command> autoIntake;
	Supplier<Command> autoIntakeTeleop;
	Supplier<Command> autoIntakeAtMax;
	Supplier<Command> runIntake;
	Supplier<Command> runIntakeWithPivotAtMax;
	Supplier<Command> stopDrivetrain;
	Supplier<Command> rumbleOperator;
	Supplier<Command> runShooter;
	Supplier<Command> runFeeder;
	Supplier<Command> spinIntake;
	Supplier<Command> blindShot;
	Supplier<Command> zeroClimb;
	Supplier<Command> stopShooter;
	Supplier<Command> justShoot;

	private boolean terminatePath = false;

	public CompBot_2024() {
		super();

		GenericRobot.setLoggingLevel(LogLevel.COMPETITION);

		ledSubsystem = new SegmentedLedSystem(0, 34, mechVisual);
		ledSubsystem.setWholeStripState((Integer i) -> GenericRobot.chooseAllianceColor().getColor8Bit());

		allLEDs = ledSubsystem.getStrip(ledSubsystem.ALL);
		allLEDs.chase(false);

		powerDistribution5010 = new PowerDistribution5010();

		// Motor Setup
		innerIntakeMotor = MotorFactory.KrakenX60(1); // 1
		outerIntakeMotor = MotorFactory.KrakenX60(5); // 5

		leftClimbMotor = MotorFactory.NEO(7); // TODO: Add correct port
		rightClimbMotor = MotorFactory.NEO(8).invert(true); // TODO: Add correct port
		pivotMotor = MotorFactory.NEO(9).invert(true);
		MotorFactory.NEO(10).setFollow(pivotMotor, true);
		feederMotor = MotorFactory.NEO(11); // MotorFactory.NEO(11);

		values.declare("FeederSpeed", 1.0);

		topShooterMotor = (KrakenX60) MotorFactory.KrakenX60(12).invert(true); // OLD RIGHT: -0.29845
		topShooterMotor.enableFOC(false);
		topShooterMotor.setCurrentLimit(100);
		bottomShooterMotor = (KrakenX60) MotorFactory.KrakenX60(14).invert(true);
		bottomShooterMotor.enableFOC(false);
		bottomShooterMotor.setCurrentLimit(100);

		visionSystem = new AprilTagPoseSystem();
		visionSystem.addCamera(new LimeLightCamera("top", 5, AprilTags.aprilTagFieldLayout, new Transform3d(), () -> RobotState.isDisabled() && neverEnabled));

		gyro = new PigeonGyro(13);

		// visionSystem.addLimeLightCameraAngle("orange", 0.3556, -10, 0, 1, null);

		pivotSubsystem = new PivotSubsystem(pivotMotor, mechVisual);

		climbSubsystem = new ClimbSubsystem(leftClimbMotor, rightClimbMotor, gyro,
				mechVisual);
		swerveConstants = new SwerveConstants(Units.inchesToMeters(Constants.Physical.TRACK_WIDTH_INCHES),
				Units.inchesToMeters(Constants.Physical.WHEEL_BASE_INCHES));

		// Setup Swerve Constants

		swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L3_KRAKEN_NEO);
		swerveConstants.configureSwerve(KrakenX60.MAXRPM, NEO.MAXRPM);
		swerveConstants.getSwerveModuleConstants().setkWheelDiameterMeters(0.103386904);
		swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontleft",
				new MotorFeedFwdConstants(0.21848, 2.3118, 0.20314)); // FL
		swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontright",
				new MotorFeedFwdConstants(0.18434, 2.3018, 0.30992)); // FR
		swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backleft",
				new MotorFeedFwdConstants(0.19542, 2.2924, 0.35934)); // BL
		swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backright",
				new MotorFeedFwdConstants(0.18096, 2.2915, 0.37156)); // BR

		swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(6);
		swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

		swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(3);
		swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
		swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(5.93);

		drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, null, swerveConstants,
				"mk4i_L3_kraken_neo");

		shooterSubsystem = new ShooterSubsystem(mechVisual, topShooterMotor, bottomShooterMotor);
		feederSubsystem = new FeederSubsystem(mechVisual, feederMotor, ledSubsystem);
		intakeSubsystem = new IntakeSubsystem(outerIntakeMotor, innerIntakeMotor, mechVisual);

		initRealOrSim();

		targetingSystem = new TargetingSystem(
				() -> TargetingSystem.getSpeakerTarget(GenericRobot.getAlliance()),
				() -> drive.getDrivetrain().getPoseEstimator().getCurrentPose3d(),
				(SwerveDrivetrain) drive.getDrivetrain(), shooterCamera, feederSubsystem, gyro);
		targetingSystem.useShooterCamera(false);
	}

	@Override
	public void configureButtonBindings(Controller driver, Controller operator) {
		/* >>>>>>>>>>>>>>>>>>>> ADDITIONAL AXIS <<<<<<<<<<<<<<<<<<<< */
		driver.setRightTrigger(driver.createRightTrigger().deadzone(0.07).negate());
		driver.setLeftTrigger(driver.createLeftTrigger().deadzone(0.07).negate());

		operator.setLeftYAxis(operator.createLeftYAxis().deadzone(0.07).negate());
		operator.setRightYAxis(operator.createRightYAxis().deadzone(0.07).negate());

		operator.setRightXAxis(operator.createRightXAxis().deadzone(0.07).negate());
		operator.setLeftXAxis(operator.createLeftXAxis().deadzone(0.07).negate());

		operator.setLeftTrigger(operator.createLeftTrigger());
		operator.setRightTrigger(operator.createRightTrigger());
		drive.configureButtonBindings(driver, operator);

		/* >>>>>>>>>>>>>>>>>>>> Common Commands <<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
		// runIntake = () -> new RunIntake(() -> -0.75, () -> 0.5, intakeSubsystem,
		// feederSubsystem,
		// pivotSubsystem,
		// null)
		// .until(() -> feederSubsystem.getNoteState() == NoteState.Loaded);

		runIntake = () -> Commands
				.deadline(
						Commands.idle().until(() -> feederSubsystem.getNoteState() != NoteState.Empty)
								.andThen(Commands.idle().withTimeout(1.5)
										.until(() -> feederSubsystem.getNoteState() == NoteState.Loaded)),
						new RunIntake(() -> -0.75, () -> 0.5, intakeSubsystem, feederSubsystem,
								pivotSubsystem, shooterSubsystem,
								null));

		runIntakeWithPivotAtMax = () -> Commands
				.deadline(
						Commands.idle().until(() -> feederSubsystem.getNoteState() != NoteState.Empty)
								.andThen(Commands.idle().withTimeout(1.5)
										.until(() -> feederSubsystem.getNoteState() == NoteState.Loaded)),
						new RunIntake(() -> -0.75, () -> 0.5, intakeSubsystem, feederSubsystem,
								pivotSubsystem, shooterSubsystem,
								null, pivotSubsystem.MAX_INTAKE_ANGLE));

		stopDrivetrain = () -> Commands.runOnce(() -> drive.getDrivetrain().drive(new ChassisSpeeds()),
				drive.getDrivetrain());

		autoIntake = () -> runIntake
				.get().until(() -> feederSubsystem.getNoteState() == NoteState.Holding).raceWith(
						new AutoIntake((SwerveDrivetrain) drive.getDrivetrain(),
								noteCamera))
				.andThen(stopDrivetrain.get()); // @TODO: Fix Feeder Spinning after Command End

		autoIntakeTeleop = () -> runIntake
				.get().until(() -> feederSubsystem.getNoteState() == NoteState.Holding).deadlineWith(
						new AutoIntake((SwerveDrivetrain) drive.getDrivetrain(),
								noteCamera))
				.andThen(stopDrivetrain.get()); // @TODO: Fix Feeder Spinning after Command End

		autoIntakeAtMax = () -> Commands.runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.MAX_INTAKE_ANGLE))
				.andThen(() -> runIntake
						.get())
				.until(() -> feederSubsystem.getNoteState() == NoteState.Holding).deadlineWith(
						new AutoIntake((SwerveDrivetrain) drive.getDrivetrain(),
								noteCamera))
				.andThen(stopDrivetrain.get()); // @TODO: Fix Feeder Spinning after Command End

		rumbleOperator = () -> Commands
				.deadline(Commands.waitSeconds(0.5), Commands.runOnce(() -> operator.setRumble(1)))
				.finallyDo(() -> operator.setRumble(0));

		runShooter = () -> Commands
				.run(() -> {
					shooterSubsystem.setShooterReference(Constants.Physical.SUBWOOFER_SHOT,
							Constants.Physical.SUBWOOFER_SHOT);

				});

		justShoot = () -> Commands.run(() -> feederSubsystem.feederStateMachine(-1.0))
				.until(() -> feederSubsystem.getNoteState() == NoteState.Empty)
				.finallyDo(() -> {
					feederSubsystem.feederStateMachine(0.0);
				});

		runFeeder = () -> Commands
				.run(() -> {
					feederSubsystem.feederStateMachine(-values.getDouble("FeederSpeed"));
					intakeSubsystem.setReference(-1000, -1000);
				},
						feederSubsystem)
				.finallyDo(() -> {
					feederSubsystem.feederStateMachine(0.0);
					intakeSubsystem.setReference(0, 0);
				});

		spinIntake = () -> Commands.startEnd(
				() -> intakeSubsystem.setReference(-500, -500),
				() -> intakeSubsystem.setReference(0, 0)).withTimeout(1.5);

		blindShot = () -> Commands.run(() -> feederSubsystem.feederStateMachine(-1.0))
				.deadlineWith(spinIntake.get())
				.until(() -> feederSubsystem.getNoteState() == NoteState.Empty)
				.finallyDo(() -> {
					feederSubsystem.feederStateMachine(0.0);
				});

		stopShooter = () -> Commands.runOnce(() -> {
			shooterSubsystem.setShooterReference(0, 0);
		});

		zeroClimb = () -> (Commands.run(() -> {
			climbSubsystem.setOverride(true);
			climbSubsystem.setLeftMotorSpeed(-0.5);
			climbSubsystem.setRightMotorSpeed(-0.5);
		}, climbSubsystem)
				.until(() -> {
					return climbSubsystem.isAtZero();
				}))
				.andThen(Commands
						.runOnce(() -> {
							climbSubsystem.setOverride(false);
							climbSubsystem.setLeftMotorSpeed(0);
							climbSubsystem.setRightMotorSpeed(0);
						},
								climbSubsystem));

		if (DriverStation.isTest()) {
			return;
		}

		/*
		 * >>>>>>>>>>>>>>>>>>>> DRIVER BUTTONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		 */
		/**
		 * Drive Axis - Left X & Y, Right X
		 * Reset Orientation - Start
		 * Lock Wheels - Left Bumper
		 * Field Oriented - B Button
		 */

		JoystickButton autoaimButton = driver.createAButton();

		autoaimButton.whileTrue(
				new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem,
						gyro, () -> (JoystickToSwerve) drive.getDefaultCommand(), false, false)
						.alongWith(spinIntake.get().withTimeout(0.25)))
				.onFalse(Commands.waitSeconds(0.05).beforeStarting(() -> feederSubsystem.setShotReadyness(false))
						.andThen(
								Commands.runOnce(() -> pivotSubsystem
										.setReference(pivotSubsystem.HOME_LEVEL))
										.unless(() -> autoaimButton.getAsBoolean())));

		driver.createYButton().whileTrue(Commands.startEnd(() -> climbSubsystem.setOverride(true),
				() -> climbSubsystem.setOverride(false)));

		driver.createXButton().whileTrue(
				autoIntake.get().until(() -> feederSubsystem.getNoteState() == NoteState.Holding));

		// driver.createRightBumper()
		// .whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
		// () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
		// () -> drive.getDrivetrain().getPoseEstimator()
		// .getPoseFromClosestVisionTarget(),
		// null,
		// TranslationConstants.noteTransform)
		// .deadlineWith(new RunIntake(() -> 1.0, () -> 0.5, intakeSubsystem,
		// feederSubsystem)));

		driver.createRightBumper()
				.onTrue(Commands.runOnce(() -> {
					climbSubsystem.enableClimb(true);
					climbSubsystem.enableAutoClimb(true);
				})
						.andThen(rumbleOperator.get()));

		driver.createUpPovButton().onTrue(Commands.runOnce(() -> {
			pivotSubsystem.setSlowdown(pivotSubsystem.getSlowdown() + 0.1);
		}, pivotSubsystem));
		driver.createDownPovButton().onTrue(Commands.runOnce(() -> {
			pivotSubsystem.setSlowdown(pivotSubsystem.getSlowdown() - 0.1);
		}, pivotSubsystem));

		driver.createBackButton()
				.whileTrue(Commands.runOnce(() -> feederSubsystem.setNoteState(NoteState.Empty)));

		/*
		 * >>>>>>>>>>>>>>>>>>>>>>>>>>>> OPERATOR BUTTONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		 */
		// Home Pivot
		operator.createAButton().onTrue(Commands.parallel(
				Commands.runOnce(() -> {
					pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
				}, pivotSubsystem),
				Commands.run(() -> {
					climbSubsystem.setLeftMotorSpeed(climbSubsystem.leftIsAtMin(-0.5) ? 0 : -1);
					climbSubsystem.setRightMotorSpeed(climbSubsystem.rightIsAtMin(-0.5) ? 0 : -1);
				}, climbSubsystem).until(() -> climbSubsystem.leftIsAtMin(-0.1)
						&& climbSubsystem.rightIsAtMin(-0.1))

		));
		JoystickButton subwooferButton = operator.createXButton();

		// Podium Pivot
		subwooferButton.onTrue(Commands.runOnce(
				() -> {
					SmartDashboard.putBoolean("X Button", true);
					pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
					shooterSubsystem.setShooterReference(Constants.Physical.SUBWOOFER_SHOT,
							Constants.Physical.SUBWOOFER_SHOT);
				}, pivotSubsystem)).onFalse(Commands.waitSeconds(0.5).beforeStarting(() -> SmartDashboard.putBoolean("X Button", false)).andThen(Commands.runOnce(() -> {
					shooterSubsystem.setShooterReference(0, 0);
					feederSubsystem.setFeederReference(0);
					pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
				}).onlyIf(() -> !subwooferButton.getAsBoolean())));

		// Amp Pivot
		operator.createYButton().whileTrue(Commands
				.runOnce(() -> {
					pivotSubsystem.setReference(pivotSubsystem.AMP_LEVEL);
					shooterSubsystem.setShooterReference(Constants.Physical.AMP_SHOOTING_SPEED,
							Constants.Physical.AMP_SHOOTING_SPEED);
				}).alongWith(spinIntake.get())

		).onFalse(Commands.runOnce(() -> {
			pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
			shooterSubsystem.setShooterReference(0, 0);
		}));

		// Trap Pivot
		operator.createBButton().onTrue(spinIntake.get().alongWith(Commands.runOnce(
				() -> {
					// pivotSubsystem.setReference(pivotSubsystem.HIGH_SHUTTLE_LEVEL);
					shooterSubsystem.setShooterReference(Constants.Physical.SHUTTLE_SPEED,
							Constants.Physical.SHUTTLE_SPEED);
				}, pivotSubsystem))).onFalse(Commands.runOnce(() -> {
					// pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
					shooterSubsystem.setShooterReference(0, 0);
				}));

		// Run Shooter motors
		operator.createRightBumper().whileTrue(
				Commands.run(() -> shooterSubsystem.setShooterReference(Constants.Physical.MANUAL_SHOOTING_SPEED,
						Constants.Physical.MANUAL_SHOOTING_SPEED)).finallyDo(() -> {
							shooterSubsystem.setShooterReference(0, 0);
							intakeSubsystem.setReference(0, 0);
						}));
		// Feed Note into Shooter
		operator.createLeftBumper().whileTrue(runFeeder.get().beforeStarting(Commands.runOnce(() -> SmartDashboard.putBoolean("Left Bumper", true)))).onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Left Bumper", false)));

		// Shooter micro adjust
		operator.createUpPovButton().onTrue(spinIntake.get());
		operator.createDownPovButton().onTrue(Commands.runOnce(() -> intakeSubsystem.setReference(0, 0)));
		// Pivot micro adjust
		operator.createLeftPovButton().onTrue(pivotSubsystem.adjustReferenceDown());
		operator.createRightPovButton().onTrue(pivotSubsystem.adjustReferenceUp());

		operator.createBackButton()
				.onTrue(zeroClimb.get());

		operator.createStartButton()
				.onTrue(Commands.runOnce(() -> climbSubsystem
						.setAutobalanceMode(!climbSubsystem.getAutobalanceMode()))
						.andThen(rumbleOperator.get()
								.onlyIf(() -> climbSubsystem.getAutobalanceMode())));

		/*
		 * >>>>>>>>>>>>>>>>>>>>>>>>>>>> BOARD BUTTONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		 */
	}

	@Override
	public void setupDefaultCommands(Controller driver, Controller operator) {
		drive.setupDefaultCommands(driver, operator);

		pivotSubsystem.setDefaultCommand(new RunPivot(
				() -> (operator.getRightTrigger() - operator.getLeftTrigger()) * 0.5, pivotSubsystem));

		shooterSubsystem.setDefaultCommand(
				new RunShooter(() -> 0.0, shooterSubsystem));

		intakeSubsystem.setDefaultCommand(
				new RunIntake(() -> driver.getRightTrigger() - driver.getLeftTrigger(),
						() -> 1.0,
						intakeSubsystem,
						feederSubsystem,
						pivotSubsystem, shooterSubsystem, driver));

		climbSubsystem.setDefaultCommand(new RunClimb(() -> operator.getLeftYAxis(),
				() -> operator.getRightYAxis(), climbSubsystem));

		// ledSubsystem.setDefaultCommand(new LEDStateHandler(ledSubsystem, () ->
		// feederSubsystem.getNoteState()));

		// feederSubsystem.setDefaultCommand(new RunFeeder(feederSubsystem, () -> 0.0));
		ledSubsystem.setDefaultCommand(Commands.run(() -> {
		}, ledSubsystem)
				.finallyDo(() -> ledSubsystem.getStrip(ledSubsystem.ALL).rainbow()));
		ledSubsystem.getStrip(ledSubsystem.ALL).setColor(GenericRobot.chooseAllianceColor());
	}

	@Override
	public void setupTestDefaultCommmands(Controller driver, Controller operator) {
		pivotSubsystem.removeDefaultCommand();
		shooterSubsystem.removeDefaultCommand();
		intakeSubsystem.removeDefaultCommand();

		drive.setupTestDefaultCommands(driver, operator);
		driver.createYButton().whileTrue(((YAGSLSwerveDrivetrain) drive.getDrivetrain()).sysIdDriveMotorCommand());
		driver.createXButton().whileTrue(((YAGSLSwerveDrivetrain) drive.getDrivetrain()).sysIdAngleMotorCommand());

		// driver.createStartButton().whileTrue(feederSubsystem.getFeederSysIdRoutineCommand());
		// // driver.createXButton().whileTrue(pivotSubsystem.getSysIdCommand());
		// driver.createLeftBumper().whileTrue(intakeSubsystem.getBottomSysIdRoutineCommand());
		// driver.createRightBumper().whileTrue(intakeSubsystem.getTopSysIdRoutineCommand());

		pivotSubsystem.setDefaultCommand(new RunPivot(
				() -> operator.getRightTrigger() - operator.getLeftTrigger(), pivotSubsystem));
		shooterSubsystem.setDefaultCommand(
				new RunShooter(() -> 0.0, shooterSubsystem));

		intakeSubsystem.setDefaultCommand(Commands.run(
				() -> intakeSubsystem.setIntakeSpeed(driver.getLeftTrigger() - driver.getRightTrigger(),
						driver.getLeftTrigger() - driver.getRightTrigger()),
				intakeSubsystem));

		climbSubsystem.setDefaultCommand(new RunClimb(() -> operator.getLeftYAxis(),
				() -> operator.getRightYAxis(), climbSubsystem));

		ledSubsystem.setDefaultCommand(Commands.run(() -> {
		}, ledSubsystem)
				.finallyDo(() -> ledSubsystem.getStrip(ledSubsystem.ALL).rainbow()));
		ledSubsystem.getStrip(ledSubsystem.ALL).setColor(GenericRobot.chooseAllianceColor());

		driver.createAButton().whileTrue(
				new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem,
						gyro, () -> (JoystickToSwerve) drive.getDefaultCommand(), false, false)
						.alongWith(spinIntake.get().withTimeout(0.25)));

		// driver.createBButton().whileTrue(
		// 		new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem,
		// 				gyro, () -> (JoystickToSwerve) drive.getDefaultCommand(), true, true)
		// 				.alongWith(spinIntake.get().withTimeout(0.25)));

		driver.createBButton().whileTrue(new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem, gyro,
						false, false, false)
						.deadlineWith(spinIntake.get().withTimeout(0.25))
						.beforeStarting(() -> {
							targetingSystem.setTolerance(0.05);
							pivotSubsystem.setTolerance(1.5);
							shooterSubsystem.setTolerance(3.0);
						})
						.finallyDo(() -> {
							pivotSubsystem
									.setReference(pivotSubsystem.HOME_LEVEL);
							targetingSystem.resetToleranceToDefaults();
							pivotSubsystem.resetToleranceToDefaults();
							shooterSubsystem.resetToleranceToDefaults();
						}));

		// Run Shooter motors
		driver.createBackButton().whileTrue(runShooter.get()
				.until(() -> shooterSubsystem.isAtTarget())
				.andThen(runFeeder.get().until(() -> feederSubsystem.isEmptied()))
				.finallyDo(() -> {
					shooterSubsystem.setShooterReference(0, 0);
					intakeSubsystem.setReference(0, 0);
					feederSubsystem.setFeederReference(0);
				}));

		// Pivot Microadjust
		driver.createLeftPovButton().onTrue(pivotSubsystem.adjustReferenceDown());
		driver.createRightPovButton().onTrue(pivotSubsystem.adjustReferenceUp());

		// Used for interpolation
		driver.createRightBumper().whileTrue(
				Commands.run(() -> shooterSubsystem.setShooterReference(targetingSystem.getShooterSpeed(),
						targetingSystem.getShooterSpeed())).finallyDo(() -> {
							shooterSubsystem.setShooterReference(0, 0);
							intakeSubsystem.setReference(0, 0);
						}));
		// Feed Note into Shooter
		driver.createLeftBumper().whileTrue(runFeeder.get());

		driver.createDownPovButton()
				.onTrue(zeroClimb.get());


		operator.createXButton().onTrue(autoIntake.get());		
		operator.createBButton().onTrue(new PredefinedAutoShot(
				AutoShotDefinition.RIGHT_SHOT_MID.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enablePivot()
				);

		operator.createRightBumper().whileTrue(
				Commands.run(() -> shooterSubsystem.setShooterReference(Constants.Physical.MANUAL_SHOOTING_SPEED,
						Constants.Physical.MANUAL_SHOOTING_SPEED)).finallyDo(() -> {
							shooterSubsystem.setShooterReference(0, 0);
							intakeSubsystem.setReference(0, 0);
						}));
		// Feed Note into Shooter
		operator.createLeftBumper().whileTrue(runFeeder.get().beforeStarting(Commands.runOnce(() -> SmartDashboard.putBoolean("Left Bumper", true)))).onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Left Bumper", false)));
	}

	@Override
	protected void initRealOrSim() {
		if (RobotBase.isReal()) {
			// visionSystem.addPhotonCamera("Left Camera", 2,
			// new Transform3d(
			// new Translation3d(0.40, // Units.inchesToMeters(-11.66)
			// .12,
			// Units.inchesToMeters(8.256)),
			// new Rotation3d(0, Units.degreesToRadians(28), 0).rotateBy(
			// new Rotation3d(0, 0,
			// Units.degreesToRadians(20)))),
			// PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			// drive.getDrivetrain().getPoseEstimator()); // Used to be
			// 28, 20
			// visionSystem.addCamera(new PhotonVisionCamera("Right Camera", 3,
			// AprilTags.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			// new Transform3d(
			// new Translation3d(Units.inchesToMeters(11.59),
			// Units.inchesToMeters(-4.682), // -.12
			// Units.inchesToMeters(8.256)),
			// new Rotation3d(0, Units.degreesToRadians(-30), 0).rotateBy( // -28
			// new Rotation3d(0, 0,
			// Units.degreesToRadians(-25)))), // -20
			// () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose()));
			// visionSystem.addPhotonCamera("Left Camera", 2,
			// new Transform3d(
			// new Translation3d(Units.inchesToMeters(11.59),
			// Units.inchesToMeters(4.682), // -.12
			// Units.inchesToMeters(8.256)),
			// new Rotation3d(0, Units.degreesToRadians(-25), 0).rotateBy( // -28
			// new Rotation3d(0, 0,
			// Units.degreesToRadians(30)))), // 20
			// PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			// drive.getDrivetrain().getPoseEstimator());
			noteCamera = new VisionLimeLight("orange", 1,
					AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
					new Transform3d(Units.inchesToMeters(-12.342), Units.inchesToMeters(-2.58),
							Units.inchesToMeters(14.264),
							new Rotation3d(0, 0, Units.degreesToRadians(180))),
					null);

			noteCamera.setUpdateValues(true);
// THIS IS THE LIMELIGHT APRIL TAG CAMERA			
			//visionSystem.addLimeLightCamera("top", 5, () -> gyro);
			//VisionLimeLight topLimeLight = ((VisionLimeLight) visionSystem.getCamera("top"));
			//topLimeLight.setPoseEstimationSupplier(() -> RobotState.isDisabled() && neverEnabled ? topLimeLight.getRobotPoseEstimateM1()
			//		: topLimeLight.getRobotPoseEstimateM2());
			
	

			shooterCamera = new VisionPhotonAprilTagTarget("Shooter Camera",
					() -> Units.inchesToMeters(15.448),
					() -> 25,
					() -> Units.inchesToMeters(10.454),
					AprilTags.aprilTagFieldLayout.getTagPose(4).get().getZ(), 4, AprilTags.aprilTagFieldLayout,
					"Vision");
			shooterCamera.cameraRotation(Rotation.DOWN);
			shooterCamera.setUpdateValues(true);
			shooterCamera.setTargetTagId(GenericRobot.chooseAllianceColor() == Color.BLUE ? 7 : 4);
		}
	}

	@Override
	public void initAutoCommands() {
		allLEDs.setColor(GenericRobot.chooseAllianceColor()).on();
		drive.initAutoCommands();
		NamedCommands.registerCommand("Intake Note",
				runIntake.get().until(() -> feederSubsystem.getNoteState() == NoteState.Holding));

		NamedCommands.registerCommand("Intake Note at Max",
				runIntakeWithPivotAtMax.get().until(() -> feederSubsystem.getNoteState() == NoteState.Holding));

		NamedCommands.registerCommand("Intake and Load Note", runIntake.get());

		NamedCommands.registerCommand("Intake And Fire", Commands.run(() -> {
			intakeSubsystem.stateMachine(-1);
			feederSubsystem.feederStateMachine(-1);
		}).until(() -> !feederSubsystem.isEmptied())
				.andThen(Commands.idle(intakeSubsystem).until(() -> feederSubsystem.isEmptied()).finallyDo(() -> {
					intakeSubsystem.stateMachine(0);
					feederSubsystem.feederStateMachine(0);
				})));

		NamedCommands.registerCommand("Auto Intake", autoIntake.get());

		NamedCommands.registerCommand("Auto Intake At Max", autoIntakeAtMax.get());

		NamedCommands.registerCommand("Auto Aim",
				new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem, gyro,
						false, false, false).until(() -> feederSubsystem.getNoteState() == NoteState.Empty)
						.alongWith(spinIntake.get().withTimeout(0.25))
						.finallyDo(() -> pivotSubsystem
								.setReference(pivotSubsystem.HOME_LEVEL)));

		NamedCommands.registerCommand("Auto Aim With Drive",
				new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem, gyro,
						true, false, false).until(() -> feederSubsystem.getNoteState() == NoteState.Empty).alongWith(spinIntake.get().withTimeout(0.25)));

		NamedCommands.registerCommand("Blind Shot",
				runShooter.get()
						.until(() -> shooterSubsystem.isAtTarget() && shooterSubsystem.getPIDReferenceTop() != 0)
						.withTimeout(0.70).andThen(blindShot.get()));

		NamedCommands.registerCommand("Alternate Blind Shot", runShooter.get().withTimeout(0.5).andThen(justShoot.get())
				.finallyDo(() -> shooterSubsystem.setShooterReference(-250.0, -250.0)));

		NamedCommands.registerCommand("Just Shoot", justShoot.get());

		NamedCommands.registerCommand("Spin Up Shooter", runShooter.get());

		NamedCommands.registerCommand("Stop Shooter", stopShooter.get());

		NamedCommands.registerCommand("Quick Auto Aim",
				new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem, gyro,
						false, false, false).until(() -> feederSubsystem.getNoteState() == NoteState.Empty)
						.deadlineWith(spinIntake.get().withTimeout(0.25))
						.beforeStarting(() -> {
							targetingSystem.setTolerance(0.05);
							pivotSubsystem.setTolerance(1.5);
							shooterSubsystem.setTolerance(3.0);
						})
						.finallyDo(() -> {
							pivotSubsystem
									.setReference(pivotSubsystem.HOME_LEVEL);
							targetingSystem.resetToleranceToDefaults();
							pivotSubsystem.resetToleranceToDefaults();
							shooterSubsystem.resetToleranceToDefaults();
						}));

		NamedCommands.registerCommand("Pivot Home",
				Commands.runOnce(() -> pivotSubsystem.setReference(PivotSubsystem.HOME_LEVEL)));

		NamedCommands.registerCommand("Aim Stage Shot Long",
				new PredefinedAutoShot(AutoShotDefinition.STAGE_SHOT_LONG.getPose(GenericRobot.getAlliance()),
						targetingSystem, pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
						.enableSpinup());

		NamedCommands.registerCommand("Aim Left Shot Long",
				new PredefinedAutoShot(AutoShotDefinition.LEFT_LONG_SHOT.getPose(GenericRobot.getAlliance()),
						targetingSystem, pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
						.enableSpinup()
						.enablePivot()
						.enableYaw());

		NamedCommands.registerCommand("Aim Center Shot Long",
				new PredefinedAutoShot(AutoShotDefinition.CENTER_SHOT_LONG.getPose(GenericRobot.getAlliance()),
						targetingSystem, pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
						.enableSpinup());

		NamedCommands.registerCommand("Aim Center Shot Mid",
				new PredefinedAutoShot(AutoShotDefinition.CENTER_SHOT_MID.getPose(GenericRobot.getAlliance()),
						targetingSystem, pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
						.enableSpinup()
						.enableYaw());

		NamedCommands.registerCommand("Aim Center Shot Short", new PredefinedAutoShot(
				AutoShotDefinition.CENTER_SHOT_SHORT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot());

		NamedCommands.registerCommand("Aim Center Shot Short NO PIVOT", new PredefinedAutoShot(
				AutoShotDefinition.CENTER_SHOT_SHORT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup());

		NamedCommands.registerCommand("Aim Right Shot Short", new PredefinedAutoShot(
				AutoShotDefinition.RIGHT_SHOT_SHORT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot()
				.enableYaw());

		NamedCommands.registerCommand("Aim Right Shot Mid", new PredefinedAutoShot(
				AutoShotDefinition.RIGHT_SHOT_MID.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot()
				.enableYaw());

		NamedCommands.registerCommand("Terminate if Empty V2", null);

		// NamedCommands.registerCommand("Pivot Shuttle",
		// spinIntake.get().alongWith(Commands.runOnce(() -> {
		// pivotSubsystem.setReference(pivotSubsystem.LOW_SHUTTLE_LEVEL);
		// shooterSubsystem.setShooterReference(Constants.Physical.TOP_SHOOTING_SPEED,
		// Constants.Physical.BOTTOM_SHOOTING_SPEED);
		// })).blindShot.get().andThen(
		// () -> {
		// pivotSubsystem.setReference(
		// pivotSubsystem.HOME_LEVEL);
		// shooterSubsystem.setShooterReference(0, 0);
		// }));

		NamedCommands.registerCommand("Pivot Shuttle", Commands.runOnce(() -> {
			pivotSubsystem.setReference(pivotSubsystem.LOW_SHUTTLE_LEVEL);
			shooterSubsystem.setShooterReference(Constants.Physical.TOP_SHOOTING_SPEED,
					Constants.Physical.BOTTOM_SHOOTING_SPEED);
		}).andThen(spinIntake.get()).alongWith(justShoot.get()).andThen(() -> {
			pivotSubsystem.setReference(
					pivotSubsystem.HOME_LEVEL);
		}));

		NamedCommands.registerCommand("Predefined Note 2", Commands.idle().beforeStarting(() -> {
			pivotSubsystem.setReference(AutoShotDefinition.B2_SHOT.getPivotAngle());
			shooterSubsystem.setShooterReference(AutoShotDefinition.B2_SHOT.getShooterSpeed(),
					AutoShotDefinition.B2_SHOT.getShooterSpeed());
		}).finallyDo(() -> pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL)));

		NamedCommands.registerCommand("Aim Left Shot Short ", new PredefinedAutoShot(
				AutoShotDefinition.LEFT_SHOT_SHORT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot());

		NamedCommands.registerCommand("Aim Left Speaker Shot", new PredefinedAutoShot(
				AutoShotDefinition.LEFT_SPEAKER_SHOT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot()
				.enableYaw());

		NamedCommands.registerCommand("Aim Right Shot Long", new PredefinedAutoShot(
				AutoShotDefinition.RIGHT_SHOT_LONG.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot()
				.enableYaw());

		NamedCommands.registerCommand("Aim Note 2", new PredefinedAutoShot(
				AutoShotDefinition.NOTE_2.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enableYaw());

		NamedCommands.registerCommand("Aim A1 Shot", new PredefinedAutoShot(
				AutoShotDefinition.A1_SHOT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot());

		NamedCommands.registerCommand("Aim C8 Shot Until Empty", new PredefinedAutoShot(
				AutoShotDefinition.C8_SHOT.getPose(GenericRobot.getAlliance()), targetingSystem,
				pivotSubsystem, shooterSubsystem, () -> feederSubsystem.getNoteState())
				.enableSpinup()
				.enablePivot()
				.enableYaw().until(() -> feederSubsystem.isEmptied()));

		NamedCommands.registerCommand("Terminate if Empty", Commands.runOnce(() -> terminatePath = true)
				.onlyIf(() -> feederSubsystem.getNoteState() == NoteState.Empty));

		NamedCommands.registerCommand("Terminate if No Note",
				Commands.runOnce(() -> terminatePath = true).onlyIf(() -> !noteCamera.isValidTarget()));

		NamedCommands.registerCommand("Terminate Once Note",
				Commands.idle().until(() -> noteCamera.isValidTarget())
						.finallyDo(() -> terminatePath = noteCamera.isValidTarget()));

		NamedCommands.registerCommand("Terminate Path Trigger",
				Commands.idle().until(() -> terminatePath).finallyDo(() -> terminatePath = false));
		NamedCommands.registerCommand("Terminate Event Marker Trigger", Commands.idle().until(() -> terminatePath));

		
	}

	public void disabledBehavior() {

	}

	@Override
	public Command generateAutoCommand(Command autoCommand) {
		neverEnabled = false;
		return drive.generateAutoCommand(autoCommand);
	}
}
