// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.LogLevel;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.FRC5010.commands.JoystickToSwerve;
import frc.robot.FRC5010.constants.MotorFeedFwdConstants;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.drive.swerve.MK4iSwerveModule;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.motors.hardware.KrakenX60;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.FRC5010.subsystems.LEDStripSegment;
import frc.robot.FRC5010.subsystems.PowerDistribution5010;
import frc.robot.FRC5010.subsystems.SegmentedLedSystem;
import frc.robot.crescendo.FeederSubsystem.NoteState;
import frc.robot.crescendo.commands.AutoAim;
import frc.robot.crescendo.commands.AutoIntake;
import frc.robot.crescendo.commands.RunClimb;
import frc.robot.crescendo.commands.RunIntake;
import frc.robot.crescendo.commands.RunPivot;
import frc.robot.crescendo.commands.RunShooter;
import swervelib.SwerveDrive;

/** Add your docs here. */
public class CompBot_2024 extends GenericMechanism {
        private SwerveConstants swerveConstants;
        private MotorController5010 pivotMotor;
        private PivotSubsystem pivotSubsystem;
        private ShooterSubsystem shooterSubsystem;
        private FeederSubsystem feederSubsystem;
        private MotorController5010 topShooterMotor;
        private MotorController5010 bottomShooterMotor;
        private MotorController5010 feederMotor;
        private IntakeSubsystem intakeSubsystem;
        private MotorController5010 innerIntakeMotor;
        private MotorController5010 outerIntakeMotor;

        private VisionMultiCam visionSystem;
        private GenericGyro gyro;
        private ClimbSubsystem climbSubsystem;
        private MotorController5010 leftClimbMotor;
        private MotorController5010 rightClimbMotor;
        private Drive drive;
        private SegmentedLedSystem ledSubsystem;
        private LEDStripSegment allLEDs;
        private TargetingSystem targetingSystem;
        private PowerDistribution5010 powerDistribution5010;

        Supplier<Command> autoIntake;
        Supplier<Command> runIntake;
        Supplier<Command> stopDrivetrain;
        Supplier<Command> rumbleOperator;
        Supplier<Command> runShooter;
        Supplier<Command> runFeeder;
        Supplier<Command> spinIntake;
        Supplier<Command> blindShot;

        public CompBot_2024(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);

                RobotContainer.setLoggingLevel(LogLevel.DEBUG);

                ledSubsystem = new SegmentedLedSystem(0, 34, visual);
                ledSubsystem.setWholeStripState((Integer i) -> RobotContainer.chooseAllianceColor().getColor8Bit());

                allLEDs = ledSubsystem.getStrip(ledSubsystem.ALL);
                allLEDs.chase(false);

                powerDistribution5010 = new PowerDistribution5010();

                // Motor Setup
                innerIntakeMotor = MotorFactory.KrakenX60(1);
                outerIntakeMotor = MotorFactory.KrakenX60(5);
                leftClimbMotor = MotorFactory.NEO(7); // TODO: Add correct port
                rightClimbMotor = MotorFactory.NEO(8).invert(true); // TODO: Add correct port
                pivotMotor = MotorFactory.NEO(9).invert(true);
                MotorFactory.NEO(10).setFollow(pivotMotor, true);
                feederMotor = MotorFactory.NEO(11); // MotorFactory.NEO(11);

                values.declare("FeederSpeed", 1.0);

                topShooterMotor = MotorFactory.KrakenX60(12).invert(true);
                bottomShooterMotor = MotorFactory.KrakenX60(14).invert(true);

                visionSystem = new VisionMultiCam("Vision", 0, AprilTags.aprilTagFieldLayout);

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
                swerveConstants.getSwerveModuleConstants().setkWheelDiameterMeters(0.115573139);
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontleft",
                                new MotorFeedFwdConstants(0.17204, 2.0698, 0.29611)); // FL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontright",
                                new MotorFeedFwdConstants(0.13752, 2.0659, 0.46677)); // FR
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backleft",
                                new MotorFeedFwdConstants(0.16356, 2.1167, 0.29725)); // BL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backright",
                                new MotorFeedFwdConstants(0.16026, 2.1268, 0.29725)); // BR

                swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(6);
                swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

                swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(1);
                swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
                swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(5.93);

                drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, null, swerveConstants,
                                "mk4i_L3_kraken_neo");
                targetingSystem = new TargetingSystem(
                                () -> TargetingSystem.getSpeakerTarget(RobotContainer.getAlliance()),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose3d(),
                                (SwerveDrivetrain) drive.getDrivetrain());

                shooterSubsystem = new ShooterSubsystem(mechVisual, topShooterMotor, bottomShooterMotor);
                feederSubsystem = new FeederSubsystem(visual, feederMotor, ledSubsystem);
                intakeSubsystem = new IntakeSubsystem(outerIntakeMotor, innerIntakeMotor, mechVisual);

                initRealOrSim();
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
                runIntake = () -> new RunIntake(() -> -0.75, () -> 0.5, intakeSubsystem, feederSubsystem,
                                pivotSubsystem,
                                null)
                                .until(() -> feederSubsystem.getNoteState() == NoteState.Loaded);

                stopDrivetrain = () -> Commands.runOnce(() -> drive.getDrivetrain().drive(new ChassisSpeeds()),
                                drive.getDrivetrain());

                autoIntake = () -> runIntake.get().deadlineWith(
                                new AutoIntake((SwerveDrivetrain) drive.getDrivetrain(),
                                                visionSystem.getCamera("orange")))
                                .andThen(stopDrivetrain.get());

                rumbleOperator = () -> Commands
                                .deadline(Commands.waitSeconds(0.5), Commands.runOnce(() -> operator.setRumble(1)))
                                .finallyDo(() -> operator.setRumble(0));

                runShooter = () -> Commands
                                .run(() -> {
                                        shooterSubsystem.setShooterReference(Constants.Physical.TOP_SHOOTING_SPEED,
                                                        Constants.Physical.BOTTOM_SHOOTING_SPEED);
                                        
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
                                () -> intakeSubsystem.setReference(-1000, -1000),
                                () -> intakeSubsystem.setReference(0, 0)).withTimeout(0.5);

                blindShot = () -> Commands.run(() -> feederSubsystem.feederStateMachine(-1.0))
                                .alongWith(spinIntake.get())
                                .until(() -> feederSubsystem.getNoteState() == NoteState.Empty)
                                .finallyDo(() -> {
                                        feederSubsystem.feederStateMachine(0.0);
                                        shooterSubsystem.setShooterReference(0, 0);
                                });

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
                                                () -> (JoystickToSwerve) drive.getDefaultCommand())
                                                .alongWith(spinIntake.get()))
                                .onFalse(Commands.waitSeconds(0.05).andThen(
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
                                .onTrue(Commands.runOnce(() -> climbSubsystem.enableClimb(true))
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
                // Podium Pivot
                operator.createXButton().onTrue(Commands.runOnce(
                                () -> {
                                        pivotSubsystem.setReference(pivotSubsystem.PODIUM_SHOT);
                                        shooterSubsystem.setShooterReference(6000, 6000);
                                }, pivotSubsystem).alongWith(spinIntake.get())).onFalse(Commands.runOnce(() -> {
                                        pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
                                        shooterSubsystem.setShooterReference(0, 0);
                                }));

                // Amp Pivot
                operator.createYButton().whileTrue(Commands
                                .runOnce(() -> {
                                        pivotSubsystem.setReference(pivotSubsystem.AMP_LEVEL);
                                        shooterSubsystem.setShooterReference(2000, 2000);
                                }).alongWith(spinIntake.get())

                ).onFalse(Commands.runOnce(() -> {
                        pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
                        shooterSubsystem.setShooterReference(0, 0);
                }));

                // Trap Pivot
                operator.createBButton().onTrue(Commands.runOnce(
                                () -> {
                                        pivotSubsystem.setReference(pivotSubsystem.HIGH_SHUTTLE_LEVEL);
                                        shooterSubsystem.setShooterReference(6000, 6000);
                                }, pivotSubsystem).alongWith(spinIntake.get())).onFalse(Commands.runOnce(() -> {
                                        pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
                                        shooterSubsystem.setShooterReference(0, 0);
                                }));

                // Run Shooter motors
                operator.createRightBumper().whileTrue(runShooter.get().finallyDo(() -> {
                        shooterSubsystem.setShooterReference(0, 0);
                        intakeSubsystem.setReference(0, 0);
                }));
                // Feed Note into Shooter
                operator.createLeftBumper().whileTrue(runFeeder.get());

                // Shooter micro adjust
                operator.createUpPovButton().onTrue(spinIntake.get());
                operator.createDownPovButton().onTrue(Commands.runOnce(() -> intakeSubsystem.setReference(0, 0)));
                // Pivot micro adjust
                operator.createLeftPovButton().onTrue(pivotSubsystem.adjustReferenceDown());
                operator.createRightPovButton().onTrue(pivotSubsystem.adjustReferenceUp());

                operator.createBackButton()
                                .onTrue((Commands.run(() -> {
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
                                                                                climbSubsystem)));

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
                                () -> operator.getRightTrigger() - operator.getLeftTrigger(), pivotSubsystem));

                shooterSubsystem.setDefaultCommand(
                                new RunShooter(() -> 0.0, shooterSubsystem));

                intakeSubsystem.setDefaultCommand(
                                new RunIntake(() -> driver.getRightTrigger() - driver.getLeftTrigger(),
                                                () -> 0.5,
                                                intakeSubsystem,
                                                feederSubsystem,
                                                pivotSubsystem, driver));

                climbSubsystem.setDefaultCommand(new RunClimb(() -> operator.getLeftYAxis(),
                                () -> operator.getRightYAxis(), climbSubsystem));

                // ledSubsystem.setDefaultCommand(new LEDStateHandler(ledSubsystem, () ->
                // feederSubsystem.getNoteState()));

                // feederSubsystem.setDefaultCommand(new RunFeeder(feederSubsystem, () -> 0.0));
                ledSubsystem.setDefaultCommand(Commands.run(() -> {
                }, ledSubsystem)
                                .finallyDo(() -> ledSubsystem.getStrip(ledSubsystem.ALL).rainbow()));
                ledSubsystem.getStrip(ledSubsystem.ALL).setColor(RobotContainer.chooseAllianceColor());
        }

        @Override
        public void setupTestDefaultCommmands(Controller driver, Controller operator) {
                pivotSubsystem.removeDefaultCommand();
                shooterSubsystem.removeDefaultCommand();
                intakeSubsystem.removeDefaultCommand();

                driver.createYButton().whileTrue(shooterSubsystem.getTopSysIdRoutineCommand());
                driver.createXButton().whileTrue(shooterSubsystem.getBottomSysIdRoutineCommand());

                driver.createStartButton().whileTrue(feederSubsystem.getFeederSysIdRoutineCommand());
                // driver.createXButton().whileTrue(pivotSubsystem.getSysIdCommand());
                driver.createLeftBumper().whileTrue(intakeSubsystem.getBottomSysIdRoutineCommand());
                driver.createRightBumper().whileTrue(intakeSubsystem.getTopSysIdRoutineCommand());
                // pivotSubsystem.setDefaultCommand(Commands.run(() ->
                // pivotSubsystem.setSpeed(operator.getLeftYAxis()), pivotSubsystem));
                // shooterSubsystem.setDefaultCommand(Commands.run(() -> {
                // feederSubsystem.setFeederSpeed(operator.getLeftTrigger() -
                // operator.getRightTrigger());
                // // shooterSubsystem.setShooterSpeed(operator.getRightXAxis(),
                // // operator.getRightXAxis());
                // }, shooterSubsystem));

                // Pivot Microadjust
                driver.createLeftPovButton().onTrue(pivotSubsystem.adjustReferenceDown());
                driver.createRightPovButton().onTrue(pivotSubsystem.adjustReferenceUp());

                intakeSubsystem.setDefaultCommand(Commands.run(
                                () -> intakeSubsystem.setIntakeSpeed(driver.getLeftTrigger() - driver.getRightTrigger(),
                                                driver.getLeftTrigger() - driver.getRightTrigger()),
                                intakeSubsystem));
                // climbSubsystem.setDefaultCommand(Commands.run(() -> {
                // climbSubsystem.setLeftMotorSpeed(operator.getLeftTrigger());
                // climbSubsystem.setRightMotorSpeed(operator.getLeftTrigger());
                // }, climbSubsystem));
                ledSubsystem.setDefaultCommand(Commands.run(() -> {
                }, ledSubsystem)
                                .finallyDo(() -> ledSubsystem.getStrip(ledSubsystem.ALL).rainbow()));
                ledSubsystem.getStrip(ledSubsystem.ALL).setColor(RobotContainer.chooseAllianceColor());
                drive.setupTestDefaultCommands(driver, operator);
        }

        @Override
        protected void initRealOrSim() {
                if (RobotBase.isReal()) {
                        visionSystem.addPhotonCamera("Left Camera", 2,
                                        new Transform3d(
                                                        new Translation3d(0.40, // Units.inchesToMeters(-11.66)
                                                                        .12,
                                                                        Units.inchesToMeters(8.256)),
                                                        new Rotation3d(0, Units.degreesToRadians(28), 0).rotateBy(
                                                                        new Rotation3d(0, 0,
                                                                                        Units.degreesToRadians(20)))),
                                        PoseStrategy.LOWEST_AMBIGUITY, drive.getDrivetrain().getPoseEstimator());
                        visionSystem.addPhotonCamera("Right Camera", 3,
                                        new Transform3d(
                                                        new Translation3d(0.4,
                                                                        -.12,
                                                                        Units.inchesToMeters(8.256)),
                                                        new Rotation3d(0, Units.degreesToRadians(28), 0).rotateBy(
                                                                        new Rotation3d(0, 0,
                                                                                        Units.degreesToRadians(-20)))),
                                        PoseStrategy.LOWEST_AMBIGUITY, drive.getDrivetrain().getPoseEstimator());
                        visionSystem.addLimeLightTargetCam("orange", Units.inchesToMeters(14.264),
                                        -10, 0, 1,
                                        new Transform3d(Units.inchesToMeters(-12.342), Units.inchesToMeters(-2.58),
                                                        Units.inchesToMeters(14.264),
                                                        new Rotation3d(0, 0, Units.degreesToRadians(180))));
                }
        }

        @Override
        public void initAutoCommands() {
                allLEDs.setColor(RobotContainer.chooseAllianceColor()).on();
                drive.initAutoCommands();
                NamedCommands.registerCommand("Intake Note", runIntake.get());
                NamedCommands.registerCommand("Auto Intake", autoIntake.get());

                NamedCommands.registerCommand("Auto Aim",
                                new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem,
                                                false).until(() -> feederSubsystem.getNoteState() == NoteState.Empty)
                                                .alongWith(spinIntake.get())
                                                .finallyDo(() -> pivotSubsystem
                                                                .setReference(pivotSubsystem.HOME_LEVEL)));
                NamedCommands.registerCommand("Auto Aim With Drive",
                                new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem,
                                                true).until(() -> feederSubsystem.getNoteState() == NoteState.Empty));
                NamedCommands.registerCommand("Blind Shot",
                                runShooter.get().withTimeout(0.75).andThen(blindShot.get()));

                // NamedCommands.registerCommand("Pivot Shuttle",
                // spinIntake.get().alongWith(Commands.runOnce(() -> {
                // pivotSubsystem.setReference(pivotSubsystem.LOW_SHUTTLE_LEVEL);
                // shooterSubsystem.setShooterReference(Constants.Physical.TOP_SHOOTING_SPEED,
                // Constants.Physical.BOTTOM_SHOOTING_SPEED);
                // })).
                // blindShot.get().andThen(
                // () -> {
                // pivotSubsystem.setReference(
                // pivotSubsystem.HOME_LEVEL);
                // shooterSubsystem.setShooterReference(0, 0);
                // })));
        }

        public void disabledBehavior() {

        }

        @Override
        public Command generateAutoCommand(Command autoCommand) {
                return drive.generateAutoCommand(autoCommand);
        }
}
