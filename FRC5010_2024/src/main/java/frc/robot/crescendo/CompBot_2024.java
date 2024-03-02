// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.FRC5010.commands.DriveToPosition;
import frc.robot.FRC5010.commands.JoystickToSwerve;
import frc.robot.FRC5010.constants.MotorFeedFwdConstants;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.TranslationConstants;
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
import frc.robot.FRC5010.subsystems.Color;
import frc.robot.FRC5010.subsystems.PowerDistribution5010;
import frc.robot.FRC5010.subsystems.SegmentedLedSystem;
import frc.robot.crescendo.FeederSubsystem.NoteState;
import frc.robot.crescendo.commands.AutoAim;
import frc.robot.crescendo.commands.RunClimb;
import frc.robot.crescendo.commands.RunFeeder;
import frc.robot.crescendo.commands.RunIntake;
import frc.robot.crescendo.commands.RunPivot;
import frc.robot.crescendo.commands.RunShooter;

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

        private Targeting2024 targetingSystem;
        private PowerDistribution5010 powerDistribution5010;

        public CompBot_2024(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);

                ledSubsystem = new SegmentedLedSystem(0, 34, visual);
                ledSubsystem.setWholeStripState((Integer i) -> Color.GREEN.getColor8Bit());
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
                swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(6);
                swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

                swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(1);
                swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
                swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(5.93);

                swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L3_KRAKEN_NEO);
                swerveConstants.getSwerveModuleConstants().setkWheelDiameterMeters(0.115573139);
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontleft",
                                new MotorFeedFwdConstants(0.13212, 1.9208, 0.20617)); // FL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontright",
                                new MotorFeedFwdConstants(0.15576, 1.9648, 0.17833)); // FR
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backleft",
                                new MotorFeedFwdConstants(0.1163, 1.9497, 0.34564)); // BL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backright",
                                new MotorFeedFwdConstants(0.1163, 1.9497, 0.34564)); // BR
                swerveConstants.configureSwerve(KrakenX60.MAXRPM, NEO.MAXRPM);

                drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, null, swerveConstants,
                                "mk4i_L3_kraken_neo");
                targetingSystem = new Targeting2024((SwerveDrivetrain) drive.getDrivetrain(),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose3d(),
                                () -> Constants.Field.BLUE_SHOT_POSE);

                shooterSubsystem = new ShooterSubsystem(mechVisual, topShooterMotor, bottomShooterMotor);
                feederSubsystem = new FeederSubsystem(visual, feederMotor);
                intakeSubsystem = new IntakeSubsystem(outerIntakeMotor, innerIntakeMotor, mechVisual);
                initRealOrSim();
        }

        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
                /* >>>>>>>>>>>>>>>>>>>> ADDITIONAL AXIS <<<<<<<<<<<<<<<<<<<< */
                driver.setRightTrigger(driver.createRightTrigger().deadzone(0.07).negate());
                driver.setLeftTrigger(driver.createLeftTrigger().deadzone(0.07).negate());

                operator.setLeftYAxis(operator.createLeftYAxis().deadzone(0.07).negate());
                operator.setRightYAxis(operator.createRightYAxis().deadzone(0.07).negate().limit(0.25).rate(0.25));

                operator.setRightXAxis(operator.createRightXAxis().deadzone(0.07).negate());
                operator.setLeftXAxis(operator.createLeftXAxis().deadzone(0.07).negate());

                operator.setLeftTrigger(operator.createLeftTrigger());
                operator.setRightTrigger(operator.createRightTrigger());
                drive.configureButtonBindings(driver, operator);

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

                driver.createAButton().whileTrue(
                                new AutoAim(pivotSubsystem, shooterSubsystem, feederSubsystem, drive, targetingSystem,
                                                () -> (JoystickToSwerve) drive.getDefaultCommand()))
                                .onFalse(Commands
                                                .runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL)));

                driver.createRightBumper()
                                .whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                                                () -> drive.getDrivetrain().getPoseEstimator()
                                                                .getPoseFromClosestVisionTarget(),
                                                null,
                                                TranslationConstants.noteTransform)
                                                .deadlineWith(new RunIntake(() -> 1.0, () -> 0.5, intakeSubsystem,
                                                                feederSubsystem)));

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
                operator.createAButton().whileTrue(Commands
                                .runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL), pivotSubsystem));
                // Podium Pivot
                operator.createXButton().onTrue(Commands.runOnce(
                                () -> {
                                        pivotSubsystem.setReference(pivotSubsystem.PODIUM_SHOT);
                                        shooterSubsystem.setShooterReference(6000, 6000);
                                }, pivotSubsystem)).onFalse(Commands.runOnce(() -> {
                                        pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
                                        shooterSubsystem.setShooterReference(0, 0);
                                }));

                // Amp Pivot
                operator.createYButton().whileTrue(Commands
                                .runOnce(() -> {
                                        pivotSubsystem.setReference(pivotSubsystem.AMP_LEVEL);
                                        shooterSubsystem.setShooterReference(2000, 2000);
                                })

                ).onFalse(Commands.runOnce(() -> {
                        pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL);
                        shooterSubsystem.setShooterReference(0, 0);
                }));

                // Trap Pivot
                operator.createBButton().onTrue(Commands
                                .runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.TRAP_LEVEL), pivotSubsystem));

                // Run Shooter motors
                operator.createRightBumper()
                                .whileTrue(Commands
                                                .run(() -> {
                                                        shooterSubsystem.setShooterReference(6000, 6000);
                                                })
                                                .finallyDo(() -> shooterSubsystem.setShooterReference(0, 0)));
                // Feed Note into Shooter
                operator.createLeftBumper().whileTrue(
                                Commands.run(() -> feederSubsystem.feederStateMachine(-values.getDouble("FeederSpeed")),
                                                feederSubsystem).finallyDo(() -> feederSubsystem.stop()));

                // Shooter micro adjust
                operator.createUpPovButton().onTrue(shooterSubsystem.adjustShooterReferenceUp());
                operator.createDownPovButton().onTrue(shooterSubsystem.adjustShooterReferenceDown());
                // Pivot micro adjust
                operator.createLeftPovButton().onTrue(pivotSubsystem.adjustReferenceDown());
                operator.createRightPovButton().onTrue(pivotSubsystem.adjustReferenceUp());

                operator.createStartButton()
                                .onTrue(Commands.runOnce(() -> climbSubsystem.zeroPosition(), climbSubsystem));
                operator.createBackButton().whileTrue(Commands.startEnd(() -> climbSubsystem.setOverride(true),
                                () -> climbSubsystem.setOverride(false)));

                /*
                 * >>>>>>>>>>>>>>>>>>>>>>>>>>>> BOARD BUTTONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                 */
        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
                drive.setupDefaultCommands(driver, operator);

                pivotSubsystem.setDefaultCommand(new RunPivot(() -> 0.0, pivotSubsystem));

                shooterSubsystem.setDefaultCommand(
                                new RunShooter(() -> 0.0, shooterSubsystem, feederSubsystem));

                intakeSubsystem.setDefaultCommand(
                                new RunIntake(() -> driver.getRightTrigger() - driver.getLeftTrigger(), () -> 0.5,
                                                intakeSubsystem,
                                                feederSubsystem));

                climbSubsystem.setDefaultCommand(new RunClimb(() -> operator.getLeftYAxis(),
                                () -> operator.getRightYAxis(), climbSubsystem));

                // ledSubsystem.setDefaultCommand(new LEDStateHandler(ledSubsystem, () ->
                // feederSubsystem.getNoteState()));

                // feederSubsystem.setDefaultCommand(new RunFeeder(feederSubsystem, () -> 0.0));
                ledSubsystem.setDefaultCommand(Commands.run(() -> {
                }, ledSubsystem)
                                .finallyDo(() -> ledSubsystem.getStrip(ledSubsystem.ALL).rainbow()));
        }

        @Override
        public void setupTestDefaultCommmands(Controller driver, Controller operator) {
                pivotSubsystem.removeDefaultCommand();
                shooterSubsystem.removeDefaultCommand();
                intakeSubsystem.removeDefaultCommand();

                // driver.createAButton().whileTrue(shooterSubsystem.getTopSysIdRoutineCommand());
                // driver.createBButton().whileTrue(shooterSubsystem.getBottomSysIdRoutineCommand());
                driver.createYButton().whileTrue(feederSubsystem.getFeederSysIdRoutineCommand());
                // driver.createXButton().whileTrue(pivotSubsystem.getSysIdCommand());
                driver.createLeftBumper().whileTrue(intakeSubsystem.getBottomSysIdRoutineCommand());
                driver.createRightBumper().whileTrue(intakeSubsystem.getTopSysIdRoutineCommand());
                // pivotSubsystem.setDefaultCommand(Commands.run(() ->
                // pivotSubsystem.setSpeed(operator.getLeftYAxis()), pivotSubsystem));
                shooterSubsystem.setDefaultCommand(Commands.run(() -> {
                        feederSubsystem.setFeederSpeed(operator.getLeftTrigger() - operator.getRightTrigger());
                        // shooterSubsystem.setShooterSpeed(operator.getRightXAxis(),
                        // operator.getRightXAxis());
                }, shooterSubsystem));
                intakeSubsystem.setDefaultCommand(Commands.run(
                                () -> intakeSubsystem.setIntakeSpeed(driver.getLeftTrigger() - driver.getRightTrigger(),
                                                driver.getLeftTrigger() - driver.getRightTrigger()),
                                intakeSubsystem));
                // climbSubsystem.setDefaultCommand(Commands.run(() -> {
                // climbSubsystem.setLeftMotorSpeed(operator.getLeftTrigger());
                // climbSubsystem.setRightMotorSpeed(operator.getLeftTrigger());
                // }, climbSubsystem));

                drive.setupTestDefaultCommands(driver, operator);
        }

        @Override
        protected void initRealOrSim() {
                if (RobotBase.isReal()) {
                        visionSystem.addPhotonCamera("Left Camera", 2,
                                        new Transform3d(
                                                        new Translation3d(Units.inchesToMeters(-11.66),
                                                                        Units.inchesToMeters(4.739),
                                                                        Units.inchesToMeters(8.256)),
                                                        new Rotation3d(0, Units.degreesToRadians(25), 0).rotateBy(
                                                                        new Rotation3d(0, 0,
                                                                                        Units.degreesToRadians(30)))),
                                        PoseStrategy.LOWEST_AMBIGUITY, drive.getDrivetrain().getPoseEstimator());
                        visionSystem.addPhotonCamera("Right Camera", 3,
                                        new Transform3d(
                                                        new Translation3d(Units.inchesToMeters(-11.66),
                                                                        Units.inchesToMeters(-4.739),
                                                                        Units.inchesToMeters(8.256)),
                                                        new Rotation3d(0, Units.degreesToRadians(25), 0).rotateBy(
                                                                        new Rotation3d(0, 0,
                                                                                        Units.degreesToRadians(-30)))),
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
                drive.initAutoCommands();
        }

        public void disabledBehavior() {

        }

        @Override
        public Command generateAutoCommand(Command autoCommand) {
                return drive.generateAutoCommand(autoCommand);
        }
}
