// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.commands.DriveToPosition;
import frc.robot.FRC5010.commands.DriveToPosition.LCR;
import frc.robot.FRC5010.constants.AutoMaps;
import frc.robot.FRC5010.constants.GenericMechanism;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.MK4iSwerveModule;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.ButtonBoard;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.FRC5010.subsystems.DriverDisplaySubsystem;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.chargedup.commands.AutoBalance;
import frc.robot.chargedup.commands.HomeElevator;
import frc.robot.chargedup.commands.HomePivot;
import frc.robot.chargedup.commands.IntakeSpin;
import frc.robot.chargedup.commands.MoveElevator;
import frc.robot.chargedup.commands.PivotElevator;

/** Add your docs here. */
public class CompBot_2023_T1G3R extends GenericMechanism {
        private SwerveConstants swerveConstants;
        private Drive drive;
        private DriverDisplaySubsystem driverDiplay;
        private GenericMechanism elevator;
        private AutoMaps autoMaps;
        private ButtonBoard buttonOperator;
        private LedSubsystem ledSubsystem;
        private GenericGyro gyro;
        private VisionMultiCam visionSystem;

        public CompBot_2023_T1G3R(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);
                // Needs to be set
                swerveConstants = new SwerveConstants(Units.inchesToMeters(22), Units.inchesToMeters(26.5));

                // Baby Swerve values need to be changed
                swerveConstants.setkFrontLeftAbsoluteOffsetRad(Units.degreesToRadians(-93.076)); //
                swerveConstants.setkFrontRightAbsoluteOffsetRad(Units.degreesToRadians(140.010)); //
                swerveConstants.setkBackLeftAbsoluteOffsetRad(Units.degreesToRadians(-1.582)); //
                swerveConstants.setkBackRightAbsoluteOffsetRad(Units.degreesToRadians(1.670)); //
                // swerveConstants.setkFrontLeftAbsoluteOffsetRad(0); //
                // swerveConstants.setkFrontRightAbsoluteOffsetRad(0); //
                // swerveConstants.setkBackLeftAbsoluteOffsetRad(0); //
                // swerveConstants.setkBackRightAbsoluteOffsetRad(0);

                swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(10);
                swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

                swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(1);
                swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

                swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L3);
                swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);

                ledSubsystem = new LedSubsystem(1, 187);
                ledSubsystem.off();

                // Will need to be changed for 2023 field
                visionSystem = new VisionMultiCam("Vision", 1, AprilTags.aprilTagFieldLayout);

                ShuffleboardTab visionTab = Shuffleboard.getTab("Drive");
                // visionTab.addCamera("DriverCam", "DriverCam",
                // "http://10.50.10.11:5800/").withPosition(0, 0).withSize(7,4);

                // Ports need to be changed when comp bot is ready
                List<SwervePorts> swervePorts = new ArrayList<>();

                swervePorts.add(new SwervePorts(1, 2, 10)); // FL
                swervePorts.add(new SwervePorts(8, 7, 11)); // FR
                swervePorts.add(new SwervePorts(3, 4, 12)); // BL
                swervePorts.add(new SwervePorts(5, 6, 9)); // BR

                gyro = new PigeonGyro(13);

                drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, swervePorts, swerveConstants,
                                "swervemk4i");

                driverDiplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

                buttonOperator = new ButtonBoard(Controller.JoystickPorts.TWO.ordinal());
                buttonOperator.createButtons(11);
                elevator = new ChargedUpMech(mechVisual, shuffleTab, buttonOperator, ledSubsystem);

                autoMaps = new AutoMaps();
                SwerveDrivetrain swerveDrivetrain = (SwerveDrivetrain) drive.getDrivetrain();
                ElevatorSubsystem elevatorSubsystem = ((ChargedUpMech) elevator).getElevatorSubsystem();
                IntakeSubsystem intakeSubsystem = ((ChargedUpMech) elevator).getIntakeSubsystem();
                PivotSubsystem pivotSubsystem = ((ChargedUpMech) elevator).getPivotSubsystem();

                // Elevator Controls
                autoMaps.addMarker("ExtendToPivotPosition",
                                new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.low));
                autoMaps.addMarker("HomeElevator", new HomeElevator(elevatorSubsystem, pivotSubsystem));

                autoMaps.addMarker("ExtendToGround",
                                new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.ground));

                autoMaps.addMarker("ExtendToLoading",
                                new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.loading)
                                                .andThen(new WaitCommand(0.1)));

                autoMaps.addMarker("ExtendToMid", new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.medium));

                autoMaps.addMarker("ExtendToHigh",
                                new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.high));

                autoMaps.addMarker("ExtendToConePickUp",
                                new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.conePickUp));

                autoMaps.addMarker("PivotToGround", new PivotElevator(pivotSubsystem,
                                ElevatorLevel.ground));
                autoMaps.addMarker("PivotToLow", new PivotElevator(pivotSubsystem,
                                ElevatorLevel.low));
                autoMaps.addMarker("PivotToMid", new PivotElevator(pivotSubsystem,
                                ElevatorLevel.medium));

                autoMaps.addMarker("PivotToHigh", new PivotElevator(pivotSubsystem,
                                ElevatorLevel.high));
                autoMaps.addMarker("HomePivot", new HomePivot(pivotSubsystem));

                // Intake Controls
                autoMaps.addMarker("ConeMode",
                                new InstantCommand(() -> intakeSubsystem.setIntakeCone(), intakeSubsystem)
                                                .andThen(new WaitCommand(0.25)));
                autoMaps.addMarker("CubeMode",
                                new InstantCommand(() -> intakeSubsystem.setIntakeCube(), intakeSubsystem));

                autoMaps.addMarker("CubeModeTimed",
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> intakeSubsystem.setIntakeCube(),
                                                                intakeSubsystem),
                                                new WaitCommand(.25)));

                autoMaps.addMarker("Yeet Cube", new IntakeSpin(intakeSubsystem, () -> -0.85).withTimeout(0.25));
                autoMaps.addMarker("OuttakeFast", (new IntakeSpin(intakeSubsystem, () -> -0.6).withTimeout(.25)));
                autoMaps.addMarker("Outtake", (new IntakeSpin(intakeSubsystem, () -> -0.5).withTimeout(.25)));
                autoMaps.addMarker("OuttakeSlow", (new IntakeSpin(intakeSubsystem, () -> -0.3).withTimeout(.25)));
                autoMaps.addMarker("OuttakeSlower", new IntakeSpin(intakeSubsystem, () -> -0.2).withTimeout(0.25));
                autoMaps.addMarker("Intake", (new IntakeSpin(intakeSubsystem, () -> 1.0).withTimeout(0.5)));
                autoMaps.addMarker("IntakeLong", (new IntakeSpin(intakeSubsystem, () -> 1.0).withTimeout(5.0)));
                // Drivetrain Controls
                autoMaps.addMarker("AutoBalance", new AutoBalance(swerveDrivetrain, () -> false, gyro));
                autoMaps.addMarker("LockWheels", new InstantCommand(() -> swerveDrivetrain.lockWheels()));
                // .beforeStarting(new InstantCommand(() -> WpiDataLogging.log("Lock
                // Wheels"))));

                autoMaps.addMarker("AutoExtendDrop", new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.medium)
                                .andThen(new IntakeSpin(intakeSubsystem, () -> -0.3).withTimeout(0.5)));

                autoMaps.addMarker("AutoGroundPickUp",
                                new PivotElevator(pivotSubsystem, ElevatorLevel.ground)
                                                .andThen(new MoveElevator(elevatorSubsystem,
                                                                () -> ElevatorLevel.ground))
                                                .deadlineWith(new IntakeSpin(intakeSubsystem, () -> 1.0)));

                autoMaps.addMarker("Cube Retract", new PivotElevator(pivotSubsystem, ElevatorLevel.medium)
                                .alongWith(new HomeElevator(elevatorSubsystem, pivotSubsystem)));

                autoMaps.addMarker("Cone Retract", new PivotElevator(pivotSubsystem, ElevatorLevel.high)
                                .alongWith(new HomeElevator(elevatorSubsystem, pivotSubsystem)));


                initRealOrSim();
        }
        // autoMaps.addPath("Command Test", new PathConstraints(4, 1.75));

       

        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
                driver.createYButton()
                                .whileTrue(new AutoBalance(drive.getDrivetrain(),
                                                () -> !driver.createAButton().getAsBoolean(), gyro));

                driver.createAButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                                () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                                ledSubsystem, LCR.center));

                driver.createBButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                                () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                                ledSubsystem, LCR.left));

                driver.createXButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                                () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                                ledSubsystem, LCR.right));

                // driver.createBButton()
                // .whileTrue(new DriveToTrajectory((SwerveDrivetrain) drive.getDrivetrain(),
                // LCR.left, swerveConstants));

                // driver.createAButton()
                // .whileTrue(new DriveToTrajectory((SwerveDrivetrain) drive.getDrivetrain(),
                // LCR.center, swerveConstants));

                // driver.createXButton()
                // .whileTrue(new DriveToTrajectory((SwerveDrivetrain) drive.getDrivetrain(),
                // LCR.right, swerveConstants));

                driver.createBackButton().onTrue(new InstantCommand(() -> drive.getDrivetrain().resetEncoders()));

                drive.configureButtonBindings(driver, operator);
                elevator.configureButtonBindings(driver, operator);
        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
                drive.setupDefaultCommands(driver, operator);
                elevator.setupDefaultCommands(driver, operator);
        }

        @Override
        protected void initRealOrSim() {
                if (RobotBase.isReal()) {
                        // Uncomment when using PhotonVision
                        visionSystem.addPhotonCamera("ForwardCam", 4,
                                        new Transform3d( // This describes the vector between the camera lens to the
                                                         // robot center on the
                                                         // ground
                                                        new Translation3d(-Units.inchesToMeters(9.469),
                                                                        -Units.inchesToMeters(5.525),
                                                                        Units.inchesToMeters(14.146)),
                                                        new Rotation3d(0, 0, 0)),
                                        PoseStrategy.LOWEST_AMBIGUITY, drive.getDrivetrain().getPoseEstimator());

                }
        }

        @Override
        public void initAutoCommands() {
                drive.initAutoCommands();
        }

        public void disabledBehavior() {
                drive.disabledBehavior();
        }

        @Override
        public Command generateAutoCommand(Command autoCommand) {
                return drive.generateAutoCommand(autoCommand);
        }
}
