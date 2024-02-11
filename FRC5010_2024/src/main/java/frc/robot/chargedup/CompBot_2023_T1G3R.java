// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.commands.DriveToPosition;
import frc.robot.FRC5010.constants.GenericMechanism;
import frc.robot.FRC5010.constants.MotorFeedFwdConstants;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.constants.TranslationConstants;
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

/** Add your docs here. */
public class CompBot_2023_T1G3R extends GenericMechanism {
        private SwerveConstants swerveConstants;
        private Drive drive;
        private DriverDisplaySubsystem driverDiplay;
        private GenericMechanism elevator;
        private ButtonBoard buttonOperator;
        private LedSubsystem ledSubsystem;
        private GenericGyro gyro;
        private VisionMultiCam visionSystem;

        public CompBot_2023_T1G3R(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);
                // Needs to be set
                swerveConstants = new SwerveConstants(Units.inchesToMeters(22), Units.inchesToMeters(26.5));

                swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(10);
                swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

                swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(1);
                swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
                swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(14.5);

                swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L3);
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontleft", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // FL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontright", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // FR
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backleft", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // BL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backright", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // BR
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
                                "t1g3r_mk4i");

                driverDiplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

                buttonOperator = new ButtonBoard(Controller.JoystickPorts.TWO.ordinal());
                buttonOperator.createButtons(11);
                elevator = new ChargedUpMech(mechVisual, shuffleTab, buttonOperator, ledSubsystem);
                initRealOrSim();
        }


        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
                driver.createYButton()
                                .whileTrue(new AutoBalance(drive.getDrivetrain(),
                                                () -> !driver.createAButton().getAsBoolean(), gyro));

                // driver.createAButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                //                 () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                //                 () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                //                 ledSubsystem, TranslationConstants.tagToLeftConeTransfrom));

                driver.createBButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                                () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                                ledSubsystem, TranslationConstants.tagToCubeTransfrom));

                driver.createXButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
                                () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                                () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                                ledSubsystem, TranslationConstants.tagToRightConeTransfrom));

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
                        // visionSystem.addPhotonCamera("ForwardCam", 4,
                        //                 new Transform3d( // This describes the vector between the camera lens to the
                        //                                  // robot center on the
                        //                                  // ground
                        //                                 new Translation3d(-Units.inchesToMeters(9.469),
                        //                                                 -Units.inchesToMeters(5.525),
                        //                                                 Units.inchesToMeters(14.146)),
                        //                                 new Rotation3d(0, 0, 0)),
                        //                 PoseStrategy.LOWEST_AMBIGUITY, drive.getDrivetrain().getPoseEstimator());

                } else {
                }
        }

        @Override
        public void initAutoCommands() {
                elevator.initAutoCommands();
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
