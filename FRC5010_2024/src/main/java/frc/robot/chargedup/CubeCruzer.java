// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.GenericMechanism;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.sensors.ButtonBoard;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.FRC5010.subsystems.DriverDisplaySubsystem;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.chargedup.commands.AutoBalance;

/** Add your docs here. */
public class CubeCruzer extends GenericMechanism {
        private SwerveConstants swerveConstants;
        private Drive drive;
        private DriverDisplaySubsystem driverDiplay;
        private ButtonBoard buttonOperator;
        private LedSubsystem ledSubsystem;
        private GenericGyro gyro;
        private VisionSystem visionSystem;

        public CubeCruzer(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);
                swerveConstants = new SwerveConstants(Units.inchesToMeters(22), Units.inchesToMeters(26.5));

                // Baby Swerve values need to be changed
                swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
                swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

                swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.1);
                swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

                ledSubsystem = new LedSubsystem(1, 187);
                ledSubsystem.off();

                // Will need to be changed for 2023 field
                VisionMultiCam multiVision = new VisionMultiCam("Vision", 0, AprilTags.aprilTagFieldLayout);
                multiVision.addLimeLightCamera("orange", 1);
                // multiVision.setUpdateValues(true);
                visionSystem = multiVision;

                // ShuffleboardTab visionTab = Shuffleboard.getTab("Drive");
                // visionTab.addCamera("DriverCam", "DriverCam",
                // "http://10.50.10.11:5800/").withPosition(0, 0).withSize(7,4);

                gyro = new PigeonGyro(15);

                drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, null, swerveConstants,
                                "swervemk4icc");
                // Uncomment when using PhotonVision
                // multiVision.addPhotonCamera("LeftCamera", 1,
                // new Transform3d( // This describes the vector between the camera lens to the
                // // robot center on the
                // // ground
                // new Translation3d(Units.inchesToMeters(27.75 / 2),
                // Units.inchesToMeters(2.5),
                // Units.inchesToMeters(36.75)),
                // new Rotation3d(0, 0, Units.degreesToRadians(90))),
                // PoseStrategy.MULTI_TAG_PNP,
                // drive.getDrivetrain().getPoseEstimator());

                driverDiplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

                buttonOperator = new ButtonBoard(Controller.JoystickPorts.TWO.ordinal());
                buttonOperator.createButtons(11);
        }

   

        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
                driver.createYButton()
                                .whileTrue(new AutoBalance(drive.getDrivetrain(),
                                                () -> !driver.createAButton().getAsBoolean(), gyro));

                // driver.createBButtdon().whileTrue(new DriveToPosition((SwerveDrivetrain)
                // drive.getDrivetrain(),
                // () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                // () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                // ledSubsystem, LCR.left));

                // driver.createAButton().whileTrue(new DriveToPosition((SwerveDrivetrain)
                // drive.getDrivetrain(),
                // () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                // () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                // ledSubsystem, LCR.center));

                // driver.createXButton().whileTrue(new DriveToPosition((SwerveDrivetrain)
                // drive.getDrivetrain(),
                // () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
                // () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
                // ledSubsystem, LCR.right));

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

        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
                drive.setupDefaultCommands(driver, operator);
        }

        @Override
        protected void initRealOrSim() {
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
