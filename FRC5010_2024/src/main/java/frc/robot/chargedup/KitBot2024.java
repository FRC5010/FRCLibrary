// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import org.frc5010.common.arch.GenericMechanism;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.swerve.MK4iSwerveModule;
import org.frc5010.common.mechanisms.Drive;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.ButtonBoard;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.sensors.gyro.PigeonGyro;
import org.frc5010.common.subsystems.AprilTagPoseSystem;
import org.frc5010.common.subsystems.DriverDisplaySubsystem;
import org.frc5010.common.subsystems.LedSubsystem;
import org.frc5010.common.subsystems.PowerDistribution5010;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.chargedup.commands.RunShooter;

/** Add your docs here. */
public class KitBot2024 extends GenericRobot {
        private SwerveConstants swerveConstants;
        private Drive drive;
        private DriverDisplaySubsystem driverDiplay;
        private ButtonBoard buttonOperator;
        private LedSubsystem ledSubsystem;
        private GenericGyro gyro;
        private AprilTagPoseSystem visionSystem;
        private ShooterSubsystem shooter;
        private MotorController5010 topShooterMotor;
        private MotorController5010 bottomShooterMotor;
        private PowerDistribution5010 powerDistribution;

        private JoystickButton shootButton;

        public KitBot2024() {
                super();
                swerveConstants = new SwerveConstants(Units.inchesToMeters(22), Units.inchesToMeters(26.5));

                // Baby Swerve values need to be changed
                swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(10);
                swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

                swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(2);
                swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
                
                swerveConstants.setkPhysicalMaxSpeedMetersPerSecond(14.5);

                swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L3);
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontleft", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // FL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("frontright", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // FR
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backleft", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // BL
                swerveConstants.getSwerveModuleConstants().addDriveMotorFF("backright", new MotorFeedFwdConstants(0.24241, 2.38, 0.43145)); // BR

                ledSubsystem = new LedSubsystem(1, 187);
                ledSubsystem.off();

                // Will need to be changed for 2023 field
                visionSystem = new AprilTagPoseSystem();

                // ShuffleboardTab visionTab = Shuffleboard.getTab("Drive");
                // visionTab.addCamera("DriverCam", "DriverCam",
                // "http://10.50.10.11:5800/").withPosition(0, 0).withSize(7,4);
                powerDistribution = new PowerDistribution5010();

                gyro = new PigeonGyro(15);

                drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, null, swerveConstants,
                                "swervemk4icc");

                topShooterMotor = MotorFactory.NEO(9);
                bottomShooterMotor = MotorFactory.NEO(33);
             
                shooter = new ShooterSubsystem(topShooterMotor, bottomShooterMotor);
                powerDistribution.registerChannel("", 0);
                // Uncomment when using PhotonVision
                // multiVision.addPhotonCamera("LeftCamera", 1,
                // new Transform3d( // This describes the vector between the camera lens to the
                // // robot center on the
                // // ground
                // new Translation3d(Units.inchesToMeters(27.75 / 2),
                // Units.inchesToMeters(2.5),
                // Units.inchesToMeters(36.75)),
                // multiVision.addLimeLightCamera("kitbot", 4, null);
                // multiVision.setUpdateValues(true);
                // new Rotation3d(0, 0, Units.degreesToRadians(90))),
                // PoseStrategy.MULTI_TAG_PNP,
                // drive.getDrivetrain().getPoseEstimator());

                driverDiplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

        
        }

        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {

                driver.createBackButton().onTrue(new InstantCommand(() -> drive.getDrivetrain().resetEncoders()));
                driver.setRightTrigger(driver.createRightTrigger().deadzone(0.07));
                driver.setLeftTrigger(driver.createLeftTrigger().deadzone(0.07));
                shootButton = driver.createRightBumper();



                drive.configureButtonBindings(driver, operator);
        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
                drive.setupDefaultCommands(driver, operator);
                shooter.setDefaultCommand(new RunShooter(shooter, () -> driver.getRightTrigger() - driver.getLeftTrigger(), 
                () -> shootButton.getAsBoolean() ? 1.0 : 0 - driver.getLeftTrigger() 
                ));
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
