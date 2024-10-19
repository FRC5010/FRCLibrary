// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.commands.JoystickToSwerve;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.constants.SwervePorts;
import org.frc5010.common.drive.swerve.MK4SwerveModule;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.mechanisms.Drive;
import org.frc5010.common.motors.hardware.NEO;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.PhotonVisionPoseCamera;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.sensors.gyro.PigeonGyro;
import org.frc5010.common.subsystems.AprilTagPoseSystem;
import org.frc5010.common.subsystems.DriverDisplaySubsystem;
import org.frc5010.common.subsystems.LedSubsystem;
import org.frc5010.common.vision.AprilTags;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** Add your docs here. */
public class PracticeBot extends GenericRobot {
  private GenericGyro gyro;
  private SwerveConstants swerveConstants;
  private DriverDisplaySubsystem driverDisplay;
  private Drive drive;
  private LedSubsystem ledSubsystem;
  private SwerveDrivetrain swerveDrivetrain;

  public PracticeBot() {
    super();
    swerveConstants = new SwerveConstants(Units.inchesToMeters(24.25), Units.inchesToMeters(20.5));
    swerveConstants.setkFrontLeftAbsoluteOffsetRad(2.856);
    swerveConstants.setkFrontRightAbsoluteOffsetRad(2.444 + Math.PI);
    swerveConstants.setkBackLeftAbsoluteOffsetRad(2.858 + Math.PI);
    swerveConstants.setkBackRightAbsoluteOffsetRad(1.967);
    swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(4);
    swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
    swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(3);
    swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
    swerveConstants.setSwerveModuleConstants(MK4SwerveModule.MK4_L1);
    swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);

    ShuffleboardTab visionTab = Shuffleboard.getTab("Drive");

    AprilTagPoseSystem multiVision =
        new AprilTagPoseSystem(new PhotonVisionPoseCamera("PhotonATSim", 2, AprilTags.aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS,
						new Transform3d(new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)),
								new Rotation3d(0, Units.degreesToRadians(-20), 0)),
						() -> drive.getDrivetrain().getPoseEstimator().getCurrentPose()), AprilTags.aprilTagFieldLayout);

    visionTab.addCamera("DriverCam", "DriverCam", "10.50.10.11").withSize(3, 3);

    // VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1,
    // AprilTags.aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP);
    // multiVision.addPhotonCamera("RightCamera",
    // new Transform3d( // This describes the vector between the camera lens to the
    // robot center on the
    // // ground
    // new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(0.0),
    // Units.inchesToMeters(3.5)),
    // new Rotation3d(0, 0, Units.degreesToRadians(-100))));
    // multiVision.addPhotonCamera("LeftCamera",
    // new Transform3d( // This describes the vector between the camera lens to the
    // robot center on the
    // // ground
    // new Translation3d(Units.inchesToMeters(-5.5), 0, Units.inchesToMeters(3.5)),
    // new Rotation3d(0, 0, Units.degreesToRadians(80))));

    List<SwervePorts> swervePorts = new ArrayList<>();
    swervePorts.add(new SwervePorts(1, 3, 21));
    swervePorts.add(new SwervePorts(10, 8, 22));
    swervePorts.add(new SwervePorts(5, 4, 23));
    swervePorts.add(new SwervePorts(2, 7, 20));

    gyro = new PigeonGyro(11);

    drive =
        new Drive(
            multiVision, gyro, Drive.Type.YAGSL_MK4_SWERVE_DRIVE, swervePorts, swerveConstants, "");
    ledSubsystem = new LedSubsystem(1, 60);
    // multiVision.setDrivetrainPoseEstimator(drive.getDrivetrain().getPoseEstimator());

    driverDisplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

    swerveDrivetrain = (SwerveDrivetrain) drive.getDrivetrain();
  }

  @Override
  public void initAutoCommands() {
    drive.initAutoCommands();
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver
        .createAButton()
        .whileTrue(
            new JoystickToSwerve(
                swerveDrivetrain,
                () -> 0.5,
                () -> 0.0,
                () -> 0.5,
                () -> true,
                () -> GenericRobot.getAlliance()));

    // driver.createBButton().whileTrue(new DriveToPosition((SwerveDrivetrain)
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

    SwerveDrivetrain swerveDrivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    driver
        .createLeftBumper()
        .whileTrue(new RunCommand(() -> swerveDrivetrain.lockWheels(), swerveDrivetrain));

    drive.configureButtonBindings(driver, operator);
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    drive.setupDefaultCommands(driver, operator);
  }

  @Override
  protected void initRealOrSim() {}

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drive.generateAutoCommand(autoCommand);
  }
}
