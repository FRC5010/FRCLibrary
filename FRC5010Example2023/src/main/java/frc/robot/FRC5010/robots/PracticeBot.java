// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.commands.AutoModes;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.MK4SwerveModule;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.chargedup.ChargedUpAutoModes;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ChaseTag;

/** Add your docs here. */
public class PracticeBot extends GenericMechanism {
  private GenericGyro gyro;
  private SwerveConstants swerveConstants;
  private AutoModes autoMaps;
  private Drive drive;

  public PracticeBot(Mechanism2d visual, ShuffleboardTab displayTab) {
    super(visual, displayTab);
    swerveConstants = new SwerveConstants(Units.inchesToMeters(24.25), Units.inchesToMeters(20.5));
    swerveConstants.setkFrontLeftAbsoluteOffsetRad(-2.357 + Math.PI);
    swerveConstants.setkFrontRightAbsoluteOffsetRad(-2.792);
    swerveConstants.setkBackLeftAbsoluteOffsetRad(0.845 + Math.PI);
    swerveConstants.setkBackRightAbsoluteOffsetRad(-0.171);
    swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(1);
    swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
    swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
    swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
    swerveConstants.setSwerveModuleConstants(MK4SwerveModule.MK4_L1);
    swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);

    VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagFieldLayout,
        PoseStrategy.AVERAGE_BEST_TARGETS);
    multiVision.addPhotonCamera("FrontCamera",
        new Transform3d( // This describes the vector between the camera lens to the robot center on the
                         // ground
            new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(0.0), Units.inchesToMeters(3.5)),
            new Rotation3d(0, 0, Units.degreesToRadians(-90))));
    multiVision.addPhotonCamera("BackCamera",
        new Transform3d( // This describes the vector between the camera lens to the robot center on the
                         // ground
            new Translation3d(Units.inchesToMeters(-5.5), 0, Units.inchesToMeters(3.5)),
            new Rotation3d(0, 0, Units.degreesToRadians(90))));

    List<SwervePorts> swervePorts = new ArrayList<>();
    swervePorts.add(new SwervePorts(1, 3, 0));
    swervePorts.add(new SwervePorts(10, 8, 1));
    swervePorts.add(new SwervePorts(5, 4, 2));
    swervePorts.add(new SwervePorts(2, 7, 3));

    gyro = new PigeonGyro(11);

    autoMaps = new ChargedUpAutoModes();
    autoMaps.loadAutoPaths();

    drive = new Drive(multiVision, gyro, Drive.Type.MK4_SWERVE_DRIVE, swervePorts, swerveConstants);
    multiVision.setDrivetrainPoseEstimator(drive.getDrivetrain().getPoseEstimator());
  }

  @Override
  public Map<String, Command> initAutoCommands() {
    return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap());
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createYButton().whileTrue(new AutoBalance(drive.getDrivetrain(), () -> !driver.createAButton().getAsBoolean(), gyro)); 
    
    driver.createBButton().whileTrue(new ChaseTag((SwerveDrivetrain) drive.getDrivetrain(), () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose())); 

    drive.configureButtonBindings(driver, operator);
    
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    drive.setupDefaultCommands(driver, operator);
  }

  @Override
  protected void initRealOrSim() {
  }
}
