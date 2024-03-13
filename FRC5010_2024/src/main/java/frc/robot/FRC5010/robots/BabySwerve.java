// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionLimeLightLib;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.ThriftySwerveModule;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.motors.hardware.NEO550;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.NavXGyro;

/** Add your docs here. */
public class BabySwerve extends GenericMechanism {
  private SwerveConstants swerveConstants;
  private VisionSystem vision;
  private Drive drive;

  public BabySwerve(Mechanism2d visual, ShuffleboardTab displayTab) {
    super(visual, displayTab);
    swerveConstants = new SwerveConstants(0.76835, 0.635);
    swerveConstants.setkFrontLeftAbsoluteOffsetRad(0.26);
    swerveConstants.setkFrontRightAbsoluteOffsetRad(-3.14);
    swerveConstants.setkBackLeftAbsoluteOffsetRad(1.0 + Math.PI);
    swerveConstants.setkBackRightAbsoluteOffsetRad(0.21 + Math.PI);
    swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
    swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
    swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
    swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
    swerveConstants.setSwerveModuleConstants(ThriftySwerveModule.moduleConstants);
    swerveConstants.configureSwerve(NEO.MAXRPM, NEO550.MAXRPM);

    // VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1,
    // AprilTags.aprilTagRoomLayout,PoseStrategy.AVERAGE_BEST_TARGETS);
    vision = new VisionLimeLightLib("orange", 2, AprilTags.aprilTagFieldLayout, new Transform3d());
    /*
     * multiVision.addPhotonCamera("Arducam_OV9281_USB_Camera",
     * new Transform3d( // This describes the vector between the camera lens to the
     * robot center on the ground
     * new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)),
     * new Rotation3d(0, Units.degreesToRadians(-20), 0)
     * )
     * );
     */

    List<SwervePorts> swervePorts = new ArrayList<>();
    swervePorts.add(new SwervePorts(1, 2, 0));
    swervePorts.add(new SwervePorts(7, 8, 1));
    swervePorts.add(new SwervePorts(3, 4, 2));
    swervePorts.add(new SwervePorts(5, 6, 3));

    GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);

    drive = new Drive(vision, gyro, Drive.Type.YAGSL_THRIFTY_SWERVE_DRIVE, swervePorts, swerveConstants, "");
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
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

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drive.generateAutoCommand(autoCommand);
  }
}
