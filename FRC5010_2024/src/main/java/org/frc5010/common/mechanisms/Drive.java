// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.mechanisms;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.arch.GenericMechanism;
import org.frc5010.common.arch.Persisted;
import org.frc5010.common.constants.DrivePorts;
import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.constants.SwerveModuleConstants;
import org.frc5010.common.constants.SwervePorts;
import org.frc5010.common.drive.DifferentialDrivetrain;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.GenericSwerveModule;
import org.frc5010.common.drive.swerve.MK4SwerveModule;
import org.frc5010.common.drive.swerve.MK4iSwerveModule;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.drive.swerve.ThriftySwerveModule;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.vision.VisionSystem;

/** Add your docs here. */
public class Drive extends GenericMechanism {
  private VisionSystem vision;
  private GenericDrivetrain drivetrain;
  private GenericGyro gyro;
  private Command defaultDriveCommand;
  private String type;
  private GenericDrivetrainConstants driveConstants;
  private List<? extends DrivePorts> motorPorts;

  public static class Type {
    public static final String DIFF_DRIVE = "DifferentialDrive";
    public static final String THRIFTY_SWERVE_DRIVE = "ThriftySwerveDrive";
    public static final String MK4_SWERVE_DRIVE = "MK4SwerveDrive";
    public static final String MK4I_SWERVE_DRIVE = "MK4ISwerveDrive";
    public static final String SDS_MK4I_SWERVE_DRIVE = "SDSMK4ISwerveDrive";
    public static final String SDS_MK4_SWERVE_DRIVE = "SDSMK4SwerveDrive";
    public static final String YAGSL_SWERVE_DRIVE = "YAGSLMK4ISwerveDrive";
    public static final String YAGSL_THRIFTY_SWERVE_DRIVE = "YAGSLThriftySwerveDrive";
    public static final String YAGSL_MK4_SWERVE_DRIVE = "YAGSLMK4SwerveDrive";
  }

  // Examples of how to use a persisted constants
  // These can live in specific constants files, however
  private static Persisted<Integer> driveVisualH;
  private static Persisted<Integer> driveVisualV;
  private String driveTrainFolder;

  public Drive(
      VisionSystem visionSystem,
      GenericGyro gyro,
      String type,
      List<? extends DrivePorts> drivePorts,
      GenericDrivetrainConstants driveConstants,
      String driveTrainFolder) {
    super(Drive.class.getSimpleName());
    this.vision = visionSystem;
    this.gyro = gyro;
    this.type = type;
    this.motorPorts = drivePorts;
    this.driveConstants = driveConstants;
    this.driveTrainFolder = driveTrainFolder;
    driveVisualH = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_H, 60);
    driveVisualV = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_V, 60);
    mechVisual = new Mechanism2d(driveVisualH.getInteger(), driveVisualV.getInteger());
    SmartDashboard.putData("Drive Visual", mechVisual);
    // SmartDashboard.putBoolean("Field Oriented", isFieldOrientedDrive);
    initRealOrSim();
  }

  @Override
  protected void initRealOrSim() {
    switch (type) {
      case Type.DIFF_DRIVE:
        {
          initializeDifferentialDrive();
          break;
        }
      case Type.THRIFTY_SWERVE_DRIVE:
        {
          initializeThriftySwerveDrive();
          break;
        }
      case Type.MK4_SWERVE_DRIVE:
        {
          initializeMK4SwerveDrive();
          break;
        }
      case Type.MK4I_SWERVE_DRIVE:
        {
          initializeMK4iSwerveDrive();
          break;
        }
      case Type.YAGSL_SWERVE_DRIVE:
        {
          initializeYAGSLSwerveDrive(driveTrainFolder);
          break;
        }
      case Type.YAGSL_MK4_SWERVE_DRIVE:
        {
          initializeYAGSLMK4SwerveDrive();
          break;
        }
      case Type.YAGSL_THRIFTY_SWERVE_DRIVE:
        {
          initializeYAGSLThriftySwerveDrive();
          break;
        }
      default:
        {
          break;
        }
    }
  }

  private void initializeYAGSLSwerveDrive(String driveTrainFolder) {
    drivetrain =
        new YAGSLSwerveDrivetrain(
            mechVisual, gyro, (SwerveConstants) driveConstants, driveTrainFolder, vision);
  }

  private void initializeYAGSLMK4SwerveDrive() {
    drivetrain =
        new YAGSLSwerveDrivetrain(
            mechVisual, gyro, (SwerveConstants) driveConstants, "swervemk4", vision);
  }

  private void initializeYAGSLThriftySwerveDrive() {
    drivetrain =
        new YAGSLSwerveDrivetrain(
            mechVisual, gyro, (SwerveConstants) driveConstants, "swervethrifty", vision);
  }

  public Command getDefaultCommand() {
    return defaultDriveCommand;
  }

  public void setupDefaultCommands(Controller driver, Controller operator) {
    // Handle real or simulation case for default commands
    if (RobotBase.isReal()) {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = drivetrain.createDefaultCommand(driver);
        drivetrain.setDefaultCommand(defaultDriveCommand);
      }
    } else {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = drivetrain.createDefaultCommand(driver);
        drivetrain.setDefaultCommand(defaultDriveCommand);
      }
    }
  }

  public void setupTestDefaultCommands(Controller driver, Controller operator) {
    if (RobotBase.isReal()) {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = drivetrain.createDefaultTestCommand(driver);
        drivetrain.setDefaultCommand(defaultDriveCommand);
      }
    } else {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = drivetrain.createDefaultCommand(driver);
        drivetrain.setDefaultCommand(defaultDriveCommand);
      }
    }
  }

  @Override
  /**
   * Drive Axis - Left X & Y, Right X Reset Orientation - Start Lock Wheels - Left Bumper Field
   * Oriented - B Button
   */
  public void configureButtonBindings(Controller driver, Controller operator) {
    // If there needs to be some commands that are real or simulation only use this
    if (!DriverStation.isTest()) {
      if (RobotBase.isReal()) {
        driver
            .createBButton()
            .onTrue(Commands.runOnce(() -> drivetrain.toggleFieldOrientedDrive()));
        driver.createStartButton().onTrue(Commands.runOnce(() -> drivetrain.resetOrientation()));
      } else {

      }
      driver
          .createLeftBumper()
          .whileTrue(
              Commands.run(
                  () -> {
                    drivetrain.lockWheels();
                  },
                  drivetrain));
    }
    // Put commands that can be both real and simulation afterwards

    driver.setLeftXAxis(driver.createLeftXAxis().negate().deadzone(0.08));
    driver.setLeftYAxis(driver.createLeftYAxis().negate().deadzone(0.08));
    driver.setRightXAxis(driver.createRightXAxis().negate().deadzone(0.08));
  }

  public GenericDrivetrain getDrivetrain() {
    return drivetrain;
  }

  private void initializeThriftySwerveDrive() {
    SwerveModuleConstants frontLeftConstants =
        new SwerveModuleConstants(0, 0, false, 0, true, true);
    SwerveModuleConstants frontRightConstants =
        new SwerveModuleConstants(0, 0, false, 0, true, true);
    SwerveModuleConstants backLeftConstants = new SwerveModuleConstants(0, 0, true, 0, true, true);
    SwerveModuleConstants backRightConstants = new SwerveModuleConstants(0, 0, true, 0, true, true);

    GenericSwerveModule frontLeft =
        new ThriftySwerveModule(
            mechVisual.getRoot("frontleft", 15, 45),
            "frontleft",
            ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(0),
            frontLeftConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule frontRight =
        new ThriftySwerveModule(
            mechVisual.getRoot("frontright", 45, 45),
            "frontright",
            ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(1),
            frontRightConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule backLeft =
        new ThriftySwerveModule(
            mechVisual.getRoot("backleft", 15, 15),
            "backleft",
            ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(2),
            backLeftConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule backRight =
        new ThriftySwerveModule(
            mechVisual.getRoot("backright", 45, 15),
            "backright",
            ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(3),
            backRightConstants,
            (SwerveConstants) driveConstants);

    drivetrain =
        new SwerveDrivetrain(
            mechVisual,
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            gyro,
            vision,
            (SwerveConstants) driveConstants);
  }

  private void initializeDifferentialDrive() {
    MotorController5010 template = MotorFactory.DriveTrainMotor(MotorFactory.NEO(1));
    List<DrivePorts> motorPorts = new ArrayList<>();

    // This assumes ports 1 & 2 are left and 3 & 4 are right
    // This is just an example of how to put a sequence of numbers into a list
    motorPorts.add(new DrivePorts(1));
    motorPorts.add(new DrivePorts(2));
    motorPorts.add(new DrivePorts(3));
    motorPorts.add(new DrivePorts(4));

    drivetrain = new DifferentialDrivetrain(template, motorPorts, gyro, vision, mechVisual);
  }

  private void initializeMK4SwerveDrive() {
    SwerveModuleConstants frontLeftConstants =
        new SwerveModuleConstants(0, 0, true, 0, false, false);
    SwerveModuleConstants frontRightConstants =
        new SwerveModuleConstants(0, 0, true, 0, false, false);
    SwerveModuleConstants backLeftConstants =
        new SwerveModuleConstants(0, 0, true, 0, false, false);
    SwerveModuleConstants backRightConstants =
        new SwerveModuleConstants(0, 0, true, 0, false, false);

    GenericSwerveModule frontLeft =
        new MK4SwerveModule(
            mechVisual.getRoot("frontleft", 15, 45),
            "frontleft",
            ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(0),
            frontLeftConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule frontRight =
        new MK4SwerveModule(
            mechVisual.getRoot("frontright", 45, 45),
            "frontright",
            ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(1),
            frontRightConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule backLeft =
        new MK4SwerveModule(
            mechVisual.getRoot("backleft", 15, 15),
            "backleft",
            ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(2),
            backLeftConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule backRight =
        new MK4SwerveModule(
            mechVisual.getRoot("backright", 45, 15),
            "backright",
            ((SwerveConstants) driveConstants).getkBackRightAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(3),
            backRightConstants,
            (SwerveConstants) driveConstants);

    drivetrain =
        new SwerveDrivetrain(
            mechVisual,
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            gyro,
            vision,
            (SwerveConstants) driveConstants);
  }

  private void initializeMK4iSwerveDrive() {
    SwerveModuleConstants frontLeftConstants =
        new SwerveModuleConstants(0, 0, false, 0, true, false);
    SwerveModuleConstants frontRightConstants =
        new SwerveModuleConstants(0, 0, false, 0, true, false);
    SwerveModuleConstants backLeftConstants =
        new SwerveModuleConstants(0, 0, false, 0, true, false);
    SwerveModuleConstants backRightConstants =
        new SwerveModuleConstants(0, 0, false, 0, true, false);

    GenericSwerveModule frontLeft =
        new MK4iSwerveModule(
            mechVisual.getRoot("frontleft", 15, 45),
            "frontleft",
            ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(0),
            frontLeftConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule frontRight =
        new MK4iSwerveModule(
            mechVisual.getRoot("frontright", 45, 45),
            "frontright",
            ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(1),
            frontRightConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule backLeft =
        new MK4iSwerveModule(
            mechVisual.getRoot("backleft", 15, 15),
            "backleft",
            ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(2),
            backLeftConstants,
            (SwerveConstants) driveConstants);
    GenericSwerveModule backRight =
        new MK4iSwerveModule(
            mechVisual.getRoot("backright", 45, 15),
            "backright",
            ((SwerveConstants) driveConstants).getkBackRightAbsoluteOffsetRad(),
            (SwervePorts) motorPorts.get(3),
            backRightConstants,
            (SwerveConstants) driveConstants);

    drivetrain =
        new SwerveDrivetrain(
            mechVisual,
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            gyro,
            vision,
            (SwerveConstants) driveConstants);
  }

  public void initAutoCommands() {
    drivetrain.setAutoBuilder();
  }

  public Command generateAutoCommand(Command autoCommand) {
    return autoCommand
        .beforeStarting(
            () -> {
              drivetrain.resetEncoders();
            })
        .until(() -> drivetrain.hasIssues());
  }

  public void disabledBehavior() {
    drivetrain.disabledBehavior();
  }
}
