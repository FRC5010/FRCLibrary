// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.constants.SwerveModuleConstants;
import org.frc5010.common.constants.SwervePorts;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.AnalogInput5010;

/** Add your docs here. */
public class ThriftySwerveModule extends GenericSwerveModule {

  // neo 550 sysid values
  public static final double kSC = 0.55641;
  public static final double kVC = 0.064889;
  public static final double kAC = 0.0025381;

  // pid values for the neo 550
  public static final double kPTurning = 0.052037 * 4;
  public static final double kITurning = 0;
  public static final double kDTurning = 0;

  // physical values
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
  public static final double kDriveMotorGearRatio = 1 / 5.25;
  public static final double kTurningMotorGearRatio =
      1 / ((5.33) * 10.5); // not 12:1 but 10.5 for gearbox, ultraplanetaries are not nominal

  // conversions
  public static final double maxAbsEncoderVolts = 4.815;
  public static final double voltsToDegrees = (360 / maxAbsEncoderVolts);
  public static final double voltsToRadians = (2 * Math.PI / maxAbsEncoderVolts);

  // limits
  public static final int neoCurrentLimit = 60;
  public static final int neo550CurrentLimit = 30;

  public static final GenericPID pid = new GenericPID(kPTurning, kITurning, kDTurning);
  public static final MotorFeedFwdConstants motorsConstants =
      new MotorFeedFwdConstants(kSC, kVC, kAC, true);
  public static final SwerveModuleConstants moduleConstants =
      new SwerveModuleConstants(
          kWheelDiameterMeters, kDriveMotorGearRatio, false, kTurningMotorGearRatio, false, false);

  public ThriftySwerveModule(
      MechanismRoot2d visualRoot,
      String key,
      double radOffset,
      SwervePorts swervePorts,
      SwerveModuleConstants swerveConstants,
      SwerveConstants swerveConstants2) {
    super(visualRoot, key, radOffset, swerveConstants2);
    super.pid = pid;
    super.motorConstants = motorConstants;
    super.moduleConstants = moduleConstants;
    drive = MotorFactory.NEO(swervePorts.getDrivePort()).invert(moduleConstants.isDrivingInv());
    turn = MotorFactory.NEO(swervePorts.getTurnPort()).invert(moduleConstants.isTurningInv());
    absoluteEncoder = new AnalogInput5010(swervePorts.getEncoderPort());
    turnEncoder = turn.getMotorEncoder();
    driveEncoder = drive.getMotorEncoder();

    // set units drive encoder to meters and meters/sec
    driveEncoder.setPositionConversion(moduleConstants.getkDriveEncoderRot2Meter());
    driveEncoder.setVelocityConversion(moduleConstants.getkDriveEncoderRPM2MeterPerSec());
    // set units turning encoder to radians and radians/sec
    turnEncoder.setPositionConversion(moduleConstants.getkTurningEncoderRot2Rad());
    turnEncoder.setVelocityConversion(moduleConstants.getkTurningEncoderRPM2RadPerSec());

    setupSwerveEncoders();
  }
}
