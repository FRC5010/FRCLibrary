// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.constants.SwerveModuleConstants;
import org.frc5010.common.constants.SwervePorts;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.encoder.CanCoderEncoder;

/** Add your docs here. */
public class MK4iSwerveModule extends GenericSwerveModule {

  public static final SwerveModuleConstants MK4I_L1 =
      new SwerveModuleConstants(
          0.10033,
          (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
          true,
          (14.0 / 50.0) * (10.0 / 60.0),
          false,
          false);
  public static final SwerveModuleConstants MK4I_L2 =
      new SwerveModuleConstants(
          0.10033,
          (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
          true,
          (14.0 / 50.0) * (10.0 / 60.0),
          false,
          false);
  public static final SwerveModuleConstants MK4I_L3 =
      new SwerveModuleConstants(
          0.10033,
          (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
          true,
          (14.0 / 50.0) * (10.0 / 60.0),
          false,
          false);
  public static final SwerveModuleConstants MK4I_L3_KRAKEN_NEO =
      new SwerveModuleConstants(
          0.1116923252,
          (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
          true,
          (14.0 / 50.0) * (10.0 / 60.0),
          false,
          false);
  private GenericPID pid = new GenericPID(3, 0.0, 0);
  private MotorFeedFwdConstants motorConstants =
      new MotorFeedFwdConstants(0, 0.0016156, 4.2354e-05);

  public MK4iSwerveModule(
      MechanismRoot2d visualRoot,
      String key,
      double radOffset,
      SwervePorts swervePorts,
      SwerveModuleConstants individualConstants,
      SwerveConstants swerveConstants) {
    super(visualRoot, key, radOffset, swerveConstants);
    super.pid = this.pid;
    super.motorConstants = this.motorConstants;
    super.moduleConstants = swerveConstants.getSwerveModuleConstants();
    drive = MotorFactory.NEO(swervePorts.getDrivePort()).invert(individualConstants.isDrivingInv());
    turn = MotorFactory.NEO(swervePorts.getTurnPort()).invert(individualConstants.isTurningInv());
    absoluteEncoder = new CanCoderEncoder(swervePorts.getEncoderPort());
    absoluteEncoder.setInverted(individualConstants.isEncoderInv());
    setupSwerveEncoders();
  }
}
