// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class SwerveConstants extends GenericDrivetrainConstants {
  // public variables
  private double DRIVETRAIN_TRACKWIDTH_METERS;
  private double DRIVETRAIN_WHEELBASE_METERS;

  private SwerveDriveKinematics kinematics;

  private double kFrontLeftAbsoluteOffsetRad;
  private double kFrontRightAbsoluteOffsetRad;
  private double kBackLeftAbsoluteOffsetRad;
  private double kBackRightAbsoluteOffsetRad;

  private SwerveModuleConstants swerveModuleConstants;

  public SwerveConstants(double trackWidth, double wheelBase) {
    this.DRIVETRAIN_TRACKWIDTH_METERS = trackWidth;
    this.DRIVETRAIN_WHEELBASE_METERS = wheelBase;

    kinematics =
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(
                DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(
                -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(
                -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void configureSwerve(double motorMaxRPM, double turnMotorMaxRPM) {
    kPhysicalMaxSpeedMetersPerSecond =
        motorMaxRPM * swerveModuleConstants.getkDriveEncoderRPM2MeterPerSec();
    kPhysicalMaxAngularSpeedRadiansPerSecond =
        turnMotorMaxRPM * swerveModuleConstants.getkTurningEncoderRPM2RadPerSec();
  }

  public double getkFrontLeftAbsoluteOffsetRad() {
    return kFrontLeftAbsoluteOffsetRad;
  }

  public void setkFrontLeftAbsoluteOffsetRad(double kFrontLeftAbsoluteOffsetRad) {
    this.kFrontLeftAbsoluteOffsetRad = kFrontLeftAbsoluteOffsetRad;
  }

  public double getkFrontRightAbsoluteOffsetRad() {
    return kFrontRightAbsoluteOffsetRad;
  }

  public void setkFrontRightAbsoluteOffsetRad(double kFrontRightAbsoluteOffsetRad) {
    this.kFrontRightAbsoluteOffsetRad = kFrontRightAbsoluteOffsetRad;
  }

  public double getkBackLeftAbsoluteOffsetRad() {
    return kBackLeftAbsoluteOffsetRad;
  }

  public void setkBackLeftAbsoluteOffsetRad(double kBackLeftAbsoluteOffsetRad) {
    this.kBackLeftAbsoluteOffsetRad = kBackLeftAbsoluteOffsetRad;
  }

  public double getkBackRightAbsoluteOffsetRad() {
    return kBackRightAbsoluteOffsetRad;
  }

  public void setkBackRightAbsoluteOffsetRad(double kBackRightAbsoluteOffsetRad) {
    this.kBackRightAbsoluteOffsetRad = kBackRightAbsoluteOffsetRad;
  }

  public SwerveModuleConstants getSwerveModuleConstants() {
    return swerveModuleConstants;
  }

  public void setSwerveModuleConstants(SwerveModuleConstants swerveModuleConstants) {
    this.swerveModuleConstants = swerveModuleConstants;
  }
}
