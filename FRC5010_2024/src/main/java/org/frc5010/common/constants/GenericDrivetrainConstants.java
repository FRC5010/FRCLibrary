// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Add your docs here. */
public class GenericDrivetrainConstants {
  protected double kPhysicalMaxAngularSpeedRadiansPerSecond;
  protected double kPhysicalMaxSpeedMetersPerSecond;

  protected double kTeleDriveMaxSpeedMetersPerSecond;
  protected double kTeleDriveMaxAngularSpeedRadiansPerSecond;
  protected double kTeleDriveMaxAccelerationUnitsPerSecond;
  protected double kTeleDriveMaxAngularAccelerationUnitsPerSecond;

  public double getkPhysicalMaxAngularSpeedRadiansPerSecond() {
    return kPhysicalMaxAngularSpeedRadiansPerSecond;
  }

  public void setkPhysicalMaxAngularSpeedRadiansPerSecond(
      double kPhysicalMaxAngularSpeedRadiansPerSecond) {
    this.kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
  }

  public double getkPhysicalMaxSpeedMetersPerSecond() {
    return kPhysicalMaxSpeedMetersPerSecond;
  }

  public void setkPhysicalMaxSpeedMetersPerSecond(double kPhysicalMaxSpeedMetersPerSecond) {
    this.kPhysicalMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
  }

  public double getkTeleDriveMaxSpeedMetersPerSecond() {
    return kTeleDriveMaxSpeedMetersPerSecond;
  }

  public void setkTeleDriveMaxSpeedMetersPerSecond(double kTeleDriveMaxSpeedMetersPerSecond) {
    this.kTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecond;
  }

  public double getkTeleDriveMaxAngularSpeedRadiansPerSecond() {
    return kTeleDriveMaxAngularSpeedRadiansPerSecond;
  }

  public void setkTeleDriveMaxAngularSpeedRadiansPerSecond(
      double kTeleDriveMaxAngularSpeedRadiansPerSecond) {
    this.kTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxAngularSpeedRadiansPerSecond;
  }

  public double getkTeleDriveMaxAccelerationUnitsPerSecond() {
    return kTeleDriveMaxAccelerationUnitsPerSecond;
  }

  public void setkTeleDriveMaxAccelerationUnitsPerSecond(
      double kTeleDriveMaxAccelerationUnitsPerSecond) {
    this.kTeleDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxAccelerationUnitsPerSecond;
  }

  public double getkTeleDriveMaxAngularAccelerationUnitsPerSecond() {
    return kTeleDriveMaxAngularAccelerationUnitsPerSecond;
  }

  public void setkTeleDriveMaxAngularAccelerationUnitsPerSecond(
      double kTeleDriveMaxAngularAccelerationUnitsPerSecond) {
    this.kTeleDriveMaxAngularAccelerationUnitsPerSecond =
        kTeleDriveMaxAngularAccelerationUnitsPerSecond;
  }
}
