// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class SwerveModuleConstants {

  private double kWheelDiameterMeters; // = Units.inchesToMeters(3);
  private double kDriveMotorGearRatio; // = 1/5.25;
  private double kTurningMotorGearRatio; // = 1/((5.33) * 10.5); // not 12:1 but 10.5 for gearbox,
  // ultraplanetaries are not nominal
  private double kDriveEncoderRot2Meter; // = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  private double kTurningEncoderRot2Rad; // = kTurningMotorGearRatio * 2 * Math.PI;
  private double kDriveEncoderRPM2MeterPerSec; // = kDriveEncoderRot2Meter / 60;
  private double kTurningEncoderRPM2RadPerSec; // = kTurningEncoderRot2Rad / 60;
  private boolean drivingInv;
  private boolean turningInv;
  private boolean encoderInv;
  private Map<String, MotorFeedFwdConstants> driveMotorFF = new HashMap<>();

  public SwerveModuleConstants(
      double kWheelDiameterMeters,
      double kDriveMotorGearRatio,
      boolean drivingInv,
      double kTurningMotorGearRatio,
      boolean turningInv,
      boolean encoderInv) {

    this.kWheelDiameterMeters = kWheelDiameterMeters;
    this.kDriveMotorGearRatio = kDriveMotorGearRatio;
    this.drivingInv = drivingInv;
    this.kTurningMotorGearRatio = kTurningMotorGearRatio;
    this.turningInv = turningInv;
    this.encoderInv = encoderInv;
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  }

  public boolean isDrivingInv() {
    return drivingInv;
  }

  public void setDrivingInv(boolean drivingInv) {
    this.drivingInv = drivingInv;
  }

  public boolean isTurningInv() {
    return turningInv;
  }

  public void setTurningInv(boolean turningInv) {
    this.turningInv = turningInv;
  }

  public boolean isEncoderInv() {
    return encoderInv;
  }

  public void setEncoderInv(boolean encoderInv) {
    this.encoderInv = encoderInv;
  }

  public double getkWheelDiameterMeters() {
    return kWheelDiameterMeters;
  }

  public void setkWheelDiameterMeters(double kWheelDiameterMeters) {
    this.kWheelDiameterMeters = kWheelDiameterMeters;
  }

  public double getkDriveMotorGearRatio() {
    return kDriveMotorGearRatio;
  }

  public void setkDriveMotorGearRatio(double kDriveMotorGearRatio) {
    this.kDriveMotorGearRatio = kDriveMotorGearRatio;
  }

  public double getkTurningMotorGearRatio() {
    return kTurningMotorGearRatio;
  }

  public void setkTurningMotorGearRatio(double kTurningMotorGearRatio) {
    this.kTurningMotorGearRatio = kTurningMotorGearRatio;
  }

  public double getkDriveEncoderRot2Meter() {
    return kDriveEncoderRot2Meter;
  }

  public void setkDriveEncoderRot2Meter(double kDriveEncoderRot2Meter) {
    this.kDriveEncoderRot2Meter = kDriveEncoderRot2Meter;
  }

  public double getkTurningEncoderRot2Rad() {
    return kTurningEncoderRot2Rad;
  }

  public void setkTurningEncoderRot2Rad(double kTurningEncoderRot2Rad) {
    this.kTurningEncoderRot2Rad = kTurningEncoderRot2Rad;
  }

  public double getkDriveEncoderRPM2MeterPerSec() {
    return kDriveEncoderRPM2MeterPerSec;
  }

  public void setkDriveEncoderRPM2MeterPerSec(double kDriveEncoderRPM2MeterPerSec) {
    this.kDriveEncoderRPM2MeterPerSec = kDriveEncoderRPM2MeterPerSec;
  }

  public double getkTurningEncoderRPM2RadPerSec() {
    return kTurningEncoderRPM2RadPerSec;
  }

  public void setkTurningEncoderRPM2RadPerSec(double kTurningEncoderRPM2RadPerSec) {
    this.kTurningEncoderRPM2RadPerSec = kTurningEncoderRPM2RadPerSec;
  }

  public void addDriveMotorFF(String module, MotorFeedFwdConstants constants) {
    driveMotorFF.put(module, constants);
  }

  public Map<String, MotorFeedFwdConstants> getDriveFeedForward() {
    return driveMotorFF;
  }
}
