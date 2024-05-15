// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

/** Add your docs here. */
public class NavXGyro implements GenericGyro {
  AHRS gyro;

  public NavXGyro(SPI.Port kmxp) {
    gyro = new AHRS(kmxp);
  }

  public NavXGyro(SerialPort.Port kmxp) {
    gyro = new AHRS(kmxp);
  }

  public NavXGyro(I2C.Port kmxp) {
    gyro = new AHRS(kmxp);
  }

  private static class NavXRunnable implements Runnable {
    NavXGyro myGyro;

    public NavXRunnable(NavXGyro me) {
      myGyro = me;
    }

    @Override
    public void run() {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
      }
      myGyro.reset();
    }
  }

  @Override
  public void reset() {
    if (gyro.isCalibrating()) {
      Runnable reset = new NavXRunnable(this);
    } else {
      gyro.reset();
      gyro.setAngleAdjustment(0);
    }
  }

  @Override
  public double getAngle() {
    return -gyro.getAngle();
  }

  @Override
  public double getRate() {
    return gyro.getRate();
  }

  @Override
  public void setAngle(double angle) {
    gyro.setAngleAdjustment(angle);
  }

  @Override
  public double getAngleX() {
    return gyro.getRoll();
  }

  @Override
  public double getAngleY() {
    return gyro.getPitch();
  }

  @Override
  public double getAngleZ() {
    return -gyro.getAngle();
  }

  public AHRS getRawGyro() {
    return gyro;
  }
}
