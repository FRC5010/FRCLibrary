// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

/** Add your docs here. */
public class PigeonGyro implements GenericGyro {
  private final Pigeon2 pigeon2;

  public PigeonGyro(int valueCAN) {
    pigeon2 = new Pigeon2(valueCAN);
  }

  public Pigeon2 getGyro() {
    return pigeon2;
  }

  @Override
  public void reset() {
    pigeon2.setYaw(0.0);
  }

  @Override
  public double getAngle() {
    return pigeon2.getAngle();
  }

  @Override
  public double getAngleX() {
    return pigeon2.getRoll().getValue();
  }

  @Override
  public double getAngleY() {
    return pigeon2.getPitch().getValue();
  }

  @Override
  public double getAngleZ() {
    return pigeon2.getYaw().getValue();
  }

  @Override
  public double getRate() {

    return pigeon2.getRate();
  }

  @Override
  public void setAngle(double angle) {
    pigeon2.setYaw(angle);
  }
  // big kahunas
}
