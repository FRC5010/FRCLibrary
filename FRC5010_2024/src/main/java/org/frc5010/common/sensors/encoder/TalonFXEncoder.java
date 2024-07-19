// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import org.frc5010.common.motors.hardware.GenericTalonFXMotor;

/** Add your docs here. */
public class TalonFXEncoder implements GenericEncoder {
  GenericTalonFXMotor motor;
  double positionConversion = 1;
  double velocityConversion = 1;

  public TalonFXEncoder(GenericTalonFXMotor motor) {
    this.motor = motor;
  }

  private double nativeToActualPosition(double position) {
    return position * positionConversion;
  }

  private double actualToNativePosition(double position) {
    return position / positionConversion;
  }

  private double nativeToActualVelocity(double velocity) {
    return velocity * velocityConversion;
  }

  private double actualToNativeVelocity(double velocity) {
    return velocity / velocityConversion;
  }

  @Override
  public double getPosition() {
    return nativeToActualPosition(motor.getPosition().getValueAsDouble());
  }

  @Override
  public double getVelocity() {
    return nativeToActualVelocity(motor.getVelocity().getValueAsDouble());
  }

  @Override
  public void reset() {
    motor.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    motor.setPosition(actualToNativePosition(position));
  }

  @Override
  public void setRate(double rate) {
    throw new UnsupportedOperationException("Not supported for TalonFX");
  }

  @Override
  public void setPositionConversion(double conversion) {
    positionConversion = conversion;
    velocityConversion = conversion * 60;
  }

  @Override
  public void setVelocityConversion(double conversion) {
    velocityConversion = conversion;
    positionConversion = conversion / 60.0;
  }

  @Override
  public void setInverted(boolean inverted) {
    throw new UnsupportedOperationException("Not supported for TalonFX");
  }
}
