// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class CanCoderEncoder implements GenericEncoder {

  private CANcoder canCoder;

  public CanCoderEncoder(int CanID) {
    this.canCoder = new CANcoder(CanID);
    CANcoderConfiguration config = new CANcoderConfiguration();
    // config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    // config.sensorCoefficient = 360.0 / 4096.0;
    // config.unitString = "degrees";
    // config.sensorTimeBase = SensorTimeBase.PerSecond;
    // canCoder.configAllSettings(config);
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    return Units.degreesToRadians(canCoder.getAbsolutePosition().getValue());
  }

  @Override
  public double getVelocity() {
    // TODO Auto-generated method stub
    return canCoder.getVelocity().getValue();
  }

  @Override
  public void reset() {
    // canCoder.setPositionToAbsolute();
    // TODO Auto-generated method stub

  }

  @Override
  public void setPositionConversion(double conversion) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setVelocityConversion(double conversion) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setPosition(double position) {
    canCoder.setPosition(position);
    // TODO Auto-generated method stub

  }

  @Override
  public void setRate(double rate) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setInverted(boolean inverted) {
    // TODO Auto-generated method stub

  }
}
