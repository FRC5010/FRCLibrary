// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import com.revrobotics.RelativeEncoder;

import frc.robot.FRC5010.GenericEncoder;

public class RevEncoder implements GenericEncoder {
  RelativeEncoder encoder;

  public RevEncoder(RelativeEncoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void reset() {
    encoder.setPosition(0); 
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public void setRate(double rate) {
  }

  @Override
  public void setPositionConversion(double conversion) {
    encoder.setPositionConversionFactor(conversion);
  }

  @Override
  public void setVelocityConversion(double conversion) {
    encoder.setVelocityConversionFactor(conversion);
  }
}
