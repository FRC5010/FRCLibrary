// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/** Add your docs here. */
public class SimulatedEncoder implements GenericEncoder {
  Encoder encoder;
  EncoderSim encoderSim;
  boolean inverted = false;

  public SimulatedEncoder(Encoder encoder) {
    this.encoder = encoder;
    init();
  }

  public SimulatedEncoder(int port1, int port2) {
    this.encoder = new Encoder(port1, port2);
    init();
  }

  private void init() {
    encoderSim = new EncoderSim(encoder);
  }

  @Override
  public double getPosition() {
    return encoderSim.getDistance();
  }

  @Override
  public double getVelocity() {
    return encoderSim.getRate();
  }

  @Override
  public void reset() {
    encoder.reset();
    encoderSim.resetData();
  }

  @Override
  public void setPosition(double position) {
    encoderSim.setDistance(position);
  }

  @Override
  public void setRate(double rate) {
    encoderSim.setRate(rate);
  }

  @Override
  public void setPositionConversion(double conversion) {
    encoder.setDistancePerPulse(conversion);
    encoderSim.setDistancePerPulse(conversion);
  }

  @Override
  public void setVelocityConversion(double conversion) {
    encoder.setDistancePerPulse(conversion);
    encoderSim.setDistancePerPulse(conversion);
  }

  @Override
  public void setInverted(boolean inverted) {
    encoderSim.setDirection(inverted);
    this.inverted = inverted;
  }
}
