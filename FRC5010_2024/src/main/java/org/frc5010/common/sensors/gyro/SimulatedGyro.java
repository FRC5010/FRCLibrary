// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.gyro;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

/** Add your docs here. */
public class SimulatedGyro implements GenericGyro {
  private final AnalogGyro gyro = new AnalogGyro(100);
  public AnalogGyroSim m_gyroSim = new AnalogGyroSim(gyro);

  @Override
  public void reset() {
    gyro.reset();
    m_gyroSim.resetData();
  }

  @Override
  public double getAngle() {
    return m_gyroSim.getAngle();
  }

  @Override
  public double getRate() {
    return m_gyroSim.getRate();
  }

  @Override
  public void setAngle(double angle) {
    m_gyroSim.setAngle(angle);
  }

  @Override
  public double getAngleX() {
    return 0;
  }

  @Override
  public double getAngleY() {
    return 0;
  }

  @Override
  public double getAngleZ() {
    return getAngle();
  }
}
