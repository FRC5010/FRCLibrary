// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import org.frc5010.common.motors.MotorConstants;

/** Add your docs here. */
public class KrakenX60 extends GenericTalonFXMotor {
  public static final double MAXRPM = 6000;

  public KrakenX60(int port) {
    super(port);
    setCurrentLimit(MotorConstants.CurrentLimits.KrakenX60);
  }
}
