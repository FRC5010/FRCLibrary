// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import org.frc5010.common.motors.MotorConstants;

/** Add your docs here. */
public class NEO extends GenericRevBrushlessMotor {
  public static final double MAXRPM = 5676;

  public NEO(int port) {
    super(port, MotorConstants.CurrentLimits.Neo);
  }

  public NEO(int port, int currentLimit) {
    super(port, currentLimit);
  }
}
