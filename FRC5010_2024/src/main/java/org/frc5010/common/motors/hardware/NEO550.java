// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import org.frc5010.common.motors.MotorConstants;

/** NEO550 extends CANSparkMax and applies code specific to a NEO550 */
public class NEO550 extends GenericRevBrushlessMotor {
  public static final double MAXRPM = 11000;

  public NEO550(int port) {
    super(port, MotorConstants.CurrentLimits.Neo550);
  }

  public NEO550(int port, int currentLimit) {
    super(port, currentLimit);
  }
}
