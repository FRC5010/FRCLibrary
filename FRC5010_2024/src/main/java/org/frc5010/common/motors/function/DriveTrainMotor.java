// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import org.frc5010.common.motors.MotorController5010;

/** Add your docs here. */
public class DriveTrainMotor extends GenericFunctionalMotor {
  public DriveTrainMotor(MotorController5010 motor) {
    super(motor);
  }

  public DriveTrainMotor(MotorController5010 motor, double slewRate) {
    super(motor);
    _motor.setSlewRate(slewRate);
  }
}
