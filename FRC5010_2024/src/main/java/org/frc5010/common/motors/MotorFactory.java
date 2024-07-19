// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import org.frc5010.common.motors.function.DriveTrainMotor;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.hardware.KrakenX60;
import org.frc5010.common.motors.hardware.NEO;
import org.frc5010.common.motors.hardware.NEO550;

/** Add your docs here. */
public class MotorFactory {
  public static MotorController5010 NEO(int port) {
    return new NEO(port);
  }

  public static MotorController5010 NEO(int port, int currentLimit) {
    return new NEO(port, currentLimit);
  }

  public static MotorController5010 NEO550(int port) {
    return new NEO550(port);
  }

  public static MotorController5010 NEO550(int port, int currentLimit) {
    return new NEO550(port, currentLimit);
  }

  public static MotorController5010 KrakenX60(int port) {
    return new KrakenX60(port);
  }

  public static MotorController5010 DriveTrainMotor(MotorController5010 motor) {
    return new DriveTrainMotor(motor);
  }

  public static MotorController5010 FollowMotor(
      MotorController5010 motor, MotorController5010 leader) {
    return new FollowerMotor(motor, leader);
  }
}
