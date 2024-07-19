// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.GenericEncoder.EncoderMeasurementType;

/** Add your docs here. */
public interface MotorController5010 extends MotorController {
  /** Sets up the same motor hardware and current limit */
  MotorController5010 duplicate(int port);

  MotorController5010 setSlewRate(double rate);

  MotorController5010 setFollow(MotorController5010 motor);

  MotorController5010 setFollow(MotorController5010 motor, boolean inverted);

  MotorController5010 invert(boolean inverted);

  MotorController5010 setCurrentLimit(int limit);

  GenericEncoder getMotorEncoder();

  GenericEncoder getMotorEncoder(EncoderMeasurementType sensorType, int countsPerRev);

  PIDController5010 getPIDController5010();

  MotorController getMotor();

  void factoryDefault();

  SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase);
}
