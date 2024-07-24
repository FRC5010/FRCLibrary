// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.GenericEncoder.EncoderMeasurementType;

/** Add your docs here. */
public class GenericFunctionalMotor implements MotorController5010 {
  protected MotorController5010 _motor;

  public GenericFunctionalMotor(MotorController5010 motor) {
    this._motor = motor;
  }

  public GenericFunctionalMotor(MotorController5010 motor, double slewRate) {
    this._motor = motor;
    _motor.setSlewRate(slewRate);
  }

  @Override
  public MotorController5010 duplicate(int port) {
    return _motor.duplicate(port);
  }

  @Override
  public MotorController5010 setCurrentLimit(int limit) {
    _motor.setCurrentLimit(limit);
    return this;
  }

  @Override
  public MotorController5010 setSlewRate(double rate) {
    _motor.setSlewRate(rate);
    return this;
  }

  @Override
  public void set(double speed) {
    _motor.set(speed);
  }

  @Override
  public double get() {
    return _motor.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    _motor.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return _motor.getInverted();
  }

  @Override
  public void disable() {
    _motor.disable();
  }

  @Override
  public void stopMotor() {
    _motor.stopMotor();
  }

  @Override
  public MotorController5010 setFollow(MotorController5010 motor) {
    _motor.setFollow(motor);
    return this;
  }

  @Override
  public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
    _motor.setFollow(motor, inverted);
    return this;
  }

  @Override
  public MotorController5010 invert(boolean inverted) {
    _motor.invert(inverted);
    return this;
  }

  @Override
  public GenericEncoder getMotorEncoder() {
    return _motor.getMotorEncoder();
  }

  @Override
  public GenericEncoder getMotorEncoder(EncoderMeasurementType sensorType, int countsPerRev) {
    return _motor.getMotorEncoder();
  }

  @Override
  public PIDController5010 getPIDController5010() {
    throw new UnsupportedOperationException("Not supported for this motor");
  }

  @Override
  public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
    return _motor.getDefaultSysId(subsystemBase);
  }

  @Override
  public MotorController getMotor() {
    return _motor.getMotor();
  }

  @Override
  public void factoryDefault() {
    // Not sure if it needs to return anything
    _motor.factoryDefault();
  }
}
