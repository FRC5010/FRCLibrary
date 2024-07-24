// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.control.RevPID;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.GenericEncoder.EncoderMeasurementType;
import org.frc5010.common.sensors.encoder.RevEncoder;

/** Add your docs here. */
public class GenericRevBrushlessMotor extends CANSparkMax implements MotorController5010 {
  protected int currentLimit;

  public GenericRevBrushlessMotor(int port, int currentLimit) {
    super(port, MotorType.kBrushless);
    factoryDefault();
    setCurrentLimit(currentLimit);
  }

  @Override
  /** Sets up the same motor hardware and current limit */
  public MotorController5010 duplicate(int port) {
    MotorController5010 duplicate = new GenericRevBrushlessMotor(port, currentLimit);
    return duplicate;
  }

  @Override
  public MotorController5010 setCurrentLimit(int limit) {
    this.currentLimit = limit;
    super.setSmartCurrentLimit(limit);
    return this;
  }

  @Override
  public MotorController5010 setSlewRate(double rate) {
    super.setOpenLoopRampRate(rate);
    return this;
  }

  @Override
  public MotorController5010 setFollow(MotorController5010 motor) {
    super.follow((CANSparkMax) motor.getMotor());
    return this;
  }

  @Override
  public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
    super.follow((CANSparkMax) motor.getMotor(), inverted);
    return this;
  }

  @Override
  public MotorController5010 invert(boolean inverted) {
    super.setInverted(inverted);
    return this;
  }

  @Override
  public GenericEncoder getMotorEncoder() {
    return new RevEncoder(super.getEncoder());
  }

  @Override
  public GenericEncoder getMotorEncoder(EncoderMeasurementType sensorType, int countsPerRev) {
    return new RevEncoder(super.getEncoder());
  }

  @Override
  public PIDController5010 getPIDController5010() {
    return new RevPID(this);
  }

  @Override
  public MotorController getMotor() {
    return this;
  }

  @Override
  public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
    return SystemIdentification.rpmSysIdRoutine(
        this, getMotorEncoder(), "Motor " + super.getDeviceId(), subsystemBase);
  }

  @Override
  public void factoryDefault() {
    super.restoreFactoryDefaults();
  }
}
