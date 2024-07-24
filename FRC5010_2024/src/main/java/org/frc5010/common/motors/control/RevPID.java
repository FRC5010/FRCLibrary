// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.control;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.hardware.GenericRevBrushlessMotor;

/** Add your docs here. */
public class RevPID implements PIDController5010 {
  GenericRevBrushlessMotor motor;
  SparkPIDController controller;
  ControlType sparkControlType = ControlType.kVoltage;
  private double reference = 0.0;
  private double tolerance = 0.0;

  public RevPID(GenericRevBrushlessMotor motor) {
    this.motor = motor;
    controller = motor.getPIDController();
  }

  @Override
  public void setP(double p) {
    controller.setP(p);
  }

  @Override
  public void setI(double i) {
    controller.setI(i);
  }

  @Override
  public void setD(double d) {
    controller.setD(d);
  }

  @Override
  public void setF(double f) {
    controller.setFF(f);
  }

  @Override
  public void setIZone(double iZone) {
    controller.setIZone(iZone);
  }

  @Override
  public void setOutputRange(double min, double max) {
    controller.setOutputRange(min, max);
  }

  @Override
  public void setReference(double reference) {
    this.reference = reference;
    controller.setReference(reference, sparkControlType);
  }

  @Override
  public void setReference(double reference, PIDControlType controlType, double feedforward) {
    setControlType(controlType);
    this.reference = reference;
    controller.setReference(feedforward, sparkControlType, 0, feedforward);
  }

  @Override
  public boolean isAtTarget() {
    switch (sparkControlType) {
      case kVelocity:
        return Math.abs(getReference() - motor.getEncoder().getVelocity()) < tolerance;
      case kPosition:
        return Math.abs(getReference() - motor.getEncoder().getPosition()) < tolerance;
      default:
        return false;
    }
  }

  @Override
  public void setTolerance(double value) {
    tolerance = value;
  }

  @Override
  public double getTolerance() {
    return tolerance;
  }

  @Override
  public void setControlType(PIDControlType controlType) {
    switch (controlType) {
      case POSITION:
        sparkControlType = ControlType.kPosition;
        break;
      case VELOCITY:
        sparkControlType = ControlType.kVelocity;
        break;
      case VOLTAGE:
        sparkControlType = ControlType.kVoltage;
        break;
      case CURRENT:
        sparkControlType = ControlType.kCurrent;
        break;
      case DUTY_CYCLE:
        sparkControlType = ControlType.kDutyCycle;
        break;
      default:
        sparkControlType = ControlType.kVoltage;
        break;
    }
  }

  @Override
  public double getP() {
    return controller.getP();
  }

  @Override
  public double getI() {
    return controller.getI();
  }

  @Override
  public double getD() {
    return controller.getD();
  }

  @Override
  public double getF() {
    return controller.getFF();
  }

  @Override
  public double getIZone() {
    return controller.getIZone();
  }

  @Override
  public double getReference() {
    return reference;
  }

  @Override
  public PIDControlType getControlType() {
    switch (sparkControlType) {
      case kPosition:
        return PIDControlType.POSITION;
      case kVelocity:
        return PIDControlType.VELOCITY;
      case kVoltage:
        return PIDControlType.VOLTAGE;
      case kCurrent:
        return PIDControlType.CURRENT;
      case kDutyCycle:
        return PIDControlType.DUTY_CYCLE;
      default:
        return PIDControlType.VOLTAGE;
    }
  }

  @Override
  public void setValues(GenericPID pid) {
    setP(pid.getkP());
    setI(pid.getkI());
    setD(pid.getkD());
  }

  @Override
  public GenericPID getValues() {
    return new GenericPID(getP(), getI(), getD());
  }
}
