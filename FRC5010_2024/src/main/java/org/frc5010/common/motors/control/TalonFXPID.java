// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.hardware.GenericTalonFXMotor;

/** Add your docs here. */
public class TalonFXPID implements PIDController5010 {
  GenericTalonFXMotor motor;
  Slot0Configs PIDConfigs = new Slot0Configs();
  PIDControlType controlType = PIDControlType.VOLTAGE;
  double reference = 0.0;
  double tolerance;

  public TalonFXPID(GenericTalonFXMotor motor) {
    this.motor = motor;
  }

  private void refreshTalonConfigs() {
    motor.getConfigurator().apply(PIDConfigs);
  }

  private void sendControlRequest(double reference, double feedforward) {
    switch (controlType) {
      case POSITION:
        motor.setControl(
            new PositionVoltage(reference)
                .withFeedForward(feedforward)
                .withEnableFOC(motor.isFOCEnabled()));
        break;
      case VELOCITY:
        motor.setControl(
            new VelocityVoltage(reference)
                .withFeedForward(feedforward)
                .withEnableFOC(motor.isFOCEnabled()));
        break;
      case DUTY_CYCLE:
        motor.setControl(new DutyCycleOut(reference).withEnableFOC(motor.isFOCEnabled()));
        break;
      case VOLTAGE:
        motor.setControl(new VoltageOut(reference).withEnableFOC(motor.isFOCEnabled()));
        break;
      default:
        throw new IllegalArgumentException("Unsupported TalonFX control type");
    }
  }

  @Override
  public boolean isAtTarget() {
    switch (controlType) {
      case VELOCITY:
        SmartDashboard.putNumber(
            "PID Velocity Difference from Reference" + motor.getDeviceID(),
            Math.abs(getReference() - motor.getVelocity().getValueAsDouble()));
        SmartDashboard.putNumber(
            "Motor Velocity" + motor.getDeviceID(), motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(
            "Closed Loop Reference" + motor.getDeviceID(),
            motor.getClosedLoopReference().getValueAsDouble());
        return Math.abs(getReference() - motor.getVelocity().getValueAsDouble()) < tolerance;
      case POSITION:
        return Math.abs(getReference() - motor.getPosition().getValueAsDouble()) < tolerance;
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
  public void setValues(GenericPID pid) {
    PIDConfigs.kP = pid.getkP();
    PIDConfigs.kI = pid.getkI();
    PIDConfigs.kD = pid.getkD();
  }

  @Override
  public void setF(double f) { // TODO: This definitely shouldn't just set kS
    PIDConfigs.kS = f;
  }

  @Override
  public void setIZone(double iZone) {}

  @Override
  public void setReference(double reference) {
    this.reference = reference;
    refreshTalonConfigs();
    sendControlRequest(reference, 0.0);
  }

  @Override
  public void setReference(double reference, PIDControlType controlType, double feedforward) {
    this.controlType = controlType;
    refreshTalonConfigs();
    sendControlRequest(reference, feedforward);
  }

  @Override
  public void setControlType(PIDControlType controlType) {
    this.controlType = controlType;
    refreshTalonConfigs();
    sendControlRequest(reference, 0.0);
  }

  @Override
  public GenericPID getValues() {
    return new GenericPID(PIDConfigs.kP, PIDConfigs.kI, PIDConfigs.kD);
  }

  @Override
  public double getP() {
    return PIDConfigs.kP;
  }

  @Override
  public double getI() {
    return PIDConfigs.kI;
  }

  @Override
  public double getD() {
    return PIDConfigs.kD;
  }

  @Override
  public double getF() {
    return PIDConfigs.kS;
  }

  @Override
  public double getIZone() {
    return 0;
  }

  @Override
  public double getReference() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public PIDControlType getControlType() {
    return controlType;
  }

  @Override
  public void setP(double p) {
    PIDConfigs.kP = p;
  }

  @Override
  public void setI(double i) {
    PIDConfigs.kI = i;
  }

  @Override
  public void setD(double d) {
    PIDConfigs.kD = d;
  }

  @Override
  public void setOutputRange(double min, double max) { // TODO: Implement
    throw new UnsupportedOperationException("Not implemented for TalonFX");
  }
}
