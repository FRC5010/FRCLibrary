// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import org.frc5010.common.constants.GenericPID;

public interface PIDController5010 {
  public static enum PIDControlType {
    POSITION,
    VELOCITY,
    VOLTAGE,
    CURRENT,
    DUTY_CYCLE
  }

  public void setTolerance(double tolerance);

  public double getTolerance();

  public void setValues(GenericPID pid);

  public void setP(double p);

  public void setI(double i);

  public void setD(double d);

  public void setF(double f);

  public void setIZone(double iZone);

  public void setOutputRange(double min, double max);

  public void setReference(double reference);

  public void setReference(double reference, PIDControlType controlType, double feedforward);

  public void setControlType(PIDControlType controlType);

  public GenericPID getValues();

  public double getP();

  public double getI();

  public double getD();

  public double getF();

  public double getIZone();

  public double getReference();

  public PIDControlType getControlType();

  public boolean isAtTarget();
}
