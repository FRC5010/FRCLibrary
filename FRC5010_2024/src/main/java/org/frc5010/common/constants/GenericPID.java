// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Add your docs here. */
public class GenericPID {

  private double kP;
  private double kI;
  private double kD;

  public GenericPID(double kP, double kI, double kD) {

    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public double getkP() {
    return kP;
  }

  public void setkP(double kP) {
    this.kP = kP;
  }

  public double getkI() {
    return kI;
  }

  public void setkI(double kI) {
    this.kI = kI;
  }

  public double getkD() {
    return kD;
  }

  public void setkD(double kD) {
    this.kD = kD;
  }
}
