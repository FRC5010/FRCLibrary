// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

/** Add your docs here. */
public class MotorModelConstants {
  private double kS;
  private double kF;
  private double kV;
  private double kA;

  public MotorModelConstants(double kS, double kG, double kV) {
    this.kS = kS;
    this.kF = kG;
    this.kV = kV;
    this.kA = 0;
  }

  public MotorModelConstants(double kS, double kG, double kV, double kA) {
    this.kS = kS;
    this.kF = kG;
    this.kV = kV;
    this.kA = kA;
  }

  public double getkS() {
    return kS;
  }

  public double getkF() {
    return kF;
  }

  public double getkV() {
    return kV;
  }

  public double getkA() {
    return kA;
  }
}
