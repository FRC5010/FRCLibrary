// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Add your docs here. */
public class MotorFeedFwdConstants {

  private double kSC;
  private double kVC;
  private double kAC;

  private double kS;
  private double kV;
  private double kA;

  public MotorFeedFwdConstants(double kSC, double kVC, double kAC, boolean convert) {
    this.kSC = kSC;
    this.kVC = kVC;
    this.kAC = kAC;
    if (convert) {
      kS = kSC / 12;
      kV = kVC / 60 / 1 / (12 - kS);
      kA = kAC / 60 / 1 / (12 - kS);
    }
  }

  public MotorFeedFwdConstants(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  public double getkSC() {
    return kSC;
  }

  public void setkSC(double kSC) {
    this.kSC = kSC;
  }

  public double getkVC() {
    return kVC;
  }

  public void setkVC(double kVC) {
    this.kVC = kVC;
  }

  public double getkAC() {
    return kAC;
  }

  public void setkAC(double kAC) {
    this.kAC = kAC;
  }

  public double getkS() {
    return kS;
  }

  public void setkS(double kS) {
    this.kS = kS;
  }

  public double getkV() {
    return kV;
  }

  public void setkV(double kV) {
    this.kV = kV;
  }

  public double getkA() {
    return kA;
  }

  public void setkA(double kA) {
    this.kA = kA;
  }
}
