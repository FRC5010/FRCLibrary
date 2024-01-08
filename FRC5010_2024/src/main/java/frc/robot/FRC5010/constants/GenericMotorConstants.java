// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

/** Add your docs here. */
public class GenericMotorConstants {

    private double kSC;
    private double kVC;
    private double kAC;

    private double kS; 
    private double kV;  
    private double kA;  
    
    public GenericMotorConstants(double kSC, double kVC, double kAC) {
        this.kSC = kSC;
        this.kVC = kVC;
        this.kAC = kAC;

        kS = kSC / 12;
        kV = kVC / 60 / 1 / (12 - kS);
        kA = kAC / 60 / 1 / (12 - kS);
        
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
