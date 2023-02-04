// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

/** Add your docs here. */
public class ElevatorConstants {
    private double kS;
    private double kG;
    private double kV;
    private double kA;
    
    public ElevatorConstants(double kS, double kG, double kV) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = 0;
    }
    public ElevatorConstants(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }
    public double getkS() {
        return kS;
    }
    public double getkG() {
        return kG;
    }
    public double getkV() {
        return kV;
    }
    public double getkA() {
        return kA;
    }
}
