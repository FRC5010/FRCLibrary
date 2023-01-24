// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

/** Add your docs here. */
public class SwervePorts {

    private int drivePort;
    private int turnPort;
    private int encoderPort;

    public SwervePorts(int drivePort, int turnPort, int encoderPort) {
        this.drivePort = drivePort;
        this.turnPort = turnPort;
        this.encoderPort = encoderPort;
    }
    
    public int getDrivePort() {
        return drivePort;
    }
    public void setDrivePort(int drivePort) {
        this.drivePort = drivePort;
    }
    public int getTurnPort() {
        return turnPort;
    }
    public void setTurnPort(int turnPort) {
        this.turnPort = turnPort;
    }
    public int getEncoderPort() {
        return encoderPort;
    }
    public void setEncoderPort(int encoderPort) {
        this.encoderPort = encoderPort;
    }
}
