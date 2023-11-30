// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

/** Add your docs here. */
public class SwervePorts extends DrivePorts {
    private int turnPort;
    private int encoderPort;
    protected GenericMotorConstants turnMotorConstants;

    public SwervePorts(int drivePort, int turnPort, int encoderPort, 
    GenericMotorConstants driveMotorConstants, GenericMotorConstants turnMotorConstants) {
        super(drivePort, driveMotorConstants);
        this.turnPort = turnPort;
        this.encoderPort = encoderPort;
        this.turnMotorConstants = turnMotorConstants;
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
    public GenericMotorConstants getTurnMotorConstants() {
        return turnMotorConstants;
    }
}
