// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

/** Add your docs here. */
public class DrivePorts {
    protected int drivePort;
    protected GenericMotorConstants driveMotorConstants;

    public DrivePorts(int drivePort, GenericMotorConstants motorConstants) {
        this.drivePort = drivePort;
        this.driveMotorConstants = motorConstants;
    }

    public int getDrivePort() {
        return drivePort;
    }

    public void setDrivePort(int drivePort) {
        this.drivePort = drivePort;
    }
    public GenericMotorConstants getMotorConstants() {
        return driveMotorConstants;
    }
}
