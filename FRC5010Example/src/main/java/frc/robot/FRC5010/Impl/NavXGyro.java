// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.FRC5010.GenericGyro;

/** Add your docs here. */
public class NavXGyro implements GenericGyro {
    AHRS gyro;
    public NavXGyro(Port sPort) {
        gyro = new AHRS(sPort);
    }
    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
    }

    @Override
    public double getRate() {
        return getRate();
    }
    @Override
    public void setAngle(double angle) {
        gyro.setAngleAdjustment(angle);
    }
    @Override
    public double getAngleX() {
        // TODO Auto-generated method stub
        return 0;
    }
    @Override
    public double getAngleY() {
        // TODO Auto-generated method stub
        return 0;
    }
    @Override
    public double getAngleZ() {
        // TODO Auto-generated method stub
        return 0;
    }
}
