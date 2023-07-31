// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/** Add your docs here. */
public class NavXGyro implements GenericGyro {
    AHRS gyro;
    public NavXGyro(SPI.Port kmxp) {
        gyro = new AHRS(kmxp);
    }
    @Override
    public void reset() {
        gyro.reset();
        gyro.setAngleAdjustment(0);
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
        return gyro.getRoll();
    }
    @Override
    public double getAngleY() {
        return gyro.getPitch();
    }
    @Override
    public double getAngleZ() {
        return gyro.getAngle();
    }
}
