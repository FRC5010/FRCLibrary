// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

/** Add your docs here. */
public class PigeonGyro implements GenericGyro {
    private final WPI_Pigeon2 pigeon2;

    public PigeonGyro(int valueCAN) {
        pigeon2 = new WPI_Pigeon2(valueCAN);
    }

    @Override
    public void reset() {
        pigeon2.setYaw(0.0);
    }

    @Override
    public double getAngle() {
        return pigeon2.getYaw();
    }

    @Override
    public double getAngleX() {
        return pigeon2.getRoll();
    }

    @Override
    public double getAngleY() {
        return pigeon2.getPitch();
    }

    @Override
    public double getAngleZ() {
        return pigeon2.getYaw();
    }

    @Override
    public double getRate() {
        double[] vel = new double[3];
        pigeon2.getRawGyro(vel);
        return vel[2];
    }

    @Override
    public void setAngle(double angle) {
        pigeon2.setYaw(angle);
    }
    // big kahunas
}
