// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors;

import com.ctre.phoenix.sensors.Pigeon2;

/** Add your docs here. */
public class PigeonGyro implements GenericGyro{
    private final Pigeon2 pigeon2;

    public PigeonGyro(int valueCAN){
        pigeon2 = new Pigeon2(valueCAN);
    }

    @Override
    public void reset() {
        pigeon2.setYaw(0.0);
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return pigeon2.getYaw();
    }

    @Override
    public double getAngleX() {
        // TODO Auto-generated method stub
        return pigeon2.getRoll();
    }

    @Override
    public double getAngleY() {
        // TODO Auto-generated method stub
        return pigeon2.getPitch();
    }

    @Override
    public double getAngleZ() {
        // TODO Auto-generated method stub
        return pigeon2.getYaw();
    }

    @Override
    public double getRate() {
        // TODO Auto-generated method stub
        return getRate();
    }

    @Override
    public void setAngle(double angle) {
        // TODO Auto-generated method stub
        pigeon2.setYaw(angle);
    }}
