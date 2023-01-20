// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors;

import com.ctre.phoenix.sensors.CANCoder;

/** Add your docs here. */
public class CanCoderEncoder implements GenericEncoder{

    private CANCoder canCoder;

    public CanCoderEncoder(int CanID){
        this.canCoder = new CANCoder(CanID);
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        return canCoder.getPosition();
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return canCoder.getVelocity();
    }

    @Override
    public void reset() {
        canCoder.setPositionToAbsolute();
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setPositionConversion(double conversion) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setVelocityConversion(double conversion) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setPosition(double position) {
        canCoder.setPosition(position);
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRate(double rate) {
        // TODO Auto-generated method stub
        
    }
}
