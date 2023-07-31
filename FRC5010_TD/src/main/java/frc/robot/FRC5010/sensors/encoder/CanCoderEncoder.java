// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors.encoder;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class CanCoderEncoder implements GenericEncoder{

    private CANCoder canCoder;
    
    public CanCoderEncoder(int CanID){
        this.canCoder = new CANCoder(CanID);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.sensorCoefficient = 360.0 / 4096.0;
        config.unitString = "degrees";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        canCoder.configAllSettings(config);
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        return Units.degreesToRadians(canCoder.getAbsolutePosition());
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

    @Override
    public void setInverted(boolean inverted) {
        // TODO Auto-generated method stub
        
    }
}
