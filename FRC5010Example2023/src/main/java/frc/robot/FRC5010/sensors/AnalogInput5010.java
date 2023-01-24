// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public class AnalogInput5010 implements GenericEncoder{

    AnalogInput analogInput;

    public AnalogInput5010(int port){
        analogInput = new AnalogInput(port);
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        return (analogInput.getVoltage() / RobotController.getVoltage5V()) * (Math.PI * 2) - Math.PI;
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void reset() {
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
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRate(double rate) {
        // TODO Auto-generated method stub
        
    }}
