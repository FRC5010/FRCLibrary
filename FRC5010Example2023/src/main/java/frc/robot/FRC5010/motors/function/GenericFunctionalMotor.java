// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.function;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.GenericEncoder;

/** Add your docs here. */
public class GenericFunctionalMotor implements MotorController5010 {
    protected MotorController5010 _motor;

    public GenericFunctionalMotor(MotorController5010 motor){
        this._motor = motor;
    }

    public GenericFunctionalMotor(MotorController5010 motor, double slewRate){
        this._motor = motor;
        _motor.setSlewRate(slewRate);
    }

    @Override
    public MotorController5010 duplicate(int port) {
        return _motor.duplicate(port);
    }

    @Override
    public MotorController5010 setCurrentLimit(int limit) {
        _motor.setCurrentLimit(limit);
        return this;
    }

    @Override
    public MotorController5010 setSlewRate(double rate){
        _motor.setSlewRate(rate);
        return this;
    }

    @Override
    public void set(double speed) {
        _motor.set(speed);
    }

    @Override
    public double get() {
        return _motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        _motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return _motor.getInverted();
    }

    @Override
    public void disable() {
        _motor.disable();
    }

    @Override
    public void stopMotor() {
        _motor.stopMotor();
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor) {
        _motor.setFollow(motor);
        return this;
    }

    @Override
    public MotorController5010 invert(boolean inverted) {
        _motor.invert(inverted);
        return this;
    }
    @Override
    public GenericEncoder getMotorEncoder(){
        return _motor.getMotorEncoder();
    }

    @Override
    public GenericEncoder getMotorEncoder(Type sensorType, int countsPerRev){
        return _motor.getMotorEncoder(sensorType, countsPerRev);
    }

    @Override
    public MotorController getMotor() {
        return _motor.getMotor();
    }

    @Override
    public void factoryDefault() {
        // Not sure if it needs to return anything
        _motor.factoryDefault();
    }
}
