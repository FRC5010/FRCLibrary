// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.hardware;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public class GenericRevPWMMotor extends Spark implements MotorController5010 {
    protected GenericEncoder motorEncoder;

    public GenericRevPWMMotor(int port) {
        super(port);
    }

    @Override
    /** Sets up the same motor hardware and current limit */
    public MotorController5010 duplicate(int port) {
        MotorController5010 duplicate = new GenericRevPWMMotor(port);
        return duplicate;
    }

    @Override
    public MotorController5010 setCurrentLimit(int limit) {
        return this;
    }

    @Override
    public MotorController5010 setSlewRate(double rate) {
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor) {
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
        return this;
    }

    @Override
    public MotorController5010 invert(boolean inverted) {
        super.setInverted(inverted);
        return this;
    }

    @Override
    public GenericEncoder getMotorEncoder() {
        return motorEncoder;
    }

    @Override
    public GenericEncoder getMotorEncoder(Type sensorType, int countsPerRev) {
        return motorEncoder;
    }

    @Override
    public MotorController getMotor() {
        return this;
    }

    @Override
    public void factoryDefault() {
    }

    public void setMotorEncoder(GenericEncoder mEncoder) {
        motorEncoder = mEncoder;
    }
}
