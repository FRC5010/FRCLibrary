// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.hardware;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.PIDController5010;
import frc.robot.FRC5010.motors.control.TalonFXPID;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder.EncoderMeasurementType;
import frc.robot.FRC5010.sensors.encoder.TalonFXEncoder;

/** Add your docs here. */
public class GenericTalonFXMotor extends TalonFX implements MotorController5010 {
    protected int motorCurrentLimit;
    protected int controllerCurrentLimit;
    
    public GenericTalonFXMotor(int port) {
        super(port);
    }

    @Override
    public MotorController5010 duplicate(int port) {
        MotorController5010 duplicate = new GenericTalonFXMotor(port);
        return duplicate;
    }

    @Override
    public MotorController5010 setCurrentLimit(int limit) {
        motorCurrentLimit = limit;
        super.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(limit)); // TODO: Add error handling
        return this;
    }

    @Override
    public MotorController5010 setSlewRate(double rate) {
        super.getConfigurator().apply(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(rate));
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor) {
        super.setControl(new Follower(((TalonFX) motor).getDeviceID(), false));
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
        super.setControl(new Follower(((TalonFX) motor).getDeviceID(), inverted));
        return this;
    }

    @Override
    public MotorController5010 invert(boolean inverted) {
        super.setInverted(inverted);
        return this;
    }

    @Override
    public GenericEncoder getMotorEncoder() {
        return new TalonFXEncoder(this);
    }

    @Override
    public GenericEncoder getMotorEncoder(EncoderMeasurementType sensorType, int countsPerRev) {
        return new TalonFXEncoder(this);
    }

    @Override
    public PIDController5010 getPIDController5010() {
        return new TalonFXPID(this);
    }

    @Override
    public MotorController getMotor() {
        return this;
    }

    @Override
    public void factoryDefault() {
        super.getConfigurator().apply(new TalonFXConfiguration());
    }
    




    

}
