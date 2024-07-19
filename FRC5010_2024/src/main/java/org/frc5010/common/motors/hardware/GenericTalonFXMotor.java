// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.control.TalonFXPID;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.GenericEncoder.EncoderMeasurementType;
import org.frc5010.common.sensors.encoder.TalonFXEncoder;

/** Add your docs here. */
public class GenericTalonFXMotor extends TalonFX implements MotorController5010 {
  protected int motorCurrentLimit;
  protected int controllerCurrentLimit;
  protected boolean enableFOC = true;

  protected TalonFXConfiguration configuration = new TalonFXConfiguration();

  public GenericTalonFXMotor(int port) {
    super(port);
    factoryDefault();
  }

  @Override
  public MotorController5010 duplicate(int port) {
    MotorController5010 duplicate = new GenericTalonFXMotor(port);
    return duplicate;
  }

  @Override
  public MotorController5010 setCurrentLimit(int limit) {
    motorCurrentLimit = limit;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = limit;
    currentLimits.SupplyCurrentLimit = 60;
    currentLimits.SupplyCurrentLimitEnable = true;

    TalonFXConfigurator cfg = super.getConfigurator();
    cfg.refresh(configuration.CurrentLimits);
    cfg.apply(currentLimits);

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
  public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
    VoltageOut sysidControl = new VoltageOut(0);
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate is acceptable
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
            null, // Default timeout is acceptable
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> super.setControl(sysidControl.withOutput(volts.in(Volts))),
            null,
            subsystemBase));
  }

  @Override
  public void factoryDefault() {
    super.getConfigurator().apply(new TalonFXConfiguration());
  }

  public boolean isFOCEnabled() {
    return enableFOC;
  }

  public void enableFOC(boolean enableFOC) {
    this.enableFOC = enableFOC;
  }
}
