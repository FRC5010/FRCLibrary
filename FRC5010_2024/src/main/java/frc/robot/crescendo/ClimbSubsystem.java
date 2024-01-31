// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.ValueSwitch;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  MotorController5010 leftMotor;
  MotorController5010 rightMotor;
  RevEncoder leftEncoder;
  RevEncoder rightEncoder;
  GenericGyro gyro;

  ValueSwitch leftCurrentSwitch;
  ValueSwitch rightCurrentSwitch;

  public ClimbSubsystem(MotorController5010 leftMotor, MotorController5010 rightMotor, GenericGyro gyro) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftEncoder = (RevEncoder) leftMotor.getMotorEncoder();
    rightEncoder = (RevEncoder) rightMotor.getMotorEncoder();
    this.gyro = gyro;

    leftCurrentSwitch = new ValueSwitch(() -> 0.0, ((CANSparkMax)leftMotor)::getOutputCurrent, 0.1); // TODO: Get "desired" current and use it as threshold
    rightCurrentSwitch = new ValueSwitch(() -> 0.0, ((CANSparkMax)rightMotor)::getOutputCurrent, 1.0); // TODO: Get "desired" current and use it as threshold
  }

  public boolean leftIsAtMin() {
    if (leftMotor.get() < 0 && rightMotor.get() < 0) { // TODO: Add current check and switch...maybe reset encoder at this point
      return true;
    }
    return false;
  }

  public boolean leftIsAtMax() {
    if (leftMotor.get() > 0 && leftCurrentSwitch.get()) { // TODO: Add current switch
      return true;
    }
    return false;
  }

  public void setLeftMotorSpeed(double speed) {
    leftMotor.set(speed);
  }

  public void setRightMotorSpeed(double speed) {
    rightMotor.set(speed);
  }

  public double getLeftMotorPosition() {
    return leftEncoder.getPosition(); // TODO: Add conversion factor
  }

  public double getRightMotorPosition() {
    return rightEncoder.getPosition(); // TODO: Add conversion factor
  }

  public double getHorizontalTilt() {
    return gyro.getAngleY(); // TODO: Fix if necessary
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle X", gyro.getAngleX());
    SmartDashboard.putNumber("Angle Y", gyro.getAngleY());
    SmartDashboard.putNumber("Angle Z", gyro.getAngleZ());
  }
}
