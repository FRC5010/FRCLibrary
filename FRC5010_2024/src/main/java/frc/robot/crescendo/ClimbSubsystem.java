// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.ValueSwitch;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

public class ClimbSubsystem extends GenericSubsystem {
  /** Creates a new ClimbSubsystem. */
  MotorController5010 leftMotor;
  MotorController5010 rightMotor;
  RevEncoder leftEncoder;
  RevEncoder rightEncoder;
  GenericGyro gyro;

  // Current Switches
  ValueSwitch leftCurrentSwitch;
  ValueSwitch rightCurrentSwitch;


  // Simulation Components
  Mechanism2d robotSim;
  MechanismRoot2d simClimbRoot;
  MechanismLigament2d simLeftClimbArm;
  MechanismLigament2d simRightClimbArm;

  SimulatedEncoder leftSimEncoder = new SimulatedEncoder(12, 13);
  SimulatedEncoder rightSimEncoder = new SimulatedEncoder(14, 15);

  private final double MAX_POSITION_DEVIANCE = 3.0;
  

  // NetworkTable names
  private final String MAX_EXTENSION = "Max Extension";
  private final double MAX_EXTENSION_DEFAULT = 15.0;
  private final String CURRENT_THRESHOLD = "Climb Current Threshold";
  private final double CURRENT_THRESHOLD_DEFAULT = 40.0;

  public ClimbSubsystem(MotorController5010 leftMotor, MotorController5010 rightMotor, GenericGyro gyro, Mechanism2d mechSim) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.gyro = gyro;

    values.declare(MAX_EXTENSION, MAX_EXTENSION_DEFAULT);
    values.declare(CURRENT_THRESHOLD, CURRENT_THRESHOLD_DEFAULT);

    leftEncoder = (RevEncoder) leftMotor.getMotorEncoder();
    rightEncoder = (RevEncoder) rightMotor.getMotorEncoder();
    
    leftEncoder.setPositionConversion(1.0);
    rightEncoder.setPositionConversion(1.0);

    leftCurrentSwitch = new ValueSwitch(() -> values.getDouble(CURRENT_THRESHOLD), ((CANSparkMax)leftMotor)::getOutputCurrent, 0.1); // TODO: Get "desired" current and use it as threshold
    rightCurrentSwitch = new ValueSwitch(() -> values.getDouble(CURRENT_THRESHOLD), ((CANSparkMax)rightMotor)::getOutputCurrent, 0.1); // TODO: Get "desired" current and use it as threshold
    
    
    // TODO: Actually Implement SImulation
    // Simulation Setup
    robotSim = mechSim;
    


  }

  public boolean leftIsAtMin() {
    if (getLeftMotorPosition() < 0.0 || leftCurrentSwitch.get()) { // TODO: Add current check and switch...maybe reset encoder at this point
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

  // Zeroes the encoder if current switch triggers and current encoder position is close enough
  private void update_encoder_extremas() {
    if (leftCurrentSwitch.get() && getLeftMotorPosition() < MAX_POSITION_DEVIANCE) {
      leftEncoder.setPosition(0);
    } else if (leftCurrentSwitch.get() && getLeftMotorPosition() > values.getDouble(MAX_EXTENSION)) {
      leftEncoder.setPosition(values.getDouble(MAX_EXTENSION));
    }

    if (rightCurrentSwitch.get() && getRightMotorPosition() < MAX_POSITION_DEVIANCE) {
      rightEncoder.setPosition(0);
    } else if (rightCurrentSwitch.get() && getRightMotorPosition() > values.getDouble(MAX_EXTENSION)) {
      rightEncoder.setPosition(values.getDouble(MAX_EXTENSION));
    }

    
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Climb Current Switch", leftCurrentSwitch.get());
    SmartDashboard.putBoolean("Right Climb Current Switch", rightCurrentSwitch.get());

    update_encoder_extremas();
  }
}
