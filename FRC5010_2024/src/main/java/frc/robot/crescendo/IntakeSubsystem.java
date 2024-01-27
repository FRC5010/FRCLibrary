// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class IntakeSubsystem extends SubsystemBase {
  private MotorController5010 intakeMotor;
  private RevEncoder encoder;
  private SparkPIDController pid;

  // Simulation Parts
  private SimulatedEncoder simEncoder;
  
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(MotorController5010 intakeMotor) {
    this.intakeMotor = intakeMotor;
    this.encoder = (RevEncoder)intakeMotor.getMotorEncoder();
    
    // Sim Init
    this.simEncoder = new SimulatedEncoder(14, 15);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
