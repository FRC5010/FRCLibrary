// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.ElevatorConstants;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.hardware.NEO; 

public class ElevatorSubsystem extends SubsystemBase {
  private NEO lift;
  private SparkMaxPIDController liftController;
  private GenericPID liftconstants;
  private ElevatorConstants elevatorConstants;
  private double KFF = 0.000156;
  private double kIz = 0;
  //TODO Implement ElevatorFeefForward
  private ElevatorFeedforward feedforward;
  public ElevatorSubsystem(NEO lift, GenericPID liftconstants, ElevatorConstants elevatorConstants) {
  this.lift = lift;
  this.liftController = lift.getPIDController(); 
  this.liftconstants = liftconstants;
  this.elevatorConstants = elevatorConstants;

  feedforward = new ElevatorFeedforward(elevatorConstants.getkS(), elevatorConstants.getkV(), elevatorConstants.getkA());
  liftController.setP(liftconstants.getkP());
  liftController.setI(liftconstants.getkI());
  liftController.setD(liftconstants.getkD());
  // TODO Set FF and IZ
  liftController.setFF(KFF);
  liftController.setIZone(kIz);
  liftController.setSmartMotionMaxVelocity(3000,0);
  liftController.setSmartMotionMinOutputVelocity(0,0);
  liftController.setSmartMotionMaxAccel(10, 0);
  }
  public void reset() {
  
  }
  public void setPosition(double position) {
    liftController.setReference(position, CANSparkMax.ControlType.kSmartMotion, 0);
    
  }

  @Override
  public void periodic() { 
  }
}
