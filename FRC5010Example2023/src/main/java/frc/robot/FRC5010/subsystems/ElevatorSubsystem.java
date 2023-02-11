// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.ElevatorConstants;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  private MotorController5010 lift;
  private MotorController5010 extend;
  private SparkMaxPIDController liftController;
  private GenericPID liftconstants;
  private ElevatorConstants elevatorConstants;
  private double KFF = 0.000156;
  private double kIz = 0;
  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d m_elevatorMech2d;
  private GenericEncoder encoder;
  private double currentPositionTarget;
  //TODO Implement ElevatorFeefForward
  private ElevatorFeedforward feedforward;
  public ElevatorSubsystem(MotorController5010 lift, GenericPID liftconstants, 
    MotorController5010 extend,  
    ElevatorConstants elevatorConstants, Mechanism2d mech2d) {
  this.currentPositionTarget = 0;
  this.lift = lift;
  this.extend = extend;
  this.liftController = ((CANSparkMax)lift).getPIDController(); 
  this.encoder = lift.getMotorEncoder();
  this.liftconstants = liftconstants;
  this.elevatorConstants = elevatorConstants;
  this.m_mech2d = mech2d;
  m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(0), 90));
  // feedforward = new ElevatorFeedforward(elevatorConstants.getkS(), elevatorConstants.getkV(), elevatorConstants.getkA());
  // liftController.setP(liftconstants.getkP());
  // liftController.setI(liftconstants.getkI());
  // liftController.setD(liftconstants.getkD());
  // // TODO Set FF and IZ
  // liftController.setFF(KFF);
  // liftController.setIZone(kIz);
  // liftController.setSmartMotionMaxVelocity(3000,0);
  // liftController.setSmartMotionMinOutputVelocity(0,0);
  // liftController.setSmartMotionMaxAccel(10, 0);
  }
  public void reset() {
  
  }
  public void setPosition(double position) {
    // this.currentPositionTarget = position;
    // liftController.setReference(this.currentPositionTarget, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public double getPositionTarget() {
    return this.currentPositionTarget;
  }
  public void winch(double pow){
    SmartDashboard.putNumber("Winch Power", pow);
    SmartDashboard.putNumber("Winch current", ((CANSparkMax)lift).getOutputCurrent());
    lift.set(pow);
  }

  public void elevate(double pow) {
    SmartDashboard.putNumber("Elevate Power", pow);
    SmartDashboard.putNumber("Elevate current", ((CANSparkMax)extend).getOutputCurrent());
    extend.set(pow);
  }

  @Override
  public void periodic() {
    // m_elevatorMech2d.setLength(encoder.getPosition());
  }
}
