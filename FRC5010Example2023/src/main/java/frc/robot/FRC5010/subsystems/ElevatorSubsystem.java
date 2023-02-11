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
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  private NEO lift;
  private SparkMaxPIDController liftController;
  private GenericPID liftconstants;
  private ElevatorConstants elevatorConstants;
  private double KFF = 0.000156;
  private double kIz = 0;
  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d m_elevatorMech2d;
  private GenericEncoder encoder;
  //TODO Implement ElevatorFeefForward
  private ElevatorFeedforward feedforward;
  public ElevatorSubsystem(NEO lift, GenericPID liftconstants, ElevatorConstants elevatorConstants, Mechanism2d mech2d) {
  this.lift = lift;
  this.liftController = lift.getPIDController(); 
  this.encoder = lift.getMotorEncoder();
  this.liftconstants = liftconstants;
  this.elevatorConstants = elevatorConstants;
  this.m_mech2d = mech2d;
  m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(0), 90));
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
    m_elevatorMech2d.setLength(encoder.getPosition());
b      //
  }
}
