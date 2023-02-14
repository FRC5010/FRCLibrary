// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

public class ElevatorSubsystem extends SubsystemBase {
  private MotorController5010 lift;
  private SparkMaxPIDController liftController;
  private MotorModelConstants liftConstants;
  private GenericPID liftPID;
  private RelativeEncoder liftEncoder;

  private MotorController5010 extend;
  private SparkMaxPIDController extendController;
  private MotorModelConstants extendConstants;
  private GenericPID extendPID;
  private RelativeEncoder extendEncoder;

  private double KFF = 0.000156;
  private double kIz = 0;

  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d m_elevatorMech2d;

  private double currentPositionTarget;
  // TODO Implement ElevatorFeefForward
  private ElevatorFeedforward extendFeedforward;
  private ElevatorSim extendSim;
  private ArmFeedforward liftFeedforward;
  private SingleJointedArmSim liftSim;

  public ElevatorSubsystem(MotorController5010 lift, GenericPID liftPID,
      MotorController5010 extend, GenericPID extendPID,
      MotorModelConstants liftConstants, MotorModelConstants extendConstants,
      Mechanism2d mech2d) {
    this.currentPositionTarget = 0;

    this.lift = lift;
    this.liftController = ((CANSparkMax) lift).getPIDController();
    this.liftEncoder = ((CANSparkMax) lift).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature , 8192);
    this.liftPID = liftPID;
    this.liftConstants = liftConstants;


    this.extend = extend;
    this.extendController = ((CANSparkMax) extend).getPIDController();
    this.extendEncoder = ((CANSparkMax) extend).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature , 8192);
    this.extendPID = extendPID;
    this.extendConstants = extendConstants;

    this.m_mech2d = mech2d;
    m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 10);
    m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Elevator", Units.metersToInches(0.05), -30));

    extendFeedforward = new ElevatorFeedforward(liftConstants.getkS(), extendConstants.getkV(),
        extendConstants.getkA());
    liftFeedforward = new ArmFeedforward(liftConstants.getkS(), liftConstants.getkF(), liftConstants.getkV());
    liftSim = new SingleJointedArmSim(DCMotor.getNEO(2), 1.0/60.0, 40, 1, Units.degreesToRadians(-30), Units.degreesToRadians(60), true);
    extendSim = new ElevatorSim(DCMotor.getNEO(1), 1.0/5.0, 10, 0.05, 0.05, 1.5, true);

    liftController.setP(liftPID.getkP());
    liftController.setI(liftPID.getkI());
    liftController.setD(liftPID.getkD());
    // TODO Set FF and IZ
    liftController.setFF(KFF);
    liftController.setIZone(kIz);
    liftController.setSmartMotionMaxVelocity(3000, 0);
    liftController.setSmartMotionMinOutputVelocity(0, 0);
    liftController.setSmartMotionMaxAccel(10, 0);
  }

  public void reset() {

  }

  public void setPosition(double position) {
    this.currentPositionTarget = position;
    liftController.setReference(this.currentPositionTarget, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public double getPositionTarget() {
    return this.currentPositionTarget;
  }

  public void winch(double pow) {
    SmartDashboard.putNumber("Winch Power", pow);
    SmartDashboard.putNumber("Winch Current", ((CANSparkMax) lift).getOutputCurrent());
    SmartDashboard.putNumber("Winch Rotation", ((CANSparkMax) lift).getEncoder().getPosition());
    lift.set(pow);
  }

  public void elevate(double pow) {
    SmartDashboard.putNumber("Elevate Power", pow);
    SmartDashboard.putNumber("Elevate Current", ((CANSparkMax) extend).getOutputCurrent());
    SmartDashboard.putNumber("Elevate Rotation", ((CANSparkMax) extend).getEncoder().getPosition());
    extend.set(pow);
  }

  @Override
  public void periodic() {
    m_elevatorMech2d.setLength(extendEncoder.getPosition());
    m_elevatorMech2d.setAngle(liftEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    extendSim.setInput(extend.get() * RobotController.getBatteryVoltage());
    liftSim.setInput(lift.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    extendSim.update(0.020);
    liftSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    liftEncoder.setPosition(extendSim.getPositionMeters());
    liftEncoder.setPosition(extendSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(extendSim.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(liftSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    m_elevatorMech2d.setLength(Units.metersToInches(extendSim.getPositionMeters()));
    m_elevatorMech2d.setAngle(Units.radiansToDegrees(liftSim.getAngleRads()));
  }
}
