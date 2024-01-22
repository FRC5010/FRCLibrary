// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class PivotSubsystem extends GenericSubsystem {
  MotorController5010 pivotMotor;
  RevEncoder encoder;
  ArmFeedforward pivotFeedforward;
  SparkPIDController pivotPID;


  Mechanism2d robotSim;
  MechanismRoot2d simPivotRoot;
  MechanismLigament2d simPivotArm;
  MechanismLigament2d simTargetArm;
  

  SingleJointedArmSim pivotSim;
  SimulatedEncoder simEncoder = new SimulatedEncoder(10, 11);

  private final double PIVOT_LENGTH = Units.inchesToMeters(19);
  private final double PIVOT_MASS = Units.lbsToKilograms(22);
  private final double PIVOT_START_ANGLE = 225;
  private final double PIVOT_kS = 0.2;

  private final String PIVOT_kG = "PivotKg";


  
  /** Creates a new Pivot. */
  public PivotSubsystem(MotorController5010 pivot, Mechanism2d mechSim) {
    pivotMotor = pivot;
    encoder = ((RevEncoder)pivotMotor.getMotorEncoder());
    pivotPID = ((CANSparkMax)pivotMotor).getPIDController();


    robotSim = mechSim;

    values.declare(PIVOT_kG, 0.0);

    // Simulation Setup

    simPivotRoot = robotSim.getRoot("Pivot Root", 50, 40);

    simPivotArm = simPivotRoot.append(new MechanismLigament2d("Arm", 25, PIVOT_START_ANGLE, 6, new Color8Bit(Color.kOrange)) );
    simTargetArm = simPivotRoot.append(new MechanismLigament2d("Target Arm", 25, PIVOT_START_ANGLE, 6, new Color8Bit(Color.kBlue)));

    
    pivotSim = new SingleJointedArmSim(DCMotor.getNEO(2), 9, SingleJointedArmSim.estimateMOI(PIVOT_LENGTH, PIVOT_MASS), PIVOT_LENGTH, -30, 255, true, PIVOT_START_ANGLE);



    SmartDashboard.putData(this);


  }

  public double getPivotPosition() {
    if (Robot.isReal()) {
      return encoder.getPosition();
    } else {
      return simEncoder.getPosition();
    }
  }

  public double getFeedFowardVoltage() {
    Double pivotKG = values.getDouble(PIVOT_kG);

    pivotFeedforward = new ArmFeedforward(PIVOT_kS, pivotKG, 0);

    double ff = pivotFeedforward.calculate(Units.degreesToRadians(getPivotPosition()), 0);

    return ff;
  }

  public void setSpeed(double speed) {
    double ff = getFeedFowardVoltage() / 12.0;
    pivotMotor.set(speed + ff);
    SmartDashboard.putNumber("Pivot Speed", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    pivotSim.setInput(pivotMotor.get() * RobotController.getBatteryVoltage());
    
    // Next, we update it. The standard loop time is 20ms.
    pivotSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    simEncoder.setPosition(Units.radiansToDegrees(pivotSim.getAngleRads()));
    //simPivotArm.setAngle(Units.degreesToRadians(pivotSim.getAngleRads()));
    simPivotArm.setAngle(simEncoder.getPosition());

    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    SmartDashboard.putNumber("Pivot Sim Rotation", Units.radiansToDegrees(pivotSim.getAngleRads()));
  }
}
