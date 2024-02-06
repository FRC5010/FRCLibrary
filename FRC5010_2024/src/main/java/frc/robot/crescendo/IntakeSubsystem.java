// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class IntakeSubsystem extends GenericSubsystem {
  private MotorController5010 topIntakeMotor;
  private MotorController5010 bottomIntakeMotor;

  private RevEncoder topEncoder;
  private RevEncoder bottomEncoder;
  private SparkPIDController topPID;
  private SparkPIDController bottomPID;

  private SimpleMotorFeedforward feedforward;

  private double topReference = 0.0;
  private double bottomReference = 0.0;

  private double topPreviousError;
  private double topPreviousTime;

  private double bottomPreviousError;
  private double bottomPreviousTime;

  // Simulation Parts
  private Mechanism2d robotSim;

  private SimulatedEncoder simTopEncoder;
  private FlywheelSim simTop;
  private MechanismLigament2d topIntakeSim;


  private SimulatedEncoder simBottomEncoder;
  private FlywheelSim simBottom;
  private MechanismLigament2d bottomIntakeSim;

  private static enum IntakeState {
    Joystick, Velocity
  }

  private IntakeState intakeState = IntakeState.Joystick;

  // NetworkTable names

  private final double topConversionFactor = 1.0;
  private final double bottomConversionFactor = 1.0;
  private SimpleMotorFeedforward topFeedFwd;
  private SimpleMotorFeedforward bottomFeedFwd;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(MotorController5010 top, MotorController5010 bottom, Mechanism2d mechSim) {
    topFeedFwd = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    bottomFeedFwd = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    topIntakeMotor = top;
    topEncoder = (RevEncoder) topIntakeMotor.getMotorEncoder();

    bottomIntakeMotor = bottom;
    bottomEncoder = (RevEncoder) bottomIntakeMotor.getMotorEncoder();

    topEncoder.setPositionConversion(topConversionFactor);
    bottomEncoder.setPositionConversion(bottomConversionFactor);

    topPID = ((CANSparkMax) topIntakeMotor.getMotor()).getPIDController();
    bottomPID = ((CANSparkMax) bottomIntakeMotor.getMotor()).getPIDController();

    topPID.setP(0.0);
    topPID.setI(0.0);
    topPID.setD(0.0);

    bottomPID.setP(0.0);
    bottomPID.setI(0.0);
    bottomPID.setD(0.0);


    simTop = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    simBottom = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    robotSim = mechSim;

    topIntakeSim = robotSim.getRoot("Top Intake", 10, 30).append(new MechanismLigament2d("Top Intake Dial", 5.0, 0, 5.0, new Color8Bit(Color.kAquamarine)));


    bottomIntakeSim = robotSim.getRoot("Bottom Intake", 10, 10).append(new MechanismLigament2d("Bottom Intake Dial", 5.0, 0, 5.0, new Color8Bit(Color.kAquamarine)));




    // Sim Init
    simTopEncoder = new SimulatedEncoder(24, 25);
    simBottomEncoder = new SimulatedEncoder(26, 27);
  }

  public double getTopFeedFwdVoltage(double velocity) {
    return topFeedFwd.calculate(velocity);
  }

  public double getBottomFeedFwdVoltage(double velocity) {
    return bottomFeedFwd.calculate(velocity);
  }

  public Command getBottomSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(bottomIntakeMotor,
        bottomIntakeMotor.getMotorEncoder(), "Bottom Motor", this), 5, 3, 3);
  }

  public Command getTopSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(topIntakeMotor, topIntakeMotor.getMotorEncoder(), "Top Motor", this), 5, 3,
        3);
  }

  public void stateMachine(double joystick) {
    switch (intakeState) {
      case Joystick:
        if (joystick == 0) {
          intakeState = IntakeState.Velocity;
          setReference(0, 0);
          topRunToReference();
          bottomRunToReference();
        } else {
          setIntakeSpeed(joystick, joystick);
        }
        break;
    
      case Velocity:
        if (joystick != 0) {
          intakeState = IntakeState.Joystick;
          setIntakeSpeed(joystick, joystick);
        } else {
          topRunToReference();
          bottomRunToReference();
        }
        break;
    }
  }

  public void setIntakeSpeed(double topSpeed, double bottomSpeed) {
    topIntakeMotor.set(topSpeed);
    bottomIntakeMotor.set(bottomSpeed);
  }

  public double getTopIntakeVelocity() {
    return Robot.isReal() ? topEncoder.getVelocity() : simTopEncoder.getVelocity();
  }

  public double getBottomIntakeVelocity() {
    return Robot.isReal() ? bottomEncoder.getVelocity() : simBottomEncoder.getVelocity();
  }

  public double getTopReference() {
    return topReference;
  }

  public double getBottomReference() {
    return bottomReference;
  }

  public void setReference(double topSpeed, double bottomSpeed) {
    topReference = topSpeed;
    bottomReference = bottomSpeed;
  }

  public void topRunToReference() {

    double currentError = getTopReference() - getTopIntakeVelocity();
    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - topPreviousError) / (currentTime - topPreviousTime);
    double voltage = currentError * topPID.getP() + (errorRate * topPID.getD());
    double feedForward = getTopFeedFwdVoltage(getTopReference());
  
    
    if (!Robot.isReal()) {
      topIntakeMotor.set(feedForward/ RobotController.getBatteryVoltage() + voltage);
    } else {
      topPID.setReference(getTopReference(), CANSparkBase.ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
    }
  

    topPreviousError = currentError;
    topPreviousTime = currentTime;
  }

  public void bottomRunToReference() {

    double currentError = getBottomReference() - getBottomIntakeVelocity();
    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - bottomPreviousError) / (currentTime - bottomPreviousTime);
    double voltage = currentError * bottomPID.getP() + (errorRate * bottomPID.getD());
    double feedForward = getBottomFeedFwdVoltage(getBottomReference());
   
    
    if (!Robot.isReal()) {
      bottomIntakeMotor.set(feedForward/ RobotController.getBatteryVoltage() + voltage);
    } else {
      bottomPID.setReference(getBottomReference(), CANSparkBase.ControlType.kPosition, 0, feedForward, ArbFFUnits.kVoltage);
    }


    bottomPreviousError = currentError;
    bottomPreviousTime = currentTime;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    topIntakeSim.setAngle(topIntakeMotor.get() * 180);
    bottomIntakeSim.setAngle(bottomIntakeMotor.get()* 180);

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    simTop.setInput(topIntakeMotor.get() * RobotController.getBatteryVoltage());
    simBottom.setInput(bottomIntakeMotor.get() * RobotController.getBatteryVoltage());
    
    // Next, we update it. The standard loop time is 20ms.
    simTop.update(0.020);
    simBottom.update(0.020);


    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    

    //simPivotArm.setAngle(Units.degreesToRadians(pivotSim.getAngleRads()));


    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simTop.getCurrentDrawAmps(), simBottom.getCurrentDrawAmps()));

  }
}
