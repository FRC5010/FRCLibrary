// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  public static final double pivotOffset = ArmLevel.ground.getPivotPosition(); // -14.04;
  private final double pivotConversionFactor = 24.242;
  private MotorController5010 pivotMotor;
  private SparkMaxPIDController pivotController;
  private MotorModelConstants pivotConstants;
  private GenericPID pivotPID;
  private RelativeEncoder pivotEncoder;
  private SimulatedEncoder pivotSimEncoder = new SimulatedEncoder(10, 11);
  private DigitalInput pivotHallEffect, pivotMaxHallEffect;
  private ArmFeedforward pivotFeedforward;
  private SingleJointedArmSim pivotSim;

  private double kIz = 0;
  private Supplier<Double> extendPos;

  private boolean override = false;
  private boolean usingTarget = false;

  private double currentPivotTarget;

  public PivotSubsystem(MotorController5010 pivot, GenericPID pivotPID, MotorModelConstants liftConstants,
      int pivotHallEffectPort, int pivotMaxHallEffectPort, Mechanism2d mech2d) {
    this.currentPivotTarget = 0;

    this.pivotMotor = pivot;
    this.pivotMotor.setInverted(false);
    this.pivotController = ((CANSparkMax) pivot).getPIDController();
    this.pivotEncoder = ((CANSparkMax) pivot).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    pivotEncoder.setPositionConversionFactor(this.pivotConversionFactor);
    this.pivotEncoder.setInverted(true);
    this.pivotPID = pivotPID;
    this.pivotConstants = liftConstants;

    pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75,
        40, 2, Units.degreesToRadians(-20),
        Units.degreesToRadians(60), false);

    pivotFeedforward = new ArmFeedforward(liftConstants.getkS(), liftConstants.getkF(), liftConstants.getkV());

    pivotController.setP(pivotPID.getkP());
    pivotController.setI(pivotPID.getkI());
    pivotController.setD(pivotPID.getkD());
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setOutputRange(-1, 1);

    pivotEncoder.setPosition(ArmLevel.auto.getPivotPosition());
    // TODO Set FF and IZ
    pivotController.setFF(0);
    pivotController.setIZone(kIz);
    pivotController.setSmartMotionMaxVelocity(3000, 0);
    pivotController.setSmartMotionMinOutputVelocity(0, 0);
    pivotController.setSmartMotionMaxAccel(100, 0);
    pivotController.setSmartMotionAllowedClosedLoopError(0.1, 0);

    this.pivotHallEffect = new DigitalInput(pivotHallEffectPort);
    this.pivotMaxHallEffect = new DigitalInput(pivotMaxHallEffectPort);

    this.extendPos = extendPos;
    SmartDashboard.putNumber("Pivot P", pivotPID.getkP());
    SmartDashboard.putBoolean("Override", override);

  }

  public void setPivotEncoderPosition(double pos) {
    this.pivotEncoder.setPosition(pos);
  }

  public double getPivotPosition() {
    if (Robot.isReal()) {
      return pivotEncoder.getPosition();
    } else {
      return pivotSimEncoder.getPosition();
    }
  }

  // public void setPivotPosition(double position) {
  // this.currentPivotTarget = position;
  // SmartDashboard.putNumber("Pivot Target", currentPivotTarget);
  // if (Robot.isReal()) {
  // SmartDashboard.putNumber("Pivot FF", getFeedFowardVoltage());
  // pivotController.setFF(getFeedFowardVoltage() / currentPivotTarget);
  // pivotController.setReference(this.currentPivotTarget,
  // CANSparkMax.ControlType.kPosition, 0);

  // } else {
  // pivotPow((currentPivotTarget - getPivotPosition())/100);
  // }
  // }

  public boolean isCloseToMinHardStop() {
    return (getPivotPosition() < -10);
  }

  public boolean isCloseToMaxHardStop() {
    return (getPivotPosition() > 30);
  }

  public boolean closeToTarget() {
    return Math.abs(getPivotPosition() - currentPivotTarget) < 1 && usingTarget;
  }

  public double getPowerFactor(double pow) {
    double powerFactor = 1;
    double sign = Math.signum(pow);
    boolean closeToTarget = closeToTarget();
    boolean atTarget = isPivotAtTarget();

    if (sign > 0) {

      if (isPivotMaxHardStop(pow) || atTarget) {
        powerFactor = 0;
      } else if ((isCloseToMaxHardStop()) || closeToTarget) {
        powerFactor = 0.25;
      }

    } else {

      if (isPivotMinHardStop(pow) || atTarget) {
        powerFactor = 0;
      } else if (isCloseToMinHardStop() || closeToTarget) {
        powerFactor = 0.25;
      }

    }
    return powerFactor;
  }

  private boolean isPivotMinHardStop(double pow) {
    return isPivotMinHallEffect() && pow < 0;
  }

  private boolean isPivotMaxHardStop(double pow) {
    return isPivotMaxPosition() && pow > 0;
  }

  public void runPivotToTarget(double position) {
    this.currentPivotTarget = position;
    usingTarget = true;
    double kP = pivotPID.getkP();// SmartDashboard.getNumber("Pivot P", pivotPID.getkP());
    double kF = getFeedFowardVoltage();
    double positionSig = Math.signum(this.currentPivotTarget - getPivotPosition());
    double pow = kP * positionSig;
    pivotMotor.setVoltage((pow * getPowerFactor(pow)) + kF);
  }

  public double getFeedFowardVoltage() {
    return 0;/*
              * (0.2 + 0.3 *
              * Math.cos(Units.degreesToRadians(getPivotPosition())));
              */
  }

  public boolean isPivotAtTarget() {
    return Math.abs(getPivotPosition() - this.currentPivotTarget) < 0.05 && usingTarget;
  }

  public double getPivotTarget() {
    return this.currentPivotTarget;
  }

  public boolean isPivotMinHallEffect() {
    if (override) {
      return false;
    }
    return !pivotHallEffect.get(); // && !override;
  }

  public boolean isPivotMaxPosition() {
    if (override) {
      return false;
    }
    return getPivotPosition() > 33.5;
  }

  public void toggleOverride() {
    override = !override;
  }

  public double getVelocity() {
    return pivotEncoder.getVelocity();
  }

  public void pivotPow(double pow, boolean feedForward) {
    usingTarget = false;
    SmartDashboard.putNumber("Pivot Power Given", pow);
    SmartDashboard.putNumber("Pivot Current", ((CANSparkMax) pivotMotor).getOutputCurrent());
    SmartDashboard.putNumber("Pivot Rotation", pivotEncoder.getPosition());

    double powerFactor = getPowerFactor(pow);
    SmartDashboard.putNumber("Pivot Power Factor", powerFactor);

    pivotMotor.set((pow * powerFactor + ((pow == 0 && feedForward) ? (getFeedFowardVoltage() / 12) : 0)));
    SmartDashboard.putNumber("getFeedFowardVoltage", getFeedFowardVoltage());
  }

  // public boolean isMovementOk(double pow){
  // if (Math.signum(pow) > 1 && isPivotMax()){

  // }
  // }

  public void stopPivot() {
    pivotMotor.set(0);
  }

  public void stopAndHoldPivot() {
    pivotMotor.setVoltage(getFeedFowardVoltage());
  }

  public boolean atMinHardStop() {
    // TODO: Test if encoder is greater
    return isPivotMinHallEffect() && pivotEncoder.getVelocity() < 0;
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {

      if (isPivotMinHallEffect()) {
        setPivotEncoderPosition(pivotOffset);
      }

      SmartDashboard.putBoolean("Pivot In: ", isPivotMinHallEffect());
      SmartDashboard.putBoolean("Pivot Max", isPivotMaxPosition());
      SmartDashboard.putBoolean("Pivot Target is good", currentPivotTarget > pivotEncoder.getPosition());
      SmartDashboard.putNumber("Pivot Motor is positive", pivotMotor.get());
    }
    SmartDashboard.putNumber("Pivot Motor Pow: ", pivotMotor.get());
    SmartDashboard.putNumber("Pivot Position: ", getPivotPosition());
    // SmartDashboard.putNumber("Abs", KFF);
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
    pivotSimEncoder.setPosition(Units.radiansToDegrees(pivotSim.getAngleRads()));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    SmartDashboard.putNumber("Pivot Sim Rotation", Units.radiansToDegrees(pivotSim.getAngleRads()));
  }
}
