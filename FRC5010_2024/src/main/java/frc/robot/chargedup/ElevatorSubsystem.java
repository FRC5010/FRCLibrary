// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class ElevatorSubsystem extends GenericSubsystem {
  /**
   *
   */

  private DigitalInput extendHallEffect;
  private DigitalInput extendMaxHallEffect;

  private MotorController5010 extendMotor;
  private SparkMaxPIDController extendController;
  private MotorModelConstants extendConstants;
  private GenericPID extendPID;
  private RelativeEncoder extendEncoder;
  private SimulatedEncoder extendSimEncoder = new SimulatedEncoder(12, 13);

  // private double KFF = 0.000156;

  private static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 10.0; // kg

  public static final double kMinElevatorHeight = Units.inchesToMeters(30);
  public static final double kMaxElevatorHeight = ElevatorLevel.high.getExtensionPosition();// Units.inchesToMeters(81);

  private double insideTheBumpers = kMinElevatorHeight + 0.1;
  private double outsideTheBumpers = kMinElevatorHeight + 0.5;
  private double aboveTheBumpers = ElevatorLevel.low.getPivotPosition();

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  // = (Pi * D) / ppr
  private static final double kElevatorEncoderDistPerPulse = Units.inchesToMeters(15.45);
  // 2.0 * Math.PI * kElevatorDrumRadius / 8192;

  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d m_elevatorMech2d;
  private static MechanismLigament2d targetPos2d;

  private double currentExtendTarget;
  private double feedForward = 0;

  private boolean usingTarget = false;
  private boolean override = false;
  private static ElevatorLevel targetLevel = ElevatorLevel.ground; // Unsure of whether this should be stored in
                                                                   // subsystem

  // TODO Implement ElevatorFeefForward
  private ElevatorFeedforward extendFeedforward;
  private ElevatorSim extendSim;

  private Supplier<Double> pivotAngle;
  private final String extendKp = "Extend kP";

  public ElevatorSubsystem(MotorController5010 extend, GenericPID extendPID,
      MotorModelConstants extendConstants,
      Mechanism2d mech2d, int extendHallEffectPort, int extendMaxHallEffect, Supplier<Double> pivotAngle) {

    this.currentExtendTarget = 0;
    values.declare(extendKp, extendPID.getkP());
    this.extendMotor = extend;
    this.extendMotor.setInverted(true);
    this.extendController = ((CANSparkMax) extend).getPIDController();
    this.extendEncoder = ((CANSparkMax) extend).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    this.extendEncoder.setPositionConversionFactor(kElevatorEncoderDistPerPulse);
    this.extendSimEncoder.setPositionConversion(0.01);
    this.extendPID = extendPID;
    this.extendConstants = extendConstants;

    setExtendEncoderPosition(ElevatorLevel.auto.getExtensionPosition());

    this.m_mech2d = mech2d;
    m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 0.05, 0.50);
    targetPos2d = m_mech2dRoot.append(
        new MechanismLigament2d("Target", kMinElevatorHeight, -30, 6,
            new Color8Bit(Color.kBlue)));
    m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Elevator", kMinElevatorHeight, -30, 6, new Color8Bit(Color.kOrange)));

    extendSim = new ElevatorSim(DCMotor.getNEO(1), 25,
        kCarriageMass, kElevatorDrumRadius, kMinElevatorHeight, kMaxElevatorHeight, false, 0);

    extendFeedforward = new ElevatorFeedforward(extendConstants.getkS(), extendConstants.getkF(), 0);

    extendController.setP(values.getDouble(extendKp));
    extendController.setI(extendPID.getkI());
    extendController.setD(extendPID.getkD());
    extendController.setFeedbackDevice(extendEncoder);

    extendController.setFF(0.0);

    extendController.setSmartMotionMaxVelocity(3000, 0);
    extendController.setSmartMotionMinOutputVelocity(0, 0);
    extendController.setSmartMotionMaxAccel(100, 0);

    this.extendHallEffect = new DigitalInput(extendHallEffectPort);
    this.extendMaxHallEffect = new DigitalInput(extendMaxHallEffect);

    this.pivotAngle = pivotAngle;

    // SmartDashboard.putNumber("Extend kP", extendPID.getkP());
    SmartDashboard.putNumber("Extend kG", extendConstants.getkF());
    SmartDashboard.putData(this);
  }

  public void toggleOverride() {
    override = !override;
  }

  public boolean isCloseToMinHardStop() {
    return (getExtendPosition() < kMinElevatorHeight + Units.inchesToMeters(3));
  }

  public boolean isCloseToMaxHardStop() {
    return (getExtendPosition() > kMaxElevatorHeight - Units.inchesToMeters(4));
  }

  public boolean closeToTarget() {
    return Math.abs(getExtendPosition() - this.currentExtendTarget) < 0.15 && usingTarget;
  }

  public double getPowerFactor(double pow) {
    double powerFactor = 1;
    double sign = Math.signum(pow);
    boolean closeToTarget = closeToTarget();
    boolean atTarget = isExtendAtTarget() && usingTarget;
    if (sign > 0) {

      if (atMaxHardStop(pow) || atTarget) {
        powerFactor = 0;
      } else if (isCloseToMaxHardStop() || closeToTarget) {
        powerFactor = 0.5;
      }

    } else {

      if (atMinHardStop(pow) || atTarget) {
        powerFactor = 0;
      } else if (isCloseToMinHardStop() || closeToTarget) {
        powerFactor = 0.5;
      }

    }
    return powerFactor;
  }

  public void initializeRunToTarget(double position) {
    usingTarget = true;
    runExtendToTarget(position);
  }

  public void runExtendToTarget(double position) {
    if (usingTarget) {
      this.currentExtendTarget = position;
      double kP = values.getDouble(extendKp);
      double kF = getFeedFowardVoltage();
      double error = position - getExtendPosition();
      double pow = kP * error;
      if (RobotBase.isReal()) {
        extendMotor.setVoltage(((pow) * getPowerFactor(pow)) + kF);
      } else {
        extendMotor.set((((pow) * getPowerFactor(pow)) + kF) / 12.0);
      }
    }
  }

  public double getFeedFowardVoltage() {
    if (RobotBase.isReal()) {
      return Math.sin(Units.degreesToRadians(pivotAngle.get())) * (.8);
    } else {
      return 0;
    }
  }

  public double getExtendPosition() {
    if (Robot.isReal()) {
      return extendEncoder.getPosition();
    } else {
      return extendSimEncoder.getPosition();
    }
  }

  public void setExtendEncoderPosition(double pos) {
    extendEncoder.setPosition(pos);
    extendSimEncoder.setPosition(pos);
  }

  public boolean isExtendAtTarget() {
    return !usingTarget || (Math.abs(getExtendPosition() - this.currentExtendTarget) < 0.0125 && usingTarget);
  }

  public double getExtendTarget() {
    return this.currentExtendTarget;
  }

  public boolean isElevatorIn() {
    if (RobotBase.isReal()) {
      return !extendHallEffect.get();
    } else {
      return getExtendPosition() < kMinElevatorHeight;
    }
  }

  public static ElevatorLevel getElevatorTargetLevel() {
    return targetLevel;
  }

  public static void setElevatorTargetLevel(ElevatorLevel level) {
    targetLevel = level;
    targetPos2d.setLength(targetLevel.getExtensionPosition());
    targetPos2d.setAngle(targetLevel.getPivotPosition());
  }

  public void extendPow(double pow) {
    if (!usingTarget || (usingTarget && pow != 0)) {
      usingTarget = false;
      double kF = getFeedFowardVoltage() / 12.0;
      SmartDashboard.putNumber("Extend Pow FF", kF);
      double powerFactor = getPowerFactor(pow);
      extendMotor.set((pow * powerFactor) + kF);
    }
    SmartDashboard.putNumber("Extend Pow Given", pow);
  }

  public boolean atMinHardStop(double pow) {
    return isElevatorIn() && pow < 0; // && !override;
  }

  public boolean atMaxHardStop(double pow) {
    return getExtendPosition() > kMaxElevatorHeight && pow > 0;
  }

  public void stopExtend() {
    extendMotor.set(0);
  }

  public void stopAndHoldExtend() {
    usingTarget = false;
    if (RobotBase.isReal()) {
      extendMotor.setVoltage(getFeedFowardVoltage());
    } else {
      extendMotor.set(getFeedFowardVoltage() / 12.0);
    }
  }

  public double extendSafe(double power, PivotSubsystem pivotSubsys, double desiredPivot) {
    double signPower = Math.signum(power);
    double extendPower = power;
    if (signPower > 0) { // if extending
      if (getExtendPosition() <= insideTheBumpers) { // if behind the bumpers
        if (pivotSubsys.getPivotPosition() < aboveTheBumpers) { // if too low to extend
          pivotSubsys.runPivotToTarget(aboveTheBumpers); // raise the pivot
          if (getExtendPosition() > (insideTheBumpers - 0.05)) { // while pivoting, stop extending and allow pivot to
                                                                 // raise
            extendPower = 0;
          }
        }
      } else if (getExtendPosition() > outsideTheBumpers) { // if outside the bumpers
        if (pivotSubsys.getPivotPosition() != desiredPivot) { // if still pivoted up
          pivotSubsys.runPivotToTarget(desiredPivot); // pivot back down
        }
      } else if (signPower < 0) { // if retracting
        if (getExtendPosition() >= outsideTheBumpers) { // if outside the bumpers
          if (pivotSubsys.getPivotPosition() < aboveTheBumpers) { // if too low to retract
            pivotSubsys.runPivotToTarget(aboveTheBumpers); // raise the pivot
            if (getExtendPosition() < (outsideTheBumpers + 0.05)) { // while pivoting, stop retracting and allow pivot
                                                                    // to raise
              extendPower = 0;
            }
          }
        } else if (getExtendPosition() < insideTheBumpers) { // if inside the bumpers
          if (pivotSubsys.getPivotPosition() != desiredPivot) { // if still pivoted up
            pivotSubsys.runPivotToTarget(desiredPivot); // pivot back down
          }
        }
      }
    }
    return extendPower;
  }

  @Override
  public void periodic() {
    m_elevatorMech2d.setAngle(pivotAngle.get());
    m_elevatorMech2d.setLength(getExtendPosition());
    if (Robot.isReal()) {
      if (isElevatorIn()) {
        setExtendEncoderPosition(kMinElevatorHeight);
      }
    }

    SmartDashboard.putNumber("Extend Percent", extendMotor.get());
    SmartDashboard.putNumber("Extend Current", ((CANSparkMax) extendMotor).getOutputCurrent());
    SmartDashboard.putBoolean("Extend In", isElevatorIn());
    SmartDashboard.putBoolean("Extend UsingTgt", usingTarget);
    SmartDashboard.putBoolean("Extend At Target", isExtendAtTarget());

    SmartDashboard.putNumber("Extend Position", getExtendPosition());

    SmartDashboard.putNumber("Extend Tgt Level", targetLevel.getExtensionPosition());

    SmartDashboard.putNumber("Pivot Tgt Level", targetLevel.getPivotPosition());
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    double motorPower = extendMotor.get() - feedForward;
    double simFF = feedForward / (0.000001 + Math.abs(Math.sin(Units.degreesToRadians(pivotAngle.get()))));
    extendSim.setInput(RobotController.getBatteryVoltage() * (motorPower + simFF));

    // Next, we update it. The standard loop time is 20ms.
    extendSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    extendSimEncoder.setPosition(extendSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(extendSim.getCurrentDrawAmps()));

    SmartDashboard.putNumber("Extend Sim Position", extendSim.getPositionMeters());
  }
}