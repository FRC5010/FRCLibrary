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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class ElevatorSubsystem extends SubsystemBase {
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
  private MechanismLigament2d targetPos2d;

  private double currentExtendTarget;

  private boolean usingTarget = false;
  private boolean override = false;
  private ElevatorLevel currentLevel = ElevatorLevel.ground; // Unsure of whether this should be stored in subsystem

  // TODO Implement ElevatorFeefForward
  private ElevatorFeedforward extendFeedforward;
  private ElevatorSim extendSim;

  private Supplier<Double> pivotAngle;

  public ElevatorSubsystem(MotorController5010 extend, GenericPID extendPID,
      MotorModelConstants extendConstants,
      Mechanism2d mech2d, int extendHallEffectPort, int extendMaxHallEffect, Supplier<Double> pivotAngle) {

    this.currentExtendTarget = 0;

    this.extendMotor = extend;
    this.extendMotor.setInverted(true);
    this.extendController = ((CANSparkMax) extend).getPIDController();
    this.extendEncoder = ((CANSparkMax) extend).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    this.extendEncoder.setPositionConversionFactor(kElevatorEncoderDistPerPulse);
    this.extendPID = extendPID;
    this.extendConstants = extendConstants;

    this.extendEncoder.setPosition(ElevatorLevel.auto.getExtensionPosition());

    this.m_mech2d = mech2d;
    m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 5, 20);
    m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Elevator", Units.metersToInches(kMinElevatorHeight), -30.0, 6, new Color8Bit(Color.kOrange)));
    targetPos2d = m_mech2dRoot.append(
        new MechanismLigament2d("Target", Units.metersToInches(kMinElevatorHeight), -30, 6,
            new Color8Bit(Color.kBlue)));

    extendSim = new ElevatorSim(DCMotor.getNEO(1), 25,
        kCarriageMass, kElevatorDrumRadius, kMinElevatorHeight, kMaxElevatorHeight, false);

    extendFeedforward = new ElevatorFeedforward(extendConstants.getkS(), extendConstants.getkV(),
        extendConstants.getkA());

    extendController.setP(extendPID.getkP());
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

    SmartDashboard.putNumber("Extend P", extendPID.getkP());

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
    boolean atTarget = isExtendAtTarget();
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

  public void runExtendToTarget(double position) {
    this.currentExtendTarget = position;
    usingTarget = true;
    double kP = SmartDashboard.getNumber("Extend P", extendPID.getkP());
    SmartDashboard.putBoolean("Is Close To Min Stop", isCloseToMinHardStop());
    double kF = getFeedFowardVoltage();
    double signum = Math.signum(position - getExtendPosition());
    // double pow = signum < 0 ? kP / 2.0 : kP * + kF;
    double pow = kP * signum;
    extendMotor.setVoltage(((pow) * getPowerFactor(pow)) + kF);
  }

  // public void runExtendToTarget(double position) {
  // this.currentExtendTarget = position;
  // SmartDashboard.putNumber("Extend Target", currentExtendTarget);
  // targetPos2d.setLength(currentExtendTarget);
  // if (Robot.isReal()) {
  // extendController.setFF(getFeedFowardVoltage() / currentExtendTarget);
  // extendController.setReference(this.currentExtendTarget,
  // CANSparkMax.ControlType.kPosition, 0);
  // SmartDashboard.putNumber("Extend FF", getFeedFowardVoltage());
  // } else {
  // extendPow((currentExtendTarget - getExtendPosition()) / kMaxElevatorHeight);
  // }
  // }

  public double getFeedFowardVoltage() {
    return Math.sin(Units.degreesToRadians(pivotAngle.get())) * (.8);
  }

  public double getExtendPosition() {
    if (Robot.isReal()) {
      return extendEncoder.getPosition();
    } else {
      return extendSimEncoder.getPosition();
    }
  }

  public void setExtendEncoderPosition(double pos) {
    this.extendEncoder.setPosition(pos);
  }

  public boolean isExtendAtTarget() {
    return Math.abs(getExtendPosition() - this.currentExtendTarget) < 0.0125 && usingTarget;
  }

  public double getExtendTarget() {
    return this.currentExtendTarget;
  }

  public boolean isElevatorIn() {
    return !extendHallEffect.get();
  }

  public ElevatorLevel getElevatorLevel() {
    return this.currentLevel;
  }

  public void setElevatorLevel(ElevatorLevel level) {
    this.currentLevel = level;
    targetPos2d.setLength(currentLevel.getExtensionPosition());
    targetPos2d.setAngle(currentLevel.getPivotPosition());
  }

  public void extendPow(double pow) {
    usingTarget = false;
    SmartDashboard.putBoolean("Is Close To Min Stop", isCloseToMinHardStop());
    extendMotor.set((pow + ((pow == 0) ? (getFeedFowardVoltage() / 12) : 0)) * getPowerFactor(pow));

    SmartDashboard.putNumber("Elevate Power", extendMotor.get());
    SmartDashboard.putNumber("Elevate Current", ((CANSparkMax) extendMotor).getOutputCurrent());
    SmartDashboard.putNumber("Extend Position", extendEncoder.getPosition());
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
    extendMotor.setVoltage(getFeedFowardVoltage());
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
    if (Robot.isReal()) {
      m_elevatorMech2d.setLength(Units.metersToInches(getExtendPosition()));

      if (isElevatorIn()) {
        setExtendEncoderPosition(kMinElevatorHeight);
      }
    }

    SmartDashboard.putBoolean("Elevator In: ", isElevatorIn());

    SmartDashboard.putNumber("Elevator Position: ", getExtendPosition());
    // SmartDashboard.putNumber("Abs", KFF);

    SmartDashboard.putNumber("Elevator Level", currentLevel.getExtensionPosition());

    SmartDashboard.putNumber("Pivot Level", currentLevel.getPivotPosition());
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    extendSim.setInput(extendMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    extendSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    extendSimEncoder.setPosition(extendSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(extendSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    m_elevatorMech2d.setLength(Units.metersToInches(extendSim.getPositionMeters()));
  }
}