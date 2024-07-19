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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.hardware.MotorModelConstants;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  public static final double pivotOffset = ElevatorLevel.ground.getPivotPosition(); // -14.04;
  private final double pivotConversionFactor = 24.242;
  private final double pivotMaxLimit = 33.5;
  private final double pivotMinLimit = ElevatorLevel.ground.getPivotPosition();
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
      int pivotHallEffectPort, int pivotMaxHallEffectPort, Supplier<Double> extendPos, Mechanism2d mech2d) {
    this.currentPivotTarget = 0;

    this.pivotMotor = pivot;
    this.pivotMotor.setInverted(false);
    this.pivotController = ((CANSparkMax) pivot).getPIDController();
    this.pivotEncoder = ((CANSparkMax) pivot).getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    pivotEncoder.setPositionConversionFactor(this.pivotConversionFactor);
    pivotSimEncoder.setPositionConversion(0.1);
    this.pivotEncoder.setInverted(true);
    this.pivotPID = pivotPID;
    this.pivotConstants = liftConstants;
    SmartDashboard.putNumber("Pivot kG", pivotConstants.getkF());
    pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 125,
        SingleJointedArmSim.estimateMOI(2, 24), 2, Units.degreesToRadians(pivotMinLimit),
        Units.degreesToRadians(pivotMaxLimit), true, 0);
    SmartDashboard.putNumber("MOI", SingleJointedArmSim.estimateMOI(2, 19));
    // From CC Code
    // pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75,
    // SingleJointedArmSim.estimateMOI(0.82, 8.5), 0.82,
    // Units.degreesToRadians(pivotMinLimit),
    // Units.degreesToRadians(pivotMaxLimit), true);

    pivotFeedforward = new ArmFeedforward(liftConstants.getkS(), liftConstants.getkF(), liftConstants.getkV());

    pivotController.setP(pivotPID.getkP());
    pivotController.setI(pivotPID.getkI());
    pivotController.setD(pivotPID.getkD());
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setOutputRange(-1, 1);

    pivotEncoder.setPosition(ElevatorLevel.auto.getPivotPosition());
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

  public boolean isCloseToMinHardStop() {
    return (getPivotPosition() < (pivotMinLimit + 5));
  }

  public boolean isCloseToMaxHardStop() {
    return (getPivotPosition() > (pivotMaxLimit - 5));
  }

  public boolean closeToTarget() {
    return Math.abs(getPivotPosition() - currentPivotTarget) < 1 && usingTarget;
  }

  public double getPowerFactor(double pow) {
    double powerFactor = 1;
    double sign = Math.signum(pow);
    boolean closeToTarget = closeToTarget();
    boolean atTarget = isPivotAtTarget() && usingTarget;

    if (sign > 0) {
      if (isPivotMaxHardStop(pow) || atTarget) {
        powerFactor = 0;
      } else if (isCloseToMaxHardStop()) {
        powerFactor = 0.25;
      }

    } else {
      if (isPivotMinHardStop(pow) || atTarget) {
        powerFactor = 0;
      } else if (isCloseToMinHardStop() || closeToTarget) {
        powerFactor = 0.25;
      }

    }
    SmartDashboard.putNumber("Pivot Power Factor", powerFactor);
    return powerFactor;
  }

  private boolean isPivotMinHardStop(double pow) {
    return isPivotMinHallEffect() && pow < 0;
  }

  private boolean isPivotMaxHardStop(double pow) {
    return isPivotMaxPosition() && pow > 0;
  }

  public void initializeRunToTarget(double position) {
    usingTarget = true;
    runPivotToTarget(position);
  }

  public void runPivotToTarget(double position) {
    if (usingTarget) {
      this.currentPivotTarget = position;
      double kP = SmartDashboard.getNumber("Pivot P", pivotPID.getkP());
      double kF = getFeedFowardVoltage();
      double error = this.currentPivotTarget - getPivotPosition();
      double pow = kP * error;
      if (RobotBase.isReal()) {
        pivotMotor.setVoltage((pow * getPowerFactor(pow)) + kF);
      } else {
        pivotMotor.set(((pow * getPowerFactor(pow)) + kF) / 12.0);
      }
    }
  }

  public double getFeedFowardVoltage() {
    if (RobotBase.isReal()) {
      return (0.1 + 0.3 *
          ((extendPos.get() - ElevatorSubsystem.kMinElevatorHeight)
              / (ElevatorSubsystem.kMaxElevatorHeight -
                  ElevatorSubsystem.kMinElevatorHeight)))
          * Math.cos(Units.degreesToRadians(getPivotPosition()));
    } else {
      // From CC code
      Double pivotKG = SmartDashboard.getNumber("Pivot kG",
          pivotConstants.getkF());
      pivotFeedforward = new ArmFeedforward(pivotConstants.getkS(), pivotKG, 0);
      double ff = pivotFeedforward.calculate(Units.degreesToRadians(getPivotPosition()), 0);
      SmartDashboard.putNumber("Pivot calc FF", ff);
      return ff;
    }
  }

  public void pivotPow(double pow, boolean feedForward) {
    if (!usingTarget || (usingTarget && pow != 0)) {
      usingTarget = false;

      double powerFactor = getPowerFactor(pow);
      SmartDashboard.putNumber("Pivot Power Factor", powerFactor);
      double ff = getFeedFowardVoltage() / 12.0;
      SmartDashboard.putNumber("Pivot Power FF", ff);

      pivotMotor.set((pow * powerFactor) + ff);
    }
    SmartDashboard.putNumber("Pivot Power Given", pow);
  }

  public void stopPivot() {
    pivotMotor.set(0);
  }

  public boolean isPivotAtTarget() {
    return (Math.abs(getPivotPosition() - this.currentPivotTarget) < 1 && usingTarget) || !usingTarget;
  }

  public double getPivotTarget() {
    return this.currentPivotTarget;
  }

  public boolean isPivotMinHallEffect() {
    if (override) {
      return false;
    }
    if (RobotBase.isReal()) {
      return !pivotHallEffect.get(); // && !override;
    } else {
      return getPivotPosition() < pivotMinLimit;
    }
  }

  public boolean isPivotMaxPosition() {
    if (override) {
      return false;
    }
    return getPivotPosition() > pivotMaxLimit;
  }

  public void toggleOverride() {
    override = !override;
  }

  public double getVelocity() {
    if (RobotBase.isReal()) {
      return pivotEncoder.getVelocity();
    } else {
      return pivotSimEncoder.getVelocity();
    }
  }

  public void stopAndHoldPivot() {
    usingTarget = false;
    if (RobotBase.isReal()) {
      pivotMotor.setVoltage(getFeedFowardVoltage());
    } else {
      pivotMotor.set(getFeedFowardVoltage() / 12.0);
    }
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
    }
    SmartDashboard.putBoolean("Pivot UsingTgt: ", usingTarget);
    SmartDashboard.putBoolean("Pivot At Target: ", isPivotAtTarget());
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
