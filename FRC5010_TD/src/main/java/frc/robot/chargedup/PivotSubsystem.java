// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private final double pivotConversionFactor = 360;
  private final double pivotMaxLimit = 150;
  private final double pivotMinLimit = 0;
  private MotorController5010 pivotMotor;
  private SparkMaxPIDController pivotController;
  private MotorModelConstants pivotConstants;
  private GenericPID pivotPID;
  private AbsoluteEncoder pivotEncoder;
  private SimulatedEncoder pivotSimEncoder = new SimulatedEncoder(10, 11);
  private ArmFeedforward pivotFeedforward;
  private SingleJointedArmSim pivotSim;

  private boolean override = false;
  private boolean usingTarget = false;

  private double currentPivotTarget;
  private MechanismLigament2d pivotArmSim;
  private MechanismRoot2d mech2dRoot;
  ShuffleboardTab shuffleTab;

  public PivotSubsystem(MotorController5010 pivot, GenericPID pivotPID, MotorModelConstants liftConstants,
      Mechanism2d mech2d, ShuffleboardTab shuffleTab) {
    this.currentPivotTarget = 0;

    this.pivotMotor = pivot;
    this.pivotMotor.setInverted(false);
    this.pivotController = ((CANSparkMax) pivot).getPIDController();
    this.pivotEncoder = ((CANSparkMax) pivot).getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(this.pivotConversionFactor);
    this.pivotEncoder.setInverted(true);
    this.pivotPID = pivotPID;
    this.pivotConstants = liftConstants;
    this.shuffleTab = shuffleTab;

    SmartDashboard.putNumber("MOI", SingleJointedArmSim.estimateMOI(0.82, 8.5));
    pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75,
        SingleJointedArmSim.estimateMOI(0.82, 8.5), 0.82, Units.degreesToRadians(0),
        Units.degreesToRadians(pivotMaxLimit), true);

    pivotFeedforward = new ArmFeedforward(pivotConstants.getkS(), pivotConstants.getkF(), pivotConstants.getkV());

    pivotController.setP(pivotPID.getkP());
    pivotController.setI(pivotPID.getkI());
    pivotController.setD(pivotPID.getkD());
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setOutputRange(-1, 1);
    pivotController.setFF(0);

    shuffleTab.addDouble("Pivot FF", this::getFeedFowardVoltage);
    if (Robot.isReal()) {

      shuffleTab.addBoolean("Pivot Max", this::isPivotMaxPosition);
      shuffleTab.addBoolean("Pivot Target is good", () -> currentPivotTarget > pivotEncoder.getPosition());
    }
    shuffleTab.addNumber("Pivot Motor Pow: ", () -> pivotMotor.get());
    shuffleTab.addDouble("Pivot Position: ", this::getPivotPosition);
    shuffleTab.addBoolean("Override", () -> override);

    SmartDashboard.putNumber("Pivot kG", pivotConstants.getkF());
    SmartDashboard.putNumber("Pivot kP", pivotPID.getkP());

    mech2dRoot = mech2d.getRoot("Intake Root", 10, 10);
    pivotArmSim = mech2dRoot.append(
        new MechanismLigament2d(
            "Pivot Arm", Units.metersToInches(0.75), 0));
  }

  public void setPivotEncoderPosition(double pos) {
    pivotArmSim.setAngle(pos);
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
    return (getPivotPosition() < pivotMinLimit + 5);
  }

  public boolean isCloseToMaxHardStop() {
    return (getPivotPosition() > pivotMaxLimit - 5);
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
    return pow < 0 && pivotEncoder.getPosition() < 0;
  }

  private boolean isPivotMaxHardStop(double pow) {
    return isPivotMaxPosition() && pow > 0;
  }

  public void runPivotToTarget(double position) {
    this.currentPivotTarget = position;
    usingTarget = true;
    double kP = SmartDashboard.getNumber("Pivot kP", pivotPID.getkP());
    double kF = getFeedFowardVoltage();
    double positionSig = Math.signum(this.currentPivotTarget - getPivotPosition());
    double pow = kP * positionSig;
    pivotMotor.setVoltage((pow * getPowerFactor(pow)) + kF);
  }

  public double getFeedFowardVoltage() {
    Double pivotKG = SmartDashboard.getNumber("Pivot kG", pivotConstants.getkF());
    pivotFeedforward = new ArmFeedforward(pivotConstants.getkS(), pivotKG, 0);
    double ff = pivotFeedforward.calculate(Units.degreesToRadians(getPivotPosition()), 0);
    SmartDashboard.putNumber("Pivot FF", ff);
    return 0;

    // (0.2 + 0.3 * 1 // replace this with the arm length
    // ((extendPos.get() - ElevatorSubsystem.kMinElevatorHeight)
    // / (ElevatorSubsystem.kMaxElevatorHeight -
    // ElevatorSubsystem.kMinElevatorHeight)))
    // * Math.cos(Units.degreesToRadians(getPivotPosition())));
  }

  public boolean isPivotAtTarget() {
    return Math.abs(getPivotPosition() - this.currentPivotTarget) < 0.05 && usingTarget;
  }

  public double getPivotTarget() {
    return this.currentPivotTarget;
  }

  public boolean isPivotMaxPosition() {
    if (override) {
      return false;
    }
    return getPivotPosition() > pivotMaxLimit;
  }

  public boolean isPivotMinPosition() {
    if (override) {
      return false;
    }
    return getPivotPosition() > pivotMinLimit;
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

    double powerFactor = getPowerFactor(pow);
    SmartDashboard.putNumber("Pivot Power Factor", powerFactor);
    double pivotPow = (pow * powerFactor + ((pow == 0 && feedForward) ? (getFeedFowardVoltage() / 12) : 0));
    SmartDashboard.putNumber("pivotPow", pivotPow);

    pivotMotor.set(pivotPow);
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
    return pivotEncoder.getVelocity() < 0 && pivotEncoder.getPosition() < 0;
  }

  @Override
  public void periodic() {
    pivotArmSim.setAngle(getPivotPosition());
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
