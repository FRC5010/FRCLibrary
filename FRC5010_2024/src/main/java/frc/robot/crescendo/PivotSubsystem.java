// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class PivotSubsystem extends GenericSubsystem {
  MotorController5010 pivotMotor;
  AbsoluteEncoder encoder;
  ArmFeedforward pivotFeedforward;
  SparkPIDController pivotPID;
  SimpleMotorFeedforward feedforward;
  TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(0.5, 0.5));
  State trapState;
  Mechanism2d robotSim;
  MechanismRoot2d simPivotRoot;
  MechanismLigament2d simPivotArm;
  MechanismLigament2d simTargetArm;

  DigitalInput leftLimitSwitch;
  DigitalInput rightLimitSwitch;

  private InterpolatingDoubleTreeMap interpolationTree;
  

  SingleJointedArmSim pivotSim;
  SimulatedEncoder simEncoder = new SimulatedEncoder(10, 11);

  private final double PIVOT_LENGTH = Units.inchesToMeters(19);
  private final double PIVOT_MASS = Units.lbsToKilograms(22);

  private final double MIN_PIVOT_POSITION = -11; // Degrees
  private final double PIVOT_START_ANGLE = MIN_PIVOT_POSITION;
  private final double PIVOT_END_ANGLE = 100;
  private final String PIVOT_kS = "PivotkS";
  private final String PIVOT_kA = "PivotkA";
  private final double PIVOT_GEARING = (5.0 * 68.0 / 24.0) *  (80.0 / 24.0);
  private final double PIVOT_CONVERSION_FACTOR = 360.0;
  private final double PIVOT_SIM_CONVERSION_FACTOR = 0.1;
  private final String PIVOT_kG = "PivotKg";
  private final String PIVOT_ANGLE = "Pivot Angle";
  private final String PIVOT_kV = "PivotkV";
  private final String PIVOT_kP = "PivotkP";
  private final String PIVOT_kD = "PivotkD";
  private final String MICRO_ADJUST = "Pivot Micro Adjustment";
  private final String SLOWDOWN = "Slowdown";
  private static enum PivotState {
    JOYSTICK, POSITION
  }
  private PivotState pivotState = PivotState.POSITION;

  public final double HOME_LEVEL = MIN_PIVOT_POSITION;
  public final double AMP_LEVEL = 49;
  public final double TRAP_LEVEL = 75;
  public final double INTAKE_LEVEL = HOME_LEVEL; // TODO: Make accurate
  public final double PODIUM_SHOT = 12.3;
  
  private double referencePosition = HOME_LEVEL;

  private double previousError = 0;
  private double previousTime = 0;
  
  /** Creates a new Pivot. */
  public PivotSubsystem(MotorController5010 pivot, Mechanism2d mechSim) {

    values.declare(PIVOT_kG, RobotBase.isReal() ? 0.57 : 6.05 ); 
    values.declare(PIVOT_kV, RobotBase.isReal() ? 0.0 : 0.01);
    values.declare(PIVOT_kP, RobotBase.isReal() ? 0.00 : 0.01);
    values.declare(PIVOT_kD, RobotBase.isReal() ? 0.0000 : 0.003);
    values.declare(PIVOT_kS, RobotBase.isReal() ? 0.12 : 0.0);
    values.declare(PIVOT_kA, 0.0);
    values.declare(MICRO_ADJUST, 10.0);
    values.declare(SLOWDOWN, 0.1);

    interpolationTree = new InterpolatingDoubleTreeMap();

    pivotMotor = pivot;
    encoder = ((CANSparkMax)pivotMotor).getAbsoluteEncoder();
    
    encoder.setPositionConversionFactor(360);
    encoder.setInverted(true);
    

    pivotPID = ((CANSparkMax)pivotMotor).getPIDController();
    pivotPID.setFeedbackDevice(encoder);
    pivotPID.setPositionPIDWrappingEnabled(true);
    pivotPID.setPositionPIDWrappingMinInput(0);
    pivotPID.setPositionPIDWrappingMaxInput(360);

    pivotPID.setP(0.0);
    pivotPID.setI(0);
    pivotPID.setD(0);
    trapState = new State(getPivotPosition(), 0);
    robotSim = mechSim;

    leftLimitSwitch = new DigitalInput(0);
    // TODO: ADD RIGHT LIMIT ONCE ACTUALLY WIRED

    

    // Simulation Setup
    simEncoder.setPositionConversion(PIVOT_SIM_CONVERSION_FACTOR);
    simEncoder.setInverted(true);
    simEncoder.setPosition(PIVOT_START_ANGLE);
    simPivotRoot = robotSim.getRoot("Pivot Root", 0.70, 0.50);
    simPivotArm = simPivotRoot.append(new MechanismLigament2d("Arm", 0.4, 180 - PIVOT_START_ANGLE, 6, new Color8Bit(Color.kOrange)) );
  //  simPivotArm.append(new MechanismLigament2d("Shooter", 0.40, 90, 6, new Color8Bit(Color.kOrange)));
    // TODO: Attach Shooter to the Arm 
    simTargetArm = simPivotRoot.append(new MechanismLigament2d("Target Arm", 0.4, 180 - PIVOT_START_ANGLE, 6, new Color8Bit(Color.kBlue)));
//    simTargetArm.append(new MechanismLigament2d("Target Shooter", 0.40, 90, 6, new Color8Bit(Color.kBlue)));
    pivotSim = new SingleJointedArmSim(DCMotor.getNEO(2), 9 , 
      SingleJointedArmSim.estimateMOI(PIVOT_LENGTH, PIVOT_MASS), PIVOT_LENGTH, 
      Units.degreesToRadians(PIVOT_START_ANGLE), Units.degreesToRadians(PIVOT_END_ANGLE), 
      true, Units.degreesToRadians(PIVOT_START_ANGLE));
    values.declare(PIVOT_ANGLE, PIVOT_START_ANGLE);
  }

  public double getReference() {
    return referencePosition;
  }

  public double getSlowdown() {
    return values.getDouble(SLOWDOWN);
  }

  public void setSlowdown(double factor) {
    values.set(SLOWDOWN, factor);
  }

  public void setReference(double value) {
    referencePosition = value;
    pivotState = PivotState.POSITION;
    simTargetArm.setAngle(180 - value);
  }

  public boolean isAtMin() {
    return (!leftLimitSwitch.get() || !rightLimitSwitch.get() || getPivotPosition() <= PIVOT_START_ANGLE);
  }

  public boolean isAtMax() {
    return getPivotPosition() >= PIVOT_END_ANGLE;
  }

  public boolean isAtTarget() {
    return Math.abs(getReference() - getPivotPosition()) < 0.5;
  }

  public void setInterpolatedShotAngle(double distance) {
    double angle = interpolationTree.get(distance);
    setReference(angle);
  }

  public void runToReference() {
    double feedForward = getFeedFowardVoltage(isAtTarget() ? 0 : getReference() -  getPivotPosition());
    SmartDashboard.putNumber("Pivot FF Voltage", feedForward);
    
    if (!Robot.isReal()) {
      double currentError = getReference() - getPivotPosition();
      double currentTime = RobotController.getFPGATime() / 1E6;
      double errorRate = (currentError - previousError) / (currentTime - previousTime);
      double voltage = currentError * values.getDouble(PIVOT_kP) + (errorRate * values.getDouble(PIVOT_kD));
      pivotMotor.set((feedForward / RobotController.getBatteryVoltage()) + voltage);
      previousError = currentError;
      previousTime = currentTime;
    } else {
      pivotPID.setP(values.getDouble(PIVOT_kP));
      pivotPID.setReference(referencePosition, CANSparkBase.ControlType.kPosition, 0, feedForward, ArbFFUnits.kVoltage);
    }
    SmartDashboard.putBoolean("Pivot Run Ref", true);
    SmartDashboard.putBoolean("Pivot Set Speed", false);
  }

  public double getPivotPosition() {
    if (Robot.isReal()) {
      return encoder.getPosition() > 180 ? encoder.getPosition() - 360 : encoder.getPosition();
    } else {
      return simEncoder.getPosition();
    }
  }

  public void setPivotPosition(double pos) {
    if (Robot.isReal()) {
    } else {
      simEncoder.setPosition(pos);
      pivotSim.setState(Units.degreesToRadians(pos), 0);
    }
    values.set(PIVOT_ANGLE, pos);
  }

  public Command getSysIdCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.angleSysIdRoutine(pivotMotor, pivotMotor.getMotorEncoder(), "Pivot Motor", this), 5, 3, 3);
  }

  public double getFeedFowardVoltage(double error) {
    Double pivotKG = values.getDouble(PIVOT_kG);
    Double pivotKV = values.getDouble(PIVOT_kV);
    Double pivotKS = values.getDouble(PIVOT_kS);
    Double pivotKA = values.getDouble(PIVOT_kA);

    pivotFeedforward = new ArmFeedforward(pivotKS, pivotKG, pivotKV, pivotKA);
    double vel = Math.signum(error) * (Math.abs(error) > 5 ? 1 : 0.25) * 2;
    double ff = pivotFeedforward.calculate(Units.degreesToRadians(getPivotPosition()), Units.degreesToRadians(vel));

    return ff;
  }

  public Command adjustReferenceUp() {
    return Commands.runOnce(() -> setReference(getReference()+values.getDouble(MICRO_ADJUST)), this);
  }
  public Command adjustReferenceDown() {
    return Commands.runOnce(() -> setReference(getReference()-values.getDouble(MICRO_ADJUST)), this);
  }

  public void setSpeed(double speed) {
    double ff = getFeedFowardVoltage(0) / RobotController.getBatteryVoltage();
    pivotMotor.set(speed * values.getDouble(SLOWDOWN) + ff);
    SmartDashboard.putBoolean("Pivot Set Speed", true);
    SmartDashboard.putBoolean("Pivot Run Ref", false);
    SmartDashboard.putNumber("Pivot Speed", speed);
  }

  public void stateMachine(double joystickInput) {
    switch(pivotState) {
      case POSITION: {
        if(joystickInput != 0) {
          pivotState = PivotState.JOYSTICK;
          setSpeed(joystickInput);
        } else {
          runToReference();
        }
      }
      break;
      case JOYSTICK: {
        if (joystickInput == 0) {
          setReference(getPivotPosition());
          runToReference();
        } else {
          setSpeed(joystickInput);
        }
      }
      break;
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    values.set(PIVOT_ANGLE, getPivotPosition());
    SmartDashboard.putNumber("Pivot Reference", getReference());
    SmartDashboard.putNumber("Pivot Reference Rotation", Units.degreesToRotations(getReference()));
    SmartDashboard.putNumber("Pivot Motor", ((CANSparkMax)pivotMotor).getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Pivot Encoder", encoder.getPosition());
    SmartDashboard.putBoolean("Pivot At Target", isAtTarget());
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
    simPivotArm.setAngle(180 - simEncoder.getPosition());
    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    SmartDashboard.putNumber("Pivot Sim Rotation", Units.radiansToDegrees(pivotSim.getAngleRads()));
  }
}
