// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.sensors.ValueSwitch;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.sensors.gyro.GenericGyro;

public class ClimbSubsystem extends GenericSubsystem {
  /** Creates a new ClimbSubsystem. */
  MotorController5010 leftMotor;
  MotorController5010 rightMotor;
  GenericEncoder leftEncoder;
  GenericEncoder rightEncoder;
  GenericGyro gyro;

  // Current Switches
  ValueSwitch leftCurrentSwitch;
  ValueSwitch rightCurrentSwitch;

  // Simulation Components
  ElevatorSim simLeft;
  Mechanism2d robotSim;
  MechanismRoot2d simLeftClimbRoot;
  MechanismLigament2d simLeftClimbArm;

  ElevatorSim simRight;
  MechanismRoot2d simRightClimbRoot;
  MechanismLigament2d simRightClimbArm;

  SimulatedEncoder leftSimEncoder = new SimulatedEncoder(12, 13);
  SimulatedEncoder rightSimEncoder = new SimulatedEncoder(14, 15);

  private final double MAX_POSITION_DEVIANCE = 3.0;

  // NetworkTable names

  private final String MAX_EXTENSION = "Max Extension";
  private final double MAX_EXTENSION_DEFAULT = 195.0;
  private final String SAFE_LIMIT = "Safe Limit";
  private final double SAFE_LIMIT_DEFAULT = 75.0;
  private final String CURRENT_THRESHOLD = "Climb Current Threshold";
  private final double CURRENT_THRESHOLD_DEFAULT = 40.0;
  private final String LEFT_CONVERSION_FACTOR = "Left Climb Conversion Factor";
  private final double LEFT_CONVERSION_DEFAULT = 1.0;
  private final String RIGHT_CONVERSION_FACTOR = "Right Climb Conversion Factor";
  private final double RIGHT_CONVERSION_DEFAULT = 1.0;
  private final String LEFT_CLIMB_CURRENT_SWITCH = "Left Climb Current Switch";
  private final String RIGHT_CLIMB_CURRENT_SWITCH = "Right Climb Current Switch";
  private final String LEFT_CLIMB_POSITION = "Left Climb Position";
  private final String RIGHT_CLIMB_POSITION = "Right Climb Position";
  private final String AUTO_BALANCE = "Auto Climb Balancing";
  private final String ENABLE_CLIMB = "Enable Climb";
  private final String LEFT_CURRENT = "Left Current";
  private final String RIGHT_CURRENT = "Right Current";

  private final double CLIMB_GEARING = 1;
  private final double CLIMB_WEIGHT_LB = 1.0;
  private final double CLIMB_DRUM_RADIUS = 0.05;
  private final double CLIMB_MIN_HEIGHT = 0.0;
  private final double CLIMB_MAX_HEIGHT = 200.0;
  private final Color8Bit CLIMB_SIM_COLOR = new Color8Bit(Color.kPurple);
  private boolean override = false;

  public ClimbSubsystem(MotorController5010 leftMotor, MotorController5010 rightMotor, GenericGyro gyro,
      Mechanism2d mechSim) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.gyro = gyro;

    values.declare(MAX_EXTENSION, MAX_EXTENSION_DEFAULT);
    values.declare(CURRENT_THRESHOLD, CURRENT_THRESHOLD_DEFAULT);
    values.declare(LEFT_CONVERSION_FACTOR, LEFT_CONVERSION_DEFAULT);
    values.declare(RIGHT_CONVERSION_FACTOR, RIGHT_CONVERSION_DEFAULT);
    values.declare(LEFT_CLIMB_CURRENT_SWITCH, false);
    values.declare(RIGHT_CLIMB_CURRENT_SWITCH, false);
    values.declare(AUTO_BALANCE, false);
    values.declare(ENABLE_CLIMB, false);
    values.declare(SAFE_LIMIT, SAFE_LIMIT_DEFAULT);
    values.declare(LEFT_CURRENT, ((CANSparkMax) leftMotor).getOutputCurrent());
    values.declare(RIGHT_CURRENT, ((CANSparkMax) rightMotor).getOutputCurrent());

    leftEncoder = leftMotor.getMotorEncoder();
    rightEncoder = rightMotor.getMotorEncoder();

    values.declare(LEFT_CLIMB_POSITION, getLeftPosition());
    values.declare(RIGHT_CLIMB_POSITION, getRightPosition());

    leftEncoder.setPositionConversion(values.getDouble(LEFT_CONVERSION_FACTOR));
    rightEncoder.setPositionConversion(values.getDouble(RIGHT_CONVERSION_FACTOR));
    leftSimEncoder.setPositionConversion(values.getDouble(LEFT_CONVERSION_FACTOR));
    rightSimEncoder.setPositionConversion(values.getDouble(RIGHT_CONVERSION_FACTOR));

    leftCurrentSwitch = new ValueSwitch(() -> values.getDouble(CURRENT_THRESHOLD),
        ((CANSparkMax) leftMotor)::getOutputCurrent, 0.1); // TODO: Get "desired" current and use it as threshold
    rightCurrentSwitch = new ValueSwitch(() -> values.getDouble(CURRENT_THRESHOLD),
        ((CANSparkMax) rightMotor)::getOutputCurrent, 0.1); // TODO: Get "desired" current and use it as threshold

    // TODO: Actually Implement SImulation
    // Simulation Setup
    robotSim = mechSim;

    simLeftClimbRoot = robotSim.getRoot("Left Climb Root", 0.60, 0.15);
    simLeftClimbArm = simLeftClimbRoot.append(
        new MechanismLigament2d("Left Climb Arm", 0.1, 90, 6, CLIMB_SIM_COLOR));
    simLeft = new ElevatorSim(DCMotor.getNEO(1), CLIMB_GEARING, CLIMB_WEIGHT_LB, CLIMB_DRUM_RADIUS, CLIMB_MIN_HEIGHT,
        CLIMB_MAX_HEIGHT, false, 0.0);

    simRightClimbRoot = robotSim.getRoot("Right Climb Root", 0.5, 0.15);
    simRightClimbArm = simRightClimbRoot.append(
        new MechanismLigament2d("Right Climb Arm", 0.1, 90, 6, CLIMB_SIM_COLOR));
    simRight = new ElevatorSim(DCMotor.getNEO(1), CLIMB_GEARING, CLIMB_WEIGHT_LB, CLIMB_DRUM_RADIUS, CLIMB_MIN_HEIGHT,
        CLIMB_MAX_HEIGHT, false, 0.0);

  }

  public void enableClimb(boolean enable) {
    values.set(ENABLE_CLIMB, enable);
  }

  public void enableAutoClimb(boolean enable) {
    values.set(AUTO_BALANCE, enable);
  }

  private double getMinPosition() {
    return values.getBoolean(ENABLE_CLIMB) ? values.getDouble(SAFE_LIMIT) : 0.0;
  }

  private double getMaxPosition() {
    return values.getBoolean(ENABLE_CLIMB) ? values.getDouble(MAX_EXTENSION) : values.getDouble(SAFE_LIMIT);
  }

  public boolean leftIsAtMin(double speed) {
    if ((speed < 0 && getLeftPosition() < getMinPosition()) && !override) {
      return true;
    }
    return false;
  }

  public boolean leftIsAtMax(double speed) {
    if ((speed > 0 && getLeftPosition() > getMaxPosition()) && !override) {
      return true;
    }
    return false;
  }

  public boolean rightIsAtMin(double speed) {
    if ((speed < 0 && getRightPosition() < getMinPosition()) && !override) {
      return true;
    }
    return false;
  }

  public boolean rightIsAtMax(double speed) {
    if ((speed > 0 && getRightPosition() > getMaxPosition()) && !override) {
      return true;
    }
    return false;
  }

  public void setAutobalanceMode(boolean state) {
    values.set(AUTO_BALANCE, state);
  }

  public boolean getAutobalanceMode() {
    return values.getBoolean(AUTO_BALANCE);
  }

  public void climbStateMachine(double left, double right) {
    if (values.getBoolean(AUTO_BALANCE)) {
      setBalancedSpeed(left);
    } else {
      setLeftMotorSpeed(left);
      setRightMotorSpeed(right);
    }
  }

  public void setLeftMotorSpeed(double speed) {
    leftMotor.set(leftIsAtMin(speed) || leftIsAtMax(speed) ? 0.0 : speed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void setOverride(boolean state) {
    override = state;
  }

  public void setRightMotorSpeed(double speed) {
    rightMotor.set(rightIsAtMax(speed) || rightIsAtMin(speed) ? 0.0 : speed);
  }

  public double getLeftPosition() {
    return Robot.isReal() ? leftEncoder.getPosition() : leftSimEncoder.getPosition();
  }

  public double getLeftVoltage() {
    return leftMotor.get() * RobotController.getBatteryVoltage();
  }

  public double getRightVoltage() {
    return rightMotor.get() * RobotController.getBatteryVoltage();
  }

  public double getRightPosition() {
    return Robot.isReal() ? rightEncoder.getPosition() : rightSimEncoder.getPosition();
  }

  public double getHorizontalTilt() {
    return gyro.getAngleY(); // TODO: Fix if necessary
  }

  public void setBalancedSpeed(double speed) {
    double robotTilt = getHorizontalTilt();
    double rightSpeed = -Math.sin(Units.degreesToRadians(robotTilt)) + speed;
    double leftSpeed = Math.sin(Units.degreesToRadians(robotTilt)) + speed;
    setLeftMotorSpeed(speed != 0 ? leftSpeed : 0);
    setRightMotorSpeed(speed != 0 ? rightSpeed : 0);
  }

  public boolean isAtZero() {
    return leftCurrentSwitch.get() && rightCurrentSwitch.get() && leftEncoder.getPosition() <= 0
        && rightEncoder.getPosition() <= 0;
  }

  public void zeroPosition() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  // Zeroes the encoder if current switch triggers and current encoder position is
  // close enough
  private void updateEncoderMinMax() {
    if (!values.getBoolean(ENABLE_CLIMB)) {
      if (leftCurrentSwitch.get() && leftMotor.get() < 0) {
        leftEncoder.setPosition(0);
      }

      if (rightCurrentSwitch.get() && rightMotor.get() < 0) {
        rightEncoder.setPosition(0);
      }
    }

    values.set(LEFT_CURRENT, ((CANSparkMax) leftMotor).getOutputCurrent());
    values.set(RIGHT_CURRENT, ((CANSparkMax) rightMotor).getOutputCurrent());
  }

  @Override
  public void periodic() {
    values.set(LEFT_CLIMB_CURRENT_SWITCH, leftCurrentSwitch.get());
    values.set(RIGHT_CLIMB_CURRENT_SWITCH, rightCurrentSwitch.get());
    values.set(LEFT_CLIMB_POSITION, getLeftPosition());
    values.set(RIGHT_CLIMB_POSITION, getRightPosition());

    updateEncoderMinMax();

    simLeftClimbArm.setLength(0.5 * getLeftPosition() / CLIMB_MAX_HEIGHT + 0.05);
    simRightClimbArm.setLength(0.5 * getRightPosition() / CLIMB_MAX_HEIGHT + 0.05);
  }

  @Override
  public void simulationPeriodic() {
    simLeft.setInput(getLeftVoltage());
    simRight.setInput(getRightVoltage());

    simLeft.update(0.020);
    simRight.update(0.020);

    leftSimEncoder.setPosition(simLeft.getPositionMeters());
    rightSimEncoder.setPosition(simRight.getPositionMeters());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simLeft.getCurrentDrawAmps(), simRight.getCurrentDrawAmps()));

  }
}
