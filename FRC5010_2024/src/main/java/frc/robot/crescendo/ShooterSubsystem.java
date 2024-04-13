// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.FRC5010.arch.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.PIDController5010;
import frc.robot.FRC5010.motors.PIDController5010.PIDControlType;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;
import frc.robot.RobotContainer.LogLevel;

public class ShooterSubsystem extends GenericSubsystem {

  private MechanismLigament2d topMotorSim;
  private MechanismLigament2d bottomMotorSim;

  private SimulatedEncoder topSimEncoder = new SimulatedEncoder(18, 19);
  private SimulatedEncoder bottomSimEncoder = new SimulatedEncoder(20, 21);

  private GenericEncoder topEncoder;
  private GenericEncoder bottomEncoder;

  private InterpolatingDoubleTreeMap interpolationTree;

  private double shooterPrevTime = 0.0;
  private double topShooterPrevError = 0.0;
  private double bottomShooterPrevError = 0.0;

  private PIDController5010 topPID;
  private PIDController5010 bottomPID;

  private MotorController5010 topMotor;
  private MotorController5010 botMotor;
  private SimpleMotorFeedforward topFeedFwd;
  private SimpleMotorFeedforward bottomFeedFwd;

  private static enum ControlState {
    Joystick,
    Velocity
  }

  private ControlState shooterState = ControlState.Joystick;

  private final String SHOOTER_REFERENCE_TOP = "Top Shooter Reference";
  private final String SHOOTER_REFERENCE_BOTTOM = "Bottom Shooter Reference";
  private final String MICRO_ADJUST = "Micro Adjust";
  private final String TOP_SHOOTER_KP = "Top Shooter Kp";
  private final String TOP_SHOOTER_KA = "Top Shooter Ka";
  private final String TOP_SHOOTER_KD = "Top Shooter Kd";
  private final String TOP_SHOOTER_KI = "Top Shooter Ki";
  private final String BOTTOM_SHOOTER_KP = "Bottom Shooter Kp";
  private final String BOTTOM_SHOOTER_KA = "Bottom Shooter Ka";
  private final String BOTTOM_SHOOTER_KD = "Bottom Shooter Kd";
  private final String BOTTOM_SHOOTER_KI = "Bottom Shooter Ki";

  private final double DEFAULT_TOLERANCE = 1;

  private static enum vals {
    REFERENCE, JOYSTICK, BOTTOM_FF, TOP_FF, BOTTOM_VELOCITY, TOP_VELOCITY, TOP_SHOOTER_KP, TOP_SHOOTER_KV,
    TOP_SHOOTER_KS, TOP_SHOOTER_KA,
    TOP_SHOOTER_KD, TOP_SHOOTER_KI, BOTTOM_SHOOTER_KP, BOTTOM_SHOOTER_KV, BOTTOM_SHOOTER_KS, BOTTOM_SHOOTER_KA,
    BOTTOM_SHOOTER_KD, BOTTOM_SHOOTER_KI
  }

  /** Creates a new Shooter. */
  public ShooterSubsystem(Mechanism2d robotSim, MotorController5010 top, MotorController5010 bottom) {

    topPID = top.getPIDController5010();
    bottomPID = bottom.getPIDController5010();

    interpolationTree = new InterpolatingDoubleTreeMap();

    values.declare(SHOOTER_REFERENCE_TOP, 0.0);
    values.declare(SHOOTER_REFERENCE_BOTTOM, 0.0);
    values.declare(MICRO_ADJUST, 100.0);
    values.declare(vals.REFERENCE.name(), false);
    values.declare(vals.JOYSTICK.name(), false);
    values.declare(vals.BOTTOM_FF.name(), 0.0);
    values.declare(vals.TOP_FF.name(), 0.0);
    values.declare(vals.BOTTOM_VELOCITY.name(), 0.0);
    values.declare(vals.TOP_VELOCITY.name(), 0.0);
    values.declare(vals.TOP_SHOOTER_KP.name(), 0.0);
    values.declare(vals.TOP_SHOOTER_KV.name(), 0.002);
    values.declare(vals.TOP_SHOOTER_KS.name(), 0.3);
    values.declare(vals.TOP_SHOOTER_KA.name(), 0.0002);
    values.declare(vals.TOP_SHOOTER_KD.name(), 0.0);
    values.declare(vals.TOP_SHOOTER_KI.name(), 0.000001);
    values.declare(vals.BOTTOM_SHOOTER_KP.name(), 0.0);
    values.declare(vals.BOTTOM_SHOOTER_KV.name(), 0.002);
    values.declare(vals.BOTTOM_SHOOTER_KS.name(), 0.3);
    values.declare(vals.BOTTOM_SHOOTER_KA.name(), 0.0001);
    values.declare(vals.BOTTOM_SHOOTER_KD.name(), 0.0);
    values.declare(vals.BOTTOM_SHOOTER_KI.name(), 0.000001);

    topFeedFwd = new SimpleMotorFeedforward(values.getDouble(vals.TOP_SHOOTER_KS.name()),
        values.getDouble(vals.TOP_SHOOTER_KV.name()), values.getDouble(vals.TOP_SHOOTER_KA.name()));
    bottomFeedFwd = new SimpleMotorFeedforward(values.getDouble(vals.BOTTOM_SHOOTER_KS.name()),
        values.getDouble(vals.BOTTOM_SHOOTER_KV.name()), values.getDouble(vals.BOTTOM_SHOOTER_KA.name()));

    topEncoder = top.getMotorEncoder();
    bottomEncoder = bottom.getMotorEncoder();

    topEncoder.setVelocityConversion(60);
    bottomEncoder.setVelocityConversion(60);

    topPID.setP(values.getDouble(vals.TOP_SHOOTER_KP.name()));
    topPID.setI(values.getDouble(vals.TOP_SHOOTER_KI.name()));
    topPID.setIZone(100);
    topPID.setD(values.getDouble(vals.TOP_SHOOTER_KD.name()));
    topPID.setTolerance(DEFAULT_TOLERANCE);

    bottomPID.setP(values.getDouble(vals.BOTTOM_SHOOTER_KP.name()));
    bottomPID.setI(values.getDouble(vals.BOTTOM_SHOOTER_KI.name()));
    bottomPID.setIZone(100);
    bottomPID.setD(values.getDouble(vals.BOTTOM_SHOOTER_KD.name()));
    bottomPID.setTolerance(DEFAULT_TOLERANCE);

    this.topMotorSim = robotSim.getRoot("Shooter Top", 0.80, 0.50)
        .append(new MechanismLigament2d("Top Motor", 0.1, 180, 5, new Color8Bit(Color.kLimeGreen)));
    this.bottomMotorSim = robotSim.getRoot("Shooter Bottom", 0.80, 0.30)
        .append(new MechanismLigament2d("Bottom Motor", 0.1, 180, 5, new Color8Bit(Color.kSpringGreen)));

    this.topMotor = top;
    this.botMotor = bottom;
  }

  public void setTolerance(double value) {
    bottomPID.setTolerance(value);
    topPID.setTolerance(value);
  }

  public void resetToleranceToDefaults() {
    bottomPID.setTolerance(DEFAULT_TOLERANCE);
    topPID.setTolerance(DEFAULT_TOLERANCE);
  }

  public double getTopFeedFwdVoltage(double velocity) {
    return topFeedFwd.calculate(velocity);
  }

  public double getBottomFeedFwdVoltage(double velocity) {
    return bottomFeedFwd.calculate(velocity);
  }

  public void setInterpolatedShotSpeed(double distance) {
    double speed = interpolationTree.get(distance);
    setShooterReference(speed, speed);
  }

  public Command getBottomSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(botMotor, bottomEncoder, "Bottom Motor", this), 10, 3, 3);
  }

  public Command getTopSysIdRoutineCommand() {
    return SystemIdentification
        .getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(topMotor, topEncoder, "Top Motor", this), 10, 3, 3);
  }

  public void shooterStateMachine(double shooter) {
    switch (shooterState) {
      case Joystick:
        values.set(vals.REFERENCE.name(), false);
        values.set(vals.JOYSTICK.name(), true);
        if (shooter == 0) {
          shooterState = ControlState.Velocity;
          // setShooterReference(0, 0);
          runToReferenceShooter();

        } else {
          setShooterSpeed(shooter, shooter);
        }
        break;

      case Velocity:
        values.set(vals.REFERENCE.name(), true);
        values.set(vals.JOYSTICK.name(), false);
        if (shooter != 0) {
          shooterState = ControlState.Joystick;
          setShooterSpeed(shooter, shooter);
        } else {
          runToReferenceShooter();
        }
        break;
    }
  }

  public void setShooterSpeed(double top, double bottom) {
    topMotor.set(top);
    botMotor.set(bottom);

  }

  public void stopMotors() {
    topMotor.set(0);
    botMotor.set(0);
    topMotorSim.setAngle(0);
    bottomMotorSim.setAngle(0);
  }

  public void runToReferenceShooter() {

    double topFeedForward = getTopFeedFwdVoltage(getTopReference());
    double bottomFeedForward = getBottomFeedFwdVoltage(getBottomReference());

    values.set(vals.BOTTOM_FF.name(), bottomFeedForward);
    values.set(vals.TOP_FF.name(), topFeedForward);

    if (Robot.isSimulation()) {
      double topCurrentError = getTopReference() - getTopVelocity();
      double bottomCurrentError = getBottomReference() - getBottomVelocity();

      double currentTime = RobotController.getFPGATime() / 1E6;
      double topErrorRate = (topCurrentError - topShooterPrevError) / (currentTime - shooterPrevTime);
      double bottomErrorRate = (bottomCurrentError - bottomShooterPrevError) / (currentTime - shooterPrevTime);

      double topVoltage = topCurrentError * topPID.getP() + (topErrorRate * topPID.getD());
      double bottomVoltage = bottomCurrentError * bottomPID.getP() + (bottomErrorRate * bottomPID.getD());

      topMotor.set(topFeedForward / RobotController.getBatteryVoltage() + topVoltage);
      botMotor.set(bottomFeedForward / RobotController.getBatteryVoltage() + bottomVoltage);

      topShooterPrevError = topCurrentError;
      bottomShooterPrevError = bottomCurrentError;
      shooterPrevTime = currentTime;
    } else {

      if (LogLevel.DEBUG == RobotContainer.getLoggingLevel()) {
        topPID.setP(values.getDouble(vals.TOP_SHOOTER_KP.name()));
        topPID.setI(values.getDouble(vals.TOP_SHOOTER_KI.name()));
        topPID.setD(values.getDouble(vals.TOP_SHOOTER_KD.name()));
        topPID.setTolerance(DEFAULT_TOLERANCE);

        bottomPID.setP(values.getDouble(vals.BOTTOM_SHOOTER_KP.name()));
        bottomPID.setI(values.getDouble(vals.BOTTOM_SHOOTER_KI.name()));
        bottomPID.setD(values.getDouble(vals.BOTTOM_SHOOTER_KD.name()));

        topFeedFwd = new SimpleMotorFeedforward(values.getDouble(vals.TOP_SHOOTER_KS.name()),
            values.getDouble(vals.TOP_SHOOTER_KV.name()), values.getDouble(vals.TOP_SHOOTER_KA.name()));
        bottomFeedFwd = new SimpleMotorFeedforward(values.getDouble(vals.BOTTOM_SHOOTER_KS.name()),
            values.getDouble(vals.BOTTOM_SHOOTER_KV.name()), values.getDouble(vals.BOTTOM_SHOOTER_KA.name()));
      }

      topPID.setReference(getTopReference() / 60.0, PIDControlType.VELOCITY, topFeedForward);
      bottomPID.setReference(getBottomReference() / 60.0, PIDControlType.VELOCITY, bottomFeedForward);
    }

  }

  public void setShooterReference(double top, double bottom) {
    values.set(SHOOTER_REFERENCE_TOP, top);
    values.set(SHOOTER_REFERENCE_BOTTOM, bottom);
    shooterState = ControlState.Velocity;

  }

  public boolean isAtTarget() {
    return topPID.isAtTarget() && bottomPID.isAtTarget();
  }

  public Double getPIDReferenceTop() {
    return topPID.getReference();
  }

  public Double getTopReference() {
    return values.getDouble(SHOOTER_REFERENCE_TOP);
  }

  public Double getBottomReference() {
    return values.getDouble(SHOOTER_REFERENCE_BOTTOM);
  }

  public Command adjustShooterReferenceUp() {
    return Commands.runOnce(() -> setShooterReference(getTopReference() + values.getDouble(MICRO_ADJUST),
        getBottomReference() + values.getDouble(MICRO_ADJUST)), this);
  }

  public Command adjustShooterReferenceDown() {
    return Commands.runOnce(() -> setShooterReference(getTopReference() - values.getDouble(MICRO_ADJUST),
        getBottomReference() - values.getDouble(MICRO_ADJUST)), this);
  }

  public double getTopVelocity() {
    return Robot.isReal() ? topEncoder.getVelocity() : topSimEncoder.getVelocity();
  }

  public double getBottomVelocity() {
    return Robot.isReal() ? bottomEncoder.getVelocity() : bottomSimEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    topMotorSim.setAngle(topMotor.get() * 180 - 90);
    bottomMotorSim.setAngle(botMotor.get() * 180 - 90);

    values.set(vals.TOP_VELOCITY.name(), topEncoder.getVelocity());
    values.set(vals.BOTTOM_VELOCITY.name(), bottomEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter At Reference", isAtTarget());
  }

  @Override
  public void simulationPeriodic() {

  }
}
