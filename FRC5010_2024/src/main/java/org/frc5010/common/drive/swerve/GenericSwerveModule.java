// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.arch.Persisted;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.constants.SwerveModuleConstants;
import org.frc5010.common.mechanisms.DriveConstantsDef;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public abstract class GenericSwerveModule extends SubsystemBase {
  private final MechanismLigament2d motorDial;
  private final MechanismLigament2d absEncDial;
  private final MechanismLigament2d expectDial;
  protected final String moduleKey;
  private PIDController turningController;

  protected MotorController5010 drive, turn;
  protected GenericEncoder turnEncoder, driveEncoder, absoluteEncoder;
  protected GenericPID pid = new GenericPID(0, 0, 0);
  protected LinearFilter velocityAdjustment = LinearFilter.movingAverage(5);
  protected Persisted<Double> swerveTurnP;
  protected Persisted<Double> swerveTurnI;
  protected Persisted<Double> swerveTurnD;
  protected MotorFeedFwdConstants motorConstants = new MotorFeedFwdConstants(0, 0, 0);
  protected SwerveModuleConstants moduleConstants =
      new SwerveModuleConstants(Units.inchesToMeters(0), 0, false, 0, false, false);
  private double radOffset;
  private SwerveConstants swerveConstants;
  // protected GenericEncoder driveEncoder = new SimulatedEncoder(0, 1);
  // protected GenericEncoder turnEncoder = new SimulatedEncoder(2, 3);
  // protected GenericGyro gyro = new SimulatedGyro();

  public GenericSwerveModule(
      MechanismRoot2d visualRoot, String key, double radOffset, SwerveConstants swerveConstants) {
    this.moduleKey = key;
    visualRoot.append(
        new MechanismLigament2d(moduleKey + "vert", 10, 90, 6.0, new Color8Bit(50, 50, 50)));
    visualRoot.append(
        new MechanismLigament2d(moduleKey + "hori", 10, 0, 6.0, new Color8Bit(50, 50, 50)));
    motorDial =
        visualRoot.append(
            new MechanismLigament2d(moduleKey, 10.0, 90, 6.0, new Color8Bit(Color.kYellow)));
    absEncDial =
        visualRoot.append(
            new MechanismLigament2d(moduleKey + "Abs", 10, 90, 6, new Color8Bit(Color.kBlue)));
    expectDial =
        visualRoot.append(
            new MechanismLigament2d(moduleKey + "Exp", 10, 90, 6, new Color8Bit(Color.kRed)));

    this.radOffset = radOffset;
    this.swerveConstants = swerveConstants;
  }

  public void setupSwerveEncoders() {
    turnEncoder = turn.getMotorEncoder();
    driveEncoder = drive.getMotorEncoder();

    // set units drive encoder to meters and meters/sec
    driveEncoder.setPositionConversion(moduleConstants.getkDriveEncoderRot2Meter());
    driveEncoder.setVelocityConversion(moduleConstants.getkDriveEncoderRPM2MeterPerSec());
    // set units turning encoder to radians and radians/sec
    turnEncoder.setPositionConversion(moduleConstants.getkTurningEncoderRot2Rad());
    turnEncoder.setVelocityConversion(moduleConstants.getkTurningEncoderRPM2RadPerSec());

    swerveTurnP = new Persisted<>(DriveConstantsDef.SWERVE_TURN_P, pid.getkP());
    swerveTurnI = new Persisted<>(DriveConstantsDef.SWERVE_TURN_I, pid.getkI());
    swerveTurnD = new Persisted<>(DriveConstantsDef.SWERVE_TURN_D, pid.getkD());
    turningController = new PIDController(swerveTurnP.get(), swerveTurnI.get(), swerveTurnD.get());

    turningController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public double getDrivePosition() {
    SmartDashboard.putNumber(moduleKey + " Drive Position", driveEncoder.getPosition());
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    SmartDashboard.putNumber(moduleKey + " Turn Position", turnEncoder.getPosition());
    return turnEncoder.getPosition();
  }

  public double getAbsoluteEncoderRad() {
    return absoluteEncoder.getPosition() - radOffset;
  }

  public double getTurningVelocity() {
    return turnEncoder.getVelocity();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public void resetAbsoluteEncoder() {}

  public boolean setState(SwerveModuleState state, boolean ready) {
    state = SwerveModuleState.optimize(state, getState().angle);
    turningController.setPID(swerveTurnP.get(), swerveTurnI.get(), swerveTurnD.get());
    double turnPow = turningController.calculate(getTurningPosition(), state.angle.getRadians());

    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return true;
    }

    double speed =
        state.speedMetersPerSecond / swerveConstants.getkPhysicalMaxSpeedMetersPerSecond();
    // calculate the percentage that the velocity is off
    double velAdj =
        (state.speedMetersPerSecond - driveEncoder.getVelocity())
            / swerveConstants.getkPhysicalMaxSpeedMetersPerSecond();
    if (ready) {
      // Add an adjustment to the overall power calc based on the average over 10
      // cycles.
      // speed += velocityAdjustment.calculate(velAdj);
      drive.set(speed);
    }

    turn.set(turnPow + (Math.signum(turnPow) * motorConstants.getkS()));
    SmartDashboard.putString(
        "Swerve [" + getKey() + "] state",
        " Vel Adj "
            + velAdj
            + " Turn "
            + turnPow
            + " Angle: "
            + state.angle.getDegrees()
            + " Speed m/s: "
            + state.speedMetersPerSecond);
    return (Math.abs(turnPow) < 0.03);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void stop() {
    drive.set(0);
    turn.set(0);
  }

  public String getKey() {
    return moduleKey;
  }

  public void periodic() {
    double turningDeg = Units.radiansToDegrees(getTurningPosition());
    double absEncDeg = Units.radiansToDegrees(getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Motor Ang: " + moduleKey, turningDeg);
    SmartDashboard.putNumber("Abs Angle: " + moduleKey, absEncDeg);
    SmartDashboard.putNumber("Abs Rads: " + moduleKey, getAbsoluteEncoderRad());
    // This method will be called once per scheduler run
    absEncDial.setAngle(absEncDeg + 90);
    motorDial.setAngle(turningDeg + 90);
    motorDial.setLength(20 * getTurningVelocity() + 5);
    expectDial.setLength(20 * getDriveVelocity() + 5);
    expectDial.setAngle(getState().angle.getDegrees() + 90);
  }
}
