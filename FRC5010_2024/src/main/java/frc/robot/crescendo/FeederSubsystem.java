// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.PIDController5010;
import frc.robot.FRC5010.motors.PIDController5010.PIDControlType;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;
import swervelib.math.SwerveMath;

public class FeederSubsystem extends GenericSubsystem {

  private MotorController5010 feederMotor;
  private PIDController5010 pid;
  private double reference;
  private GenericEncoder encoder;
  private SimulatedEncoder feederSimEncoder = new SimulatedEncoder(22, 23);
  private SimpleMotorFeedforward feederFeedFwd;

  private static enum ControlState {
    Joystick,
    Velocity
  }

  private ControlState feederState = ControlState.Joystick;

  private double feederReference = 0.0;
  private double feederPrevTime = 0.0;
  private double feederPrevError = 0.0;

  private double microAdjust = 10.0;

  private MechanismLigament2d feederMotorSim;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(Mechanism2d robotSim, MotorController5010 feeder) {
    feederMotor = feeder;
    encoder = feeder.getMotorEncoder();
    pid = feeder.getPIDController5010();
    feederFeedFwd = SwerveMath.createDriveFeedforward(12, NEO.MAXRPM, 1.19);


    pid.setValues(new GenericPID(0,0, 0));

    this.feederMotorSim = robotSim.getRoot("Feeder Motor", 0.40, 0.30)
      .append(new MechanismLigament2d("Feeder Motor", 0.1, 180, 5, new Color8Bit(Color.kMagenta)));
  }

  public Command getFeederSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(feederMotor, feederMotor.getMotorEncoder(), "Top Motor", this), 5, 3, 3);
  }

  public void setFeederReference(double speed) {
    reference = speed;
  }

  public double getFeederReference() {
    return reference;
  }

  public double getFeederVelocity() {
    return Robot.isReal() ? encoder.getVelocity() : feederSimEncoder.getVelocity();
  }

  public void feederStateMachine(double feeder) {
    switch (feederState) {
      case Joystick:

        if (feeder == 0) {
          feederState = ControlState.Velocity;
         
          setFeederReference(0.0);
          runToReferenceFeeder();

        } else {
          setFeederSpeed(feeder);
        }
        break;
    
      case Velocity:

        if (feeder != 0) {
          feederState = ControlState.Joystick;
          setFeederSpeed(feeder);
        } else {
          runToReferenceFeeder();
        }
        break;
    }
  }

  public void runToReferenceFeeder() {
    double currentError = getFeederReference() - getFeederVelocity();

    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - feederPrevError) / (currentTime - feederPrevTime);


    double voltage = currentError * pid.getP() + (errorRate * pid.getD());

    double feedforward = getFeederFeedFwdVoltage(voltage);

    
    if (!Robot.isReal()) {

      feederMotor.set(feedforward/ RobotController.getBatteryVoltage() + feedforward);
    } else {
      pid.setReference(getFeederReference(), PIDControlType.VELOCITY, feedforward);
    }


    feederPrevTime = currentTime;
    feederPrevError = currentError;

  }

  public double getFeederFeedFwdVoltage(double velocity) {
    return feederFeedFwd.calculate(velocity);
  }


  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void stop() {
    feederMotor.set(0);
    feederMotorSim.setAngle(0);
  }

  public Command adjustFeederReferenceUp() {
    return Commands.runOnce(() -> setFeederReference(getFeederReference()+microAdjust), this);
  }
  public Command adjustFeederReferenceDown() {
    return Commands.runOnce(() -> setFeederReference(getFeederReference()-microAdjust), this);
  }

  @Override
  public void periodic() {
    feederMotorSim.setAngle(feederMotor.get() * 180 - 90);
  }
}
