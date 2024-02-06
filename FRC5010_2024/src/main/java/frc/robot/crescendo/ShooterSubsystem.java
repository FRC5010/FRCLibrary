// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;

public class ShooterSubsystem extends GenericSubsystem {

  private DigitalInput beambreak;

  private MechanismRoot2d shooterSimRoot;
  private MechanismLigament2d topMotorSim;
  private MechanismLigament2d bottomMotorSim;

  private SimulatedEncoder topSimEncoder = new SimulatedEncoder(18, 19);
  private SimulatedEncoder bottomSimEncoder = new SimulatedEncoder(20, 21);
  private SimulatedEncoder feederSimEncoder = new SimulatedEncoder(22, 23);

  private GenericEncoder topEncoder;
  private GenericEncoder bottomEncoder;
  private GenericEncoder feederEncoder;


  private double topReference = 0.0;
  private double bottomReference = 0.0;
  private double shooterPrevTime = 0.0;
  private double topShooterPrevError = 0.0;
  private double bottomShooterPrevError = 0.0;

  private double feederReference = 0.0;
  private double feederPrevTime = 0.0;
  private double feederPrevError = 0.0;
 

  private SparkPIDController topPID;
  private SparkPIDController bottomPID;
  private SparkPIDController feederPID;

  private MotorController5010 topMotor;
  private MotorController5010 botMotor;
  private SimpleMotorFeedforward topFeedFwd;
  private SimpleMotorFeedforward bottomFeedFwd;

  private MotorController5010 feederMotor;
  private SimpleMotorFeedforward feederFeedFwd;

  private static enum ShooterState {
    Joystick,
    Velocity
  }

  private ShooterState shooterState = ShooterState.Joystick;


  private final String FEEDER_CONVERSION_FACTOR = "Feeder Conversion Factor";
  private final double FEEDER_CONVERSION_DEFAULT = 1.0;
  

  /** Creates a new Shooter. */
  public ShooterSubsystem(Mechanism2d robotSim, MotorController5010 top, MotorController5010 feeder, MotorController5010 bottom) {
    
    topPID = ((CANSparkMax) top.getMotor()).getPIDController();
    bottomPID = ((CANSparkMax) bottom.getMotor()).getPIDController();
    feederPID = ((CANSparkMax) feeder.getMotor()).getPIDController();

    values.declare(FEEDER_CONVERSION_FACTOR, FEEDER_CONVERSION_DEFAULT);
    
    topFeedFwd = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    bottomFeedFwd = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    feederFeedFwd = new SimpleMotorFeedforward(0, 0, 0);

    topEncoder = top.getMotorEncoder();
    bottomEncoder = bottom.getMotorEncoder();
    feederEncoder = feeder.getMotorEncoder();

    topPID.setP(0);
    topPID.setI(0);    
    topPID.setD(0);
  
    bottomPID.setP(0);
    bottomPID.setI(0);    
    bottomPID.setD(0);

    feederPID.setP(0);
    feederPID.setI(0);    
    feederPID.setD(0);

    beambreak = new DigitalInput(0);

    this.topMotorSim = robotSim.getRoot("Shooter Top", 50, 50)
    .append(new MechanismLigament2d("Top Motor", 5, 180, 5, new Color8Bit(Color.kGreen)));
    this.bottomMotorSim = robotSim.getRoot("Shooter Bottom", 50, 30)
    .append(new MechanismLigament2d("Bottom Motor", 5, 180, 5, new Color8Bit(Color.kGreen)));
    this.topMotor = top;
    this.botMotor = bottom;
    this.feederMotor = feeder;




    SmartDashboard.putData(this);
  }

  public double getTopFeedFwdVoltage(double velocity) {
    return topFeedFwd.calculate(velocity);
  }

  public double getBottomFeedFwdVoltage(double velocity) {
    return bottomFeedFwd.calculate(velocity);
  }
  public double getFeederFeedFwdVoltage(double velocity) {
    return feederFeedFwd.calculate(velocity);
  }



  public Command getBottomSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(botMotor, botMotor.getMotorEncoder(), "Bottom Motor", this), 5, 3, 3);
  }

  public Command getTopSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(topMotor, topMotor.getMotorEncoder(), "Top Motor", this), 5, 3, 3);
  }

  public Command getFeederSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(feederMotor, feederMotor.getMotorEncoder(), "Top Motor", this), 5, 3, 3);
  }

  public void stateMachine(double shooter, double feeder) {
    switch (shooterState) {
      case Joystick:
        if (shooter == 0 || feeder == 0) {
          shooterState = ShooterState.Velocity;
          setReference(0, 0, 0);
          runToReferenceShooter();
          runToReferenceFeeder();
        } else {
          setShooterSpeed(shooter, shooter);
        }
        break;
    
      case Velocity:
        if (shooter != 0 || feeder != 0) {
          shooterState = ShooterState.Joystick;
          setShooterSpeed(shooter, shooter);
          setFeederSpeed(feeder);
        } else {
          runToReferenceFeeder();
          runToReferenceShooter();
        }
        break;
    }
  }

  public void setShooterSpeed(double top, double bottom) {
    topMotor.set(top);
    botMotor.set(bottom);

  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
    
  }

  public void stopMotors() {
    topMotor.set(0);
    botMotor.set(0);
    feederMotor.set(0);
    topMotorSim.setAngle(0);
    bottomMotorSim.setAngle(0);
  }

  public void runToReferenceShooter() {
    double topCurrentError = getTopReference() - getTopVelocity();
    double bottomCurrentError = getBottomReference() - getBottomVelocity();

    double currentTime = RobotController.getFPGATime() / 1E6;
    double topErrorRate = (topCurrentError - topShooterPrevError) / (currentTime - shooterPrevTime);
    double bottomErrorRate = (bottomCurrentError - bottomShooterPrevError) / (currentTime - shooterPrevTime);

    double topVoltage = topCurrentError * topPID.getP() + (topErrorRate * topPID.getD());
    double bottomVoltage = bottomCurrentError * bottomPID.getP() + (bottomErrorRate * bottomPID.getD());
    double topFeedForward = getTopFeedFwdVoltage(getTopReference());
    double bottomFeedForward = getBottomFeedFwdVoltage(getBottomReference());

    
    if (!Robot.isReal()) {
      topMotor.set(topFeedForward/ RobotController.getBatteryVoltage() + topVoltage);
      topMotor.set(bottomFeedForward/ RobotController.getBatteryVoltage() + bottomVoltage);
    } else {
      topPID.setReference(getTopReference(), CANSparkBase.ControlType.kVelocity, 0, topFeedForward, ArbFFUnits.kVoltage);
      bottomPID.setReference(getTopReference(), CANSparkBase.ControlType.kVelocity, 0, bottomFeedForward, ArbFFUnits.kVoltage);
    }


    topShooterPrevError = topCurrentError;
    bottomShooterPrevError = bottomCurrentError;
    shooterPrevTime = currentTime;

  }

  public void runToReferenceFeeder() {
    double currentError = getFeederReference() - getFeederVelocity();

    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - topShooterPrevError) / (currentTime - feederPrevTime);


    double voltage = currentError * feederPID.getP() + (errorRate * feederPID.getD());

    double feedforward = getFeederFeedFwdVoltage(voltage);

    
    if (!Robot.isReal()) {

      feederMotor.set(feedforward/ RobotController.getBatteryVoltage() + feedforward);
    } else {
      feederPID.setReference(getFeederReference(), CANSparkBase.ControlType.kVelocity, 0, feedforward, ArbFFUnits.kVoltage);
    }


    feederPrevTime = currentTime;
    feederPrevError = currentError;

  }

  public void setReference(double top, double bottom, double feeder) {
    topReference = top;
    bottomReference = bottom;
    feederReference = feeder;
  }

  public double getTopReference() {
    return topReference;
  }

  public double getBottomReference() {
    return bottomReference;
  }

  public double getFeederReference() {
    return feederReference;
  }

  public double getTopVelocity() {
    return Robot.isReal() ? topEncoder.getVelocity() : topSimEncoder.getVelocity();
  }
  public double getBottomVelocity() {
    return Robot.isReal() ? bottomEncoder.getVelocity() : bottomSimEncoder.getVelocity();
  }
  public double getFeederVelocity() {
    return Robot.isReal() ? feederEncoder.getVelocity() : feederSimEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    topMotorSim.setAngle(topMotor.get() * 180 - 90);
    bottomMotorSim.setAngle(botMotor.get() * 180 - 90);
  }

  @Override
  public void simulationPeriodic() {

    
  }
}
