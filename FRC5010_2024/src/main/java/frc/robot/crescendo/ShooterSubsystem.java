// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.FRC5010.motors.PIDController5010;
import frc.robot.FRC5010.motors.PIDController5010.PIDControlType;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;
import swervelib.math.SwerveMath;

public class ShooterSubsystem extends GenericSubsystem {

  private DigitalInput beambreak;

  private MechanismRoot2d shooterSimRoot;
  private MechanismLigament2d topMotorSim;
  private MechanismLigament2d bottomMotorSim;
  private MechanismLigament2d feederMotorSim;

  private SimulatedEncoder topSimEncoder = new SimulatedEncoder(18, 19);
  private SimulatedEncoder bottomSimEncoder = new SimulatedEncoder(20, 21);
  private SimulatedEncoder feederSimEncoder = new SimulatedEncoder(22, 23);

  private GenericEncoder topEncoder;
  private GenericEncoder bottomEncoder;
  private GenericEncoder feederEncoder;

  private InterpolatingDoubleTreeMap interpolationTree;



  private double shooterPrevTime = 0.0;
  private double topShooterPrevError = 0.0;
  private double bottomShooterPrevError = 0.0;

  private double feederReference = 0.0;
  private double feederPrevTime = 0.0;
  private double feederPrevError = 0.0;
 

  private PIDController5010 topPID;
  private PIDController5010 bottomPID;
  private PIDController5010 feederPID;

  private MotorController5010 topMotor;
  private MotorController5010 botMotor;
  private SimpleMotorFeedforward topFeedFwd;
  private SimpleMotorFeedforward bottomFeedFwd;

  private MotorController5010 feederMotor;
  private SimpleMotorFeedforward feederFeedFwd;

  private static enum ControlState {
    Joystick,
    Velocity
  }

  private ControlState shooterState = ControlState.Joystick;
  private ControlState feederState = ControlState.Joystick;


  private final String FEEDER_CONVERSION_FACTOR = "Feeder Conversion Factor";
  private final double FEEDER_CONVERSION_DEFAULT = 1.0;
  
  private final String SHOOTER_REFERENCE_TOP = "Top Shooter Reference";
  private final String SHOOTER_REFERENCE_BOTTOM = "Bottom Shooter Reference";

  private final String MICRO_ADJUST = "Micro Adjust";

  private final String BEAM_BREAK_STATE = "Beam Break State";

  /** Creates a new Shooter. */
  public ShooterSubsystem(Mechanism2d robotSim, MotorController5010 top, MotorController5010 feeder, MotorController5010 bottom) {
    
    topPID = top.getPIDController5010();
    bottomPID = bottom.getPIDController5010();
    feederPID = feeder.getPIDController5010();

    interpolationTree = new InterpolatingDoubleTreeMap();

    values.declare(FEEDER_CONVERSION_FACTOR, FEEDER_CONVERSION_DEFAULT);
    values.declare(SHOOTER_REFERENCE_TOP, 0.0);
    values.declare(SHOOTER_REFERENCE_BOTTOM, 0.0);
    values.declare(MICRO_ADJUST, 10.0);
    values.declare(BEAM_BREAK_STATE, false);
    
    topFeedFwd = SwerveMath.createDriveFeedforward(12, NEO.MAXRPM, 1.19);
    bottomFeedFwd = SwerveMath.createDriveFeedforward(12, NEO.MAXRPM, 1.19);
    feederFeedFwd = SwerveMath.createDriveFeedforward(12, NEO.MAXRPM, 1.19);
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
    this.feederMotorSim = robotSim.getRoot("Feeder Motor", 40, 30)
      .append(new MechanismLigament2d("Feeder Motor", 5, 180, 5, new Color8Bit(Color.kAntiqueWhite)));
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

  public void setInterpolatedShotSpeed(double distance) {
    double speed = interpolationTree.get(distance);
    setShooterReference(speed, speed);
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

  public boolean getBeambreak() {
    return beambreak.get(); 
  }



  public void shooterStateMachine(double shooter) {
    switch (shooterState) {
      case Joystick:
        SmartDashboard.putBoolean("Running Shooter Reference", false);
        SmartDashboard.putBoolean("Running Shooter Joystick", true);
        if (shooter == 0) {
          shooterState = ControlState.Velocity;
          //setShooterReference(0, 0);
          runToReferenceShooter();

        } else {
          setShooterSpeed(shooter, shooter);
        }
        break;
    
      case Velocity:
        SmartDashboard.putBoolean("Running Shooter Reference", true);
        SmartDashboard.putBoolean("Running Shooter Joystick", false);
        if (shooter != 0) {
          shooterState = ControlState.Joystick;
          setShooterSpeed(shooter, shooter);
        } else {
          runToReferenceShooter();
        }
        break;
    }
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

    SmartDashboard.putNumber("Shooter Bottom FF", bottomFeedForward);
    SmartDashboard.putNumber("Shooter Top FF", topFeedForward);

    
    if (!Robot.isReal()) {
      topMotor.set(topFeedForward/ RobotController.getBatteryVoltage() + topVoltage);
      botMotor.set(bottomFeedForward/ RobotController.getBatteryVoltage() + bottomVoltage);
    } else {
      topPID.setReference(getTopReference(), PIDControlType.VELOCITY, topFeedForward);
      bottomPID.setReference(getTopReference(), PIDControlType.VELOCITY, bottomFeedForward);
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
      feederPID.setReference(getFeederReference(), PIDControlType.VELOCITY, feedforward);
    }


    feederPrevTime = currentTime;
    feederPrevError = currentError;

  }

  public void setShooterReference(double top, double bottom) {
    values.set(SHOOTER_REFERENCE_TOP, top);
    values.set(SHOOTER_REFERENCE_BOTTOM, bottom);
    shooterState = ControlState.Velocity;
 
  }

  public void setFeederReference(double speed) {
    feederReference = speed;
  }

  public Double getTopReference() {
    return values.getDouble(SHOOTER_REFERENCE_TOP);
  }

  public Double getBottomReference() {
    return values.getDouble(SHOOTER_REFERENCE_BOTTOM);
  }

  public double getFeederReference() {
    return feederReference;
  }

  public Command adjustShooterReferenceUp() {
    return Commands.runOnce(() -> setShooterReference(getTopReference()+values.getDouble(MICRO_ADJUST), getBottomReference()+values.getDouble(MICRO_ADJUST)), this);
  }
  public Command adjustShooterReferenceDown() {
    return Commands.runOnce(() -> setShooterReference(getTopReference()-values.getDouble(MICRO_ADJUST), getBottomReference()-values.getDouble(MICRO_ADJUST)), this);
  }

  public Command adjustFeederReferenceUp() {
    return Commands.runOnce(() -> setFeederReference(getFeederReference()+values.getDouble(MICRO_ADJUST)), this);
  }
  public Command adjustFeederReferenceDown() {
    return Commands.runOnce(() -> setFeederReference(getFeederReference()-values.getDouble(MICRO_ADJUST)), this);
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
    feederMotorSim.setAngle(feederMotor.get() * 180 - 90);
    values.set(BEAM_BREAK_STATE, getBeambreak());
    SmartDashboard.putBoolean("Beam Break State", getBeambreak());
  }

  @Override
  public void simulationPeriodic() {

    
  }
}
