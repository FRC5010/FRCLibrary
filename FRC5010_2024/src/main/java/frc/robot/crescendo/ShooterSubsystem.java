// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
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
import frc.robot.FRC5010.motors.hardware.KrakenX60;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;
import swervelib.math.SwerveMath;

public class ShooterSubsystem extends GenericSubsystem {

  private DigitalInput beambreak;


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

  private final String BEAM_BREAK_STATE = "Beam Break State";

  /** Creates a new Shooter. */
  public ShooterSubsystem(Mechanism2d robotSim, MotorController5010 top, MotorController5010 bottom) {
    
    topPID = top.getPIDController5010();
    bottomPID = bottom.getPIDController5010();


    interpolationTree = new InterpolatingDoubleTreeMap();

    values.declare(SHOOTER_REFERENCE_TOP, 0.0);
    values.declare(SHOOTER_REFERENCE_BOTTOM, 0.0);
    values.declare(MICRO_ADJUST, 100.0);
    values.declare(BEAM_BREAK_STATE, false);
    
    topFeedFwd = new SimpleMotorFeedforward(0.24361, 0.0020052, 7.9531E-05);
    bottomFeedFwd = new SimpleMotorFeedforward(0.18458, 0.0020311, 8.4766E-05);
    
    topEncoder = top.getMotorEncoder();
    bottomEncoder = bottom.getMotorEncoder();

    topEncoder.setVelocityConversion(60);
    bottomEncoder.setVelocityConversion(60);

    topPID.setP(0.0004543);
    topPID.setI(0.0000001);    
    topPID.setIZone(100);
    topPID.setD(0);
  
    bottomPID.setP(0.00049915);
    bottomPID.setI(0.0000001);  
    bottomPID.setIZone(100);  
    bottomPID.setD(0);



    beambreak = new DigitalInput(1);
    

    this.topMotorSim = robotSim.getRoot("Shooter Top", 0.80, 0.50)
    .append(new MechanismLigament2d("Top Motor", 0.1, 180, 5, new Color8Bit(Color.kLimeGreen)));
    this.bottomMotorSim = robotSim.getRoot("Shooter Bottom", 0.80, 0.30)
    .append(new MechanismLigament2d("Bottom Motor", 0.1, 180, 5, new Color8Bit(Color.kSpringGreen)));
    
    this.topMotor = top;
    this.botMotor = bottom;
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
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(botMotor, bottomEncoder, "Bottom Motor", this), 5, 3, 3);
  }

  public Command getTopSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(topMotor, topEncoder, "Top Motor", this), 5, 3, 3);
  }



  public boolean isBeamBroken() {
    return !beambreak.get(); 
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
      topPID.setReference(getTopReference()/60.0, PIDControlType.VELOCITY, topFeedForward);
      bottomPID.setReference(getBottomReference()/60.0, PIDControlType.VELOCITY, bottomFeedForward);
    }


    topShooterPrevError = topCurrentError;
    bottomShooterPrevError = bottomCurrentError;
    shooterPrevTime = currentTime;

  }



  public void setShooterReference(double top, double bottom) {
    values.set(SHOOTER_REFERENCE_TOP, top);
    values.set(SHOOTER_REFERENCE_BOTTOM, bottom);
    shooterState = ControlState.Velocity;
 
  }

  

  public Double getTopReference() {
    return values.getDouble(SHOOTER_REFERENCE_TOP);
  }

  public Double getBottomReference() {
    return values.getDouble(SHOOTER_REFERENCE_BOTTOM);
  }



  public Command adjustShooterReferenceUp() {
    return Commands.runOnce(() -> setShooterReference(getTopReference()+values.getDouble(MICRO_ADJUST), getBottomReference()+values.getDouble(MICRO_ADJUST)), this);
  }
  public Command adjustShooterReferenceDown() {
    return Commands.runOnce(() -> setShooterReference(getTopReference()-values.getDouble(MICRO_ADJUST), getBottomReference()-values.getDouble(MICRO_ADJUST)), this);
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
    values.set(BEAM_BREAK_STATE, isBeamBroken());
    SmartDashboard.putBoolean("Beam Break State", isBeamBroken());
    SmartDashboard.putNumber("Shooter Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Bottom Velocity", bottomEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {

    
  }
}
