// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.constants.GenericSubsystem;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.SystemIdentification;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.RevEncoder;

public class ShooterSubsystem extends GenericSubsystem {

  private DigitalInput beambreak;

  private MechanismRoot2d shooterSimRoot;
  private MechanismLigament2d topMotorSim;
  private MechanismLigament2d bottomMotorSim;

  private SparkPIDController topPID;
  private SparkPIDController bottomPID;

  private MotorController5010 topMotor;
  private MotorController5010 botMotor;
  private MotorController5010 feederMotor;


  private final String FEEDER_CONVERSION_FACTOR = "Feeder Conversion Factor";
  private final double FEEDER_CONVERSION_DEFAULT = 1.0;
  

  /** Creates a new Shooter. */
  public ShooterSubsystem(Mechanism2d robotSim, MotorController5010 top, MotorController5010 feeder, MotorController5010 bottom) {
    
    values.declare(FEEDER_CONVERSION_FACTOR, FEEDER_CONVERSION_DEFAULT);
    
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

  public Command getBottomSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(botMotor, botMotor.getMotorEncoder(), "Bottom Motor", this), 5, 3, 3);
  }

  public Command getTopSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(topMotor, topMotor.getMotorEncoder(), "Top Motor", this), 5, 3, 3);
  }

  public Command getFeederSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(feederMotor, feederMotor.getMotorEncoder(), "Top Motor", this), 5, 3, 3);
  }

  public void setMotorSpeed(double speed) {
    topMotor.set(speed);
    botMotor.set(speed);
    topMotorSim.setAngle(speed * 180 - 90);
    bottomMotorSim.setAngle(speed * 180 - 90);
  }

  public void stopMotors() {
    topMotor.set(0);
    botMotor.set(0);
    topMotorSim.setAngle(0);
    bottomMotorSim.setAngle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
