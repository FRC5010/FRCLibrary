// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

public class IntakeSubsystem extends SubsystemBase {
  private String intakeState;
  private MotorController5010 intake;
  private SparkMaxPIDController intakeController;
  private MotorModelConstants intakeConstants;
  private GenericPID intakePID;
  private GenericEncoder intakeEncoder;
  
  private DoubleSolenoid intakeSolenoid;

  private double setPoint = 0;
  private double KFF = 0;
  private double kIz = 0;
 // When piston is closed = cube, open = cone
  private Mechanism2d m_mech2d;
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d m_intakeLeft1Mech2d;
  private MechanismLigament2d m_intakeLeft2Mech2d;
  private MechanismLigament2d m_intakeRight1Mech2d;
  private MechanismLigament2d m_intakeRight2Mech2d;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(MotorController5010 intake, MotorController5010 intake2, SparkMaxPIDController intakeController,
      MotorModelConstants intakeConstants, GenericPID intakePID, GenericEncoder intakeEncoder,
      DoubleSolenoid intakeSolenoid, Mechanism2d m_mech2d) {
    this.intake = intake;
    intake2.setInverted(true);
    intake2.setFollow(intake);
    this.intakeController = ((CANSparkMax) intake).getPIDController(); 
    this.intakeEncoder = intake.getMotorEncoder();
    this.intakeConstants = intakeConstants;
    this.intakePID = intakePID;
    this.intakeSolenoid = intakeSolenoid;
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);

    this.m_mech2d = m_mech2d;
    // TODO: Find values for mech 2D once intake is made "0"
    m_mech2dRoot = m_mech2d.getRoot("Intake Root", 0, 0);
    m_intakeLeft1Mech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Intake L1", Units.metersToInches(0), 0));
    m_intakeLeft2Mech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Intake L2", Units.metersToInches(0), 0));
    m_intakeRight1Mech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Intake R1", Units.metersToInches(0), 0));
    m_intakeRight2Mech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Intake R2", Units.metersToInches(0), 0));

    intakeController.setP(intakePID.getkP());
    intakeController.setI(intakePID.getkI());
    intakeController.setD(intakePID.getkD());
    // TODO Set values
    intakeController.setFF(KFF);
    intakeController.setIZone(kIz);
  }

  public void setVelocity(double velocity) {
    this.setPoint = velocity;
    intakeController.setReference(this.setPoint, CANSparkMax.ControlType.kVelocity, 0);
  }

  public void stopIntake(){
    this.setPoint = 0;
    intakeController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
    intake.set(0);
  }

  public void toggleIntake(){
    if (intakeSolenoid.isFwdSolenoidDisabled()) {
      setIntakeCone();
    }
    intakeSolenoid.toggle();
  }
  
  public void setIntakeCube(){ // Cube
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setIntakeCone(){ // Cone
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void disableIntake(){
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);
  }
  
  public boolean isIntakeCone(){
    return intakeSolenoid.get().equals(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    if (isIntakeCone()) {
      intakeState = "Cone";
    } else {
      intakeState = "Cube";
    }

    // This method will be called once per scheduler run
    Shuffleboard.getTab("Robot")
      .add("Intake State", intakeState);
  }
}
