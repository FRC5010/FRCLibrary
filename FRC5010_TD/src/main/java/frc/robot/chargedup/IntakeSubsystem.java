// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

public class IntakeSubsystem extends SubsystemBase {
  private MotorController5010 intake;
  private SparkMaxPIDController intakeController;
  private MotorModelConstants intakeConstants;
  private GenericPID intakePID;
  private GenericEncoder intakeEncoder;

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
  public IntakeSubsystem(MotorController5010 intake, MotorController5010 intake2, MotorController5010 intake3,
      MotorModelConstants intakeConstants, GenericPID intakePID,
      Mechanism2d m_mech2d) {
    this.intake = intake.invert(false);
    intake2.setFollow(intake, true);
    intake3.setFollow(intake, false);
    this.intakeController = ((CANSparkMax) intake).getPIDController();
    this.intakeEncoder = intake.getMotorEncoder();
    this.intakeConstants = intakeConstants;
    this.intakePID = intakePID;

    this.m_mech2d = m_mech2d;
    // TODO: Find values for mech 2D once intake is made "0"
    m_mech2dRoot = m_mech2d.getRoot("Intake Root", 10, 10);
    m_intakeLeft1Mech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Intake L1", Units.metersToInches(0.1), 0));
    m_intakeRight1Mech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Intake R1", Units.metersToInches(0.1), 0));

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
    m_intakeLeft1Mech2d.setAngle(velocity / 5500 * 360);
    m_intakeRight1Mech2d.setAngle(velocity / 5500 * 360);
  }

  public void stopIntake() {
    this.setPoint = 0;
    // intakeController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
    intake.set(0);
    m_intakeLeft1Mech2d.setAngle(0);
    m_intakeRight1Mech2d.setAngle(0);
  }

  public void toggleIntake() {

  }

  public void setIntakeCube() { // Cube
  }

  public void setIntakeCone() { // Cone
  }

  public void disableIntake() {
  }

  public void setMotor(double speed) {
    intake.set(speed);
    m_intakeLeft1Mech2d.setAngle(speed * 360);
    m_intakeRight1Mech2d.setAngle(speed * 360);
  }

  public boolean isIntakeCone() {
    return false;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // Shuffleboard.getTab("Robot").addString("Intake Mode Cone/Cube", () ->
    // intakeState);
  }
}
