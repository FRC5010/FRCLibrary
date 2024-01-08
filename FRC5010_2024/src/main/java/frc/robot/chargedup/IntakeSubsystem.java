// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
  private MechanismRoot2d m_mech2dRoot;
  private MechanismLigament2d intakeLeftMotor;
  private MechanismLigament2d intakeLeftSolenoid;
  private MechanismLigament2d intakeRightMotor;
  private MechanismLigament2d intakeRightSolenoid;
  private Color8Bit coneColor = new Color8Bit(Color.kYellow);
  private Color8Bit cubeColor = new Color8Bit(Color.kPurple);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(MotorController5010 intake, MotorController5010 intake2,
      MotorModelConstants intakeConstants, GenericPID intakePID,
      DoubleSolenoid intakeSolenoid, Mechanism2d m_mech2d) {
    this.intake = intake.invert(false);
    intake2.setFollow(intake, true);
    this.intakeController = ((CANSparkMax) intake).getPIDController();
    this.intakeEncoder = intake.getMotorEncoder();
    this.intakeConstants = intakeConstants;
    this.intakePID = intakePID;
    this.intakeSolenoid = intakeSolenoid;
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);

    // TODO: Find values for mech 2D once intake is made "0"
    intakeLeftMotor = m_mech2d.getRoot("Intake Left", 15, 50).append(
        new MechanismLigament2d(
            "Intake LM", 5, 180, 5, cubeColor));
    intakeLeftSolenoid = m_mech2d.getRoot("Intake Left Sol", 15, 50).append(
        new MechanismLigament2d(
            "Intake LS", 5, 45, 5, cubeColor));
    intakeRightMotor = m_mech2d.getRoot("Intake Right", 50, 50).append(
        new MechanismLigament2d(
            "Intake RM", 5, 0, 5, cubeColor));
    intakeRightSolenoid = m_mech2d.getRoot("Intake Right Sol", 50, 50).append(
        new MechanismLigament2d(
            "Intake RS", 5, 135, 5, cubeColor));

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
    intakeLeftMotor.setAngle(180 + (velocity / 5500 * 360));
    intakeRightMotor.setAngle(velocity / 5500 * 360);
  }

  public void stopIntake() {
    this.setPoint = 0;
    // intakeController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
    intake.set(0);
    intakeLeftMotor.setAngle(180);
    intakeRightMotor.setAngle(0);
  }

  public void toggleIntake() {
    if (intakeSolenoid.isFwdSolenoidDisabled()) {
      setIntakeCone();
    }
    intakeSolenoid.toggle();
  }

  public void setIntakeCube() { // Cube
    intakeLeftMotor.setColor(cubeColor);
    intakeRightMotor.setColor(cubeColor);
    intakeLeftSolenoid.setColor(cubeColor);
    intakeRightSolenoid.setColor(cubeColor);
    intakeLeftSolenoid.setAngle(45);
    intakeRightSolenoid.setAngle(135);
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setIntakeCone() { // Cone
    intakeLeftMotor.setColor(coneColor);
    intakeRightMotor.setColor(coneColor);
    intakeLeftSolenoid.setColor(coneColor);
    intakeRightSolenoid.setColor(coneColor);
    intakeLeftSolenoid.setAngle(0);
    intakeRightSolenoid.setAngle(180);
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void disableIntake() {
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void setMotor(double speed) {
    intake.set(speed);
    intakeLeftMotor.setAngle(180 + (speed * 360));
    intakeRightMotor.setAngle(speed * 360);
  }

  public boolean isIntakeCone() {
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
    // Shuffleboard.getTab("Robot").addString("Intake Mode Cone/Cube", () ->
    // intakeState);
  }
}
