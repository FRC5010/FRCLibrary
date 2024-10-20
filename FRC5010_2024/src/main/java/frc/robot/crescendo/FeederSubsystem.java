// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.hardware.NEO;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.subsystems.Color;
import org.frc5010.common.subsystems.SegmentedLedSystem;
import swervelib.math.SwerveMath;

public class FeederSubsystem extends GenericSubsystem {

  private SegmentedLedSystem ledSubsystem;
  private MotorController5010 feederMotor;
  private PIDController5010 pid;
  private double reference;
  private GenericEncoder encoder;
  private SimulatedEncoder feederSimEncoder = new SimulatedEncoder(22, 23);
  private SimpleMotorFeedforward feederFeedFwd;
  private DigitalInput stopBeamBreak;
  private DigitalInput detectBeamBreak;
  private final String SIM_STOP_BEAMBREAK = "Simulated Stop Beambreak";
  private final String SIM_DETECT_BEAMBREAK = "Simulated Detect Beambreak";
  private boolean ReadyToShoot = false;
  private boolean shooterHasTarget = false;

  private static enum ControlState {
    Joystick,
    Velocity
  }

  /*
   * Empty: No Note
   * Holding: Note but not positioned for shooting
   * Loaded: Positioned Well, Ready for Shooting
   * Shooting: During shooting phase
   */
  public static enum NoteState {
    Empty,
    Holding,
    Locked,
    Loaded,
    Shooting,

  }

  private ControlState feederState = ControlState.Joystick;
  private NoteState noteState = NoteState.Empty;

  private double feederReference = 0.0;
  private double feederPrevTime = 0.0;
  private double feederPrevError = 0.0;

  private double microAdjust = 10.0;

  private MechanismLigament2d feederMotorSim;
  private MechanismRoot2d noteRoot;
  private MechanismLigament2d note;
  private double noteX = 0.1;
  private double noteY = 0.1;
  private double noteMotion = 0.05;

  private final String STOP_BEAM_BREAK_STATE = "Stop Beam Break State";
  private final String DETECT_BEAM_BREAK_STATE = "Detect Beam Break State";
  private final String FEEDER_MOTOR_SPEED = "Feeder Motor";
  private final String NOTE_STATE = "Note State";
  private final String INTAKE_SPEED_FACTOR = "Intake Speed Factor";

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(Mechanism2d robotSim, MotorController5010 feeder, SegmentedLedSystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    feederMotor = feeder;
    encoder = feeder.getMotorEncoder();
    pid = feeder.getPIDController5010();
    feederFeedFwd = SwerveMath.createDriveFeedforward(12, NEO.MAXRPM, 1.19);
    stopBeamBreak = new DigitalInput(1);
    detectBeamBreak = new DigitalInput(3);
    values.declare(SIM_STOP_BEAMBREAK, true);
    values.declare(SIM_DETECT_BEAMBREAK, true);
    values.declare(INTAKE_SPEED_FACTOR, 1.1);

    noteState = DriverStation.isFMSAttached() ? NoteState.Loaded
        : isDetectBeamBroken() ? NoteState.Loaded : NoteState.Empty;

    values.declare(STOP_BEAM_BREAK_STATE, false);
    values.declare(DETECT_BEAM_BREAK_STATE, false);
    values.declare(FEEDER_MOTOR_SPEED, 0.0);
    values.declare(NOTE_STATE, noteState.toString());

    pid.setValues(new GenericPID(0, 0, 0));

    this.feederMotorSim = robotSim.getRoot("Feeder Motor", 0.40, 0.30)
        .append(new MechanismLigament2d("Feeder Motor", 0.1, 180, 5, Color.PURPLE.getColor8Bit()));
    noteRoot = robotSim.getRoot("Note Root", noteX, noteY);
    note = noteRoot
        .append(
            new MechanismLigament2d("Note", Units.inchesToMeters(14), 45, 5, Color.FIFTY_TEN_ORANGE.getColor8Bit()));
  }

  public Command getFeederSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(feederMotor, feederMotor.getMotorEncoder(), "Top Motor", this), 5, 3, 3);
  }

  public void setFeederReference(double speed) {
    reference = speed;
  }

  public double getFeederReference() {
    return reference;
  }

  public void setShotReadyness(boolean value) {
    ReadyToShoot = value;
  }

  public void setShooterHasTarget(boolean value) {
    shooterHasTarget = value;
  }

  public boolean getShotReadyness() {
    return ReadyToShoot;
  }

  public Command loadNote() {
    return Commands.runEnd(() -> setFeederSpeed(-0.1), () -> {
      stop();
      setNoteState(NoteState.Loaded);
    }, this).until(() -> !isStopBeamBroken());
  }

  public Command shootNote() {
    return Commands.runEnd(() -> {
      if (noteState == NoteState.Holding && isStopBeamBroken()) {
        noteState = NoteState.Shooting;
      } else if (noteState == NoteState.Shooting && !isStopBeamBroken()) {
        noteState = NoteState.Empty;
      }
    }, () -> {
      stop();
    }, this).beforeStarting(() -> setFeederSpeed(1.0), this).until(() -> getNoteState().equals(NoteState.Empty));

  }

  public double getFeederVelocity() {
    return Robot.isReal() ? encoder.getVelocity() : feederSimEncoder.getVelocity();
  }

  public void transitionNoteState() {
    switch (noteState) {
      case Empty:
        if (isStopBeamBroken() && isDetectBeamBroken()) {
          noteState = NoteState.Holding;
        }
        break;
      case Holding:
        if (!isDetectBeamBroken()) {
          noteState = NoteState.Empty;
        } else if (!isStopBeamBroken()) {
          noteState = NoteState.Loaded;
        }
        break;
      case Loaded:
        if (isDetectBeamBroken() && isStopBeamBroken()) {
          noteState = NoteState.Shooting;
        } else if (!isDetectBeamBroken()) {
          noteState = NoteState.Empty;
        }
        break;
      case Shooting:
      if (!isStopBeamBroken() && !isDetectBeamBroken()) {
          noteState = NoteState.Empty;
        } else if (!isStopBeamBroken() && isDetectBeamBroken()) {
          noteState = NoteState.Loaded;
        }
    }
    if (ReadyToShoot) {
      ledSubsystem.setWholeStripState((Integer i) -> Color.PURPLE.getColor8Bit());
    } else if (shooterHasTarget) {
      ledSubsystem.setWholeStripState((Integer i) -> Color.FIFTY_TEN_ORANGE.getColor8Bit());
    } else {
      switch (noteState) {
        case Empty:
          ledSubsystem.setWholeStripState((Integer i) -> Color.RED.getColor8Bit());
          break;
        case Holding:
          ledSubsystem.setWholeStripState((Integer i) -> Color.BLUE.getColor8Bit());
          break;
        case Loaded:
          ledSubsystem.setWholeStripState((Integer i) -> Color.GREEN.getColor8Bit());
          break;
        case Shooting:
          ledSubsystem.setWholeStripState((Integer i) -> Color.ORANGE.getColor8Bit());
          break;
      }
    }
  }

  public void feederStateMachine(double feeder) {
    switch (feederState) {
      case Joystick:

        if (feeder == 0) {
          feederState = ControlState.Velocity;

          setFeederReference(0.0);

          setFeederSpeed(0.0);
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
    transitionNoteState();
  }

  public NoteState getNoteState() {
    transitionNoteState();
    return noteState;
  }

  public void setNoteState(NoteState state) {
    noteState = state;
  }

  public double getSpeedFactor() {
    return values.getDouble(INTAKE_SPEED_FACTOR);
  }

  public void setSpeedFactor(double factor) {
    values.set(INTAKE_SPEED_FACTOR, factor);
  }

  public void emptied() {
    setNoteState(NoteState.Empty);
  }

  public boolean isEmptied() {
    return NoteState.Empty == getNoteState();
  }

  public boolean isHolding() {
    return NoteState.Holding == getNoteState();
  }

  public void holding() {
    setNoteState(NoteState.Holding);
  }

  public boolean isLoaded() {
    return NoteState.Loaded == getNoteState();
  }

  public void loaded() {
    setNoteState(NoteState.Loaded);
  }

  public void runToReferenceFeeder() {
    double currentError = getFeederReference() - getFeederVelocity();

    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - feederPrevError) / (currentTime - feederPrevTime);

    double voltage = currentError * pid.getP() + (errorRate * pid.getD());

    double feedforward = getFeederFeedFwdVoltage(voltage);

    if (!Robot.isReal()) {

      feederMotor.set(feedforward / RobotController.getBatteryVoltage() + feedforward);
    } else {
      pid.setReference(getFeederReference(), PIDControlType.VELOCITY, feedforward);
    }

    feederPrevTime = currentTime;
    feederPrevError = currentError;

  }

  public void setSimulatedStopBeambreak(boolean value) {
    values.set(SIM_STOP_BEAMBREAK, value);
  }

  public void setSimulatedDetectBeambreak(boolean value) {
    values.set(SIM_DETECT_BEAMBREAK, value);
  }

  public boolean isStopBeamBroken() {
    return Robot.isReal() ? !stopBeamBreak.get() : !values.getBoolean(SIM_STOP_BEAMBREAK);
  }

  public boolean isDetectBeamBroken() {
    return Robot.isReal() ? !detectBeamBreak.get() : !values.getBoolean(SIM_DETECT_BEAMBREAK);
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
    return Commands.runOnce(() -> setFeederReference(getFeederReference() + microAdjust), this);
  }

  public Command adjustFeederReferenceDown() {
    return Commands.runOnce(() -> setFeederReference(getFeederReference() - microAdjust), this);
  }

  @Override
  public void periodic() {
    double feederSpeed = feederMotor.get();
    feederMotorSim.setAngle(feederSpeed * 180 - 90);
    noteX -= Math.signum(feederSpeed) * noteMotion;
    noteY -= Math.signum(feederSpeed) * noteMotion;
    values.set(STOP_BEAM_BREAK_STATE, isStopBeamBroken());
    values.set(DETECT_BEAM_BREAK_STATE, isDetectBeamBroken());
    values.set(FEEDER_MOTOR_SPEED, feederMotor.getMotorEncoder().getVelocity());
    values.set(NOTE_STATE, noteState.toString());
    if (noteX < 0.1 || noteX > 1.0)
      noteX = 0.1;
    if (noteY < 0.1 || noteY > 1.0)
      noteY = 0.1;
    noteRoot.setPosition(noteX, noteY);
    if (noteX > 0.5 && noteX < 0.55)
      values.set(SIM_STOP_BEAMBREAK, false);
    values.set(SIM_DETECT_BEAMBREAK, false);
  }
}
