// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private DigitalInput beambreak;
  private final String SIM_BEAMBREAK = "Simulated Beambreak";

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
  private final String BEAM_BREAK_STATE = "Beam Break State";

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(Mechanism2d robotSim, MotorController5010 feeder) {
    feederMotor = feeder;
    encoder = feeder.getMotorEncoder();
    pid = feeder.getPIDController5010();
    feederFeedFwd = SwerveMath.createDriveFeedforward(12, NEO.MAXRPM, 1.19);
    beambreak = new DigitalInput(1);
    values.declare(SIM_BEAMBREAK, false);
  
    

    values.declare(BEAM_BREAK_STATE, false);

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

  public Command loadNote() {
    return Commands.runEnd(() -> setFeederSpeed(-0.1), () -> {
      stop();
      setNoteState(NoteState.Loaded);
    }, this).until(() -> !isBeamBroken());
  }

  public Command shootNote() {
    return Commands.runEnd(() -> {
      if (noteState == NoteState.Holding && isBeamBroken()) {
        noteState = NoteState.Shooting;
      } else if (noteState == NoteState.Shooting && !isBeamBroken()) {
        noteState = NoteState.Empty;
      }
    }, () -> {
      stop();
    }, this).beforeStarting(() -> setFeederSpeed(1.0), this).until(() ->getNoteState().equals(NoteState.Empty));
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

  public NoteState getNoteState() {
    return noteState;
  }

  public void setNoteState(NoteState state) {
    noteState = state;
  }

  public void emptied() {
    setNoteState(NoteState.Empty);
  }

  public void holding() {
    setNoteState(NoteState.Holding);
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

      feederMotor.set(feedforward/ RobotController.getBatteryVoltage() + feedforward);
    } else {
      pid.setReference(getFeederReference(), PIDControlType.VELOCITY, feedforward);
    }


    feederPrevTime = currentTime;
    feederPrevError = currentError;

  }

  public void setSimulatedBeambreak(boolean value) {
    values.set(SIM_BEAMBREAK, value);
  }

  public boolean isBeamBroken() {
    return Robot.isReal() ? !beambreak.get() : !values.getBoolean(SIM_BEAMBREAK);
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
    values.set(BEAM_BREAK_STATE, isBeamBroken());
  }
}
