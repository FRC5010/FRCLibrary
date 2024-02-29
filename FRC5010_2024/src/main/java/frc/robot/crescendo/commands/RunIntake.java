// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FRC5010.arch.GenericCommand;
import frc.robot.crescendo.FeederSubsystem;
import frc.robot.crescendo.IntakeSubsystem;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.FeederSubsystem.NoteState;

public class RunIntake extends GenericCommand {

  DoubleSupplier speed;
  DoubleSupplier feederSpeed;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  FeederSubsystem feederSubsystem;

  boolean feederStopFlag = false;

  Command feederCommand;

  /** Creates a new RunIntake. */
  public RunIntake(DoubleSupplier joystick, DoubleSupplier feederSpeed, IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem) {
    this.speed = joystick;
    this.intakeSubsystem = intakeSubsystem;
    this.feederSpeed = feederSpeed;
    this.feederSubsystem = feederSubsystem;

    feederCommand = Commands
        .run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble()) * feederSpeed.getAsDouble()),
            feederSubsystem)
        .onlyIf(() -> feederSubsystem.getNoteState() == NoteState.Empty
            || feederSubsystem.getNoteState() == NoteState.Holding)
        .until(() -> feederSubsystem.getNoteState() == NoteState.Holding || 0 == speed.getAsDouble())
        .finallyDo(() -> {
          feederSubsystem.stop();
          if (feederSubsystem.isStopBeamBroken()) {
          feederSubsystem.setNoteState(NoteState.Holding);
          }
        })
        .andThen(Commands.run(() -> feederSubsystem.setFeederSpeed(0.7 * feederSpeed.getAsDouble()))
            .onlyIf(() -> feederSubsystem.isStopBeamBroken() || NoteState.Holding == feederSubsystem.getNoteState())
            .until(() -> !feederSubsystem.isStopBeamBroken() || feederSubsystem.getNoteState() == NoteState.Loaded)
            .finallyDo(() -> {
              feederSubsystem.setNoteState(NoteState.Loaded);
              feederSubsystem.stop();
            }))
        .andThen(Commands.waitSeconds(0.1))    
        .andThen(Commands.run(() -> feederSubsystem.setFeederSpeed(-0.7 * feederSpeed.getAsDouble()))
            .onlyIf(() -> !feederSubsystem.isStopBeamBroken() || NoteState.Loaded == feederSubsystem.getNoteState())
            .until(() -> feederSubsystem.isStopBeamBroken() || feederSubsystem.getNoteState() == NoteState.Shooting)
            .finallyDo(() -> {
              feederSubsystem.setNoteState(NoteState.Shooting);
              feederSubsystem.stop();
            }));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = speed.getAsDouble();
    //intakeSubsystem.stateMachine(feederSubsystem.getNoteState() == NoteState.Empty ? velocity : 0.0);
    intakeSubsystem.setIntakeSpeed(velocity, velocity);
    if (velocity != 0) {
      // feederStopFlag = true;
      // if (feederSubsystem.isBeamBroken()) {
      // feederSubsystem.setFeederSpeed(0.0);
      // feederSubsystem.setNoteState(FeederSubsystem.NoteState.Holding);
      // } else {
      // feederSubsystem.setFeederSpeed(Math.signum(velocity) *
      // feederSpeed.getAsDouble());
      // }

      // Just need to do this if welocity is != 0
      CommandScheduler.getInstance().schedule(feederCommand);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    feederSubsystem.stop();
    intakeSubsystem.stateMachine(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
