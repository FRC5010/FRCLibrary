// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FRC5010.constants.GenericCommand;
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

  /** Creates a new RunIntake. */
  public RunIntake(DoubleSupplier joystick, DoubleSupplier feeder, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    this.speed = joystick;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.feederSpeed = feeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {

  }

  Command feederCommand = Commands
      .run(() -> feederSubsystem.setFeederSpeed(Math.signum(speed.getAsDouble()) * feederSpeed.getAsDouble()),
          feederSubsystem)
      .until(() -> feederSubsystem.isBeamBroken() || 0 == speed.getAsDouble())
      .finallyDo(() -> {
        feederSubsystem.stop();
        if (feederSubsystem.isBeamBroken()) {
          feederSubsystem.setNoteState(NoteState.Holding);
        }
      })
      .andThen(Commands.run(() -> feederSubsystem.setFeederSpeed(-0.1))
          .onlyIf(() -> feederSubsystem.isBeamBroken() || NoteState.Holding == feederSubsystem.getNoteState())
          .until(() -> !feederSubsystem.isBeamBroken())
          .finallyDo(() -> {
            feederSubsystem.setNoteState(NoteState.Loaded);
            feederSubsystem.stop();
          })
      );

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = speed.getAsDouble();
    intakeSubsystem.stateMachine(!feederSubsystem.isBeamBroken() ? velocity : 0.0);
    if (velocity != 0) {
      feederStopFlag = true;
      if (feederSubsystem.isBeamBroken()) {
        feederSubsystem.setFeederSpeed(0.0);
        feederSubsystem.setNoteState(FeederSubsystem.NoteState.Holding);
      } else {
        feederSubsystem.setFeederSpeed(Math.signum(velocity) * feederSpeed.getAsDouble());
      }

      // Just need to do this if welocity is != 0
      // CommandScheduler.getInstance().schedule(feederCommand);
    } else if (feederStopFlag) {
      feederStopFlag = false;
      feederSubsystem.stop();
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
