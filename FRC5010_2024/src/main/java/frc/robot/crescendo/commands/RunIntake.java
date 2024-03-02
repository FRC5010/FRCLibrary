// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FRC5010.arch.GenericCommand;
import frc.robot.crescendo.FeederSubsystem;
import frc.robot.crescendo.FeederSubsystem.NoteState;
import frc.robot.crescendo.IntakeSubsystem;
import frc.robot.crescendo.ShooterSubsystem;

public class RunIntake extends GenericCommand {

  DoubleSupplier speed;
  DoubleSupplier feederSpeed;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  FeederSubsystem feederSubsystem;

  boolean feederStopFlag = false;

  Command feederCommand;
  Command holdingCommand;
  Command selectorCommand;
  Command loadedCommand;

  /** Creates a new RunIntake. */
  public RunIntake(DoubleSupplier joystick, DoubleSupplier feederSpeed, IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem) {
    this.speed = joystick;
    this.intakeSubsystem = intakeSubsystem;
    this.feederSpeed = feederSpeed;
    this.feederSubsystem = feederSubsystem;

    Map<NoteState, Command> intakeCommands = new HashMap<>();
    feederCommand = Commands
        .run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble()) * feederSpeed.getAsDouble()),
            feederSubsystem)
        .until(() -> feederSubsystem.getNoteState() == NoteState.Holding || 0 == speed.getAsDouble())
        .finallyDo(() -> {
          feederSubsystem.stop();
        })
        .andThen(Commands.run(() -> feederSubsystem.feederStateMachine(0.2 * feederSpeed.getAsDouble()))
            .onlyIf(() -> feederSubsystem.isStopBeamBroken() || NoteState.Holding == feederSubsystem.getNoteState())
            .until(() -> !feederSubsystem.isStopBeamBroken() || feederSubsystem.getNoteState() == NoteState.Loaded)
            .finallyDo(() -> {
              feederSubsystem.stop();
            }));

    holdingCommand = Commands.run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble() > 0 ? speed.getAsDouble() : 0) * feederSpeed.getAsDouble()),
            feederSubsystem).until(() -> 0 == speed.getAsDouble());
    loadedCommand = Commands.run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble() > 0 ? speed.getAsDouble() : 0) * feederSpeed.getAsDouble()),
            feederSubsystem).until(() -> 0 == speed.getAsDouble());
    intakeCommands.put(NoteState.Empty, feederCommand);
    intakeCommands.put(NoteState.Holding, holdingCommand);
    intakeCommands.put(NoteState.Loaded, loadedCommand);

    selectorCommand = Commands.select(intakeCommands, () -> feederSubsystem.getNoteState());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {

  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double velocity = speed.getAsDouble();
    //intakeSubsystem.stateMachine(feederSubsystem.getNoteState() == NoteState.Empty ? velocity : 0.0);
    // intakeSubsystem.setIntakeSpeed(velocity, velocity);
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
      CommandScheduler.getInstance().schedule(selectorCommand);
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
