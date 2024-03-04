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
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.crescendo.PivotSubsystem;
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
  PivotSubsystem pivotSubsystem;

  boolean feederStopFlag = false;
  Controller rumbleController;

  Command feederCommand;
  Command holdingCommand;
  Command selectorCommand;
  Command loadedCommand;

  /** Creates a new RunIntake. */
  public RunIntake(DoubleSupplier joystick, DoubleSupplier feederSpeed, IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem, Controller rumbleController) {
    this.speed = joystick;
    this.intakeSubsystem = intakeSubsystem;
    this.feederSpeed = feederSpeed;
    this.feederSubsystem = feederSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.rumbleController = rumbleController;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    Map<NoteState, Command> intakeCommands = new HashMap<>();
    feederCommand = Commands
        .run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble()) * feederSpeed.getAsDouble()),
            feederSubsystem)
        .until(() -> feederSubsystem.getNoteState() == NoteState.Holding || 0 == speed.getAsDouble())
        .finallyDo(() -> {
          feederSubsystem.feederStateMachine(0);

        })
        .andThen(

          Commands.run(() -> feederSubsystem.feederStateMachine(0.5 * feederSpeed.getAsDouble()))
            .onlyIf(() -> feederSubsystem.isStopBeamBroken() || NoteState.Holding == feederSubsystem.getNoteState())
            .until(() -> !feederSubsystem.isStopBeamBroken() || feederSubsystem.getNoteState() == NoteState.Loaded)
            .finallyDo(() -> {
              feederSubsystem.feederStateMachine(0);
            
            }).andThen(
              Commands.runOnce(() -> rumbleController.setRumble(1)).andThen(
              Commands.waitSeconds(1)).finallyDo(() -> rumbleController.setRumble(0))
            ).onlyIf(() -> null != rumbleController && feederSubsystem.getNoteState() == NoteState.Loaded)
          );

    holdingCommand = Commands.run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble() > 0 ? speed.getAsDouble() : 0) * feederSpeed.getAsDouble()),
            feederSubsystem).until(() -> 0 == speed.getAsDouble());
    loadedCommand = Commands.run(() -> feederSubsystem.feederStateMachine(Math.signum(speed.getAsDouble() > 0 ? speed.getAsDouble() : 0) * feederSpeed.getAsDouble()),
            feederSubsystem).until(() -> 0 == speed.getAsDouble());
    intakeCommands.put(NoteState.Empty, feederCommand);
    intakeCommands.put(NoteState.Holding, holdingCommand);
    intakeCommands.put(NoteState.Loaded, loadedCommand);

    selectorCommand = Commands.select(intakeCommands, () -> feederSubsystem.getNoteState());
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    double velocity = speed.getAsDouble();
    intakeSubsystem.stateMachine(feederSubsystem.getNoteState() == NoteState.Empty ? velocity : feederSubsystem.getNoteState() == NoteState.Holding ? -velocity : 0);
    
    // intakeSubsystem.setIntakeSpeed(velocity, velocity);
    if (velocity != 0) {
      if (velocity < 0) {
        pivotSubsystem.setReference(pivotSubsystem.INTAKE_LEVEL);
      }
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
