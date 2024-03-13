// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import frc.robot.FRC5010.arch.GenericCommand;
import frc.robot.chargedup.PivotSubsystem;

public class HomePivot extends GenericCommand {
  /** Creates a new HomePivot. */
  PivotSubsystem pivotSubsystem;

  public HomePivot(PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.runPivotToTarget(-45);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    pivotSubsystem.stopAndHoldPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSubsystem.isPivotMinHallEffect();
  }
}
