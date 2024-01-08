// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import frc.robot.FRC5010.constants.GenericCommand;
import frc.robot.chargedup.PivotSubsystem;

public class PivotReset extends GenericCommand {
  private PivotSubsystem pivotSubsystem;

  /** Creates a new PivotReset. */
  public PivotReset(PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    pivotSubsystem.pivotPow(-0.5, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    pivotSubsystem.pivotPow(0, false);
    pivotSubsystem.setPivotEncoderPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSubsystem.isPivotMinHallEffect();
  }
}
