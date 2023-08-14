// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.CubeCruzerPivotSubsystem;

public class PivotReset extends CommandBase {
  private CubeCruzerPivotSubsystem pivotSubsystem;

  /** Creates a new PivotReset. */
  public PivotReset(CubeCruzerPivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSubsystem.pivotPow(-0.5, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.pivotPow(0, false);
    pivotSubsystem.setPivotEncoderPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSubsystem.isPivotMinHallEffect();
  }
}
