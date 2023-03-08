// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.PivotSubsystem;

public class HomePivot extends CommandBase {
  /** Creates a new HomePivot. */
  PivotSubsystem pivotSubsystem;
  public HomePivot(PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("test");
    pivotSubsystem.pivotPow(-0.8, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopAndHoldPivot();
    System.out.println("Break");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSubsystem.isPivotMin();
  }
}
