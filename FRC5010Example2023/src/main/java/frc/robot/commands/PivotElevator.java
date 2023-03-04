// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.PivotSubsystem;

public class PivotElevator extends CommandBase {
  PivotSubsystem pivotSubsystem;
  ElevatorLevel elevatorLevel;
  /** Creates a new PivotElevator. */
  public PivotElevator(PivotSubsystem pivot, ElevatorLevel elevatorLevel) {
    this.pivotSubsystem = pivot;
    this.elevatorLevel = elevatorLevel;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.runPivotToTarget(elevatorLevel.getPivotPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (interrupted){
    //   pivotSubsystem.stopPivot();
    // } else {
    //   pivotSubsystem.stopAndHoldPivot();
    // 
    pivotSubsystem.stopAndHoldPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double goalPos = pivotSubsystem.getPivotTarget();
    double currPos = elevatorLevel.getPivotPosition();
    return pivotSubsystem.isPivotAtTarget()
    || (pivotSubsystem.isPivotMax() && pivotSubsystem.getVelocity() > 0)  
    || (pivotSubsystem.isPivotIn() && pivotSubsystem.getVelocity() < 0);
    // || (elevatorLevel.getPivotPosition() > 0 && pivotSubsystem.getPivotTarget() > 0);
  }
}
